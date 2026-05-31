#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <algorithm>

// Verfuegbare Zustaende des Roboters
enum class RobotMode { MANUAL, AUTONOMOUS };
enum class RobotState { DRIVE_FORWARD, TURNING, GOAL_REACHED };

class MazeRunnerControl : public rclcpp::Node {
public:
    MazeRunnerControl() : Node("maze_runner_control") {
        
        // --- 1. ZENTRALE PARAMETER ---

        // Controller
        this->declare_parameter("toggle_mode_button", 0);   // Modus wechseln (A-Taste) 
        this->declare_parameter("reset_button", 1);         // Reset & Teleport (B-Taste)
        this->declare_parameter("linear_axis", 1);          // Vor-/Rueckwaerts (linker Stick vertikal)
        this->declare_parameter("angular_axis", 2);         // Drehung (rechter Stick horizontal)

        // Antrieb
        this->declare_parameter("speed_linear", 0.8);        // Vorwaertsgeschwindigkeit
        this->declare_parameter("speed_turn_min", 0.2);      // Min. Drehgeschwindigkeit (gegen Steckenbleiben)
        this->declare_parameter("speed_turn_max", 0.6);      // Max. Drehgeschwindigkeit
        
        // Sensorik & Distanzen
        this->declare_parameter("dist_turn", 0.75);          // Bremsdistanz vor einer Wand
        this->declare_parameter("dist_free_path", 1.2);      // Ab wann ein seitlicher Gang als "frei" gilt
        this->declare_parameter("exit_open_space_dist", 2.0);// Schwellenwert zur Erkennung des Ausgangs
        
        // Regler-Gewichte (PID)
        this->declare_parameter("kp_heading", 1.0);          // Staerke des Kompass-Reglers (Parallelfahrt)
        this->declare_parameter("kp_center", 0.3);           // Staerke der Wand-Zentrierung
        this->declare_parameter("kd_center", 0.6);           // Daempfung (verhindert Schlangenlinien)

        // Startmodus evaluieren
        this->declare_parameter<bool>("use_controller", false);
        bool use_controller;
        this->get_parameter("use_controller", use_controller);
        
        current_mode_ = use_controller ? RobotMode::MANUAL : RobotMode::AUTONOMOUS;
        RCLCPP_INFO(this->get_logger(), "Startmodus: %s", use_controller ? "MANUELL" : "AUTONOM");

        // TF2 initialisieren fuer die Kompass-Lokalisierung
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // ROS 2 Kommunikation
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&MazeRunnerControl::scan_callback, this, std::placeholders::_1));
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, std::bind(&MazeRunnerControl::joy_callback, this, std::placeholders::_1));
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        bool btn_a = msg->buttons[this->get_parameter("toggle_mode_button").as_int()] == 1; // A-Taste: Modus wechseln
        bool btn_b = msg->buttons[this->get_parameter("reset_button").as_int()] == 1; // B-Taste: Reset & Teleport

        if (btn_a && !last_btn_a_state_) toggle_mode();
        
        // --- HARD-RESET & TELEPORT ---
        if (btn_b && !last_btn_b_state_) {
            cmd_pub_->publish(geometry_msgs::msg::Twist()); // Sicherer Stopp
            
            bool use_controller;
            this->get_parameter("use_controller", use_controller);
            current_mode_ = use_controller ? RobotMode::MANUAL : RobotMode::AUTONOMOUS;
            
            // Logik-Reset durchfuehren
            auto_state_ = RobotState::DRIVE_FORWARD;
            is_initialized_ = false; 
            prev_center_error_ = 0.0; 
            
            RCLCPP_WARN(this->get_logger(), "[RESET] Zustand genullt. Teleportiere...");

            // Asynchroner Teleport via Gazebo Service Call zum Startpunkt
            system("gz service -s /world/maze_world/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000 "
                   "--req \"name: 'maze_runner', position: {x: 1.5, y: 1.5, z: 0.2}, orientation: {x: 0, y: 0, z: 0.7071, w: 0.7071}\" &");
        }
        
        last_btn_a_state_ = btn_a;
        last_btn_b_state_ = btn_b;

        // Manuelle Controller-Steuerung
        if (current_mode_ == RobotMode::MANUAL) {
            auto twist = geometry_msgs::msg::Twist();
            float linear_input = msg->axes[this->get_parameter("linear_axis").as_int()];  
            float angular_input = msg->axes[this->get_parameter("angular_axis").as_int()]; 

            // Totzone und Geradeaus-Fahrassistent
            if (std::abs(linear_input) > 0.5 && std::abs(angular_input) < 0.3) 
            {
                angular_input = 0.0;
            }
            if (std::abs(linear_input) > 0.15) {
                twist.linear.x = linear_input * this->get_parameter("speed_linear").as_double();
            }
            if (std::abs(angular_input) > 0.15) {
                twist.angular.z = angular_input * this->get_parameter("speed_turn_max").as_double() * 2.0;
            }
            cmd_pub_->publish(twist);
        }
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (current_mode_ != RobotMode::AUTONOMOUS) return;
        if (auto_state_ == RobotState::GOAL_REACHED) return; // Nichts tun, wenn fertig

        // --- 2. ZIELERKENNUNG (Sensorbasiert) ---
        // Der Sensor tastet 360 Grad ab (-180 bis +180).
        // Wir prüfen das vordere Halbfeld (Index 90 bis 270 entspricht -90 bis +90 Grad).
        float absolute_min_dist = 10.0f; // Startwert (Unendlich)
        int start_idx = msg->ranges.size() / 4;       // ~ -90 Grad (Rechts)
        int end_idx = 3 * msg->ranges.size() / 4;     // ~ +90 Grad (Links)
        
        for (int i = start_idx; i <= end_idx; ++i) {
            float d = msg->ranges[i];
            // Ignoriere Sensor-Rauschen oder 'Infinity' Werte ins Leere
            if (!std::isnan(d) && !std::isinf(d) && d > msg->range_min) {
                if (d < absolute_min_dist) absolute_min_dist = d;
            }
        }
        
        // Wenn das naechste Objekt weiter als 2.0m weg ist, sind wir aus dem Labyrinth ausgebrochen!
        double exit_dist = this->get_parameter("exit_open_space_dist").as_double();
        if (absolute_min_dist > exit_dist) {
            RCLCPP_INFO(this->get_logger(), "ZIEL ERREICHT!");
            auto_state_ = RobotState::GOAL_REACHED;
            cmd_pub_->publish(geometry_msgs::msg::Twist()); // Dauerhaft bremsen
            return;
        }

        //Aktuelle Position bestimmen
        geometry_msgs::msg::TransformStamped transform;
        try {
            // SLAM map-Frame nutzen fuer absolute Praezision
            transform = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
        } catch (const tf2::TransformException & ex) {
            try {
                // Fallback auf Odometrie waehrend der SLAM Startphase
                transform = tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
            } catch (const tf2::TransformException & ex2) { return; }
        }
        
        // Aktuelle Rotatian aus Quaternion extrahieren
        tf2::Quaternion q(transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        // Roll, Pitch und Yaw (Ausrichtung) berechnen
        double roll, pitch, current_yaw;
        m.getRPY(roll, pitch, current_yaw); // Ausrichtung extrahieren

        // Startwinkel einrasten lassen
        if (!is_initialized_) {
            target_yaw_ = std::round(current_yaw / (M_PI / 2.0)) * (M_PI / 2.0);
            is_initialized_ = true;
        }

        auto twist = geometry_msgs::msg::Twist();
        
        // Parameter fuer den aktuellen Berechnungszyklus abgreifen
        double v_lin = this->get_parameter("speed_linear").as_double();
        double w_min = this->get_parameter("speed_turn_min").as_double();
        double w_max = this->get_parameter("speed_turn_max").as_double();
        double turn_dist = this->get_parameter("dist_turn").as_double();
        double free_path = this->get_parameter("dist_free_path").as_double();

        // --- 3. DREHMANOEVER ---
        if (auto_state_ == RobotState::TURNING) {
            double yaw_diff = target_yaw_ - current_yaw;
            yaw_diff = std::atan2(std::sin(yaw_diff), std::cos(yaw_diff)); // auf -PI bis PI zwingen

            // Stoppen bei Erreichen des Zielwinkels
            if (std::abs(yaw_diff) < 0.05) {
                auto_state_ = RobotState::DRIVE_FORWARD; 
                twist.angular.z = 0.0; 
                prev_center_error_ = 0.0; 
            } else {
                twist.linear.x = 0.0; 
                float turn_speed = 1.5 * yaw_diff;
                // Geschwindigkeit kappen, um nicht umzukippen oder steckenzubleiben
                if (turn_speed > 0) turn_speed = std::clamp(turn_speed, (float)w_min, (float)w_max);
                else turn_speed = std::clamp(turn_speed, (float)-w_max, (float)-w_min);
                
                twist.angular.z = turn_speed;
            }
            cmd_pub_->publish(twist);
            return; 
        }

        // --- 4. VORWAERTS FAHREN UND ZENTRIEREN ---
        auto get_min_dist_cone = [&](double angle, int spread) {
            int center = std::round((angle - msg->angle_min) / msg->angle_increment);
            float min_d = 10.0f;
            for (int i = center - spread; i <= center + spread; ++i) {
                int idx = (i + msg->ranges.size()) % msg->ranges.size();
                float d = msg->ranges[idx];
                if (!std::isnan(d) && !std::isinf(d) && d > 0.05) {
                    if (d < min_d) min_d = d;
                }
            }
            return min_d;
        };

        float dist_front = get_min_dist_cone(0.0, 5);
        float dist_left  = get_min_dist_cone(M_PI / 2.0, 5);
        float dist_right = get_min_dist_cone(-M_PI / 2.0, 5);

        if (dist_front > turn_dist) {
            twist.linear.x = v_lin; 
            
            // Fehlerberechnung
            double yaw_error = target_yaw_ - current_yaw;
            yaw_error = std::atan2(std::sin(yaw_error), std::cos(yaw_error));
            float center_error = 0.0;
            float d_error = 0.0; 
            
            // Nur eingreifen, wenn beidseitig Waende abgetastet werden
            if (dist_left < free_path && dist_right < free_path) {
                center_error = dist_left - dist_right;
                d_error = center_error - prev_center_error_; 
            }
            prev_center_error_ = center_error; 

            // PD-Heading Regler anwenden
            double kp_h = this->get_parameter("kp_heading").as_double();
            double kp_c = this->get_parameter("kp_center").as_double();
            double kd_c = this->get_parameter("kd_center").as_double();

            twist.angular.z = (kp_h * yaw_error) + (kp_c * center_error) + (kd_c * d_error);
            twist.angular.z = std::clamp(twist.angular.z, -w_max, w_max); // Motor schützen
            
        } else {
            // Wand blockiert -> Wegfindung anwenden
            auto_state_ = RobotState::TURNING;
            twist.linear.x = 0.0;
            
            if (dist_left > free_path) target_yaw_ += (M_PI / 2.0);   ble kd_c = this->get_parameter("kd_center").as_double();

            twist.angular.z = (kp_h * yaw_error) + (kp_c * center_error) + (kd_c * d_   // Links frei
            else if (dist_right > free_path) target_yaw_ -= (M_PI / 2.0);// Rechts frei
            else target_yaw_ += M_PI;                                    // Sackgasse
            
            // Kompass-Wert wieder auf Himmelsrichtung arretieren
            target_yaw_ = std::round(target_yaw_ / (M_PI / 2.0)) * (M_PI / 2.0);
        }
        
        cmd_pub_->publish(twist);
    }

    void toggle_mode() {
        cmd_pub_->publish(geometry_msgs::msg::Twist()); 

        if (current_mode_ == RobotMode::MANUAL) {
            current_mode_ = RobotMode::AUTONOMOUS;
            auto_state_ = RobotState::DRIVE_FORWARD; 
            is_initialized_ = false; 
            prev_center_error_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "[MODUS] Gewechselt zu: AUTONOM");
        } else {
            current_mode_ = RobotMode::MANUAL;
            RCLCPP_INFO(this->get_logger(), "[MODUS] Gewechselt zu: MANUELL");
        }
    }

    RobotMode current_mode_ = RobotMode::MANUAL;
    RobotState auto_state_ = RobotState::DRIVE_FORWARD;
    
    double target_yaw_ = 0.0;
    bool is_initialized_ = false;
    float prev_center_error_ = 0.0; 
    
    bool last_btn_a_state_ = false;
    bool last_btn_b_state_ = false;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MazeRunnerControl>());
    rclcpp::shutdown();
    return 0;
}