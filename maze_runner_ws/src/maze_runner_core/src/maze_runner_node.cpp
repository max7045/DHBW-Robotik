#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <algorithm>
#include <map>

// Definition der Roboter-Zustände
enum class RobotMode { MANUAL, AUTONOMOUS };
enum class RobotState { DRIVE_FORWARD, TURNING, GOAL_REACHED };

// Parameter-Strukturen für sauberes Datenmanagement
struct ControllerParams {
    int toggle_btn;
    int reset_btn;
    int axis_linear;
    int axis_angular;
    bool start_manual;
};

struct DriveParams {
    double v_linear;
    double w_min;
    double w_max;
    double dist_turn;
    double dist_free_path;
    double exit_thresh;
};

struct PidParams {
    double kp_heading;
    double kp_center;
    double kd_center;
};

class MazeRunnerControl : public rclcpp::Node {
public:
    MazeRunnerControl() : Node("maze_runner_control") {
        load_parameters();
        
        current_mode_ = ctrl_params_.start_manual ? RobotMode::MANUAL : RobotMode::AUTONOMOUS;
        RCLCPP_INFO(this->get_logger(), "Startmodus: %s", ctrl_params_.start_manual ? "MANUELL" : "AUTONOM");
        
        // TF2 initialisieren fuer die Kompass-Lokalisierung
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // ROS 2 Kommunikation aufbauen
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, 
            std::bind(&MazeRunnerControl::scan_callback, this, std::placeholders::_1));
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, 
            std::bind(&MazeRunnerControl::joy_callback, this, std::placeholders::_1));
    }

private:
    // --- VARIABLEN ---
    ControllerParams ctrl_params_;
    DriveParams drive_params_;
    PidParams pid_params_;

    RobotMode current_mode_ = RobotMode::MANUAL;
    RobotState auto_state_ = RobotState::DRIVE_FORWARD;
    
    // Odometrie & Zustand
    double current_x_ = 0.0, current_y_ = 0.0, current_yaw_ = 0.0;
    double target_yaw_ = 0.0;
    bool is_initialized_ = false;
    float prev_center_error_ = 0.0; 
    
    // Labyrinth-Gedächtnis (Trémaux)
    double maze_scale_ = 0.0;
    bool evaluated_this_cell_ = false;
    std::map<std::pair<int, int>, int> grid_memory_;

    bool last_btn_a_state_ = false;
    bool last_btn_b_state_ = false;

    // ROS 2 Objekte
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    // --- INITIALISIERUNG ---
    
    // Lädt alle ROS 2 Parameter einmalig in strukturierte Variablen
    void load_parameters() {
        ctrl_params_.toggle_btn = this->declare_parameter("toggle_mode_button", 0);
        ctrl_params_.reset_btn = this->declare_parameter("reset_button", 1);
        ctrl_params_.axis_linear = this->declare_parameter("linear_axis", 1);
        ctrl_params_.axis_angular = this->declare_parameter("angular_axis", 2);
        ctrl_params_.start_manual = this->declare_parameter("use_controller", false);

        drive_params_.v_linear = this->declare_parameter("speed_linear", 1.1);
        drive_params_.w_min = this->declare_parameter("speed_turn_min", 0.2);
        drive_params_.w_max = this->declare_parameter("speed_turn_max", 0.6);
        drive_params_.dist_turn = this->declare_parameter("dist_turn", 0.75);
        drive_params_.dist_free_path = this->declare_parameter("dist_free_path", 1.2);
        drive_params_.exit_thresh = this->declare_parameter("exit_open_space_dist", 2.0);

        pid_params_.kp_heading = this->declare_parameter("kp_heading", 1.0);
        pid_params_.kp_center = this->declare_parameter("kp_center", 0.3);
        pid_params_.kd_center = this->declare_parameter("kd_center", 0.6);
    }

    // --- EINGABE-VERARBEITUNG ---

    // Verarbeitet Gamepad-Eingaben für Moduswechsel und Handsteuerung
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        bool btn_a = msg->buttons[ctrl_params_.toggle_btn] == 1;
        bool btn_b = msg->buttons[ctrl_params_.reset_btn] == 1;

        // Moduswechsel auslösen
        if (btn_a && !last_btn_a_state_) toggle_mode();
        
        // Reset und Teleport auslösen
        if (btn_b && !last_btn_b_state_) handle_reset();
        
        last_btn_a_state_ = btn_a;
        last_btn_b_state_ = btn_b;

        // Handsteuerung ausführen, falls aktiv
        if (current_mode_ == RobotMode::MANUAL) {
            execute_manual_drive(msg);
        }
    }

    // --- AUTONOMIE-PIPELINE ---

    // Hauptschleife für die autonome Navigation, strukturiert als Pipeline
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (current_mode_ != RobotMode::AUTONOMOUS || auto_state_ == RobotState::GOAL_REACHED) {
            return;
        }

        if (!measure_maze_scale(msg)) return; // Warten auf Gangerkennung
        if (check_goal_reached(msg)) return;  // Prüfen ob Ausgang erreicht
        if (!update_robot_pose()) return;     // Warten auf TF-Daten

        auto twist = geometry_msgs::msg::Twist();

        if (auto_state_ == RobotState::TURNING) {
            execute_turn(twist);
        } else {
            evaluate_intersection(msg, twist);
            
            // Nur fahren, wenn keine neue Drehung eingeleitet wurde
            if (auto_state_ != RobotState::TURNING) {
                drive_and_center(msg, twist);
            }
        }

        cmd_pub_->publish(twist);
    }

    // --- UNTERPROGRAMME AUTONOMIE ---

    // Ermittelt initial die Gangbreite des Labyrinths
    bool measure_maze_scale(const sensor_msgs::msg::LaserScan::SharedPtr& msg) {
        if (maze_scale_ > 0.0) return true; // Bereits skaliert

        float dl = get_min_dist_cone(msg, M_PI / 2.0, 5);
        float dr = get_min_dist_cone(msg, -M_PI / 2.0, 5);
        
        if (dl < 5.0 && dr < 5.0) {
            maze_scale_ = dl + dr;
            RCLCPP_INFO(this->get_logger(), "Labyrinth skaliert: %.2f m", maze_scale_);
            return true;
        }
        return false;
    }

    // Überprüft anhand freier Sicht nach vorne, ob das Ziel erreicht ist
    bool check_goal_reached(const sensor_msgs::msg::LaserScan::SharedPtr& msg) {
        float min_front_dist = std::numeric_limits<float>::max();
        int start_idx = msg->ranges.size() / 4;       // -90 Grad
        int end_idx = 3 * msg->ranges.size() / 4;     // +90 Grad
        
        for (int i = start_idx; i <= end_idx; ++i) {
            float d = msg->ranges[i];
            if (!std::isnan(d) && !std::isinf(d) && d > msg->range_min) {
                min_front_dist = std::min(min_front_dist, d);
            }
        }
        
        if (min_front_dist > drive_params_.exit_thresh) {
            RCLCPP_INFO(this->get_logger(), "ZIEL ERREICHT!");
            auto_state_ = RobotState::GOAL_REACHED;
            cmd_pub_->publish(geometry_msgs::msg::Twist()); // Stoppen
            return true;
        }
        return false;
    }

    // Aktualisiert die globale Roboterposition via TF2
    bool update_robot_pose() {
        geometry_msgs::msg::TransformStamped tf;
        try {
            tf = tf_buffer_->lookupTransform("map", "base_footprint", tf2::TimePointZero);
        } catch (const tf2::TransformException &ex) {
            try { // Fallback auf Odom
                tf = tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
            } catch (const tf2::TransformException &ex2) { return false; }
        }

        current_x_ = tf.transform.translation.x;
        current_y_ = tf.transform.translation.y;
        
        tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y, 
                          tf.transform.rotation.z, tf.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, current_yaw_);

        // Startwinkel beim ersten Durchlauf speichern
        if (!is_initialized_) {
            target_yaw_ = current_yaw_;
            is_initialized_ = true;
        }
        return true;
    }

    // Führt 90-Grad-Drehungen aus und beendet den Turn-State
    void execute_turn(geometry_msgs::msg::Twist& twist) {
        double yaw_diff = target_yaw_ - current_yaw_;
        yaw_diff = std::atan2(std::sin(yaw_diff), std::cos(yaw_diff)); // Normalisieren

        if (std::abs(yaw_diff) < 0.01) {
            auto_state_ = RobotState::DRIVE_FORWARD; 
            twist.angular.z = 0.0; 
            prev_center_error_ = 0.0; 
        } else {
            twist.linear.x = 0.0; 
            float turn_speed = 1.5 * yaw_diff;
            
            // Begrenzen der Drehgeschwindigkeit (Clamp)
            if (turn_speed > 0) twist.angular.z = std::clamp(turn_speed, (float)drive_params_.w_min, (float)drive_params_.w_max);
            else twist.angular.z = std::clamp(turn_speed, (float)-drive_params_.w_max, (float)-drive_params_.w_min);
        }
    }

    // Trémaux-Algorithmus: Evaluiert Kreuzungen und wählt den Pfad
    void evaluate_intersection(const sensor_msgs::msg::LaserScan::SharedPtr& msg, geometry_msgs::msg::Twist& twist) {
        int current_gx = std::round(current_x_ / maze_scale_);
        int current_gy = std::round(current_y_ / maze_scale_);
        double center_x = current_gx * maze_scale_;
        double center_y = current_gy * maze_scale_;
        
        double dist_to_center = std::hypot(current_x_ - center_x, current_y_ - center_y);

        // Reset der Evaluierung, wenn wir die Zellmitte verlassen
        if (dist_to_center > maze_scale_ * 0.35) {
            evaluated_this_cell_ = false; 
        }

        // Nur im Zentrum einer Zelle entscheiden
        if (dist_to_center >= 0.15 || evaluated_this_cell_) return;

        evaluated_this_cell_ = true;
        grid_memory_[{current_gx, current_gy}] += 1; 

        float dist_front = get_min_dist_cone(msg, 0.0, 5);
        float dist_left  = get_min_dist_cone(msg, M_PI / 2.0, 5);
        float dist_right = get_min_dist_cone(msg, -M_PI / 2.0, 5);

        double open_thresh = maze_scale_ * 0.65; 
        bool can_go_straight = (dist_front > open_thresh);
        bool can_go_left   = (dist_left > open_thresh);
        bool can_go_right  = (dist_right > open_thresh);

        // Nur agieren bei Abzweigungen oder Sackgassen
        if (can_go_straight && !can_go_left && !can_go_right) return;

        double yaw_straight = target_yaw_;
        double yaw_left = target_yaw_ + (M_PI / 2.0);
        double yaw_right = target_yaw_ - (M_PI / 2.0);

        auto get_gx = [&](double yaw) { return current_gx + std::round(std::cos(yaw)); };
        auto get_gy = [&](double yaw) { return current_gy + std::round(std::sin(yaw)); };

        // Zähle Besuche benachbarter Zellen (9999 = Wand)
        int v_straight = can_go_straight ? grid_memory_[{get_gx(yaw_straight), get_gy(yaw_straight)}] : 9999;
        int v_left = can_go_left ? grid_memory_[{get_gx(yaw_left), get_gy(yaw_left)}] : 9999;
        int v_right = can_go_right ? grid_memory_[{get_gx(yaw_right), get_gy(yaw_right)}] : 9999;
        
        int min_visits = std::min({v_straight, v_left, v_right});

        // Weg priorisieren: Bevorzuge unbesuchte Wege und Geradeausfahrt
        if (min_visits < 9999) {
            if (v_straight == min_visits) {
                target_yaw_ = yaw_straight; 
            } else if (v_left == min_visits) {
                target_yaw_ = yaw_left;
                auto_state_ = RobotState::TURNING;
            } else {
                target_yaw_ = yaw_right;
                auto_state_ = RobotState::TURNING;
            }
        } else {
            // Sackgasse: 180 Grad drehen
            target_yaw_ += M_PI; 
            auto_state_ = RobotState::TURNING;
        }
        
        // Zielwinkel am Labyrinth-Raster einrasten
        target_yaw_ = std::round(target_yaw_ / (M_PI / 2.0)) * (M_PI / 2.0);
        
        // Bei neuer Drehung sofort Bremsen
        if (auto_state_ == RobotState::TURNING) {
            twist.linear.x = 0.0;
        }
    }

    // Geradeausfahrt mit PD-Regler zur perfekten Wandzentrierung
    void drive_and_center(const sensor_msgs::msg::LaserScan::SharedPtr& msg, geometry_msgs::msg::Twist& twist) {
        float dist_front = get_min_dist_cone(msg, 0.0, 5);
        float dist_left  = get_min_dist_cone(msg, M_PI / 2.0, 5);
        float dist_right = get_min_dist_cone(msg, -M_PI / 2.0, 5);

        twist.linear.x = drive_params_.v_linear; 
        
        // Notbremse vor Wänden
        if (dist_front < maze_scale_ * 0.3) twist.linear.x = 0.0;

        double yaw_error = target_yaw_ - current_yaw_;
        yaw_error = std::atan2(std::sin(yaw_error), std::cos(yaw_error));
        
        float center_error = 0.0;
        float d_error = 0.0; 
        
        // Zentrierung nur anwenden, wenn links UND rechts Wände existieren
        if (dist_left < drive_params_.dist_free_path && dist_right < drive_params_.dist_free_path) {
            center_error = dist_left - dist_right;
            d_error = center_error - prev_center_error_; 
        }
        prev_center_error_ = center_error; 

        // PD-Regler Ausgang berechnen
        twist.angular.z = (pid_params_.kp_heading * yaw_error) + 
                          (pid_params_.kp_center * center_error) + 
                          (pid_params_.kd_center * d_error);
                          
        twist.angular.z = std::clamp(twist.angular.z, -drive_params_.w_max, drive_params_.w_max);
    }

    // --- HILFSFUNKTIONEN ---

    // Ermittelt den geringsten Abstand in einem bestimmten Laserscan-Kegel
    float get_min_dist_cone(const sensor_msgs::msg::LaserScan::SharedPtr& msg, double angle, int spread) {
        int center = std::round((angle - msg->angle_min) / msg->angle_increment);
        float min_d = 10.0f;
        for (int i = center - spread; i <= center + spread; ++i) {
            int idx = (i + msg->ranges.size()) % msg->ranges.size();
            float d = msg->ranges[idx];
            if (!std::isnan(d) && !std::isinf(d) && d > 0.05) {
                min_d = std::min(min_d, d);
            }
        }
        return min_d;
    }

    // Verarbeitet manuelle Gamepad-Eingaben
    void execute_manual_drive(const sensor_msgs::msg::Joy::SharedPtr& msg) {
        auto twist = geometry_msgs::msg::Twist();
        float lin_in = msg->axes[ctrl_params_.axis_linear];  
        float ang_in = msg->axes[ctrl_params_.axis_angular]; 

        // Fahrassistent: Geradeaus korrigieren bei starken Vorwärtsbewegungen
        if (std::abs(lin_in) > 0.5 && std::abs(ang_in) < 0.3) ang_in = 0.0;
        
        if (std::abs(lin_in) > 0.15) twist.linear.x = lin_in * drive_params_.v_linear;
        if (std::abs(ang_in) > 0.15) twist.angular.z = ang_in * drive_params_.w_max * 2.0;
        
        cmd_pub_->publish(twist);
    }

    // Wechselt zwischen autonomen und manuellen Modus
    void toggle_mode() {
        cmd_pub_->publish(geometry_msgs::msg::Twist()); // Stoppbefehl senden

        if (current_mode_ == RobotMode::MANUAL) {
            current_mode_ = RobotMode::AUTONOMOUS;
            auto_state_ = RobotState::DRIVE_FORWARD; 
            is_initialized_ = false; 
            prev_center_error_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "Modus: AUTONOM");
        } else {
            current_mode_ = RobotMode::MANUAL;
            RCLCPP_INFO(this->get_logger(), "Modus: MANUELL");
        }
    }

    // Setzt Algorithmus zurück und teleportiert via Gazebo-Befehl
    void handle_reset() {
        cmd_pub_->publish(geometry_msgs::msg::Twist()); 
        
        current_mode_ = ctrl_params_.start_manual ? RobotMode::MANUAL : RobotMode::AUTONOMOUS;
        auto_state_ = RobotState::DRIVE_FORWARD;
        is_initialized_ = false; 
        prev_center_error_ = 0.0; 
        maze_scale_ = 0.0;
        grid_memory_.clear();
        evaluated_this_cell_ = false;
        
        RCLCPP_WARN(this->get_logger(), "Reset ausgeführt. Teleportiere Roboter...");

        // Gazebo System-Call bleibt bestehen
        system("gz service -s /world/maze_world/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000 "
               "--req \"name: 'maze_runner', position: {x: 1.5, y: 0, z: 0.2}, orientation: {x: 0, y: 0, z: 0.7071, w: 0.7071}\" &");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MazeRunnerControl>());
    rclcpp::shutdown();
    return 0;
}