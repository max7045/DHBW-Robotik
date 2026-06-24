#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <algorithm>
#include <map>

// Definition der Roboter-Zustände
enum class RobotMode { MANUAL, AUTONOMOUS };
enum class RobotState { DRIVE_FORWARD, DECELERATING, TURNING, GOAL_REACHED };

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
        RCLCPP_INFO(this->get_logger(), "\033[1;34mStartmodus: %s\033[0m", ctrl_params_.start_manual ? "MANUELL" : "AUTONOM");
        
        // TF2 initialisieren fuer die Kompass-Lokalisierung
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // ROS 2 Kommunikation aufbauen
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, 
            std::bind(&MazeRunnerControl::scan_callback, this, std::placeholders::_1));
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("/joy", 10, 
            std::bind(&MazeRunnerControl::joy_callback, this, std::placeholders::_1));
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/tremaux_markers_array", 10);

        RCLCPP_INFO(this->get_logger(), "\033[1;34mMazeRunnerControl Node erfolgreich initialisiert.\033[0m");
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
    bool evaluated_this_cell_ = true; // Startet auf true, um die Startbox nicht fälschlicherweise zu evaluieren
    std::map<std::pair<int, int>, int> grid_memory_;
    bool grid_initialized_ = false;
    int last_gx_ = 0;
    int last_gy_ = 0;
    std::string pose_frame_id_ = "map";
    bool enable_debug_ = true;

    bool last_btn_a_state_ = false;
    bool last_btn_b_state_ = false;
    rclcpp::Time last_reset_time_;
    bool is_reset_time_set_ = false;
    rclcpp::Time decel_start_time_;

    // ROS 2 Objekte
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
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
        enable_debug_ = this->declare_parameter("enable_debug", true);
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
        if (current_mode_ != RobotMode::AUTONOMOUS) {
            return;
        }
        if (auto_state_ == RobotState::GOAL_REACHED) {
            return;
        }

        // 1. Sicherheits-Check: Leere Laserscans ignorieren (Schutz vor Absturz bei leeren Daten)
        if (msg->ranges.empty()) {
            return;
        }

        // 2. Startup-/Reset-Verzögerung (2.0 Sekunden Sim-Zeit):
        // Verhindert das Auslesen fehlerhafter Sensordaten während des Spawns/Drops
        // oder Teleports, und gibt EKF/TF-Tree Zeit zum Einschwingen.
        if (!is_reset_time_set_) {
            // Warte, bis der Clock-Empfänger die erste gültige Simulationszeit geliefert hat (> 0)
            if (this->now().nanoseconds() > 0) {
                last_reset_time_ = this->now();
                is_reset_time_set_ = true;
                is_initialized_ = false; // Bei Reset/Start neu initialisieren
                RCLCPP_INFO(this->get_logger(), "\033[1;34mStabilisierungsphase gestartet. Warte 2s Sim-Zeit...\033[0m");
            } else {
                // Keine Uhrzeit vorhanden, zur Sicherheit stoppen und warten
                geometry_msgs::msg::Twist stop_twist;
                cmd_pub_->publish(stop_twist);
                return;
            }
        }

        double elapsed_seconds = (this->now() - last_reset_time_).seconds();
        if (elapsed_seconds < 2.0) {
            // Während der Stabilisierungsphase aktualisieren wir den Zielwinkel kontinuierlich,
            // damit er sich an den voll eingeschwungenen EKF-Wert anpasst.
            if (update_robot_pose()) {
                target_yaw_ = std::round(current_yaw_ / (M_PI / 2.0)) * (M_PI / 2.0);
                is_initialized_ = true;
            }
            geometry_msgs::msg::Twist stop_twist;
            cmd_pub_->publish(stop_twist);
            return;
        }

        // Einmalige Logmeldung, wenn die Stabilisierung beendet ist
        static bool stabilization_done_logged = false;
        if (!stabilization_done_logged) {
            RCLCPP_INFO(this->get_logger(), "\033[1;34mStabilisierungsphase beendet. Start der Navigation.\033[0m");
            stabilization_done_logged = true;
        }

        if (maze_scale_ <= 0.0) {
            if (!measure_maze_scale(msg)) {
                // Nur alle 2 Sekunden loggen, um Fluten zu verhindern
                static double last_scale_log_time = 0.0;
                if (this->now().seconds() - last_scale_log_time > 2.0) {
                    float dl = get_min_dist_cone(msg, M_PI / 2.0, 5);
                    float dr = get_min_dist_cone(msg, -M_PI / 2.0, 5);
                    RCLCPP_INFO(this->get_logger(), "\033[1;34mWarte auf Gangerkennung (Wände links/rechts)... dl: %.2f, dr: %.2f\033[0m", dl, dr);
                    last_scale_log_time = this->now().seconds();
                }
                return;
            }
        }

        if (check_goal_reached(msg)) {
            RCLCPP_INFO(this->get_logger(), "\033[1;34mZiel erreicht laut Laserdaten.\033[0m");
            return;
        }

        if (!update_robot_pose()) {
            static double last_pose_log_time = 0.0;
            if (this->now().seconds() - last_pose_log_time > 2.0) {
                RCLCPP_WARN(this->get_logger(), "\033[1;34mWarte auf gültige TF-Daten (map/odom -> base_footprint)...\033[0m");
                last_pose_log_time = this->now().seconds();
            }
            return;
        }

        // Einmalige Logmeldung nach erfolgreicher Pose-Initialisierung
        static bool pose_init_logged = false;
        if (!pose_init_logged) {
            RCLCPP_INFO(this->get_logger(), "\033[1;34mTF-Pose erfolgreich empfangen. Position: (%.2f, %.2f), Yaw: %.2f\033[0m", 
                        current_x_, current_y_, current_yaw_);
            pose_init_logged = true;
        }

        // Gitter-Zellengrenzenerkennung zur Evaluierungssteuerung
        update_grid_cell_tracking();

        // Navigations-Loop logs (Blau)
        log_navigation_status();


        auto twist = geometry_msgs::msg::Twist();
  
        if (auto_state_ == RobotState::TURNING) {
            execute_turn(twist);
        } else if (auto_state_ == RobotState::DECELERATING) {
            execute_deceleration(msg, twist);
        } else {
            evaluate_intersection(msg, twist);
            
            // Nur fahren, wenn keine neue Drehung eingeleitet wurde
            if (auto_state_ == RobotState::DRIVE_FORWARD) {
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
            double measured = dl + dr;
            // Snappen auf das nächste Vielfache von 0.5m, um Messrauschen zu eliminieren (z. B. 1.5m)
            maze_scale_ = std::round(measured * 2.0) / 2.0;
            RCLCPP_INFO(this->get_logger(), "\033[1;34mLabyrinth skaliert: %.2f m (gemessen: %.2f m)\033[0m", maze_scale_, measured);
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
            RCLCPP_INFO(this->get_logger(), "\033[1;34mZIEL ERREICHT!\033[0m");
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
            pose_frame_id_ = "map";
        } catch (const tf2::TransformException &ex) {
            try { // Fallback auf Odom
                tf = tf_buffer_->lookupTransform("odom", "base_footprint", tf2::TimePointZero);
                pose_frame_id_ = "odom";
            } catch (const tf2::TransformException &ex2) {
                return false;
            }
        }

        current_x_ = tf.transform.translation.x;
        current_y_ = tf.transform.translation.y;
        
        tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y, 
                          tf.transform.rotation.z, tf.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, current_yaw_);

        return true;
    }

    // Aktualisiert das Gitterzellen-Tracking und setzt die Auswertung bei Zellwechsel zurueck
    void update_grid_cell_tracking() {
        if (maze_scale_ <= 0.0) {
            return;
        }

        int current_gx = std::round(current_x_ / maze_scale_);
        int current_gy = std::round(current_y_ / maze_scale_);

        if (!grid_initialized_) {
            last_gx_ = current_gx;
            last_gy_ = current_gy;
            grid_initialized_ = true;
            return;
        }

        if (current_gx != last_gx_ || current_gy != last_gy_) {
            if (enable_debug_) {
                RCLCPP_INFO(this->get_logger(), 
                            "\033[1;34m[DEBUG] Zellwechsel erkannt: von (%d, %d) nach (%d, %d). Setze evaluated_this_cell auf false.\033[0m", 
                            last_gx_, last_gy_, current_gx, current_gy);
            }
            evaluated_this_cell_ = false;
            last_gx_ = current_gx;
            last_gy_ = current_gy;
        }
    }

    // Loggt den aktuellen Navigationsstatus in regelmaessigen Abstaenden (0.5s)
    void log_navigation_status() {
        if (!enable_debug_) return;
        if (maze_scale_ <= 0.0) {
            return;
        }

        static double last_loop_log_time = 0.0;
        double current_time = this->now().seconds();
        if (current_time - last_loop_log_time > 0.5) {
            double dx = current_x_ - (std::round(current_x_ / maze_scale_) * maze_scale_);
            double dy = current_y_ - (std::round(current_y_ / maze_scale_) * maze_scale_);
            double dist_longitudinal = dx * std::cos(target_yaw_) + dy * std::sin(target_yaw_);
            double yaw_error = normalize_angle(target_yaw_ - current_yaw_);
            
            RCLCPP_INFO(this->get_logger(), 
                        "\033[1;34m[DEBUG] Pose: (%.2f, %.2f) | Yaw: %.3f (Target: %.3f, Err: %.3f) | DistLong: %.3f | Eval: %d\033[0m",
                        current_x_, current_y_, current_yaw_, target_yaw_, yaw_error, dist_longitudinal, evaluated_this_cell_);
            last_loop_log_time = current_time;
        }
    }

    // Verzögert das Fahrzeug bis zum Stillstand vor einer Drehung mit einer linearen Geschwindigkeitsrampe
    void execute_deceleration(const sensor_msgs::msg::LaserScan::SharedPtr& msg, geometry_msgs::msg::Twist& twist) {
        // Erst Zentrierungsregler ausführen, um in der Spur zu bleiben
        drive_and_center(msg, twist);

        double elapsed = (this->now() - decel_start_time_).seconds();
        double duration = 0.5; // Bremsdauer in Sekunden

        if (elapsed >= duration) {
            auto_state_ = RobotState::TURNING;
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
        } else {
            double scale = std::clamp(1.0 - (elapsed / duration), 0.0, 1.0);
            twist.linear.x *= scale;
        }
    }

    // Führt 90-Grad-Drehungen aus und beendet den Turn-State
    void execute_turn(geometry_msgs::msg::Twist& twist) {
        double yaw_diff = normalize_angle(target_yaw_ - current_yaw_);

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
        
        // Berechnung des Abstands entlang der Fahrtrichtung (longitudinal)
        double dx = current_x_ - center_x;
        double dy = current_y_ - center_y;
        double dist_longitudinal = dx * std::cos(target_yaw_) + dy * std::sin(target_yaw_);

        float dist_front = get_min_dist_cone(msg, 0.0, 5);
        float dist_left  = get_min_dist_cone(msg, M_PI / 2.0, 5);
        float dist_right = get_min_dist_cone(msg, -M_PI / 2.0, 5);

        // Bestimme, ob wir nah genug am Zentrum sind, um eine Entscheidung zu treffen.
        // Wir nutzen eine Kombination aus EKF-Distanz (für offene Kreuzungen) und 
        // präziser LiDAR-Distanz zur Vorderwand (für Ecken/Sackgassen/T-Kreuzungen).
        // Triggerpunkt auf -0.30m vorverlegt, um den Bremsweg bei der Geschwindigkeitsrampe zu kompensieren.
        bool close_to_center = (dist_longitudinal >= -0.30);
        double trigger_dist_front = (maze_scale_ / 2.0) + 0.30; // z.B. 1.05m bei 1.5m Scale
        if (dist_front < trigger_dist_front) {
            close_to_center = true;
        }

        // Nur nahe dem Zentrum der Zelle entscheiden
        if (!close_to_center || evaluated_this_cell_) {
            return;
        }

        double open_thresh = drive_params_.dist_free_path; 
        double dist_front_projected = dist_front + dist_longitudinal;
        bool can_go_straight = (dist_front_projected > open_thresh);
        bool can_go_left   = (dist_left > open_thresh);
        bool can_go_right  = (dist_right > open_thresh);

        // Nur agieren bei Abzweigungen oder Sackgassen
        if (can_go_straight && !can_go_left && !can_go_right) {
            // Keine Drehung erforderlich, Zelle als evaluiert markieren
            evaluated_this_cell_ = true;
            grid_memory_[{current_gx, current_gy}] += 1;
            publish_tremaux_markers();
            return;
        }

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

        double new_target_yaw = target_yaw_;
        bool turn_needed = false;

        // Weg priorisieren: Bevorzuge unbesuchte Wege und Geradeausfahrt
        if (min_visits < 9999) {
            if (v_straight == min_visits) {
                new_target_yaw = yaw_straight; 
            } else if (v_left == min_visits) {
                new_target_yaw = yaw_left;
                turn_needed = true;
            } else {
                new_target_yaw = yaw_right;
                turn_needed = true;
            }
        } else {
            // Sackgasse: 180 Grad drehen
            new_target_yaw = target_yaw_ + M_PI; 
            turn_needed = true;
        }
        
        // Zielwinkel am Labyrinth-Raster einrasten
        new_target_yaw = std::round(new_target_yaw / (M_PI / 2.0)) * (M_PI / 2.0);
        
        if (turn_needed) {
            target_yaw_ = new_target_yaw;
            auto_state_ = RobotState::DECELERATING;
            decel_start_time_ = this->now();
            twist.linear.x = 0.0;
            twist.angular.z = 0.0;
        } else {
            // Keine Drehung erforderlich, wir fahren geradeaus weiter
        }
        
        evaluated_this_cell_ = true;
        grid_memory_[{current_gx, current_gy}] += 1;
        publish_tremaux_markers();
    }

    // Geradeausfahrt mit PD-Regler zur perfekten Wandzentrierung
    void drive_and_center(const sensor_msgs::msg::LaserScan::SharedPtr& msg, geometry_msgs::msg::Twist& twist) {
        float dist_front = get_min_dist_cone(msg, 0.0, 5);
        float dist_left  = get_min_dist_cone(msg, M_PI / 2.0, 5);
        float dist_right = get_min_dist_cone(msg, -M_PI / 2.0, 5);

        twist.linear.x = drive_params_.v_linear; 
        
        // Notbremse vor Wänden
        if (dist_front < maze_scale_ * 0.3) twist.linear.x = 0.0;

        double yaw_error = normalize_angle(target_yaw_ - current_yaw_);
        
        float center_error = 0.0;
        float d_error = 0.0; 
        
        bool has_left_wall = (dist_left < drive_params_.dist_free_path);
        bool has_right_wall = (dist_right < drive_params_.dist_free_path);

        if (has_left_wall && has_right_wall) {
            center_error = dist_left - dist_right;
            d_error = center_error - prev_center_error_; 
        } else if (has_left_wall && maze_scale_ > 0.0) {
            // Einwandige Zentrierung links (Soll-Abstand ist die halbe Labyrinth-Breite)
            center_error = 2.0 * (dist_left - (maze_scale_ / 2.0));
            d_error = center_error - prev_center_error_;
        } else if (has_right_wall && maze_scale_ > 0.0) {
            // Einwandige Zentrierung rechts (Soll-Abstand ist die halbe Labyrinth-Breite)
            center_error = 2.0 * ((maze_scale_ / 2.0) - dist_right);
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

    // Normalisiert einen Winkel auf den Bereich [-pi, pi]
    double normalize_angle(double angle) {
        return std::atan2(std::sin(angle), std::cos(angle));
    }

    // Ermittelt den geringsten Abstand in einem bestimmten Laserscan-Kegel
    float get_min_dist_cone(const sensor_msgs::msg::LaserScan::SharedPtr& msg, double angle, int spread) {
        // Schutz vor Division durch Null oder leeren Scans
        if (msg->ranges.empty() || msg->angle_increment == 0.0f) {
            return 10.0f;
        }
        int center = std::round((angle - msg->angle_min) / msg->angle_increment);
        float min_d = 10.0f;
        int num_ranges = static_cast<int>(msg->ranges.size());
        for (int i = center - spread; i <= center + spread; ++i) {
            // Sicherstellen, dass der Index immer im gültigen Bereich [0, num_ranges-1] liegt
            int idx = (i % num_ranges + num_ranges) % num_ranges;
            float d = msg->ranges[idx];
            if (!std::isnan(d) && !std::isinf(d) && d > 0.05) {
                min_d = std::min(min_d, d);
            }
        }
        return min_d;
    }

    // Veröffentlicht die Trémaux-Karte als farbige Marker in RViz
    void publish_tremaux_markers() {
        if (maze_scale_ <= 0.0) {
            return;
        }

        auto marker_array = visualization_msgs::msg::MarkerArray();

        // 1. Zuerst alle bisherigen Marker loeschen (verhindert Geister-Marker nach Reset)
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.header.frame_id = pose_frame_id_;
        clear_marker.header.stamp = this->now();
        clear_marker.ns = "tremaux";
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);
        marker_pub_->publish(marker_array);
        marker_array.markers.clear();

        // 2. Neue Marker erstellen
        int id = 0;
        for (const auto& [cell, visits] : grid_memory_) {
            // Wand-Zellen (9999) nicht einzeichnen
            if (visits >= 9999) {
                continue;
            }

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = pose_frame_id_;
            marker.header.stamp = this->now();
            marker.ns = "tremaux";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;

            // Position (Zellzentrum auf Bodenhöhe)
            marker.pose.position.x = cell.first * maze_scale_;
            marker.pose.position.y = cell.second * maze_scale_;
            marker.pose.position.z = 0.01; // minimal ueber dem Boden, um Z-Fighting zu vermeiden
            marker.pose.orientation.w = 1.0;

            // Groesse (etwas kleiner als die Zelle, z.B. 80% der Skala, damit man die Zellgrenzen sieht)
            marker.scale.x = maze_scale_ * 0.8;
            marker.scale.y = maze_scale_ * 0.8;
            marker.scale.z = 0.02;

            // Farbkodierung nach Besuchsanzahl:
            // 0 Besuche (in grid_memory registriert) -> Blau
            // 1 Besuch -> Gruen (semi-transparent)
            // 2 Besuche -> Gelb/Orange (semi-transparent)
            // >= 3 Besuche -> Rot (semi-transparent)
            if (visits == 0) {
                marker.color.r = 0.0f; marker.color.g = 0.5f; marker.color.b = 1.0f; marker.color.a = 0.3f;
            } else if (visits == 1) {
                marker.color.r = 0.0f; marker.color.g = 1.0f; marker.color.b = 0.0f; marker.color.a = 0.4f;
            } else if (visits == 2) {
                marker.color.r = 1.0f; marker.color.g = 0.7f; marker.color.b = 0.0f; marker.color.a = 0.4f;
            } else {
                marker.color.r = 1.0f; marker.color.g = 0.0f; marker.color.b = 0.0f; marker.color.a = 0.5f;
            }

            marker_array.markers.push_back(marker);
        }

        marker_pub_->publish(marker_array);
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
            grid_initialized_ = false;
            prev_center_error_ = 0.0;
            evaluated_this_cell_ = true; // Zelle bei Moduswechsel als bereits evaluiert markieren
            is_reset_time_set_ = false; // Stabilisierungsphase bei Moduswechsel erneut aktivieren
            RCLCPP_INFO(this->get_logger(), "\033[1;34mModus: AUTONOM\033[0m");
        } else {
            current_mode_ = RobotMode::MANUAL;
            RCLCPP_INFO(this->get_logger(), "\033[1;34mModus: MANUELL\033[0m");
        }
    }

    // Setzt Algorithmus zurück und teleportiert via Gazebo-Befehl
    void handle_reset() {
        cmd_pub_->publish(geometry_msgs::msg::Twist()); 
        
        current_mode_ = ctrl_params_.start_manual ? RobotMode::MANUAL : RobotMode::AUTONOMOUS;
        auto_state_ = RobotState::DRIVE_FORWARD;
        is_initialized_ = false; 
        grid_initialized_ = false;
        prev_center_error_ = 0.0; 
        maze_scale_ = 0.0;
        grid_memory_.clear();
        publish_tremaux_markers();
        evaluated_this_cell_ = true; // Startzelle als bereits evaluiert markieren
        
        RCLCPP_WARN(this->get_logger(), "\033[1;34mReset ausgeführt. Teleportiere Roboter...\033[0m");

        // Gazebo System-Call bleibt bestehen
        system("gz service -s /world/maze_world/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000 "
               "--req \"name: 'maze_runner', position: {x: 1.5, y: 0, z: 0.2}, orientation: {x: 0, y: 0, z: 0.7071, w: 0.7071}\" &");

        // Stabilisierungszeitpunkt nach Reset neu triggern
        is_reset_time_set_ = false;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MazeRunnerControl>());
    rclcpp::shutdown();
    return 0;
}