#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MazeRunnerControl : public rclcpp::Node {
public:
    MazeRunnerControl() : Node("maze_runner_control"), is_auto_mode_(true), last_button_state_(false) {
        // Lese Parameter ein. Falls 'use_controller' aktiv, starte im manuellen Modus.
        this->declare_parameter<bool>("use_controller", false);
        bool use_controller;
        this->get_parameter("use_controller", use_controller);
        is_auto_mode_ = !use_controller;

        // Erstelle Publisher für die Motoren
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Erstelle Subscriber für Lidar und Controller
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&MazeRunnerControl::scan_callback, this, std::placeholders::_1));
        
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&MazeRunnerControl::joy_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "MazeRunner gestartet. Auto-Modus: %s", is_auto_mode_ ? "AN" : "AUS");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        // A-Taste (Index 0 beim Xbox-Controller) zum Umschalten des Modus
        bool current_button_state = msg->buttons[0] == 1;

        // Flankenerkennung: Nur umschalten, wenn die Taste neu gedrückt wird
        if (current_button_state && !last_button_state_) {
            is_auto_mode_ = !is_auto_mode_;
            RCLCPP_INFO(this->get_logger(), "Modus gewechselt. Auto-Modus: %s", is_auto_mode_ ? "AN" : "AUS");
        }
        last_button_state_ = current_button_state;

        // Manuelle Steuerung nur ausführen, wenn Auto-Modus deaktiviert ist
        if (!is_auto_mode_) {
            auto twist = geometry_msgs::msg::Twist();
            // Linker Stick hoch/runter (Index 1) steuert Geschwindigkeit
            twist.linear.x = msg->axes[1] * 1; // Max 0.5 m/s
            // Rechter Stick links/rechts (Index 3) steuert Drehung
            twist.angular.z = msg->axes[2] * 2.0; // Max 1.0 rad/s
            
            cmd_pub_->publish(twist);
        }
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Automatische Steuerung überspringen, wenn der manuelle Modus aktiv ist
        if (!is_auto_mode_) return;

        auto twist = geometry_msgs::msg::Twist();
        float right_distance = msg->ranges[90];
        float front_distance = msg->ranges[180];

        // Einfache Wandverfolgungs-Logik (Rechte-Hand-Regel)
        if (front_distance < 0.6) {
            twist.linear.x = 0.0;
            twist.angular.z = 1.0;  // Drehe scharf links bei Frontalkollision
        } else if (right_distance < 0.4) {
            twist.linear.x = 0.15;
            twist.angular.z = 0.4;  // Korrigiere leicht nach links (Wand zu nah)
        } else if (right_distance > 0.6) {
            twist.linear.x = 0.15;
            twist.angular.z = -0.4; // Korrigiere leicht nach rechts (Wand zu weit weg)
        } else {
            twist.linear.x = 0.2;
            twist.angular.z = 0.0;  // Fahre geradeaus
        }

        cmd_pub_->publish(twist);
    }

    bool is_auto_mode_;
    bool last_button_state_;
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