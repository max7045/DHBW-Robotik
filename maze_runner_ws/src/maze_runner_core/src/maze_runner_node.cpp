#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"

class MazeRunnerControl : public rclcpp::Node {
public:
    MazeRunnerControl() : Node("maze_runner_control"), is_auto_mode_(true), last_button_state_(false) {

        // use_controller Parameter
        this->declare_parameter<bool>("use_controller", false);
        bool use_controller;
        this->get_parameter("use_controller", use_controller);
        is_auto_mode_ = !use_controller;

        // Motor Publisher
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        
        // Lidar Subscription
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&MazeRunnerControl::scan_callback, this, std::placeholders::_1));
        
        // Joystick Subscription
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10, std::bind(&MazeRunnerControl::joy_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "MazeRunner gestartet. Auto-Modus: %s", is_auto_mode_ ? "AN" : "AUS");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        // Status A-Taste 
        bool current_button_state = msg->buttons[0] == 1;

        // Änderung erkennen
        if (current_button_state && !last_button_state_) {
            is_auto_mode_ = !is_auto_mode_;
            RCLCPP_INFO(this->get_logger(), "Modus gewechselt. Auto-Modus: %s", is_auto_mode_ ? "AN" : "AUS");
        }
        last_button_state_ = current_button_state;

        // Manueller Moudus: Steuerung über Controller-Eingaben
        if (!is_auto_mode_) {
            auto twist = geometry_msgs::msg::Twist();

            // Auslenkungen der Joystick-Achsen lesen
            float linear_input = msg->axes[1];
            float angular_input = msg->axes[2];

            // Deadzone
            const float deadzone = 0.05;

            // Linear- und Winkelgeschwindigkeit berechnen
            if (std::abs(linear_input) > deadzone) {
                twist.linear.x = linear_input * 1; 
            } else {
                twist.linear.x = 0.0; 
            }

            if (std::abs(angular_input) > deadzone) {
                twist.angular.z = angular_input * 2.0; 
            } else {
                twist.angular.z = 0.0;
            }
            
            cmd_pub_->publish(twist);
        }
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (is_auto_mode_) {
            auto twist = geometry_msgs::msg::Twist();
            twist.linear.x = 5.0;
            twist.angular.z = 1.0;
            cmd_pub_->publish(twist);
        }
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