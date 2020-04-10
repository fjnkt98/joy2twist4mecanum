#include <chrono>
#include <memory>
#include <cstdio>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rosidl_generator_cpp/message_initialization.hpp>

#include "joy2twist4mecanum/joy2twist4mecanum.hpp"

using namespace std::chrono_literals;

namespace joy2twist4mecanum {
  Joy2Twist4Mecanum::Joy2Twist4Mecanum(const rclcpp::NodeOptions &options)
    : Node("joy2twist4mecanum", options),
      twist_(rosidl_generator_cpp::MessageInitialization::ZERO),
      wheel_radius_(0.0),
      wheel_tread_half_(0.0),
      max_wheel_ang_vel_(0.0)
  {
    using namespace std::placeholders;

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    // Create publisher
    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    // Create subscriber
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", rclcpp::QoS(10),
             std::bind(&Joy2Twist4Mecanum::joyCallback, this, _1));

    // Initialize and declare parameters and substitute into member variables
    enable_button_ = this->declare_parameter("enable_button", 1);
    wheel_radius_ = this->declare_parameter("wheel_radius", 0.022);
    wheel_tread_half_ = this->declare_parameter("wheel_tread_half", 0.045);
    max_wheel_ang_vel_ = this->declare_parameter("max_wheel_ang_vel", 5.7596);
    x_axis_ = this->declare_parameter("x_axis", 1);
    y_axis_ = this->declare_parameter("y_axis", 0);
    yaw_axis_ = this->declare_parameter("yaw_axis", 2);
  }

  void Joy2Twist4Mecanum::publishTwist(const sensor_msgs::msg::Joy::SharedPtr joy_msg) {
    double alpha = joy_msg->axes[x_axis_];
    double beta = joy_msg->axes[y_axis_];
    double gamma = joy_msg->axes[yaw_axis_];

    double vel_max = wheel_radius_ * max_wheel_ang_vel_;
    double denom = std::fabs(alpha) + std::fabs(beta) + std::fabs(gamma);

    double coeff = 0.0;

    if (std::fabs(alpha) == 0.0 && std::fabs(beta) == 0.0 && std::fabs(gamma) == 0.0) {
      twist_.linear.x = 0.0;
      twist_.linear.y = 0.0;
      twist_.linear.z = 0.0;
      twist_.angular.x = 0.0;
      twist_.angular.y = 0.0;
      twist_.angular.z = 0.0;

      twist_pub_->publish(twist_);
    } else {
      if (std::fabs(alpha) >= std::fabs(beta) && std::fabs(alpha) >= std::fabs(gamma)) {
        coeff = std::fabs(alpha);
      } else if (std::fabs(beta) >= std::fabs(alpha) && std::fabs(beta) >= std::fabs(gamma)) {
        coeff = std::fabs(beta);
      } else if (std::fabs(gamma) >= std::fabs(alpha) && std::fabs(gamma) >= std::fabs(beta)) {
        coeff = std::fabs(gamma);
      }

      twist_.linear.x = (coeff * alpha * vel_max) / denom ;
      twist_.linear.y = (coeff * beta * vel_max) / denom;
      twist_.linear.z = 0.0;
      twist_.angular.x = 0.0;
      twist_.angular.y = 0.0;
      twist_.angular.z = (coeff * gamma * vel_max) / (2 * wheel_tread_half_ * denom);

      twist_pub_->publish(twist_);
    }   
  }

  void Joy2Twist4Mecanum::joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    if (static_cast<int>(msg->buttons.size()) > enable_button_ &&
        msg->buttons[enable_button_]) {
      this->publishTwist(msg);
    } else {
      auto cmd_vel_msg = std::make_unique<geometry_msgs::msg::Twist>();
      twist_pub_->publish(std::move(cmd_vel_msg));
    }
  }
} // namespace joy2twist4mecanum 

RCLCPP_COMPONENTS_REGISTER_NODE(joy2twist4mecanum::Joy2Twist4Mecanum)
