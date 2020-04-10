#ifndef JOY2TWIST4MECANUM__JOY2TWIST4MECANUM_HPP_
#define JOY2TWIST4MECANUM__JOY2TWIST4MECANUM_HPP_ 

#include <map>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include "joy2twist4mecanum/visibility_control.h"

namespace joy2twist4mecanum {
  class Joy2Twist4Mecanum : public rclcpp::Node {
    public:
      JOY2TWIST4MECANUM_PUBLIC

      explicit Joy2Twist4Mecanum(const rclcpp::NodeOptions &options);

    private:
      geometry_msgs::msg::Twist twist_;

      // Publisher
      rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
      // Subscriber
      rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

      // Joystick parameters
      int64_t enable_button_;
      int64_t x_axis_;
      int64_t y_axis_;
      int64_t yaw_axis_;

      // Robot parameters
      double wheel_radius_;
      double wheel_tread_half_;
      double max_wheel_ang_vel_;

      // Callback function for Joy message
      void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
      // function that publish computed twist message
      void publishTwist(const sensor_msgs::msg::Joy::SharedPtr joy_msg);
  };
} // namespace joy2twist4mecanum 

#endif // JOY2TWIST4MECANUM__JOY2TWIST4MECANUM_HPP_
