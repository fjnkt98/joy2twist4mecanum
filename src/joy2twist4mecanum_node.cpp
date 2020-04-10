#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "joy2twist4mecanum/joy2twist4mecanum.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  rclcpp::NodeOptions options;

  auto node = std::make_shared<joy2twist4mecanum::Joy2Twist4Mecanum>(options);
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
}
