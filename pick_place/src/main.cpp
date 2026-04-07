#include "pick_place/mtc_task_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);

  // Spin the node to handle subscriptions and services
  rclcpp::spin(mtc_task_node);

  rclcpp::shutdown();
  return 0;
}