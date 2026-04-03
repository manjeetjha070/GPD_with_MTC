#include "pick_place/mtc_task_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  auto mtc_task_node = std::make_shared<MTCTaskNode>(options);

  rclcpp::executors::MultiThreadedExecutor executor;

  auto spin_thread = std::make_unique<std::thread>(
    [&executor, &mtc_task_node]() {
      executor.add_node(mtc_task_node->getNodeBaseInterface());
      executor.spin();
      executor.remove_node(mtc_task_node->getNodeBaseInterface());
    });

  mtc_task_node->setupPlanningScene();
  mtc_task_node->doTask();

  spin_thread->join();
  rclcpp::shutdown();
  return 0;
}