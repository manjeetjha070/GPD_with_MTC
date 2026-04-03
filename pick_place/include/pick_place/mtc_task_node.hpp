#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>

namespace mtc = moveit::task_constructor;

class MTCTaskNode
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr getNodeBaseInterface();

  void doTask();
  void setupPlanningScene();

private:
  mtc::Task createTask();

  mtc::Task task_;
  rclcpp::Node::SharedPtr node_;

  std::string open = "Open";
  std::string close = "Close";
  std::string home = "Home";
};