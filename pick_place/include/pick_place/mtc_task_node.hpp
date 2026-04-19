#pragma once

#include <rclcpp/rclcpp.hpp>
#include <moveit/task_constructor/task.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <gpd_ros/msg/grasp_config_list.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace mtc = moveit::task_constructor;

class MTCTaskNode : public rclcpp::Node
{
public:
  MTCTaskNode(const rclcpp::NodeOptions& options);

  void doTask();
  void setupPlanningScene();

private:
  mtc::Task createTask();
  void graspsCallback(const gpd_ros::msg::GraspConfigList::SharedPtr msg);

  mtc::Task task_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  geometry_msgs::msg::PoseStamped object_pose_;
  rclcpp::Subscription<gpd_ros::msg::GraspConfigList>::SharedPtr grasps_sub_;

  void publishTfFrames();

  std::string open = "Open";
  std::string close = "Close";
  std::string home = "Home";
};