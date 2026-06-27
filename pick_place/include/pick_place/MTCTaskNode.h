#ifndef INCLUDED_MYPROJECT_MTC_TASK_NODE_H
#define INCLUDED_MYPROJECT_MTC_TASK_NODE_H

#include <string>

// Third-party library headers (alphabetical within group)
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

// Headers from non-third-party libraries (alphabetical within group)
#include <moveit/task_constructor/task.h>

// Headers from the current project (alphabetical within group)
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <gpd_ros/msg/grasp_config_list.hpp>
#include <rclcpp/rclcpp.hpp>

namespace mtc = moveit::task_constructor;

class MTCTaskNode : public rclcpp::Node
{
public:
    MTCTaskNode(const rclcpp::NodeOptions& options);

    void setupPlanningScene();

private:
    mtc::Task createTask();
    double computeGraspScore(
        const gpd_ros::msg::GraspConfig& grasp,
        const geometry_msgs::msg::PoseStamped& poseWorld,
        double maxGpd);
    void graspsCallback(const gpd_ros::msg::GraspConfigList::SharedPtr msg);
    void publishTfFrames();

    // Member variables
    mtc::Task task;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tfBroadcaster;
    geometry_msgs::msg::PoseStamped objectPose;
    rclcpp::Subscription<gpd_ros::msg::GraspConfigList>::SharedPtr graspsSubscriber;

    // String constants 
    std::string openState = "Open";
    std::string closeState = "Close";
    std::string homeState = "Home";
    std::string topDown = "pick";

    double zSign;
};

#endif  // INCLUDED_MYPROJECT_MTC_TASK_NODE_H