#ifndef INCLUDED_MYPROJECT_GRASP_MESSAGES_H
#define INCLUDED_MYPROJECT_GRASP_MESSAGES_H

//  C++ standard library headers
#include <memory>
#include <vector>

//  Third-party library headers (alphabetical within group)
#include <tf2_eigen/tf2_eigen.hpp>

//  Headers from non-third-party libraries (alphabetical within group)
#include <gpd/candidate/hand.h>

//  Headers from the current project (alphabetical within group)
#include <gpd_ros/msg/grasp_config.hpp>
#include <gpd_ros/msg/grasp_config_list.hpp>
#include <std_msgs/msg/header.hpp>

namespace GraspMessages
{

/**
 * Convert a vector of GPD hand candidates to a ROS2 grasp list message
 */
gpd_ros::msg::GraspConfigList createGraspListMsg(
    const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands,
    const std_msgs::msg::Header& header);

/**
 * Convert a single GPD hand to a ROS2 grasp message
 */
gpd_ros::msg::GraspConfig convertToGraspMsg(
    const gpd::candidate::Hand& hand);

}  // namespace GraspMessages

#endif  // INCLUDED_MYPROJECT_GRASP_MESSAGES_H