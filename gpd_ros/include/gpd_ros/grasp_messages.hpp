/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Andreas ten Pas
 *  All rights reserved.
 */

#ifndef GRASP_MESSAGES_H_
#define GRASP_MESSAGES_H_

#include <vector>
#include <memory>

// Eigen conversions
#include <tf2_eigen/tf2_eigen.hpp> 

// GPD
#include <gpd/candidate/hand.h>

// ROS2 messages
#include <std_msgs/msg/header.hpp>
#include <gpd_ros/msg/grasp_config.hpp>
#include <gpd_ros/msg/grasp_config_list.hpp>

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

}

#endif /* GRASP_MESSAGES_H_ */