/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, Andreas ten Pas
 *  All rights reserved.
 */

#ifndef GRASP_PLOTTER_H_
#define GRASP_PLOTTER_H_

// ROS2
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// Eigen
#include <Eigen/Dense>

// GPD
#include <gpd/candidate/hand.h>
#include <gpd/candidate/hand_geometry.h>

/**
 * GraspPlotter class
 *
 * Draw grasps in RViz2
 */
class GraspPlotter
{
public:
    /**
     * Constructor
     */
    GraspPlotter (
        const rclcpp::Clock::SharedPtr& clock,
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& rviz_pub,
        const gpd::candidate::HandGeometry& params);

    /**
     * Visualize grasps in RViz2
     */
    void drawGrasps (const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands, const std::string& frame);

    /**
     * Convert grasps to MarkerArray message
     */
    visualization_msgs::msg::MarkerArray convertToVisualGraspMsg (
        const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands, const std::string& frame_id);

    /**
     * Create finger marker
     */
    visualization_msgs::msg::Marker createFingerMarker (
        const Eigen::Vector3d& center,
        const Eigen::Matrix3d& rot,
        const Eigen::Vector3d& lwh,
        int id,
        const std::string& frame_id);

    /**
     * Create hand base marker
     */
    visualization_msgs::msg::Marker createHandBaseMarker (
        const Eigen::Vector3d& start,
        const Eigen::Vector3d& end,
        const Eigen::Matrix3d& frame,
        double length,
        double height,
        int id,
        const std::string& frame_id);

private:

    double outer_diameter_;
    double hand_depth_;
    double finger_width_;
    double hand_height_;

    rclcpp::Clock::SharedPtr clock;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rviz_pub_;
};

#endif