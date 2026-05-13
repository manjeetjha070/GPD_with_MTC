#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "std_srvs/srv/trigger.hpp"

#include <gpd/grasp_detector.h>
#include <gpd/util/cloud.h>
#include <gpd_ros/grasp_messages.hpp>
#include <gpd_ros/grasp_plotter.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloudRGBA;
typedef pcl::PointCloud<pcl::PointNormal> PointCloudPointNormal;

class GraspDetectionNode : public rclcpp::Node
{
public:
    GraspDetectionNode();
    ~GraspDetectionNode();

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void detectGrasps();
    void serviceCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // Subscribers and Publishers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<gpd_ros::msg::GraspConfigList>::SharedPtr grasps_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rviz_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;

    // GPD pointers
    gpd::GraspDetector* grasp_detector_ = nullptr;
    gpd::util::Cloud* cloud_camera_ = nullptr;
    GraspPlotter* rviz_plotter_ = nullptr;    

    // Parameters
    std::string cloud_topic_;
    std::string config_file_;
    std::string rviz_topic_;

    // Data
    std_msgs::msg::Header cloud_header_;
    std::string frame_id_;
    bool has_cloud_;
};