#ifndef INCLUDED_MYPROJECT_GRASP_DETECTION_NODE_H
#define INCLUDED_MYPROJECT_GRASP_DETECTION_NODE_H

#include <string>

// Third-party library headers (alphabetical within group)
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// Headers from non-third-party libraries
#include "std_srvs/srv/trigger.hpp"

// Headers from the current project (alphabetical within group)
#include <gpd/grasp_detector.h>
#include <gpd/util/cloud.h>
#include <gpd_ros/GraspMessages.h>
#include <gpd_ros/GraspPlotter.h>

using pcl::PointCloud;          
using pcl::PointNormal;
using pcl::PointXYZRGBA;

using PointCloudRGBA = PointCloud<PointXYZRGBA>;
using PointCloudPointNormal = PointCloud<PointNormal>;

class GraspDetectionNode : public rclcpp::Node
{
public:
    GraspDetectionNode();
    ~GraspDetectionNode();

private:    
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void detectGrasps();
    void serviceCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // Subscribers and Publishers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloudSubscriber;
    rclcpp::Publisher<gpd_ros::msg::GraspConfigList>::SharedPtr graspsPublisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rvizPublisher;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service;

    // GPD pointers 
    gpd::GraspDetector* graspDetector = nullptr;
    gpd::util::Cloud* cloudCamera = nullptr;
    GraspPlotter* rvizPlotter = nullptr;

    // Parameters 
    std::string cloudTopic;
    std::string configFile;
    std::string rvizTopic;

    // Data 
    std_msgs::msg::Header cloudHeader;
    std::string frameId;
    bool hasCloud;
};

#endif  // INCLUDED_MYPROJECT_GRASP_DETECTION_NODE_H