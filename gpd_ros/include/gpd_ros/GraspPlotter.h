#ifndef INCLUDED_MYPROJECT_GRASP_PLOTTER_H
#define INCLUDED_MYPROJECT_GRASP_PLOTTER_H

// 1. Related header (none for .h file)
// 2. C standard library headers (none needed)
// 3. C++ standard library headers
#include <memory>
#include <string>
#include <vector>

// 4. Third-party library headers (alphabetical within group)
#include <Eigen/Dense>

// 5. Headers from non-third-party libraries (alphabetical within group)
#include <gpd/candidate/hand.h>
#include <gpd/candidate/hand_geometry.h>

// 6. Headers from the current project (alphabetical within group)
#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

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
    GraspPlotter(
        const rclcpp::Clock::SharedPtr& clock,
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& rvizPublisher,
        const gpd::candidate::HandGeometry& parameters);

    /**
     * Visualize grasps in RViz2
     */
    void drawGrasps(
        const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands,
        const std::string& frame);

    /**
     * Convert grasps to MarkerArray message
     */
    visualization_msgs::msg::MarkerArray convertToVisualGraspMsg(
        const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands,
        const std::string& frameId);

    /**
     * Create finger marker
     */
    visualization_msgs::msg::Marker createFingerMarker(
        const Eigen::Vector3d& center,
        const Eigen::Matrix3d& rotation,
        const Eigen::Vector3d& lengthWidthHeight,
        int id,
        const std::string& frameId);

    /**
     * Create hand base marker
     */
    visualization_msgs::msg::Marker createHandBaseMarker(
        const Eigen::Vector3d& start,
        const Eigen::Vector3d& end,
        const Eigen::Matrix3d& frame,
        double length,
        double height,
        int id,
        const std::string& frameId);

private:
    // Member variables (camelCase)
    double outerDiameter;
    double handDepth;
    double fingerWidth;
    double handHeight;

    rclcpp::Clock::SharedPtr clock;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rvizPublisher;
};

#endif  // INCLUDED_MYPROJECT_GRASP_PLOTTER_H