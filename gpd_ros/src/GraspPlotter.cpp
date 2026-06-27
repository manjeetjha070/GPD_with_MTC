#include "gpd_ros/GraspPlotter.h"

// 1. Related header (already included above)
// 2. C standard library headers (none needed)
// 3. C++ standard library headers
#include <memory>

// 4. Third-party library headers (none needed beyond what's in .hpp)
// 5. Non-third-party library headers (none needed)
// 6. Current project headers (none needed beyond what's in .hpp)

// Constructor
GraspPlotter::GraspPlotter(
    const rclcpp::Clock::SharedPtr& clock,
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& rvizPublisher,
    const gpd::candidate::HandGeometry& parameters)
: clock(clock)
, rvizPublisher(rvizPublisher)
, handDepth(parameters.depth_)
, handHeight(parameters.height_)
, outerDiameter(parameters.outer_diameter_)
, fingerWidth(parameters.finger_width_)
{
    // Constructor body
}

void GraspPlotter::drawGrasps(
    const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands,
    const std::string& frame)
{
    visualization_msgs::msg::MarkerArray markers;
    markers = convertToVisualGraspMsg(hands, frame);
    rvizPublisher->publish(markers);
}

visualization_msgs::msg::MarkerArray GraspPlotter::convertToVisualGraspMsg(
    const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands,
    const std::string& frameId)
{
    double halfWidth = 0.5 * outerDiameter - 0.5 * fingerWidth;

    visualization_msgs::msg::MarkerArray markerArray;
    visualization_msgs::msg::Marker leftFinger;
    visualization_msgs::msg::Marker rightFinger;
    visualization_msgs::msg::Marker base;
    visualization_msgs::msg::Marker approach;

    Eigen::Vector3d leftBottom;
    Eigen::Vector3d rightBottom;
    Eigen::Vector3d leftTop;
    Eigen::Vector3d rightTop;
    Eigen::Vector3d leftCenter;
    Eigen::Vector3d rightCenter;
    Eigen::Vector3d approachCenter;
    Eigen::Vector3d baseCenter;

    for (size_t i = 0; i < hands.size(); ++i)
    {
        leftBottom  = hands[i]->getPosition() - halfWidth * hands[i]->getBinormal();
        rightBottom = hands[i]->getPosition() + halfWidth * hands[i]->getBinormal();

        leftTop     = leftBottom + handDepth * hands[i]->getApproach();
        rightTop    = rightBottom + handDepth * hands[i]->getApproach();

        leftCenter  = leftBottom + 0.5 * (leftTop - leftBottom);
        rightCenter = rightBottom + 0.5 * (rightTop - rightBottom);

        baseCenter = leftBottom + 0.5 * (rightBottom - leftBottom)
                     - 0.01 * hands[i]->getApproach();

        approachCenter = baseCenter - 0.04 * hands[i]->getApproach();

        Eigen::Vector3d fingerLengthWidthHeight;
        Eigen::Vector3d approachLengthWidthHeight;

        fingerLengthWidthHeight << handDepth, fingerWidth, handHeight;
        approachLengthWidthHeight << 0.08, fingerWidth, handHeight;

        base = createHandBaseMarker(
            leftBottom,
            rightBottom,
            hands[i]->getFrame(),
            0.02,
            handHeight,
            i,
            frameId);

        leftFinger = createFingerMarker(
            leftCenter,
            hands[i]->getFrame(),
            fingerLengthWidthHeight,
            i * 3,
            frameId);

        rightFinger = createFingerMarker(
            rightCenter,
            hands[i]->getFrame(),
            fingerLengthWidthHeight,
            i * 3 + 1,
            frameId);

        approach = createFingerMarker(
            approachCenter,
            hands[i]->getFrame(),
            approachLengthWidthHeight,
            i * 3 + 2,
            frameId);

        markerArray.markers.push_back(leftFinger);
        markerArray.markers.push_back(rightFinger);
        markerArray.markers.push_back(approach);
        markerArray.markers.push_back(base);
    }

    return markerArray;
}

visualization_msgs::msg::Marker GraspPlotter::createFingerMarker(
    const Eigen::Vector3d& center,
    const Eigen::Matrix3d& frame,
    const Eigen::Vector3d& lengthWidthHeight,
    int id,
    const std::string& frameId)
{
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = frameId;
    marker.header.stamp    = clock->now();
    marker.header.stamp.sec -= 1;

    marker.ns      = "finger";
    marker.id      = id;

    marker.type    = visualization_msgs::msg::Marker::CUBE;
    marker.action  = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = center(0);
    marker.pose.position.y = center(1);
    marker.pose.position.z = center(2);

    // marker.lifetime = rclcpp::Duration::from_seconds(10.0);

    // Use orientation of hand frame
    Eigen::Quaterniond quaternion(frame);

    marker.pose.orientation.x = quaternion.x();
    marker.pose.orientation.y = quaternion.y();
    marker.pose.orientation.z = quaternion.z();
    marker.pose.orientation.w = quaternion.w();

    // These scales are relative to the hand frame (unit: meters)
    marker.scale.x = lengthWidthHeight(0);
    marker.scale.y = lengthWidthHeight(1);
    marker.scale.z = lengthWidthHeight(2);

    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.5;

    return marker;
}

visualization_msgs::msg::Marker GraspPlotter::createHandBaseMarker(
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    const Eigen::Matrix3d& frame,
    double length,
    double height,
    int id,
    const std::string& frameId)
{
    Eigen::Vector3d center = start + 0.5 * (end - start);

    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = frameId;
    marker.header.stamp    = clock->now();
    marker.header.stamp.sec -= 1;

    marker.ns      = "hand_base";
    marker.id      = id;

    marker.type    = visualization_msgs::msg::Marker::CUBE;
    marker.action  = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = center(0);
    marker.pose.position.y = center(1);
    marker.pose.position.z = center(2);

    // marker.lifetime = rclcpp::Duration::from_seconds(10.0);

    Eigen::Quaterniond quaternion(frame);

    marker.pose.orientation.x = quaternion.x();
    marker.pose.orientation.y = quaternion.y();
    marker.pose.orientation.z = quaternion.z();
    marker.pose.orientation.w = quaternion.w();

    marker.scale.x = length;
    marker.scale.y = (end - start).norm();
    marker.scale.z = height;

    marker.color.a = 0.5;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;

    return marker;
}