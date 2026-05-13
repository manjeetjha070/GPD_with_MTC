#include <gpd_ros/grasp_plotter.hpp>
#include <memory>
#include <rclcpp/node.hpp>

GraspPlotter::GraspPlotter (
    const rclcpp::Clock::SharedPtr& clock,
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& rviz_pub,
    const gpd::candidate::HandGeometry& params)
    : clock (clock),
      rviz_pub_ (rviz_pub)
{
    hand_depth_     = params.depth_;
    hand_height_    = params.height_;
    outer_diameter_ = params.outer_diameter_;
    finger_width_   = params.finger_width_;
}


void GraspPlotter::drawGrasps (
    const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands, const std::string& frame)
{
    visualization_msgs::msg::MarkerArray markers;
    markers = convertToVisualGraspMsg (hands, frame);
    rviz_pub_->publish (markers);
}


visualization_msgs::msg::MarkerArray GraspPlotter::convertToVisualGraspMsg (
    const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands, const std::string& frame_id)
{
    double hw = 0.5 * outer_diameter_ - 0.5 * finger_width_;

    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker left_finger, right_finger, base, approach;

    Eigen::Vector3d left_bottom, right_bottom, left_top, right_top;
    Eigen::Vector3d left_center, right_center, approach_center, base_center;

    for (size_t i = 0; i < hands.size(); i++)
    {
        left_bottom     = hands[i]->getPosition() - hw * hands[i]->getBinormal();
        right_bottom    = hands[i]->getPosition() + hw * hands[i]->getBinormal();

        left_top        = left_bottom + hand_depth_ * hands[i]->getApproach();
        right_top       = right_bottom + hand_depth_ * hands[i]->getApproach();

        left_center     = left_bottom + 0.5 * (left_top - left_bottom);
        right_center    = right_bottom + 0.5 * (right_top - right_bottom);

        base_center     = left_bottom + 0.5 * (right_bottom - left_bottom) - 0.01 * hands[i]->getApproach();
        approach_center = base_center - 0.04 * hands[i]->getApproach();

        Eigen::Vector3d finger_lwh, approach_lwh;
        finger_lwh << hand_depth_, finger_width_, hand_height_;
        approach_lwh << 0.08, finger_width_, hand_height_;

        base = createHandBaseMarker (left_bottom, right_bottom, hands[i]->getFrame(), 0.02, hand_height_, i, frame_id);

        left_finger  = createFingerMarker (left_center, hands[i]->getFrame(), finger_lwh, i * 3, frame_id);

        right_finger = createFingerMarker (right_center, hands[i]->getFrame(), finger_lwh, i * 3 + 1, frame_id);

        approach     = createFingerMarker (approach_center, hands[i]->getFrame(), approach_lwh, i * 3 + 2, frame_id);

        marker_array.markers.push_back (left_finger);
        marker_array.markers.push_back (right_finger);
        marker_array.markers.push_back (approach);
        marker_array.markers.push_back (base);
    }

    return marker_array;
}


visualization_msgs::msg::Marker GraspPlotter::createFingerMarker (
    const Eigen::Vector3d& center,
    const Eigen::Matrix3d& frame,
    const Eigen::Vector3d& lwh,
    int id,
    const std::string& frame_id)
{
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id   = frame_id;
    marker.header.stamp      = clock->now();
    marker.header.stamp.sec -= 1;

    marker.ns                = "finger";
    marker.id                = id;

    marker.type              = visualization_msgs::msg::Marker::CUBE;
    marker.action            = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x   = center (0);
    marker.pose.position.y   = center (1);
    marker.pose.position.z   = center (2);

    //marker.lifetime          = rclcpp::Duration::from_seconds (10.0);

    // use orientation of hand frame
    Eigen::Quaterniond quat (frame);

    marker.pose.orientation.x = quat.x();
    marker.pose.orientation.y = quat.y();
    marker.pose.orientation.z = quat.z();
    marker.pose.orientation.w = quat.w();

    // these scales are relative to the hand frame (unit: meters)
    marker.scale.x            = lwh (0);
    marker.scale.y            = lwh (1);
    marker.scale.z            = lwh (2);

    marker.color.a            = 0.5;
    marker.color.r            = 0.0;
    marker.color.g            = 0.0;
    marker.color.b            = 0.5;

    return marker;
}


visualization_msgs::msg::Marker GraspPlotter::createHandBaseMarker (
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    const Eigen::Matrix3d& frame,
    double length,
    double height,
    int id,
    const std::string& frame_id)
{
    Eigen::Vector3d center = start + 0.5 * (end - start);

    visualization_msgs::msg::Marker marker;

    marker.header.frame_id   = frame_id;
    marker.header.stamp      = clock->now();
    marker.header.stamp.sec -= 1;

    marker.ns                = "hand_base";
    marker.id                = id;

    marker.type              = visualization_msgs::msg::Marker::CUBE;
    marker.action            = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x   = center (0);
    marker.pose.position.y   = center (1);
    marker.pose.position.z   = center (2);

    //marker.lifetime          = rclcpp::Duration::from_seconds (10.0);

    Eigen::Quaterniond quat (frame);

    marker.pose.orientation.x = quat.x();
    marker.pose.orientation.y = quat.y();
    marker.pose.orientation.z = quat.z();
    marker.pose.orientation.w = quat.w();

    marker.scale.x            = length;
    marker.scale.y            = (end - start).norm();
    marker.scale.z            = height;

    marker.color.a            = 0.5;
    marker.color.r            = 0.0;
    marker.color.g            = 0.0;
    marker.color.b            = 1.0;

    return marker;
}