#include "gpd_ros/GraspMessages.h"

namespace GraspMessages
{

gpd_ros::msg::GraspConfigList createGraspListMsg(
    const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands,
    const std_msgs::msg::Header& header)
{
    gpd_ros::msg::GraspConfigList msg;

    msg.header = header;
    msg.grasps.reserve(hands.size());

    for (size_t i = 0; i < hands.size(); ++i)
    {
        msg.grasps.push_back(convertToGraspMsg(*hands[i]));
    }

    return msg;
}

gpd_ros::msg::GraspConfig convertToGraspMsg(
    const gpd::candidate::Hand& hand)
{
    gpd_ros::msg::GraspConfig msg;

    // Position (Point)
    auto position = hand.getPosition();
    msg.position.x = position.x();
    msg.position.y = position.y();
    msg.position.z = position.z();

    // Approach (Vector3)
    auto approach = hand.getApproach();
    msg.approach.x = approach.x();
    msg.approach.y = approach.y();
    msg.approach.z = approach.z();

    // Binormal (Vector3)
    auto binormal = hand.getBinormal();
    msg.binormal.x = binormal.x();
    msg.binormal.y = binormal.y();
    msg.binormal.z = binormal.z();

    // Axis (Vector3)
    auto axis = hand.getAxis();
    msg.axis.x = axis.x();
    msg.axis.y = axis.y();
    msg.axis.z = axis.z();

    msg.width.data = hand.getGraspWidth();
    msg.score.data = hand.getScore();

    // Sample (Point)
    auto sample = hand.getSample();
    msg.sample.x = sample.x();
    msg.sample.y = sample.y();
    msg.sample.z = sample.z();

    return msg;
}

}  // namespace GraspMessages