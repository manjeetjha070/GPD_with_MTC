#include <gpd_ros/grasp_messages.hpp>

namespace GraspMessages
{

gpd_ros::msg::GraspConfigList createGraspListMsg(
    const std::vector<std::unique_ptr<gpd::candidate::Hand>>& hands,
    const std_msgs::msg::Header& header)
{
  gpd_ros::msg::GraspConfigList msg;

  msg.header = header;
  msg.grasps.reserve(hands.size());

  for (size_t i = 0; i < hands.size(); i++)
  {
    msg.grasps.push_back(convertToGraspMsg(*hands[i]));
  }

  return msg;
}


gpd_ros::msg::GraspConfig convertToGraspMsg(
    const gpd::candidate::Hand& hand)
{
  gpd_ros::msg::GraspConfig msg;

  // --- position (Point)
  auto pos = hand.getPosition();
  msg.position.x = pos.x();
  msg.position.y = pos.y();
  msg.position.z = pos.z();

  // --- approach (Vector3)
  auto a = hand.getApproach();
  msg.approach.x = a.x();
  msg.approach.y = a.y();
  msg.approach.z = a.z();

  // --- binormal (Vector3)
  auto b = hand.getBinormal();
  msg.binormal.x = b.x();
  msg.binormal.y = b.y();
  msg.binormal.z = b.z();

  // --- axis (Vector3)
  auto ax = hand.getAxis();
  msg.axis.x = ax.x();
  msg.axis.y = ax.y();
  msg.axis.z = ax.z();

  msg.width.data = hand.getGraspWidth();
  msg.score.data = hand.getScore();

  // --- sample (Point)
  auto s = hand.getSample();
  msg.sample.x = s.x();
  msg.sample.y = s.y();
  msg.sample.z = s.z();

  return msg;
}

}