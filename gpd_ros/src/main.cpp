#include "gpd_ros/grasp_detection_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GraspDetectionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}