#include "pick_place/MTCTaskNode.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto mtcTaskNode = std::make_shared<MTCTaskNode>(options);

    // Spin the node to handle subscriptions and services
    rclcpp::spin(mtcTaskNode);

    rclcpp::shutdown();
    return 0;
}
