#include "gpd_ros/grasp_detection_node.hpp"

GraspDetectionNode::GraspDetectionNode()
: Node("grasp_detection_node"), has_cloud_(false)
{
    RCLCPP_INFO(this->get_logger(), "Initializing node...");

    // Parameters
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/camera/depth/points");
    config_file_ = declare_parameter<std::string>("config_file", "");
    rviz_topic_  = declare_parameter<std::string>("rviz_topic", "plot_grasps");

    // Initialize GPD
    grasp_detector_ = new gpd::GraspDetector(config_file_.c_str());

    // Subscriber
    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloud_topic_, 10,
        std::bind(&GraspDetectionNode::cloudCallback, this, std::placeholders::_1));

    // Publishers
    grasps_pub_ = this->create_publisher<gpd_ros::msg::GraspConfigList>("clustered_grasps", 10);
    rviz_pub_   = this->create_publisher<visualization_msgs::msg::MarkerArray>(rviz_topic_, 10);

    // RViz plotter
    rviz_plotter_ = new GraspPlotter(this->get_clock(), rviz_pub_, grasp_detector_->getHandSearchParameters().hand_geometry_);

    //service
    service_ = this->create_service<std_srvs::srv::Trigger>(
    "detect_grasps",
    std::bind(&GraspDetectionNode::serviceCallback, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Node ready. Waiting for cloud...");
}

GraspDetectionNode::~GraspDetectionNode()
{
    if (cloud_camera_)
    {
        delete cloud_camera_;
        cloud_camera_ = nullptr;
    }

    if (grasp_detector_)
    {
        delete grasp_detector_;
        grasp_detector_ = nullptr;
    }

    if (rviz_plotter_)
    {
        delete rviz_plotter_;
        rviz_plotter_ = nullptr;
    }
}

void GraspDetectionNode::cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // if (has_cloud_) return;

    Eigen::Matrix3Xd view_points(3,1);
    view_points.setZero();

    // Reset previous cloud
    if (cloud_camera_)
    {
        delete cloud_camera_;
        cloud_camera_ = nullptr;
    }

    // Check for normals
    if (msg->fields.size() >= 6 &&
        msg->fields[3].name == "normal_x" &&
        msg->fields[4].name == "normal_y" &&
        msg->fields[5].name == "normal_z")
    {
        PointCloudPointNormal::Ptr cloud(new PointCloudPointNormal);
        pcl::fromROSMsg(*msg, *cloud);
        cloud_camera_ = new gpd::util::Cloud(cloud, 0, view_points);

        // RCLCPP_INFO(this->get_logger(), "Received cloud with normals: %zu points", cloud->size());
    }
    else
    {
        PointCloudRGBA::Ptr cloud(new PointCloudRGBA);
        pcl::fromROSMsg(*msg, *cloud);
        cloud_camera_ = new gpd::util::Cloud(cloud, 0, view_points);

        // RCLCPP_INFO(this->get_logger(), "Received cloud without normals: %zu points", cloud->size());
    }

    cloud_header_ = msg->header;
    frame_id_     = msg->header.frame_id;
    has_cloud_    = true;

}

void GraspDetectionNode::serviceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if (!has_cloud_ || !cloud_camera_)
    {
        response->success = false;
        response->message = "No point cloud available.";
        return;
    }

    detectGrasps();

    response->success = true;
    response->message = "Grasp detection executed.";
}

void GraspDetectionNode::detectGrasps()
{    
    cloud_camera_->voxelizeCloud(0.003); //comment this out while using the simulation
    // Preprocess
    grasp_detector_->preprocessPointCloud(*cloud_camera_);

    // Detect grasps
    auto grasps = grasp_detector_->detectGrasps(*cloud_camera_);

    // Publish
    auto msg = GraspMessages::createGraspListMsg(grasps, cloud_header_);
    grasps_pub_->publish(msg);

    RCLCPP_INFO(this->get_logger(), "Published %zu grasps", msg.grasps.size());

    // RViz
    if (rviz_plotter_)
    {
        rviz_plotter_->drawGrasps(grasps, frame_id_);
    }

    has_cloud_ = false;
}