#include "gpd_ros/GraspDetectionNode.h"

// Constructor
GraspDetectionNode::GraspDetectionNode()
: Node("grasp_detection_node")
, hasCloud(false)
{
    RCLCPP_INFO(this->get_logger(), "Initializing node...");

    // Parameters (camelCase)
    cloudTopic = declare_parameter<std::string>("cloud_topic", "/camera/depth/points");
    configFile = declare_parameter<std::string>("config_file", "");
    rvizTopic  = declare_parameter<std::string>("rviz_topic", "plot_grasps");

    // Initialize GPD
    graspDetector = new gpd::GraspDetector(configFile.c_str());

    // Subscriber 
    cloudSubscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        cloudTopic,
        10,
        std::bind(&GraspDetectionNode::cloudCallback, this, std::placeholders::_1));

    // Publishers 
    graspsPublisher = this->create_publisher<gpd_ros::msg::GraspConfigList>(
        "clustered_grasps",
        10);

    rvizPublisher = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        rvizTopic,
        10);

    // RViz plotter
    rvizPlotter = new GraspPlotter(
        this->get_clock(),
        rvizPublisher,
        graspDetector->getHandSearchParameters().hand_geometry_);

    // Service 
    service = this->create_service<std_srvs::srv::Trigger>(
        "detect_grasps",
        std::bind(
            &GraspDetectionNode::serviceCallback,
            this,
            std::placeholders::_1,
            std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Node ready. Waiting for cloud...");
}

// Destructor
GraspDetectionNode::~GraspDetectionNode()
{
    if (cloudCamera)
    {
        delete cloudCamera;
        cloudCamera = nullptr;
    }

    if (graspDetector)
    {
        delete graspDetector;
        graspDetector = nullptr;
    }

    if (rvizPlotter)
    {
        delete rvizPlotter;
        rvizPlotter = nullptr;
    }
}

// Private member functions 
void GraspDetectionNode::cloudCallback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    // if (hasCloud) return;

    Eigen::Matrix3Xd viewPoints(3, 1);
    viewPoints.setZero();

    // Reset previous cloud
    if (cloudCamera)
    {
        delete cloudCamera;
        cloudCamera = nullptr;
    }

    // Check for normals
    if (msg->fields.size() >= 6
        && msg->fields[3].name == "normal_x"
        && msg->fields[4].name == "normal_y"
        && msg->fields[5].name == "normal_z")
    {
        PointCloudPointNormal::Ptr cloud(new PointCloudPointNormal);
        pcl::fromROSMsg(*msg, *cloud);
        cloudCamera = new gpd::util::Cloud(cloud, 0, viewPoints);

    }
    else
    {
        PointCloudRGBA::Ptr cloud(new PointCloudRGBA);
        pcl::fromROSMsg(*msg, *cloud);
        cloudCamera = new gpd::util::Cloud(cloud, 0, viewPoints);

    }

    cloudHeader = msg->header;
    frameId     = msg->header.frame_id;
    hasCloud    = true;
}

void GraspDetectionNode::serviceCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if (!hasCloud || !cloudCamera)
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
    cloudCamera->voxelizeCloud(0.003); // Comment this out while using the simulation

    // Preprocess
    graspDetector->preprocessPointCloud(*cloudCamera);

    // Detect grasps
    auto grasps = graspDetector->detectGrasps(*cloudCamera);

    // Publish
    auto msg = GraspMessages::createGraspListMsg(grasps, cloudHeader);
    graspsPublisher->publish(msg);

    RCLCPP_INFO(this->get_logger(), "Published %zu grasps", msg.grasps.size());

    // RViz
    if (rvizPlotter)
    {
        rvizPlotter->drawGrasps(grasps, frameId);
    }

    hasCloud = false;
}