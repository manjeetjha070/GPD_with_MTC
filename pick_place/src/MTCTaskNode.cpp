#include "pick_place/MTCTaskNode.h"

#include <cmath>
#include <limits>

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>
#include <moveit/task_constructor/stages/generate_random_pose.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");

// ============================================================================
// Constructor
// ============================================================================
MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
: rclcpp::Node("mtc_node", options)
, tfBuffer(this->get_clock())
, tfListener(tfBuffer)
{
    this->declare_parameter("velocity_scaling", 0.1);
    this->declare_parameter("acceleration_scaling", 0.1);
    this->declare_parameter("w_gpd", 1.0);
    this->declare_parameter("w_height", 0.5);
    this->declare_parameter("w_top", 0.8);
    this->declare_parameter("w_dist", 0.3);

    tfBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    graspsSubscriber = this->create_subscription<gpd_ros::msg::GraspConfigList>(
        "clustered_grasps",
        10,
        std::bind(
            &MTCTaskNode::graspsCallback,
            this,
            std::placeholders::_1));
}

// ============================================================================
// Compute Grasp Score
// ============================================================================
double MTCTaskNode::computeGraspScore(
    const gpd_ros::msg::GraspConfig& grasp,
    const geometry_msgs::msg::PoseStamped& poseWorld,
    double maxGpd)
{
    // ---- Weights (tune these!) ----
    const double wGpd = this->get_parameter("w_gpd").as_double();
    const double wHeight = this->get_parameter("w_height").as_double();
    const double wTop = this->get_parameter("w_top").as_double();
    const double wDist = this->get_parameter("w_dist").as_double();

    // ---- 1. GPD score ----
    double gpdScore = grasp.score.data;
    double gpdScoreNormalized = gpdScore / (maxGpd + 1e-6);  // Normalize to [0,1]

    // ---- 2. Height (prefer higher grasps) ----
    double height = poseWorld.pose.position.z;

    // ---- 3. Top-down preference ----
    // Approach vector from GPD
    Eigen::Vector3d approach(grasp.approach.x,
                             grasp.approach.y,
                             grasp.approach.z);
    approach.normalize();

    // Prefer alignment with -Z (downward grasp)
    Eigen::Vector3d worldDown(0, 0, -1);
    double topScore = approach.dot(worldDown);  // [-1,1]

    // Normalize to [0,1]
    topScore = (topScore + 1.0) / 2.0;

    // ---- 4. Distance penalty (simple proxy) ----
    // Distance from robot base (you can improve this later)
    double distance = std::sqrt(
        poseWorld.pose.position.x * poseWorld.pose.position.x
        + poseWorld.pose.position.y * poseWorld.pose.position.y
        + poseWorld.pose.position.z * poseWorld.pose.position.z);

    // Invert so closer = higher score
    double distScore = 1.0 / (1.0 + distance);

    // ---- Final score ----
    double finalScore = wGpd * gpdScoreNormalized
                        + wHeight * height
                        + wTop * topScore
                        + wDist * distScore;

    return finalScore;
}

// ============================================================================
// Grasps Callback
// ============================================================================
void MTCTaskNode::graspsCallback(const gpd_ros::msg::GraspConfigList::SharedPtr msg)
{
    if (msg->grasps.empty())
    {
        RCLCPP_WARN(LOGGER, "No grasps received");
        return;
    }

    // Copy + sort grasps (highest score first)
    std::vector<gpd_ros::msg::GraspConfig> grasps = msg->grasps;

    double maxGpd = std::numeric_limits<double>::lowest();

    for (const auto& grasp : grasps)
    {
        maxGpd = std::max(maxGpd, static_cast<double>(grasp.score.data));
    }

    std::vector<std::pair<double, gpd_ros::msg::GraspConfig>> scoredGrasps;

    for (const auto& grasp : msg->grasps)
    {
        try
        {
            // --- Build pose (same as your code) ---
            geometry_msgs::msg::PoseStamped poseIn;
            poseIn.header = msg->header;
            poseIn.pose.position = grasp.position;

            Eigen::Matrix3d rotation;
            rotation.col(0) = Eigen::Vector3d(
                grasp.approach.x,
                grasp.approach.y,
                grasp.approach.z);

            rotation.col(1) = Eigen::Vector3d(
                grasp.binormal.x,
                grasp.binormal.y,
                grasp.binormal.z);

            rotation.col(2) = Eigen::Vector3d(
                grasp.axis.x,
                grasp.axis.y,
                grasp.axis.z);

            Eigen::Quaterniond quaternion(rotation);
            poseIn.pose.orientation = tf2::toMsg(quaternion);

            poseIn.header.stamp = this->get_clock()->now();
            poseIn.header.stamp.sec -= 1;

            geometry_msgs::msg::PoseStamped poseWorld;
            tfBuffer.transform(poseIn, poseWorld, "world");

            // --- Compute combined score ---
            double score = computeGraspScore(grasp, poseWorld, maxGpd);

            scoredGrasps.emplace_back(score, grasp);
        }
        catch (tf2::TransformException& exception)
        {
            continue;
        }
    }

    // ---- Sort by combined score ----
    std::sort(
        scoredGrasps.begin(),
        scoredGrasps.end(),
        [](const auto& a, const auto& b)
        {
            return a.first > b.first;
        });

    for (size_t i = 0; i < scoredGrasps.size(); ++i)
    {
        const auto& grasp = scoredGrasps[i].second;
        double combinedScore = scoredGrasps[i].first;

        RCLCPP_INFO(
            LOGGER,
            "Trying grasp %zu (combined=%.3f, gpd=%.3f)",
            i,
            combinedScore,
            grasp.score.data);

        try
        {
            // ---------------- Transform grasp ----------------
            geometry_msgs::msg::PoseStamped poseInObject;
            poseInObject.header = msg->header;
            poseInObject.pose.position = grasp.position;

            Eigen::Matrix3d rotationObject;
            rotationObject.col(0) = Eigen::Vector3d(
                grasp.approach.x,
                grasp.approach.y,
                grasp.approach.z);

            rotationObject.col(1) = Eigen::Vector3d(
                grasp.binormal.x,
                grasp.binormal.y,
                grasp.binormal.z);

            rotationObject.col(2) = Eigen::Vector3d(
                grasp.axis.x,
                grasp.axis.y,
                grasp.axis.z);

            Eigen::Quaterniond quaternionObject(rotationObject);
            poseInObject.pose.orientation = tf2::toMsg(quaternionObject);

            poseInObject.header.stamp = this->get_clock()->now();
            poseInObject.header.stamp.sec -= 1;

            geometry_msgs::msg::PoseStamped poseOutObject;
            tfBuffer.transform(poseInObject, poseOutObject, "world");

            // -------- Compute zSign --------
            tf2::Quaternion quaternion;
            tf2::fromMsg(poseOutObject.pose.orientation, quaternion);
            tf2::Matrix3x3 matrix(quaternion);
            tf2::Vector3 zAxis = matrix.getColumn(2);
            zSign = (zAxis.z() >= 0.0) ? 1.0 : -1.0;

            objectPose = poseOutObject;

            // ---------------- Reset planning scene ----------------
            moveit::planning_interface::PlanningSceneInterface psi;
            psi.removeCollisionObjects({"object"});

            // ---------------- Setup + plan ----------------
            publishTfFrames();
            setupPlanningScene();

            task = createTask();

            try
            {
                task.init();
            }
            catch (mtc::InitStageException& exception)
            {
                RCLCPP_WARN(LOGGER, "Init failed for grasp %zu", i);
                continue;
            }

            if (!task.plan(5))
            {
                RCLCPP_WARN(LOGGER, "Planning failed for grasp %zu", i);
                continue;
            }

            RCLCPP_INFO(LOGGER, "Planning SUCCESS for grasp %zu", i);

            task.introspection().publishSolution(*task.solutions().front());

            auto result = task.execute(*task.solutions().front());

            if (result.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS)
            {
                RCLCPP_INFO(LOGGER, "Execution SUCCESS for grasp %zu", i);
            }
            else
            {
                RCLCPP_WARN(LOGGER, "Execution failed for grasp %zu", i);
            }

            return;  // Exit after first successful grasp
        }
        catch (tf2::TransformException& exception)
        {
            RCLCPP_WARN(
                LOGGER,
                "TF failed for grasp %zu: %s",
                i,
                exception.what());

            continue;
        }
    }

    RCLCPP_ERROR(LOGGER, "All grasps failed");
}

// ============================================================================
// Planning Scene
// ============================================================================
void MTCTaskNode::setupPlanningScene()
{
    moveit_msgs::msg::CollisionObject object;
    object.id = "object";
    object.header.frame_id = "world";

    object.primitives.resize(1);
    object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
    object.primitives[0].dimensions = { 0.2, 0.04 };

    geometry_msgs::msg::Pose pose = objectPose.pose;

    // Define offset in local object frame (x direction)
    tf2::Vector3 localOffset(0.04, 0.0, 0.0);  // 4 cm along object's x

    // Convert orientation to tf2
    tf2::Quaternion quaternion;
    tf2::fromMsg(pose.orientation, quaternion);

    // Rotate offset into world frame
    tf2::Vector3 worldOffset = tf2::quatRotate(quaternion, localOffset);

    // Apply offset
    pose.position.x += worldOffset.x();
    pose.position.y += worldOffset.y();
    pose.position.z += worldOffset.z();

    object.pose = pose;
    objectPose.pose = pose;

    moveit::planning_interface::PlanningSceneInterface psi;
    psi.applyCollisionObject(object);
}

void MTCTaskNode::publishTfFrames()
{
    auto makeTransform = [&](const std::string& childFrame,
                             const geometry_msgs::msg::PoseStamped& pose)
    {
        geometry_msgs::msg::TransformStamped transformMessage;
        transformMessage.header.stamp = this->get_clock()->now();
        transformMessage.header.frame_id = pose.header.frame_id;
        transformMessage.child_frame_id = childFrame;
        transformMessage.transform.translation.x = pose.pose.position.x;
        transformMessage.transform.translation.y = pose.pose.position.y;
        transformMessage.transform.translation.z = pose.pose.position.z;
        transformMessage.transform.rotation = pose.pose.orientation;
        tfBroadcaster->sendTransform(transformMessage);
    };

    makeTransform("object_pose", objectPose);
}

// ============================================================================
// Create Task
// ============================================================================
mtc::Task MTCTaskNode::createTask()
{
    mtc::Task task;
    task.stages()->setName("demo task");
    task.loadRobotModel(this->shared_from_this());

    double velocityScaling = this->get_parameter("velocity_scaling").as_double();
    double accelerationScaling = this->get_parameter("acceleration_scaling").as_double();

    const auto& armGroupName = "manipulator";
    const auto& handGroupName = "gripper";
    const auto& handFrame = "end_effector_link";

    // Set task properties
    task.setProperty("group", armGroupName);
    task.setProperty("eef", handGroupName);
    task.setProperty("ik_frame", handFrame);

    // Current state stage
    auto stageStateCurrent = std::make_unique<mtc::stages::CurrentState>("current");
    task.add(std::move(stageStateCurrent));

    // Planners
    auto samplingPlanner = std::make_shared<mtc::solvers::PipelinePlanner>(
        this->shared_from_this());

    samplingPlanner->setMaxVelocityScalingFactor(velocityScaling);
    samplingPlanner->setMaxAccelerationScalingFactor(accelerationScaling);

    auto interpolationPlanner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
    interpolationPlanner->setMaxVelocityScalingFactor(velocityScaling);
    interpolationPlanner->setMaxAccelerationScalingFactor(accelerationScaling);

    auto cartesianPlanner = std::make_shared<mtc::solvers::CartesianPath>();
    cartesianPlanner->setMaxVelocityScalingFactor(velocityScaling);
    cartesianPlanner->setMaxAccelerationScalingFactor(accelerationScaling);
    cartesianPlanner->setStepSize(0.01);

    // ---- Open hand ----
    auto stageOpenHand = std::make_unique<mtc::stages::MoveTo>(
        "open hand",
        interpolationPlanner);

    mtc::Stage* openStatePtr = nullptr;
    openStatePtr = stageOpenHand.get();

    stageOpenHand->setGroup(handGroupName);
    stageOpenHand->setGoal(openState);
    task.add(std::move(stageOpenHand));

    // ---- Move to pick ----
    auto stageMoveToPick = std::make_unique<mtc::stages::Connect>(
        "move to pick",
        mtc::stages::Connect::GroupPlannerVector{
            { armGroupName, samplingPlanner } });

    stageMoveToPick->setTimeout(10.0);
    stageMoveToPick->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stageMoveToPick));

    // ---- Pick object (SerialContainer) ----
    mtc::Stage* attachObjectStage = nullptr;

    {
        auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
        task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
        grasp->properties().configureInitFrom(
            mtc::Stage::PARENT,
            { "eef", "group", "ik_frame" });

        // Approach object
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>(
                "approach object",
                cartesianPlanner);

            stage->properties().set("marker_ns", "approach_object");
            stage->properties().set("link", handFrame);
            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(0.1, 0.15);

            // Set hand forward direction
            geometry_msgs::msg::Vector3Stamped vector;
            vector.header.frame_id = handFrame;
            vector.vector.z = 1.0;

            stage->setDirection(vector);
            grasp->insert(std::move(stage));
        }

        // Generate grasp pose
        {
            auto stage = std::make_unique<mtc::stages::GeneratePose>(
                "generate grasp pose");

            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->properties().set("marker_ns", "grasp_pose");
            stage->setMonitoredStage(openStatePtr);
            stage->setPose(objectPose);

            // This is the transform from the object frame to the end-effector frame
            Eigen::Isometry3d graspFrameTransform;

            if (zSign == 1)
            {
                Eigen::Quaterniond quaternion =
                    Eigen::AngleAxisd((-M_PI) / 2, Eigen::Vector3d::UnitX())
                    * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd((-M_PI) / 2, Eigen::Vector3d::UnitZ());

                graspFrameTransform.linear() = quaternion.matrix();
            }
            else
            {
                Eigen::Quaterniond quaternion =
                    Eigen::AngleAxisd((M_PI) / 2, Eigen::Vector3d::UnitX())
                    * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd((M_PI) / 2, Eigen::Vector3d::UnitZ());

                graspFrameTransform.linear() = quaternion.matrix();
            }

            graspFrameTransform.translation().z() = 0.14; // Adjust this offset based on your gripper geometry, e.g., the distance from the end-effector frame to the grasp point on the object.

            // Compute IK
            auto wrapper = std::make_unique<mtc::stages::ComputeIK>(
                "grasp pose IK",
                std::move(stage));

            wrapper->setMaxIKSolutions(8);
            wrapper->setMinSolutionDistance(1.0);
            wrapper->setIKFrame(graspFrameTransform, handFrame);
            wrapper->properties().configureInitFrom(
                mtc::Stage::PARENT,
                { "eef", "group" });

            wrapper->properties().configureInitFrom(
                mtc::Stage::INTERFACE,
                { "target_pose" });

            grasp->insert(std::move(wrapper));
        }

        // Allow collision (hand, object)
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
                "allow collision (hand,object)");

            stage->allowCollisions(
                "object",
                task.getRobotModel()
                    ->getJointModelGroup(handGroupName)
                    ->getLinkModelNamesWithCollisionGeometry(),
                true);

            grasp->insert(std::move(stage));
        }

        // Close hand
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>(
                "close hand",
                interpolationPlanner);

            stage->setGroup(handGroupName);
            stage->setGoal(closeState);
            grasp->insert(std::move(stage));
        }

        // Attach object
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
                "attach object");

            stage->attachObject("object", handFrame);
            attachObjectStage = stage.get();
            grasp->insert(std::move(stage));
        }

        // Lift object
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>(
                "lift object",
                cartesianPlanner);

            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(0.05, 0.3);
            stage->setIKFrame(handFrame);
            stage->properties().set("marker_ns", "lift_object");

            // Set upward direction
            geometry_msgs::msg::Vector3Stamped vector;
            vector.header.frame_id = "world";
            vector.vector.z = 1.0;
            stage->setDirection(vector);

            grasp->insert(std::move(stage));
        }

        task.add(std::move(grasp));
    }

    // ---- Move to place ----
    {
        auto stageMoveToPlace = std::make_unique<mtc::stages::Connect>(
            "move to place",
            mtc::stages::Connect::GroupPlannerVector{
                { armGroupName, samplingPlanner } });

        stageMoveToPlace->setTimeout(5.0);
        stageMoveToPlace->properties().configureInitFrom(mtc::Stage::PARENT);
        task.add(std::move(stageMoveToPlace));
    }

    // ---- Place object (SerialContainer) ----
    {
        auto place = std::make_unique<mtc::SerialContainer>("place object");
        task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
        place->properties().configureInitFrom(
            mtc::Stage::PARENT,
            { "eef", "group", "ik_frame" });

        // Generate place pose
        {
            auto stage = std::make_unique<mtc::stages::GeneratePlacePose>(
                "generate place pose");

            stage->properties().configureInitFrom(mtc::Stage::PARENT);
            stage->properties().set("marker_ns", "place_pose");
            stage->setObject("object");

            geometry_msgs::msg::PoseStamped targetPoseMsg;
            targetPoseMsg.header.frame_id = "world";
            targetPoseMsg.pose = objectPose.pose;
            targetPoseMsg.pose.position.y += 0.5;
            targetPoseMsg.pose.position.z = 0.1;

            stage->setPose(targetPoseMsg);
            stage->setMonitoredStage(attachObjectStage);

            // Compute IK
            auto wrapper = std::make_unique<mtc::stages::ComputeIK>(
                "place pose IK",
                std::move(stage));

            wrapper->setMaxIKSolutions(2);
            wrapper->setMinSolutionDistance(1.0);
            wrapper->setIKFrame("object");
            wrapper->properties().configureInitFrom(
                mtc::Stage::PARENT,
                { "eef", "group" });

            wrapper->properties().configureInitFrom(
                mtc::Stage::INTERFACE,
                { "target_pose" });

            place->insert(std::move(wrapper));
        }

        // Open hand
        {
            auto stage = std::make_unique<mtc::stages::MoveTo>(
                "open hand",
                interpolationPlanner);

            stage->setGroup(handGroupName);
            stage->setGoal(openState);
            place->insert(std::move(stage));
        }

        // Forbid collision (hand, object)
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
                "forbid collision (hand,object)");

            stage->allowCollisions(
                "object",
                task.getRobotModel()
                    ->getJointModelGroup(handGroupName)
                    ->getLinkModelNamesWithCollisionGeometry(),
                false);

            place->insert(std::move(stage));
        }

        // Detach object
        {
            auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>(
                "detach object");

            stage->detachObject("object", handFrame);
            place->insert(std::move(stage));
        }

        // Retreat
        {
            auto stage = std::make_unique<mtc::stages::MoveRelative>(
                "retreat",
                cartesianPlanner);

            stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
            stage->setMinMaxDistance(0.05, 0.3);
            stage->setIKFrame(handFrame);
            stage->properties().set("marker_ns", "retreat");

            // Set retreat direction
            geometry_msgs::msg::Vector3Stamped vector;
            vector.header.frame_id = "world";
            vector.vector.z = 0.05;
            stage->setDirection(vector);

            place->insert(std::move(stage));
        }

        task.add(std::move(place));
    }

    // ---- Return home ----
    {
        auto stage = std::make_unique<mtc::stages::MoveTo>(
            "return home",
            interpolationPlanner);

        stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
        stage->setGoal(topDown);
        task.add(std::move(stage));
    }

    return task;
}