#include "pick_place/mtc_task_node.hpp"

#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/task_constructor/solvers.h>
#include <moveit/task_constructor/stages.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker.hpp>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("mtc_tutorial");

// ================= CONSTRUCTOR =================
MTCTaskNode::MTCTaskNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("mtc_node", options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  this->declare_parameter("velocity_scaling", 0.1);
  this->declare_parameter("acceleration_scaling", 0.1);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  grasps_sub_ = this->create_subscription<gpd_ros::msg::GraspConfigList>(
      "clustered_grasps", 10, std::bind(&MTCTaskNode::graspsCallback, this, std::placeholders::_1));
}

// ================= GRASPS CALLBACK =================
void MTCTaskNode::graspsCallback(const gpd_ros::msg::GraspConfigList::SharedPtr msg)
{
  if (msg->grasps.empty()) {
    RCLCPP_WARN(LOGGER, "No grasps received");
    return;
  }

  try {
    // Find the grasp with the highest score
    auto best_grasp = std::max_element(msg->grasps.begin(), msg->grasps.end(),
                                       [](const auto& a, const auto& b) {
                                         return a.score.data < b.score.data;
                                       });

//------------------------------------ Transform GPD grasp pose for selected object to world frame ------------------------------//

    geometry_msgs::msg::PoseStamped pose_in_object;
    pose_in_object.header = msg->header;
    pose_in_object.pose.position = best_grasp->position;

    // Construct orientation from approach, binormal, axis for object pose
    Eigen::Matrix3d rot_object;
    rot_object.col(0) = Eigen::Vector3d(best_grasp->approach.x, best_grasp->approach.y, best_grasp->approach.z);
    rot_object.col(1) = Eigen::Vector3d(best_grasp->binormal.x, best_grasp->binormal.y, best_grasp->binormal.z);
    rot_object.col(2) = Eigen::Vector3d(best_grasp->axis.x, best_grasp->axis.y, best_grasp->axis.z);    

    Eigen::Quaterniond q_object(rot_object);
    pose_in_object.pose.orientation = tf2::toMsg(q_object); 
    
    // Update timestamps to current time to avoid TF extrapolation errors
    pose_in_object.header.stamp = this->get_clock()->now();
    pose_in_object.header.stamp.sec -= 1;
    
    // Transform to world frame                                    
    geometry_msgs::msg::PoseStamped pose_out_object;
    tf_buffer_.transform(pose_in_object, pose_out_object, "world");  
    // Convert to tf2
    tf2::Quaternion q;
    tf2::fromMsg(pose_out_object.pose.orientation, q);

    tf2::Matrix3x3 m(q);

    // Extract axes
    tf2::Vector3 x_axis = m.getColumn(0);
    tf2::Vector3 y_axis = m.getColumn(1);
    tf2::Vector3 z_axis = m.getColumn(2);

    // If Z is pointing downward → flip entire frame
    if (z_axis.z() < 0) {
        x_axis = -x_axis;
        y_axis = -y_axis;
        z_axis = -z_axis;
    }

    // Rebuild rotation matrix
    tf2::Matrix3x3 m_new(
        x_axis.x(), y_axis.x(), z_axis.x(),
        x_axis.y(), y_axis.y(), z_axis.y(),
        x_axis.z(), y_axis.z(), z_axis.z()
    );

    // Convert back to quaternion
    tf2::Quaternion q_new;
    m_new.getRotation(q_new);

    pose_out_object.pose.orientation = tf2::toMsg(q_new);
    
    object_pose_ = pose_out_object;
    object_pose_.pose.position.x += 0.01; // Add small offset

//------------------Publish TF frames for visualization, Update planning scene and execute task ------------------------------//    
    
    publishTfFrames();
    setupPlanningScene();
    doTask();

  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(LOGGER, "Could not transform pose: %s", ex.what());
  }
}

// ================= PLANNING SCENE =================
void MTCTaskNode::setupPlanningScene()
{
  moveit_msgs::msg::CollisionObject object;
  object.id = "object";
  object.header.frame_id = "world";

  object.primitives.resize(1);
  object.primitives[0].type = shape_msgs::msg::SolidPrimitive::CYLINDER;
  object.primitives[0].dimensions = { 0.1, 0.02 };  
  object.pose = object_pose_.pose;
  


  moveit::planning_interface::PlanningSceneInterface psi;
  psi.applyCollisionObject(object);
}

// ================= EXECUTE TASK =================
void MTCTaskNode::doTask()
{
  task_ = createTask();

  try {
    task_.init();
  } catch (mtc::InitStageException& e) {
    RCLCPP_ERROR_STREAM(LOGGER, e);
    return;
  }

  if (!task_.plan(5)) {
    RCLCPP_ERROR_STREAM(LOGGER, "Task planning failed");
    return;
  }

  task_.introspection().publishSolution(*task_.solutions().front());

  auto result = task_.execute(*task_.solutions().front());
  if (result.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    RCLCPP_ERROR_STREAM(LOGGER, "Task execution failed");
  }
}

void MTCTaskNode::publishTfFrames()
{
  auto make_tf = [&](const std::string& child_frame, const geometry_msgs::msg::PoseStamped& pose) {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = this->get_clock()->now();
    tf_msg.header.frame_id = pose.header.frame_id;
    tf_msg.child_frame_id = child_frame;
    tf_msg.transform.translation.x = pose.pose.position.x;
    tf_msg.transform.translation.y = pose.pose.position.y;
    tf_msg.transform.translation.z = pose.pose.position.z;
    tf_msg.transform.rotation = pose.pose.orientation;
    tf_broadcaster_->sendTransform(tf_msg);
  };
  
  make_tf("object_pose", object_pose_);
}

// ================= CREATE TASK =================
mtc::Task MTCTaskNode::createTask()
{
  
  mtc::Task task;
  task.stages()->setName("demo task");
  task.loadRobotModel(this->shared_from_this());

  double velocity_scaling =
    this->get_parameter("velocity_scaling").as_double();

  double acceleration_scaling =
    this->get_parameter("acceleration_scaling").as_double();

  const auto& arm_group_name = "manipulator";
  const auto& hand_group_name = "gripper";
  const auto& hand_frame = "end_effector_link";

  // Set task properties
  task.setProperty("group", arm_group_name);
  task.setProperty("eef", hand_group_name);
  task.setProperty("ik_frame", hand_frame);

  //mtc::Stage* current_state_ptr = nullptr;  // Forward current_state on to grasp pose generator (use this with grasp_pose_generator stage)
  auto stage_state_current = std::make_unique<mtc::stages::CurrentState>("current");
  //current_state_ptr = stage_state_current.get();
  task.add(std::move(stage_state_current));

  auto sampling_planner = std::make_shared<mtc::solvers::PipelinePlanner>(this->shared_from_this());
  sampling_planner->setMaxVelocityScalingFactor(velocity_scaling);
  sampling_planner->setMaxAccelerationScalingFactor(acceleration_scaling);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();
  interpolation_planner->setMaxVelocityScalingFactor(velocity_scaling);
  interpolation_planner->setMaxAccelerationScalingFactor(acceleration_scaling);

  auto cartesian_planner = std::make_shared<mtc::solvers::CartesianPath>();
  cartesian_planner->setMaxVelocityScalingFactor(velocity_scaling);
  cartesian_planner->setMaxAccelerationScalingFactor(acceleration_scaling);
  cartesian_planner->setStepSize(.01);

  // clang-format off
  auto stage_open_hand =
      std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
  mtc::Stage* open_state_ptr = nullptr;  // Forward current_state on to pose generator (use this with pose_generator stage)
  open_state_ptr = stage_open_hand.get();
  // clang-format on
  stage_open_hand->setGroup(hand_group_name);
  stage_open_hand->setGoal(open);
  task.add(std::move(stage_open_hand));


  // clang-format off
  auto stage_move_to_pick = std::make_unique<mtc::stages::Connect>(
      "move to pick",
      mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
  // clang-format on
  stage_move_to_pick->setTimeout(10.0);
  stage_move_to_pick->properties().configureInitFrom(mtc::Stage::PARENT);
  task.add(std::move(stage_move_to_pick));

  // clang-format off
  mtc::Stage* attach_object_stage =
      nullptr;  // Forward attach_object_stage to place pose generator
  // clang-format on

  // This is an example of SerialContainer usage. It's not strictly needed here.
  // In fact, `task` itself is a SerialContainer by default.
  {
    auto grasp = std::make_unique<mtc::SerialContainer>("pick object");
    task.properties().exposeTo(grasp->properties(), { "eef", "group", "ik_frame" });
    // clang-format off
    grasp->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });
    // clang-format on

    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("approach object", cartesian_planner);
      // clang-format on
      stage->properties().set("marker_ns", "approach_object");
      stage->properties().set("link", hand_frame);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.15);

      // Set hand forward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = hand_frame;
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }
    
    /****************************************************
  ---- *            Use a Fixed GPD Grasp Pose           *
     ***************************************************/

    {
      // Sample grasp pose
      auto stage = std::make_unique<mtc::stages::GeneratePose>("generate grasp pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "grasp_pose");
      //stage->setMonitoredStage(current_state_ptr);  // Hook into current state
      stage->setMonitoredStage (open_state_ptr); // Hook into open state
      stage->setPose(object_pose_); // Use the transformed GPD grasp pose in world frame

      // This is the transform from the object frame to the end-effector frame
      Eigen::Isometry3d grasp_frame_transform;
      Eigen::Quaterniond q = Eigen::AngleAxisd((-M_PI )/ 2, Eigen::Vector3d::UnitX()) *
                             Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                             Eigen::AngleAxisd((-M_PI) / 2, Eigen::Vector3d::UnitZ());
      grasp_frame_transform.linear() = q.matrix();
      grasp_frame_transform.translation().z() = 0.14; // Adjust this offset based on your gripper geometry


      // Compute IK      
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("grasp pose IK", std::move(stage));
      // clang-format on
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_frame_transform, hand_frame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      grasp->insert(std::move(wrapper));
    }
    
    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("allow collision (hand,object)");
      stage->allowCollisions("object",
                             task.getRobotModel()
                                 ->getJointModelGroup(hand_group_name)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             true);
      // clang-format on
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("close hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal(close);
      grasp->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
      stage->attachObject("object", hand_frame);
      attach_object_stage = stage.get();
      grasp->insert(std::move(stage));
    }

    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::MoveRelative>("lift object", cartesian_planner);
      // clang-format on
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.05, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "lift_object");

      // Set upward direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.z = 1.0;
      stage->setDirection(vec);
      grasp->insert(std::move(stage));
    }
    task.add(std::move(grasp));
  }

  {
    // clang-format off
    auto stage_move_to_place = std::make_unique<mtc::stages::Connect>(
        "move to place",
        mtc::stages::Connect::GroupPlannerVector{ { arm_group_name, sampling_planner } });
    // clang-format on
    stage_move_to_place->setTimeout(5.0);
    stage_move_to_place->properties().configureInitFrom(mtc::Stage::PARENT);
    task.add(std::move(stage_move_to_place));
  }

  {
    auto place = std::make_unique<mtc::SerialContainer>("place object");
    task.properties().exposeTo(place->properties(), { "eef", "group", "ik_frame" });
    // clang-format off
    place->properties().configureInitFrom(mtc::Stage::PARENT,
                                          { "eef", "group", "ik_frame" });
    // clang-format on

    /****************************************************
  ---- *               Generate Place Pose                *
     ***************************************************/
    {
      // Sample place pose
      auto stage = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      stage->properties().configureInitFrom(mtc::Stage::PARENT);
      stage->properties().set("marker_ns", "place_pose");
      stage->setObject("object");

      geometry_msgs::msg::PoseStamped target_pose_msg;
      target_pose_msg.header.frame_id = "world";
      target_pose_msg.pose=object_pose_.pose;
      target_pose_msg.pose.position.y += 0.5; // Offset the place position in y direction
      stage->setPose(target_pose_msg);
      stage->setMonitoredStage(attach_object_stage);  // Hook into attach_object_stage

      // Compute IK
      // clang-format off
      auto wrapper =
          std::make_unique<mtc::stages::ComputeIK>("place pose IK", std::move(stage));
      // clang-format on
      wrapper->setMaxIKSolutions(2);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame("object");
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, { "eef", "group" });
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, { "target_pose" });
      place->insert(std::move(wrapper));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open hand", interpolation_planner);
      stage->setGroup(hand_group_name);
      stage->setGoal(open);
      place->insert(std::move(stage));
    }

    {
      // clang-format off
      auto stage =
          std::make_unique<mtc::stages::ModifyPlanningScene>("forbid collision (hand,object)");
      stage->allowCollisions("object",
                             task.getRobotModel()
                                 ->getJointModelGroup(hand_group_name)
                                 ->getLinkModelNamesWithCollisionGeometry(),
                             false);
      // clang-format on
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject("object", hand_frame);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
      stage->setMinMaxDistance(0.1, 0.3);
      stage->setIKFrame(hand_frame);
      stage->properties().set("marker_ns", "retreat");

      // Set retreat direction
      geometry_msgs::msg::Vector3Stamped vec;
      vec.header.frame_id = "world";
      vec.vector.x = -0.5;
      stage->setDirection(vec);
      place->insert(std::move(stage));
    }
    task.add(std::move(place));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveTo>("return home", interpolation_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, { "group" });
    stage->setGoal(home);
    task.add(std::move(stage));
  }
  return task;

}