#include "baxter_pnp/bt_nodes.h"
#include "baxter_pnp/logging_utils.h"

#include <string>
#include <chrono>

#include <std_msgs/Bool.h>

namespace baxter_pnp
{

// ══════════════════════════════════════════════════════════════════════════
//  Conditions
// ══════════════════════════════════════════════════════════════════════════

BaxterEnabledCondition::BaxterEnabledCondition(
    const std::string& name, const BT::NodeConfig& config,
    BaxterHardwareInterface::Ptr hw)
  : BT::ConditionNode(name, config), hw_(std::move(hw)) {}

BT::PortsList BaxterEnabledCondition::providedPorts()
{
  return { BT::OutputPort<bool>("baxter_enabled") };
}

BT::NodeStatus BaxterEnabledCondition::tick()
{
  if (!hw_) return BT::NodeStatus::FAILURE;
  bool enabled = hw_->isInitialized() && hw_->getStatus().enabled;
  setOutput("baxter_enabled", enabled);

  if (!enabled) {
    ROS_WARN_THROTTLE(5, "[Condition] Baxter NOT enabled — check e-stop");
  }
  return enabled ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

ForceThresholdReachedCondition::ForceThresholdReachedCondition(
    const std::string& name, const BT::NodeConfig& config,
    BaxterHardwareInterface::Ptr hw)
  : BT::ConditionNode(name, config), hw_(std::move(hw)) {}

BT::PortsList ForceThresholdReachedCondition::providedPorts()
{
  return {
    BT::InputPort<double>("force_threshold"),
    BT::OutputPort<bool>("threshold_reached")
  };
}

BT::NodeStatus ForceThresholdReachedCondition::tick()
{
  if (!hw_) return BT::NodeStatus::FAILURE;
  auto force = hw_->getEndEffectorForce();
  double threshold = -8.0;
  getInput("force_threshold", threshold);

  bool reached = force.z < threshold;
  setOutput("threshold_reached", reached);

  if (reached) {
    ROS_INFO("[Condition] Contact detected — force_z=%.2f < threshold=%.1f",
             force.z, threshold);
  }
  return reached ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

GripperClosedCondition::GripperClosedCondition(
    const std::string& name, const BT::NodeConfig& config,
    BaxterHardwareInterface::Ptr hw)
  : BT::ConditionNode(name, config), hw_(std::move(hw)) {}

BT::PortsList GripperClosedCondition::providedPorts()
{
  return { BT::OutputPort<bool>("gripper_closed") };
}

BT::NodeStatus GripperClosedCondition::tick()
{
  if (!hw_) return BT::NodeStatus::FAILURE;
  bool closed = hw_->isGripperClosed();
  setOutput("gripper_closed", closed);

  if (closed) LOG_PNP_EVENT("GraspVerified", "Object held");
  else LOG_PNP_EVENT("GraspCheck", "Gripper not closed");

  return closed ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ══════════════════════════════════════════════════════════════════════════
//  Actions
// ══════════════════════════════════════════════════════════════════════════

GoHomeAction::GoHomeAction(const std::string& name,
                           const BT::NodeConfig& config,
                           BaxterHardwareInterface::Ptr hw)
  : BT::StatefulActionNode(name, config), hw_(std::move(hw)) {}

BT::PortsList GoHomeAction::providedPorts()
{
  return { BT::OutputPort<bool>("at_home") };
}

BT::NodeStatus GoHomeAction::onStart()
{
  ROS_INFO("[GoHome] Starting");
  motion_started_ = false;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GoHomeAction::onRunning()
{
  if (!motion_started_) {
    hw_->moveToHome();
    motion_started_ = true;
    return BT::NodeStatus::RUNNING;
  }
  setOutput("at_home", true);
  ROS_INFO("[GoHome] Complete");
  return BT::NodeStatus::SUCCESS;
}

void GoHomeAction::onHalted()
{
  ROS_WARN("[GoHome] Halted");
  motion_started_ = false;
}

// ── DetectObjectAction ─────────────────────────────────────────────────

DetectObjectAction::DetectObjectAction(const std::string& name,
                                       const BT::NodeConfig& config,
                                       BaxterHardwareInterface::Ptr hw)
  : BT::StatefulActionNode(name, config), hw_(std::move(hw)) {}

BT::PortsList DetectObjectAction::providedPorts()
{
  return {
    BT::OutputPort<geometry_msgs::PoseStamped>("detected_pose"),
    BT::OutputPort<bool>("object_detected")
  };
}

BT::NodeStatus DetectObjectAction::onStart()
{
  LOG_PNP_EVENT("DetectionStarted", "Waiting for grasp pose...");
  pose_received_ = false;

  // Get ROS node handle from blackboard (registered in pnp_main.cpp)
  ros::NodeHandle* nh = nullptr;
  if (config().blackboard) {
    config().blackboard->get("ros_nh", nh);
  }
  if (nh) {
    sub_ = nh->subscribe("/baxter_pnp/grasp_pose", 1,
                         &DetectObjectAction::graspPoseCB, this);
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DetectObjectAction::onRunning()
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (pose_received_) {
    setOutput("detected_pose", latest_pose_);
    setOutput("object_detected", true);

    ROS_INFO("[Detect] Pose received: [%.3f, %.3f, %.3f]",
             latest_pose_.pose.position.x,
             latest_pose_.pose.position.y,
             latest_pose_.pose.position.z);
    LOG_PNP_EVENT("DetectionComplete",
      "(" + std::to_string(latest_pose_.pose.position.x) + "," +
           std::to_string(latest_pose_.pose.position.y) + "," +
           std::to_string(latest_pose_.pose.position.z) + ")");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void DetectObjectAction::onHalted()
{
  ROS_WARN("[Detect] Halted");
  std::lock_guard<std::mutex> lock(mutex_);
  pose_received_ = false;
  sub_.shutdown();
}

void DetectObjectAction::graspPoseCB(const geometry_msgs::PoseStampedConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_pose_ = *msg;
  pose_received_ = true;
}

// ── PlanPathAction ─────────────────────────────────────────────────────

PlanPathAction::PlanPathAction(const std::string& name,
                               const BT::NodeConfig& config,
                               BaxterHardwareInterface::Ptr hw)
  : BT::StatefulActionNode(name, config), hw_(std::move(hw)) {}

BT::PortsList PlanPathAction::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::PoseStamped>("detected_pose"),
    BT::OutputPort<trajectory_msgs::JointTrajectory>("planned_trajectory"),
    BT::OutputPort<bool>("ik_success")
  };
}

BT::NodeStatus PlanPathAction::onStart()
{
  auto pose = getInput<geometry_msgs::PoseStamped>("detected_pose");
  if (!pose) {
    ROS_ERROR("[PlanPath] No detected_pose on blackboard");
    return BT::NodeStatus::FAILURE;
  }

  ik_done_ = false;
  LOG_PNP_EVENT("IKStarted",
    "target=" + std::to_string(pose->pose.position.x) + "," +
                std::to_string(pose->pose.position.y) + "," +
                std::to_string(pose->pose.position.z));

  // Async IK request — callback sets ik_done_
  hw_->requestIK(*pose, [this](const IKResult& res) {
    ik_result_ = res;
    ik_done_ = true;
  });

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus PlanPathAction::onRunning()
{
  if (ik_done_) {
    if (ik_result_.success) {
      // Build JointTrajectory from IK result
      trajectory_msgs::JointTrajectory traj;
      traj.joint_names = {"s0", "s1", "e0", "e1", "w0", "w1", "w2"};
      traj.points.resize(1);
      traj.points[0].positions = ik_result_.joints;
      traj.points[0].time_from_start = ros::Duration(2.0);

      setOutput("planned_trajectory", traj);
      setOutput("ik_success", true);
      LOG_PNP_EVENT("IKComplete", "SUCCESS");
      return BT::NodeStatus::SUCCESS;
    }
    setOutput("ik_success", false);
    LOG_PNP_EVENT("IKComplete", "FAILED — " + ik_result_.error_msg);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void PlanPathAction::onHalted()
{
  ROS_WARN("[PlanPath] Halted");
}

// ── ApproachPoseAction ─────────────────────────────────────────────────

ApproachPoseAction::ApproachPoseAction(const std::string& name,
                                       const BT::NodeConfig& config,
                                       BaxterHardwareInterface::Ptr hw)
  : BT::StatefulActionNode(name, config), hw_(std::move(hw)) {}

BT::PortsList ApproachPoseAction::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::PoseStamped>("target_pose"),
    BT::InputPort<double>("hover_distance"),
    BT::InputPort<double>("step_size"),
    BT::InputPort<double>("force_threshold"),
    BT::OutputPort<bool>("approach_complete"),
    BT::OutputPort<bool>("contact_made")
  };
}

BT::NodeStatus ApproachPoseAction::onStart()
{
  auto target = getInput<geometry_msgs::PoseStamped>("target_pose");
  if (!target) return BT::NodeStatus::FAILURE;

  double hover = 0.12, step = 0.05;
  getInput("hover_distance", hover);
  getInput("step_size", step);

  // Build approach steps (hover → step down → target)
  steps_.clear();
  current_step_ = 0;
  contact_made_ = false;

  geometry_msgs::PoseStamped hover_pose = *target;
  hover_pose.pose.position.z += hover;
  steps_.push_back(hover_pose);

  double z = hover_pose.pose.position.z;
  while (z > target->pose.position.z) {
    z = std::max(z - step, target->pose.position.z);
    geometry_msgs::PoseStamped step_pose = *target;
    step_pose.pose.position.z = z;
    steps_.push_back(step_pose);
  }

  ROS_INFO("[Approach] %zu steps from hover=%.2f to target=%.2f",
           steps_.size(), hover_pose.pose.position.z, target->pose.position.z);

  // Execute first step
  hw_->moveToPose(steps_[0]);
  current_step_ = 1;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ApproachPoseAction::onRunning()
{
  // Check force at current position
  auto force = hw_->getEndEffectorForce();
  double threshold = -8.0;
  getInput("force_threshold", threshold);

  if (force.z < threshold) {
    ROS_WARN("[Approach] Contact at step %zu/%zu — force_z=%.2f",
             current_step_, steps_.size(), force.z);
    contact_made_ = true;
    setOutput("approach_complete", true);
    setOutput("contact_made", true);
    return BT::NodeStatus::SUCCESS;
  }

  if (current_step_ >= steps_.size()) {
    setOutput("approach_complete", true);
    setOutput("contact_made", false);
    ROS_INFO("[Approach] Complete — no contact");
    return BT::NodeStatus::SUCCESS;
  }

  // Move to next step
  hw_->moveToPose(steps_[current_step_]);
  current_step_++;
  return BT::NodeStatus::RUNNING;
}

void ApproachPoseAction::onHalted()
{
  ROS_WARN("[Approach] Halted");
  current_step_ = steps_.size();  // Prevent further steps
}

// ── GraspObjectAction ──────────────────────────────────────────────────

GraspObjectAction::GraspObjectAction(const std::string& name,
                                     const BT::NodeConfig& config,
                                     BaxterHardwareInterface::Ptr hw)
  : BT::StatefulActionNode(name, config), hw_(std::move(hw)) {}

BT::PortsList GraspObjectAction::providedPorts()
{
  return { BT::OutputPort<bool>("grasp_success") };
}

BT::NodeStatus GraspObjectAction::onStart()
{
  LOG_PNP_EVENT("GraspStarted", "");
  hw_->closeGripper();
  start_time_ = ros::Time::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus GraspObjectAction::onRunning()
{
  // Wait 1s for grasp to stabilize (mirrors Python rospy.sleep(1.0))
  // But we use elapsed time instead of blocking sleep!
  if ((ros::Time::now() - start_time_).toSec() >= 1.0) {
    bool success = hw_->isGripperClosed();
    setOutput("grasp_success", success);

    if (success) {
      LOG_PNP_EVENT("GraspSuccess", "");
      return BT::NodeStatus::SUCCESS;
    }
    LOG_PNP_EVENT("GraspFailure",
      "gripper_pos=" + std::to_string(hw_->getGripperPosition()));
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void GraspObjectAction::onHalted()
{
  ROS_WARN("[Grasp] Halted — releasing gripper");
  if (hw_) hw_->openGripper();
}

// ── RetractAction ──────────────────────────────────────────────────────

RetractAction::RetractAction(const std::string& name,
                             const BT::NodeConfig& config,
                             BaxterHardwareInterface::Ptr hw)
  : BT::StatefulActionNode(name, config), hw_(std::move(hw)) {}

BT::PortsList RetractAction::providedPorts()
{
  return { BT::OutputPort<bool>("retract_complete") };
}

BT::NodeStatus RetractAction::onStart()
{
  auto pose = hw_->getCurrentEePose();
  pose.pose.position.z += 0.12;  // hover distance
  hw_->moveToPose(pose);
  started_ = true;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus RetractAction::onRunning()
{
  setOutput("retract_complete", true);
  return BT::NodeStatus::SUCCESS;
}

void RetractAction::onHalted() { ROS_WARN("[Retract] Halted"); }

// ── MoveToPlaceAction ──────────────────────────────────────────────────

MoveToPlaceAction::MoveToPlaceAction(const std::string& name,
                                     const BT::NodeConfig& config,
                                     BaxterHardwareInterface::Ptr hw)
  : BT::StatefulActionNode(name, config), hw_(std::move(hw)) {}

BT::PortsList MoveToPlaceAction::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::PoseStamped>("place_position"),
    BT::OutputPort<bool>("at_place_pose")
  };
}

BT::NodeStatus MoveToPlaceAction::onStart()
{
  auto pos = getInput<geometry_msgs::PoseStamped>("place_position");
  if (!pos) return BT::NodeStatus::FAILURE;

  LOG_PNP_EVENT("MoveToPlaceStarted",
    "target=" + std::to_string(pos->pose.position.x) + "," +
                std::to_string(pos->pose.position.y) + "," +
                std::to_string(pos->pose.position.z));
  hw_->moveToPose(*pos);
  started_ = true;
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveToPlaceAction::onRunning()
{
  setOutput("at_place_pose", true);
  LOG_PNP_EVENT("MoveToPlaceComplete", "");
  return BT::NodeStatus::SUCCESS;
}

void MoveToPlaceAction::onHalted() { ROS_WARN("[MoveToPlace] Halted"); }

// ── ReleaseObjectAction ────────────────────────────────────────────────

ReleaseObjectAction::ReleaseObjectAction(const std::string& name,
                                         const BT::NodeConfig& config,
                                         BaxterHardwareInterface::Ptr hw)
  : BT::StatefulActionNode(name, config), hw_(std::move(hw)) {}

BT::PortsList ReleaseObjectAction::providedPorts()
{
  return { BT::OutputPort<bool>("release_complete") };
}

BT::NodeStatus ReleaseObjectAction::onStart()
{
  LOG_PNP_EVENT("ReleaseStarted", "");
  hw_->openGripper();
  start_time_ = ros::Time::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ReleaseObjectAction::onRunning()
{
  if ((ros::Time::now() - start_time_).toSec() >= 1.0) {
    setOutput("release_complete", true);
    LOG_PNP_EVENT("ReleaseComplete", "");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void ReleaseObjectAction::onHalted() { ROS_WARN("[Release] Halted"); }

// ── RequestGraspPoseAction ─────────────────────────────────────────────

RequestGraspPoseAction::RequestGraspPoseAction(const std::string& name,
                                               const BT::NodeConfig& config,
                                               BaxterHardwareInterface::Ptr hw)
  : BT::SimpleActionNode(name, config), hw_(std::move(hw))
{
  ros::NodeHandle* nh = nullptr;
  if (config.blackboard) config.blackboard->get("ros_nh", nh);
  if (nh) pub_ = nh->advertise<std_msgs::Bool>("/baxter_pnp/grasp_request", 1);
}

BT::PortsList RequestGraspPoseAction::providedPorts()
{
  return { BT::OutputPort<bool>("request_sent") };
}

BT::NodeStatus RequestGraspPoseAction::tick()
{
  std_msgs::Bool msg;
  msg.data = true;
  pub_.publish(msg);
  setOutput("request_sent", true);
  ROS_INFO("[RequestGrasp] Requested next pose");
  return BT::NodeStatus::SUCCESS;
}

// ── AlarmTriggerAction ─────────────────────────────────────────────────

AlarmTriggerAction::AlarmTriggerAction(const std::string& name,
                                       const BT::NodeConfig& config,
                                       BaxterHardwareInterface::Ptr hw)
  : BT::SimpleActionNode(name, config), hw_(std::move(hw)) {}

BT::PortsList AlarmTriggerAction::providedPorts()
{
  return { BT::InputPort<std::string>("error_reason") };
}

BT::NodeStatus AlarmTriggerAction::tick()
{
  std::string reason;
  getInput("error_reason", reason);

  ROS_FATAL("[ALARM] %s", reason.c_str());
  LOG_PNP_EVENT("AlarmTriggered", reason);

  if (hw_) hw_->shutdown();
  return BT::NodeStatus::SUCCESS;
}

// ══════════════════════════════════════════════════════════════════════════
//  Factory Registration
// ══════════════════════════════════════════════════════════════════════════

void registerAllNodes(BT::BehaviorTreeFactory& factory,
                      BaxterHardwareInterface::Ptr hw)
{
  factory.registerNodeType<BaxterEnabledCondition>("BaxterEnabled", hw);
  factory.registerNodeType<ForceThresholdReachedCondition>("ForceThresholdReached", hw);
  factory.registerNodeType<GripperClosedCondition>("GripperClosed", hw);

  factory.registerNodeType<GoHomeAction>("GoHome", hw);
  factory.registerNodeType<DetectObjectAction>("DetectObject", hw);
  factory.registerNodeType<PlanPathAction>("PlanPath", hw);
  factory.registerNodeType<ApproachPoseAction>("ApproachPose", hw);
  factory.registerNodeType<GraspObjectAction>("GraspObject", hw);
  factory.registerNodeType<RetractAction>("Retract", hw);
  factory.registerNodeType<MoveToPlaceAction>("MoveToPlace", hw);
  factory.registerNodeType<ReleaseObjectAction>("ReleaseObject", hw);
  factory.registerNodeType<RequestGraspPoseAction>("RequestGraspPose", hw);
  factory.registerNodeType<AlarmTriggerAction>("AlarmTrigger", hw);
}

}  // namespace baxter_pnp
