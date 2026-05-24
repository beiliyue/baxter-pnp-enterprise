#include "baxter_pnp/robot_lifecycle.h"
#include "baxter_pnp/logging_utils.h"

#include <memory>
#include <string>
#include <chrono>
#include <thread>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <baxter_core_msgs/SolvePositionIK.h>
#include <baxter_core_msgs/EndEffectorCommand.h>

namespace baxter_pnp
{

// ═══════════════════════════════════════════════════════════════════════════
//  Constructor / Destructor
// ═══════════════════════════════════════════════════════════════════════════

BaxterRobotLifecycle::BaxterRobotLifecycle(ros::NodeHandle& nh,
                                           const std::string& limb,
                                           bool auto_recover)
  : nh_(nh)
  , limb_(limb)
  , auto_recover_(auto_recover)
  , traj_as_(nh, limb + "_follow_joint_trajectory",
             boost::bind(&BaxterRobotLifecycle::trajectoryActionCB, this), false)
  , diagnostic_(nh_, ros::this_node::getName(), ros::this_node::getName())
{
  ROS_INFO("[Lifecycle] BaxterRobotLifecycle created — limb=%s, auto_recover=%s",
           limb_.c_str(), auto_recover_ ? "true" : "false");

  // Register diagnostic callback (publishes to /diagnostics)
  diagnostic_.add("Baxter Robot Status", this, &BaxterRobotLifecycle::diagnosticCB);
}

BaxterRobotLifecycle::~BaxterRobotLifecycle()
{
  shutdown();
}

// ═══════════════════════════════════════════════════════════════════════════
//  Lifecycle
// ═══════════════════════════════════════════════════════════════════════════

bool BaxterRobotLifecycle::initialize()
{
  std::lock_guard<std::mutex> lock(mutex_);
  state_ = RobotState::INITIALIZING;

  ROS_INFO("[Lifecycle] Initializing Baxter robot...");

  // ── Publishers ──────────────────────────────────────────────────────
  joint_cmd_pub_ = nh_.advertise<baxter_core_msgs::JointCommand>(
    "/robot/limb/" + limb_ + "/joint_command", 1);

  gripper_cmd_pub_ = nh_.advertise<baxter_core_msgs::EndEffectorCommand>(
    "/robot/end_effector/" + limb_ + "/gripper_command", 1);

  // ── Subscribers ─────────────────────────────────────────────────────
  joint_state_sub_ = nh_.subscribe("/robot/joint_states", 10,
    &BaxterRobotLifecycle::jointStateCB, this);

  endpoint_state_sub_ = nh_.subscribe(
    "/robot/limb/" + limb_ + "/endpoint_state", 10,
    &BaxterRobotLifecycle::endpointStateCB, this);

  // ── IK Service Client ───────────────────────────────────────────────
  std::string ik_service = "ExternalTools/" + limb_ + "/PositionKinematicsNode/IKService";
  ik_client_ = nh_.serviceClient<baxter_core_msgs::SolvePositionIK>(ik_service);

  // ── Connect to robot ────────────────────────────────────────────────
  // NOTE: In real Baxter, this calls baxter_interface::RobotEnable
  // Here we demonstrate the pattern with a simulated connection.
  if (!connectToRobot()) {
    state_ = RobotState::ERROR;
    ROS_ERROR("[Lifecycle] Failed to connect to Baxter");
    return false;
  }

  // ── Start health monitoring timer ───────────────────────────────────
  // Checks connection status every 5 seconds
  health_timer_ = nh_.createTimer(ros::Duration(5.0),
    [this](const ros::TimerEvent&) { checkConnectionHealth(); });

  // ── Start action server ─────────────────────────────────────────────
  traj_as_.start();
  diagnostic_.force_update();

  state_ = RobotState::RUNNING;
  status_.enabled = true;
  LOG_PNP_EVENT("SystemInitialized", "limb=" + limb_);
  ROS_INFO("[Lifecycle] Baxter initialized successfully");
  return true;
}

bool BaxterRobotLifecycle::shutdown()
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (state_ == RobotState::SHUTDOWN ||
      state_ == RobotState::UNINITIALIZED) {
    return true;  // Already shut down
  }

  ROS_WARN("[Lifecycle] Shutting down Baxter...");
  state_ = RobotState::SHUTDOWN;

  // Cancel health timer
  health_timer_.stop();

  // Disconnect robot hardware
  disconnectFromRobot();

  // Shutdown publishers
  joint_cmd_pub_.shutdown();
  gripper_cmd_pub_.shutdown();

  // Stop action server
  traj_as_.shutdown();

  status_.enabled = false;
  LOG_PNP_EVENT("SystemShutdown", "");
  ROS_INFO("[Lifecycle] Baxter shut down");
  return true;
}

bool BaxterRobotLifecycle::isInitialized() const
{
  return state_ == RobotState::RUNNING;
}

RobotStatus BaxterRobotLifecycle::getStatus() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return status_;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Motion
// ═══════════════════════════════════════════════════════════════════════════

bool BaxterRobotLifecycle::moveToPose(const geometry_msgs::PoseStamped& target,
                                      double timeout_sec)
{
  if (state_ != RobotState::RUNNING) {
    ROS_WARN_THROTTLE(5, "[Motion] Cannot move — state=%s", stateToString().c_str());
    return false;
  }

  ROS_INFO("[Motion] moveToPose: [%.3f, %.3f, %.3f] timeout=%.1fs",
           target.pose.position.x, target.pose.position.y,
           target.pose.position.z, timeout_sec);

  // Use actionlib client approach — for brevity, call IK then publish
  baxter_core_msgs::SolvePositionIK srv;
  geometry_msgs::PoseStamped ps = target;
  ps.header.frame_id = "base";
  srv.request.pose_stamp.push_back(ps);

  if (!ik_client_.call(srv)) {
    ROS_ERROR("[Motion] IK service call failed");
    return false;
  }

  if (srv.response.isValid[0]) {
    // Publish joint command
    baxter_core_msgs::JointCommand cmd;
    cmd.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
    cmd.names = joint_names_;
    cmd.command = srv.response.joints[0].position;
    joint_cmd_pub_.publish(cmd);

    logEePose(limb_, target);
    return true;
  }

  ROS_ERROR("[Motion] IK solution invalid");
  return false;
}

bool BaxterRobotLifecycle::moveToJointPositions(
    const std::vector<double>& joint_angles, double timeout_sec)
{
  if (joint_angles.size() != 7) {
    ROS_ERROR("[Motion] Expected 7 joints, got %zu", joint_angles.size());
    return false;
  }

  baxter_core_msgs::JointCommand cmd;
  cmd.mode = baxter_core_msgs::JointCommand::POSITION_MODE;
  cmd.names = joint_names_;
  cmd.command = joint_angles;
  joint_cmd_pub_.publish(cmd);
  return true;
}

bool BaxterRobotLifecycle::moveToHome()
{
  ROS_INFO("[Motion] Moving to home (neutral) position");
  std::vector<double> neutral = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  return moveToJointPositions(neutral, 10.0);
}

geometry_msgs::PoseStamped BaxterRobotLifecycle::getCurrentEePose() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return current_pose_;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Gripper
// ═══════════════════════════════════════════════════════════════════════════

bool BaxterRobotLifecycle::openGripper()
{
  ROS_INFO("[Gripper] Opening");
  baxter_core_msgs::EndEffectorCommand cmd;
  cmd.command = baxter_core_msgs::EndEffectorCommand::CMD_OPEN;
  cmd.id = 65536;  // Baxter gripper ID
  gripper_cmd_pub_.publish(cmd);

  // ⚠️ DO NOT use sleep() here — use a timer or wait for state feedback.
  // The actionlib GripperCommandAction provides proper feedback.
  // This demo uses a short sleep, but in production use the action server.
  ros::Duration(0.5).sleep();

  {
    std::lock_guard<std::mutex> lock(mutex_);
    status_.gripper_position = 0.0;
  }
  return true;
}

bool BaxterRobotLifecycle::closeGripper()
{
  ROS_INFO("[Gripper] Closing");
  baxter_core_msgs::EndEffectorCommand cmd;
  cmd.command = baxter_core_msgs::EndEffectorCommand::CMD_CLOSE;
  cmd.id = 65536;
  gripper_cmd_pub_.publish(cmd);

  ros::Duration(0.5).sleep();

  {
    std::lock_guard<std::mutex> lock(mutex_);
    status_.gripper_position = 1.0;
  }
  return true;
}

bool BaxterRobotLifecycle::calibrateGripper()
{
  ROS_INFO("[Gripper] Calibrating");
  baxter_core_msgs::EndEffectorCommand cmd;
  cmd.command = baxter_core_msgs::EndEffectorCommand::CMD_CALIBRATE;
  cmd.id = 65536;
  gripper_cmd_pub_.publish(cmd);

  ros::Duration(2.0).sleep();

  {
    std::lock_guard<std::mutex> lock(mutex_);
    status_.gripper_calibrated = true;
  }
  return true;
}

bool BaxterRobotLifecycle::isGripperClosed() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return status_.gripper_position > 0.9;
}

double BaxterRobotLifecycle::getGripperPosition() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return status_.gripper_position;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Force / IK
// ═══════════════════════════════════════════════════════════════════════════

EndEffectorForce BaxterRobotLifecycle::getEndEffectorForce() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return last_force_;
}

void BaxterRobotLifecycle::requestIK(
    const geometry_msgs::PoseStamped& pose,
    std::function<void(const IKResult&)> callback)
{
  // In a full implementation, this queues the request to IKSolver.
  // For brevity, we call synchronously here (but in production,
  // use the IKSolver thread pool).
  baxter_core_msgs::SolvePositionIK srv;
  srv.request.pose_stamp.push_back(pose);

  IKResult result;
  if (ik_client_.call(srv)) {
    result.success = srv.response.isValid[0];
    if (result.success) {
      result.joints = srv.response.joints[0].position;
    }
  }

  if (callback) callback(result);
}

// ═══════════════════════════════════════════════════════════════════════════
//  Private: ROS Callbacks
// ═══════════════════════════════════════════════════════════════════════════

void BaxterRobotLifecycle::jointStateCB(const sensor_msgs::JointStateConstPtr& msg)
{
  // ⚠️ This runs on AsyncSpinner thread — do minimal work, no blocking
  std::lock_guard<std::mutex> lock(mutex_);
  last_joint_state_ = *msg;

  // Throttle high-frequency log (100Hz → 1Hz)
  ROS_INFO_THROTTLE(1, "[JointState] %zu joints received", msg->name.size());
}

void BaxterRobotLifecycle::endpointStateCB(
    const baxter_core_msgs::EndpointStateConstPtr& msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  current_pose_.pose = msg->pose;
  current_pose_.header = msg->header;
  last_force_.x = msg->wrench.force.x;
  last_force_.y = msg->wrench.force.y;
  last_force_.z = msg->wrench.force.z;
}

void BaxterRobotLifecycle::checkConnectionHealth()
{
  // Periodically check if we lost connection to the robot
  // In production, check the time since last joint state message
  if (!status_.enabled && auto_recover_) {
    ROS_WARN("[Health] Connection lost — attempting recovery...");
    triggerRecovery();
  }
}

bool BaxterRobotLifecycle::triggerRecovery()
{
  ROS_WARN("[Lifecycle] Triggering recovery...");
  state_ = RobotState::INITIALIZING;

  bool ok = connectToRobot();

  std::lock_guard<std::mutex> lock(mutex_);
  if (ok) {
    state_ = RobotState::RUNNING;
    status_.enabled = true;
    ROS_INFO("[Lifecycle] Recovery successful");
  } else {
    state_ = RobotState::ERROR;
    ROS_ERROR("[Lifecycle] Recovery failed");
  }
  return ok;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Private: Connection
// ═══════════════════════════════════════════════════════════════════════════

bool BaxterRobotLifecycle::connectToRobot()
{
  ROS_INFO("[Connection] Enabling robot...");

  // Simulated connection — in production:
  //   baxter_interface::RobotEnable enable(true);
  //   enable.enable();
  ROS_INFO("[Connection] Robot enabled");
  return true;
}

bool BaxterRobotLifecycle::disconnectFromRobot()
{
  ROS_WARN("[Connection] Disabling robot...");
  // In production:
  //   baxter_interface::RobotEnable enable(true);
  //   enable.disable();
  return true;
}

// ═══════════════════════════════════════════════════════════════════════════
//  Private: Diagnostics / Action
// ═══════════════════════════════════════════════════════════════════════════

void BaxterRobotLifecycle::diagnosticCB(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  std::lock_guard<std::mutex> lock(mutex_);

  stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Baxter Running");

  if (state_ == RobotState::ERROR) {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Baxter in ERROR state");
  } else if (state_ == RobotState::FATAL_ERROR) {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Baxter FATAL ERROR");
  }

  stat.add("Limb", limb_);
  stat.add("State", stateToString());
  stat.add("Gripper Calibrated", status_.gripper_calibrated ? "yes" : "no");
  stat.add("Gripper Position", status_.gripper_position);
}

void BaxterRobotLifecycle::trajectoryActionCB()
{
  // Handle follow_joint_trajectory action goal
  auto goal = traj_as_.acceptNewGoal();
  // ... implementation would execute the trajectory ...
  traj_as_.setSucceeded();
}

std::string BaxterRobotLifecycle::stateToString() const
{
  switch (state_) {
    case RobotState::UNINITIALIZED: return "UNINITIALIZED";
    case RobotState::INITIALIZING:  return "INITIALIZING";
    case RobotState::RUNNING:       return "RUNNING";
    case RobotState::ERROR:         return "ERROR";
    case RobotState::FATAL_ERROR:   return "FATAL_ERROR";
    case RobotState::SHUTDOWN:      return "SHUTDOWN";
    default:                        return "UNKNOWN";
  }
}

}  // namespace baxter_pnp
