#ifndef BAXTER_PNP_ROBOT_LIFECYCLE_H_
#define BAXTER_PNP_ROBOT_LIFECYCLE_H_

/**
 * @file robot_lifecycle.h
 * @brief Manages Baxter robot lifecycle (enable/disable/error recovery).
 *
 * Enterprise Design:
 *   ROS1 does not have a LifecycleNode (that's ROS2-only).
 *   We implement a robust lifecycle state machine ourselves:
 *
 *   UNINITIALIZED → INITIALIZING → RUNNING → ERROR → RECOVERING → RUNNING
 *                       ↓                                       ↓
 *                   SHUTDOWN ←───────────────────────────── SHUTDOWN
 *
 * Key differences from the original Python ROS1 code (robot.py):
 *   1. State machine instead of linear init
 *   2. Automatic recovery on connection loss
 *   3. diagnostic_updater publishes health status
 *   4. Thread-safe status accessors with std::mutex
 *
 * 面试常考:
 *   Q: "ROS1没有Lifecycle Node，你怎么保证机器人安全启动和关闭？"
 *   A: "手动实现状态机，每个状态有entry/exit动作，错误时自动回到INITIALIZING。
 *      结合diagnostic_updater上报健康状态，结合rosbag做故障现场保留。"
 */

#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <atomic>

#include <ros/ros.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <baxter_core_msgs/EndpointState.h>
#include <baxter_core_msgs/JointCommand.h>
#include <sensor_msgs/JointState.h>

#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include "hardware_interface.h"

namespace baxter_pnp
{

/**
 * @brief Robot lifecycle states.
 */
enum class RobotState
{
  UNINITIALIZED,   ///< After construction
  INITIALIZING,    ///< Connecting to Baxter, enabling motors
  RUNNING,         ///< Normal operation
  ERROR,           ///< Non-fatal error (will auto-recover)
  FATAL_ERROR,     ///< Fatal error (requires manual restart)
  SHUTDOWN         ///< Cleanly shut down
};

/**
 * @class BaxterRobotLifecycle
 * @brief Concrete implementation of BaxterHardwareInterface.
 *
 * Thread Safety Model:
 *   - All public methods acquire mutex_ before touching shared state.
 *   - ROS callbacks (jointStateCB, endpointStateCB) run on AsyncSpinner threads
 *     and store data under lock_guard.
 *   - The BT executor calls tick() on its own thread — it reads via getStatus()
 *     which returns a copy (not a reference) under lock.
 *
 * Zombie Process Prevention (面试高频):
 *   This class does NOT fork(). For child processes in ROS:
 *   - Use nodelet for in-process zero-copy (no zombie risk)
 *   - If you must fork(), capture SIGCHLD and waitpid() in a loop
 *   - Never use system() or popen() in production code
 */
class BaxterRobotLifecycle : public BaxterHardwareInterface
{
public:
  /**
   * @param nh           ROS node handle (private)
   * @param limb         "left" or "right"
   * @param auto_recover Automatically retry connection on failure
   */
  explicit BaxterRobotLifecycle(ros::NodeHandle& nh,
                                const std::string& limb = "right",
                                bool auto_recover = true);

  ~BaxterRobotLifecycle() override;

  // ── BaxterHardwareInterface ────────────────────────────────────────────
  bool initialize() override;
  bool shutdown() override;
  bool isInitialized() const override;
  RobotStatus getStatus() const override;

  bool moveToPose(const geometry_msgs::PoseStamped& target,
                  double timeout_sec = 15.0) override;
  bool moveToJointPositions(const std::vector<double>& joint_angles,
                            double timeout_sec = 15.0) override;
  bool moveToHome() override;
  geometry_msgs::PoseStamped getCurrentEePose() const override;

  bool openGripper() override;
  bool closeGripper() override;
  bool calibrateGripper() override;
  bool isGripperClosed() const override;
  double getGripperPosition() const override;

  EndEffectorForce getEndEffectorForce() const override;

  void requestIK(const geometry_msgs::PoseStamped& pose,
                 std::function<void(const IKResult&)> callback) override;

  // ── Enterprise-specific ────────────────────────────────────────────────
  RobotState getState() const { return state_; }
  std::string stateToString() const;
  bool triggerRecovery();           ///< Attempt auto-recovery from ERROR

private:
  // ── ROS Callbacks (run on AsyncSpinner threads) ────────────────────────
  void jointStateCB(const sensor_msgs::JointStateConstPtr& msg);
  void endpointStateCB(const baxter_core_msgs::EndpointStateConstPtr& msg);

  // ── Lifecycle helpers ─────────────────────────────────────────────────
  bool connectToRobot();            ///< Enable Baxter motors
  bool disconnectFromRobot();       ///< Disable Baxter motors
  void checkConnectionHealth();     ///< Timer callback — monitor connection

  // ── Diagnostic updater ────────────────────────────────────────────────
  void diagnosticCB(diagnostic_updater::DiagnosticStatusWrapper& stat);

  // ── Trajectory Action Server ──────────────────────────────────────────
  // Exposes follow_joint_trajectory action for MoveIt! / RViz integration
  void trajectoryActionCB();
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> traj_as_;

  // ── Members ────────────────────────────────────────────────────────────
  ros::NodeHandle nh_;
  std::string limb_;

  // Thread synchronization
  mutable std::mutex mutex_;

  // State
  std::atomic<RobotState> state_{RobotState::UNINITIALIZED};
  bool auto_recover_;

  // Publishers
  ros::Publisher joint_cmd_pub_;
  ros::Publisher gripper_cmd_pub_;

  // Subscribers
  ros::Subscriber joint_state_sub_;
  ros::Subscriber endpoint_state_sub_;

  // Service clients
  ros::ServiceClient ik_client_;

  // Timer for connection health monitoring
  ros::Timer health_timer_;

  // Shared state (guarded by mutex_)
  RobotStatus status_;
  EndEffectorForce last_force_;
  geometry_msgs::PoseStamped current_pose_;
  sensor_msgs::JointState last_joint_state_;

  // Diagnostic updater
  diagnostic_updater::Updater diagnostic_;

  // Joint names (Baxter 7-DOF)
  const std::vector<std::string> joint_names_{
    "s0", "s1", "e0", "e1", "w0", "w1", "w2"
  };
};

}  // namespace baxter_pnp

#endif  // BAXTER_PNP_ROBOT_LIFECYCLE_H_
