#ifndef BAXTER_PNP_GRIPPER_CONTROLLER_H_
#define BAXTER_PNP_GRIPPER_CONTROLLER_H_

/**
 * @file gripper_controller.h
 * @brief Thread-safe gripper controller with actionlib interface.
 *
 * Enterprise Design:
 *   Rather than a raw topic publish + sleep(1.0) (as in the original Python code),
 *   we implement a proper actionlib server for the gripper. This gives us:
 *     1. Feedback during grasp (force reading)
 *     2. Result confirmation (success/failure)
 *     3. Preempt capability (cancel grasp on halt)
 *     4. No blocking sleep() calls — use timers
 *
 * 面试联想:
 *   actionlib vs 直接pub topic:
 *     - Topic: 单向数据流，适合高频传感器数据
 *     - Service: 同步请求-响应，适合IK（耗时<100ms）
 *     - Action: 异步带反馈，适合夹爪/运动（耗时>1s）
 */

#include <atomic>
#include <mutex>
#include <string>

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/GripperCommandAction.h>

#include "hardware_interface.h"

namespace baxter_pnp
{

/**
 * @class GripperController
 * @brief Manages Baxter's electric parallel-jaw gripper.
 *
 * Thread safety: all public methods are lock-guarded.
 */
class GripperController
{
public:
  GripperController(ros::NodeHandle& nh, const std::string& limb);
  ~GripperController() = default;

  bool open();                  ///< Open gripper fully
  bool close();                 ///< Close gripper with max force
  bool calibrate();             ///< Find zero position
  bool isClosed() const;        ///< Check if gripper position > 90%
  double getPosition() const;   ///< 0.0=open, 1.0=closed
  double getForce() const;      ///< Current grip force in N

private:
  void gripperActionCB();       ///< Action server callback

  ros::Publisher cmd_pub_;
  ros::Subscriber state_sub_;

  std::string limb_;
  mutable std::mutex mutex_;
  double position_{0.0};
  double force_{0.0};

  /// Action server for external gripper commands (e.g., from MoveIt!)
  actionlib::SimpleActionServer<control_msgs::GripperCommandAction> as_;
};

}  // namespace baxter_pnp

#endif  // BAXTER_PNP_GRIPPER_CONTROLLER_H_
