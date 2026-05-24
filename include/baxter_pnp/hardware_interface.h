#ifndef BAXTER_PNP_HARDWARE_INTERFACE_H_
#define BAXTER_PNP_HARDWARE_INTERFACE_H_

/**
 * @file hardware_interface.h
 * @brief Abstract hardware interface for Baxter PnP.
 *
 * Enterprise Design Rationale:
 *   This abstract class decouples the BehaviorTree nodes from the physical robot.
 *   By programming to an interface, we can:
 *     1. Swap Baxter for any other robot (UR5, Panda, custom) by writing one
 *        new implementation — zero changes to BT nodes or decision logic.
 *     2. Inject a simulator or mock for unit testing without real hardware.
 *     3. Add logging/profiling/debugging via a Decorator pattern wrapper.
 *
 * Thread Safety:
 *   All implementations of this interface MUST be thread-safe.
 *   The BT executor and ROS callback threads may call these methods concurrently.
 *   Use std::mutex + std::lock_guard in derived classes (see robot_lifecycle.cpp).
 *
 * Memory Model:
 *   Implementations are managed via std::shared_ptr to allow shared ownership
 *   between the BT factory and the main control loop.
 */

#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <baxter_core_msgs/EndEffectorCommand.h>

namespace baxter_pnp
{

/**
 * @brief 3D force vector at the end-effector.
 * Maps to baxter_core_msgs/EndpointState.force
 */
struct EndEffectorForce
{
  double x{0.0}, y{0.0}, z{0.0};
};

/**
 * @brief Result of an Inverse Kinematics computation.
 */
struct IKResult
{
  bool success{false};
  std::vector<double> joints;   ///< 7 angles: [s0, s1, e0, e1, w0, w1, w2]
  std::string error_msg;
  double solution_cost{0.0};    ///< For comparing multiple IK solutions
};

/**
 * @brief Runtime status of the Baxter robot.
 */
struct RobotStatus
{
  bool enabled{false};
  bool gripper_calibrated{false};
  double gripper_position{0.0};
  bool has_error{false};
  std::string error_message;
};

/**
 * @class BaxterHardwareInterface
 * @brief Abstract interface for controlling a robotic arm.
 *
 * All PnP operations go through this interface. The concrete implementation
 * (BaxterRobotLifecycle) wires it to the actual Baxter SDK via ROS1 topics/services.
 *
 * For interviews (企业面试常考):
 *   "为什么用抽象接口而不是直接调用BaxterSDK？"
 *   答：依赖倒置原则（DIP）—— 高层模块（BT节点）不依赖低层模块（BaxterSDK），
 *   两者都依赖抽象接口。便于单元测试、机器人替换、功能扩展。
 */
class BaxterHardwareInterface : public std::enable_shared_from_this<BaxterHardwareInterface>
{
public:
  using Ptr = std::shared_ptr<BaxterHardwareInterface>;
  using ConstPtr = std::shared_ptr<const BaxterHardwareInterface>;

  virtual ~BaxterHardwareInterface() = default;

  // ── Lifecycle Management ──────────────────────────────────────────────
  virtual bool initialize() = 0;           ///< Enable robot, calibrate
  virtual bool shutdown() = 0;             ///< Disable robot safely
  virtual bool isInitialized() const = 0;  ///< Thread-safe check
  virtual RobotStatus getStatus() const = 0;

  // ── Motion Control ────────────────────────────────────────────────────
  virtual bool moveToPose(const geometry_msgs::PoseStamped& target,
                          double timeout_sec = 15.0) = 0;
  virtual bool moveToJointPositions(const std::vector<double>& joint_angles,
                                    double timeout_sec = 15.0) = 0;
  virtual bool moveToHome() = 0;
  virtual geometry_msgs::PoseStamped getCurrentEePose() const = 0;

  // ── Gripper Control ───────────────────────────────────────────────────
  virtual bool openGripper() = 0;
  virtual bool closeGripper() = 0;
  virtual bool calibrateGripper() = 0;
  virtual bool isGripperClosed() const = 0;
  virtual double getGripperPosition() const = 0;

  // ── Force / Torque Sensing ────────────────────────────────────────────
  virtual EndEffectorForce getEndEffectorForce() const = 0;

  // ── IK Service ────────────────────────────────────────────────────────
  /**
   * @brief Asynchronous IK request.
   *
   * @param pose         Target end-effector pose.
   * @param callback     Called on the ROS callback thread when IK completes.
   *                     The BT node checks the result in its onRunning().
   *
   * Enterprise note: Asynchronous avoids blocking the BT tick().
   * The callback runs on a separate thread — be thread-safe!
   */
  virtual void requestIK(const geometry_msgs::PoseStamped& pose,
                         std::function<void(const IKResult&)> callback) = 0;
};

}  // namespace baxter_pnp

#endif  // BAXTER_PNP_HARDWARE_INTERFACE_H_
