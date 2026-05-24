#ifndef BAXTER_PNP_BT_NODES_H_
#define BAXTER_PNP_BT_NODES_H_

/**
 * @file bt_nodes.h
 * @brief BehaviorTree.CPP v4 Action and Condition nodes for Baxter PnP.
 *
 * ========================================================
 * 企业级 BehaviorTree 节点设计
 * ========================================================
 *
 * 为什么用 BehaviorTree 而不是传统状态机 (FSM/SMACH)？
 *   (面试高频问题)
 *
 *   1. 可组合性 (Composability):
 *      状态机的状态转移是硬编码的，每个新状态需要修改转移矩阵。
 *      BT节点通过XML组合，新增"重试3次"只需要加一个 <Retry> 装饰器，
 *      不需要改一行C++代码。
 *
 *   2. 可视化 (Debuggability):
 *      BT.CPP 自带 Groot2 可视化工具，可以实时观察每个节点的
 *      SUCCESS/FAILURE/RUNNING 状态。传统状态机需要手动打log。
 *
 *   3. 异常处理 (Exception Handling):
 *      Fallback + Retry + Sequence 的嵌套结构天然支持异常处理。
 *      状态机需要每个状态添加error transition，容易遗漏。
 *
 *   4. 模块复用 (Reusability):
 *      "ApproachPose" 节点可以在"抓取"和"放置"两个场景复用，
 *      只需要不同的参数（force_threshold）。状态机需要复制状态。
 *
 * 节点分类:
 *   - Condition: 瞬时检查，无副作用，不返回RUNNING
 *   - Action:    有副作用，可返回RUNNING（StatefulActionNode）
 *
 * ========================================================
 * 多线程安全模型
 * ========================================================
 *   BT节点在 tick() 线程上执行。
 *   ROS回调在 AsyncSpinner 线程上执行。
 *   共享数据通过 BaxterHardwareInterface（内部加锁）访问。
 *
 *   ⚠️ 不要在 tick() 里：
 *     - 调用 ros::spinOnce()  ❌
 *     - 创建 ros::NodeHandle  ❌（在节点构造时创建）
 *     - 阻塞等待 >10ms        ❌（用StatefulActionNode的onRunning）
 */

#include <memory>
#include <string>
#include <mutex>
#include <atomic>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/bt_factory.h>

#include "hardware_interface.h"
#include "logging_utils.h"

namespace baxter_pnp
{

// ══════════════════════════════════════════════════════════════════════════
//  Conditions (瞬时检查，无副作用)
// ══════════════════════════════════════════════════════════════════════════

/**
 * @brief 检查Baxter机器人是否已使能。
 *
 * 对应Python: rs.state().enabled (robot.py:51)
 *
 * Blackboard Ports:
 *   Output: "baxter_enabled" (bool)
 */
class BaxterEnabledCondition : public BT::ConditionNode
{
public:
  BaxterEnabledCondition(const std::string& name,
                         const BT::NodeConfig& config,
                         BaxterHardwareInterface::Ptr hw);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  BaxterHardwareInterface::Ptr hw_;
};

/**
 * @brief 检测是否力超过了阈值（接近时接触物体）。
 *
 * 对应Python: if force.z < self.force_threshold: (pnp.py:38)
 *
 * Blackboard Ports:
 *   Input:  "force_threshold" (double)
 *   Output: "threshold_reached" (bool)
 */
class ForceThresholdReachedCondition : public BT::ConditionNode
{
public:
  ForceThresholdReachedCondition(const std::string& name,
                                 const BT::NodeConfig& config,
                                 BaxterHardwareInterface::Ptr hw);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  BaxterHardwareInterface::Ptr hw_;
};

/**
 * @brief 检查夹爪是否闭合（抓取验证）。
 *
 * Blackboard Ports:
 *   Output: "gripper_closed" (bool)
 */
class GripperClosedCondition : public BT::ConditionNode
{
public:
  GripperClosedCondition(const std::string& name,
                         const BT::NodeConfig& config,
                         BaxterHardwareInterface::Ptr hw);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  BaxterHardwareInterface::Ptr hw_;
};

// ══════════════════════════════════════════════════════════════════════════
//  Actions (有副作用，返回RUNNING)
// ══════════════════════════════════════════════════════════════════════════

/**
 * @brief 移动到HOME位置。
 *
 * 对应Python: self.robot.go_home() (pnp.py:88)
 *
 * StatefulActionNode + actionlib client:
 *   onStart()  → 发送 goal 到轨迹action server
 *   onRunning() → 等待 actionlib 反馈
 *   onHalted()  → 取消 goal
 */
class GoHomeAction : public BT::StatefulActionNode
{
public:
  GoHomeAction(const std::string& name,
               const BT::NodeConfig& config,
               BaxterHardwareInterface::Ptr hw);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  BaxterHardwareInterface::Ptr hw_;
  bool motion_started_{false};
};

/**
 * @brief 等待视觉节点发布抓取位姿。
 *
 * 对应Python: while not np.load(grasp_available): time.sleep(0.1)
 *            (pnp.py:97-99)
 *
 * 订阅 /baxter_pnp/grasp_pose topic，收到后写入Blackboard。
 *
 * Blackboard Ports:
 *   Output: "detected_pose" (PoseStamped)
 *   Output: "object_detected" (bool)
 */
class DetectObjectAction : public BT::StatefulActionNode
{
public:
  DetectObjectAction(const std::string& name,
                     const BT::NodeConfig& config,
                     BaxterHardwareInterface::Ptr hw);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  void graspPoseCB(const geometry_msgs::PoseStampedConstPtr& msg);
  BaxterHardwareInterface::Ptr hw_;
  geometry_msgs::PoseStamped latest_pose_;
  bool pose_received_{false};
  ros::Subscriber sub_;
  mutable std::mutex mutex_;
};

/**
 * @brief IK路径规划（后台非阻塞）。
 *
 * onStart() → 调用 hw_->requestIK() 异步求解
 * onRunning() → 检查IK是否完成
 *
 * Blackboard Ports:
 *   Input:  "detected_pose" (PoseStamped)
 *   Output: "planned_trajectory" (JointTrajectory)
 *   Output: "ik_success" (bool)
 */
class PlanPathAction : public BT::StatefulActionNode
{
public:
  PlanPathAction(const std::string& name,
                 const BT::NodeConfig& config,
                 BaxterHardwareInterface::Ptr hw);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  BaxterHardwareInterface::Ptr hw_;
  IKResult ik_result_;
  bool ik_done_{false};
};

/**
 * @brief 力控接近物体（逐步下降+力监控）。
 *
 * 对应Python: _approach(pose) (pnp.py:28-41)
 *
 * 核心逻辑:
 *   1. 先移到hover位置 (z + hover_distance)
 *   2. 按step_size逐步下降
 *   3. 每一步检查末端力是否超过阈值
 *   4. 超过阈值即停止（接触物体）
 */
class ApproachPoseAction : public BT::StatefulActionNode
{
public:
  ApproachPoseAction(const std::string& name,
                     const BT::NodeConfig& config,
                     BaxterHardwareInterface::Ptr hw);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  BaxterHardwareInterface::Ptr hw_;
  std::vector<geometry_msgs::PoseStamped> steps_;
  size_t current_step_{0};
  bool contact_made_{false};
};

/**
 * @brief 闭合夹爪抓取物体。
 *
 * 对应Python: self.robot.close_gripper() (robot.py:63)
 *
 * Blackboard Ports:
 *   Output: "grasp_success" (bool)
 */
class GraspObjectAction : public BT::StatefulActionNode
{
public:
  GraspObjectAction(const std::string& name,
                    const BT::NodeConfig& config,
                    BaxterHardwareInterface::Ptr hw);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  BaxterHardwareInterface::Ptr hw_;
  ros::Time start_time_;
};

/**
 * @brief 抓取后抬臂升起。
 *
 * 对应Python: _retract() (pnp.py:44-56)
 */
class RetractAction : public BT::StatefulActionNode
{
public:
  RetractAction(const std::string& name,
                const BT::NodeConfig& config,
                BaxterHardwareInterface::Ptr hw);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  BaxterHardwareInterface::Ptr hw_;
  bool started_{false};
};

/**
 * @brief 移动到放置位置。
 *
 * Blackboard Ports:
 *   Input: "place_position" (PoseStamped)
 *   Output: "at_place_pose" (bool)
 */
class MoveToPlaceAction : public BT::StatefulActionNode
{
public:
  MoveToPlaceAction(const std::string& name,
                    const BT::NodeConfig& config,
                    BaxterHardwareInterface::Ptr hw);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  BaxterHardwareInterface::Ptr hw_;
  bool started_{false};
};

/**
 * @brief 打开夹爪释放物体。
 *
 * 对应Python: self.robot.open_gripper() (robot.py:68)
 */
class ReleaseObjectAction : public BT::StatefulActionNode
{
public:
  ReleaseObjectAction(const std::string& name,
                      const BT::NodeConfig& config,
                      BaxterHardwareInterface::Ptr hw);
  static BT::PortsList providedPorts();
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  BaxterHardwareInterface::Ptr hw_;
  ros::Time start_time_;
};

/**
 * @brief 请求视觉节点提供下一个抓取位姿。
 *
 * 对应Python: np.save(self.grasp_request, 1) (pnp.py:93)
 */
class RequestGraspPoseAction : public BT::SimpleActionNode
{
public:
  RequestGraspPoseAction(const std::string& name,
                         const BT::NodeConfig& config,
                         BaxterHardwareInterface::Ptr hw);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  BaxterHardwareInterface::Ptr hw_;
  ros::Publisher pub_;
};

/**
 * @brief 多次抓取失败后触发报警。
 *
 * Fallback的最后一个子节点 —— 持久化记录失败事件、停止机器人。
 */
class AlarmTriggerAction : public BT::SimpleActionNode
{
public:
  AlarmTriggerAction(const std::string& name,
                     const BT::NodeConfig& config,
                     BaxterHardwareInterface::Ptr hw);
  static BT::PortsList providedPorts();
  BT::NodeStatus tick() override;
private:
  BaxterHardwareInterface::Ptr hw_;
};

// ══════════════════════════════════════════════════════════════════════════
//  Factory Registration
// ══════════════════════════════════════════════════════════════════════════

/**
 * @brief 将所有BT节点注册到factory中。
 *
 * 在 pnp_main.cpp 中调用:
 *   BT::BehaviorTreeFactory factory;
 *   baxter_pnp::registerAllNodes(factory, hardware);
 */
void registerAllNodes(BT::BehaviorTreeFactory& factory,
                      BaxterHardwareInterface::Ptr hw);

}  // namespace baxter_pnp

#endif  // BAXTER_PNP_BT_NODES_H_
