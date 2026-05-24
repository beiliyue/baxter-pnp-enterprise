#ifndef BAXTER_PNP_LOGGING_UTILS_H_
#define BAXTER_PNP_LOGGING_UTILS_H_

/**
 * @file logging_utils.h
 * @brief Enterprise-grade logging, event markers, and diagnostics.
 *
 * ========== ROS1 日志最佳实践 (面试常考) ==========
 *
 * 1. 日志级别:
 *    DEBUG   → 高频调试（关节角度），默认关闭
 *    INFO    → 正常事件（运动完成、检测到物体）
 *    WARN    → 可恢复异常（力阈值触发、重试第2次）
 *    ERROR   → 不可恢复但系统可继续（IK求解失败）
 *    FATAL   → 系统不可继续（机器人断连）
 *
 * 2. 高频率日志节制:
 *    关节状态 100Hz → RCLCPP_INFO_THROTTLE (ROS2) 或
 *                      ROS_INFO_THROTTLE(ros::Rate) (ROS1)
 *    ⚠️ 不要在100Hz循环里 ROS_INFO —— 会淹没控制台并拖慢executor
 *
 * 3. 结构化事件标记:
 *    用 "[PNP_EVENT]" 前缀标记关键里程碑，方便 grepping rosbag:
 *      rosbag play bag.bag | grep PNP_EVENT
 *
 * 4. rosbag 脱机分析:
 *    企业实践中，所有baxter数据通过 rosbag record 持续录制。
 *    故障发生后:
 *      rosbag info fault.bag
 *      rosbag play fault.bag /rosout:=/rosout_replay
 *      grep "PNP_EVENT" rosout_replay.log
 *      grep "GraspFailure" rosout_replay.log
 */

#include <string>
#include <sstream>
#include <iomanip>
#include <chrono>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

namespace baxter_pnp
{

/**
 * @brief 格式化并打印末端执行器位姿。
 *
 * 输出格式:
 *   [EE_POSE] [right] t=2.500 pos:[0.532,-0.284,0.012] ori:[0.707,0.000,0.707,0.000]
 *
 * 对应Python: current_pose = self._limb.endpoint_pose() (robot.py:81)
 *             print(current_pose)  // 原始代码
 */
inline void logEePose(const std::string& arm,
                      const geometry_msgs::PoseStamped& pose,
                      const std::string& level = "info")
{
  std::stringstream ss;
  ss << std::fixed << std::setprecision(3);
  ss << "[EE_POSE] [" << arm << "] "
     << "t:" << pose.header.stamp.toSec() << " "
     << "pos:[" << pose.pose.position.x << ","
                << pose.pose.position.y << ","
                << pose.pose.position.z << "] "
     << "ori:[" << pose.pose.orientation.x << ","
                << pose.pose.orientation.y << ","
                << pose.pose.orientation.z << ","
                << pose.pose.orientation.w << "]";

  if (level == "warn") ROS_WARN("%s", ss.str().c_str());
  else if (level == "error") ROS_ERROR("%s", ss.str().c_str());
  else ROS_INFO("%s", ss.str().c_str());
}

/**
 * @brief PnP关键事件标记。
 *
 * 每个PnP里程碑（GraspSuccess、PickComplete等）都应该用此宏记录。
 * 格式: [PNP_EVENT] [timestamp] EventName — detail
 *
 * 用法:
 *   LOG_PNP_EVENT("GraspSuccess", "object_id=003 force_z=-12.5");
 */
inline void logPnpEvent(const std::string& event,
                        const std::string& detail = "")
{
  auto now = std::chrono::system_clock::now();
  auto now_c = std::chrono::system_clock::to_time_t(now);
  auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    now.time_since_epoch()) % 1000000000;

  std::stringstream ss;
  ss << "[PNP_EVENT] ["
     << std::put_time(std::localtime(&now_c), "%Y-%m-%d %H:%M:%S")
     << "." << std::setw(9) << std::setfill('0') << now_ns.count() << "] "
     << event;
  if (!detail.empty()) ss << " — " << detail;

  ROS_INFO("%s", ss.str().c_str());
}

#define LOG_PNP_EVENT(event, detail) \
  baxter_pnp::logPnpEvent(event, detail)

/**
 * @brief 安全位姿差比较。
 *
 * 用于判断机器人是否到达目标位置。由于Baxter的精度约为±5mm，
 * 阈值为0.01m。
 */
inline bool poseApproxEqual(const geometry_msgs::Pose& a,
                            const geometry_msgs::Pose& b,
                            double eps = 0.01)
{
  return (std::abs(a.position.x - b.position.x) < eps &&
          std::abs(a.position.y - b.position.y) < eps &&
          std::abs(a.position.z - b.position.z) < eps);
}

}  // namespace baxter_pnp

#endif  // BAXTER_PNP_LOGGING_UTILS_H_
