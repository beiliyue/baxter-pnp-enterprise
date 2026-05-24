#ifndef BAXTER_PNP_PERCEPTION_NODE_H_
#define BAXTER_PNP_PERCEPTION_NODE_H_

/**
 * @file perception_node.h
 * @brief ROS1感知节点 — 深度学习模型推理 + 抓取位姿发布。
 *
 * ========================================================
 * 架构设计 (面试重点)
 * ========================================================
 *
 * 本节点是一个**独立ROS进程**，与 pnp_main 进程隔离。
 *
 * ┌──────────────────────────────────────────────────────┐
 * │  perception_node (独立进程)                          │
 * │                                                      │
 * │  Camera feed ──► [BEST_EFFORT QoS]                  │
 * │       │                                              │
 * │       ▼                                              │
 * │  Image Queue (线程安全, bounded)                     │
 * │       │                                              │
 * │       ▼                                              │
 * │  std::thread: Inference Worker                       │
 * │  ┌─────────────────────────────────────────────────┐ │
 * │  │ while(running) {                                 │ │
 * │  │   img = queue_.pop();           ← 不阻塞ROS回调  │ │
 * │  │   result = model_.forward(img); ← 100-500ms      │ │
 * │  │   pub_.publish(result);         ← 发布抓取位姿   │ │
 * │  │ }                                                 │ │
 * │  └─────────────────────────────────────────────────┘ │
 * │       │                                              │
 * │       ▼                                              │
 * │  /baxter_pnp/grasp_pose  (PoseStamped)              │
 * └──────────────────────┬───────────────────────────────┘
 *                        │ topic
 * ┌──────────────────────▼───────────────────────────────┐
 * │  pnp_main (独立进程)                                  │
 * │  DetectObjectAction::onRunning()                     │
 * │    → 收到 grasp_pose → 写入 Blackboard               │
 * │    → BT tick 继续执行 → PlanPath → Grasp → ...       │
 * └──────────────────────────────────────────────────────┘
 *
 * 为什么要独立进程而不是线程？
 *   面试高频题:
 *   Q: "ROS1中为什么感知要单独一个node？"
 *   A: "因为深度学习模型可能crash（OOM、段错误等），
 *       独立进程崩溃不会带走运动控制节点，Baxter停在安全位置。
 *       如果用nodelet可以零拷贝传输图像，但牺牲隔离性。"
 */

#include <atomic>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>

#include "perception_interface.h"

namespace baxter_pnp
{

/**
 * @class PerceptionNode
 * @brief ROS1感知节点，负责深度学习推理和抓取位姿发布。
 *
 * 线程模型:
 *   Thread 1 (ROS AsyncSpinner): 相机图像回调 → push to queue
 *   Thread 2 (Worker):           pop from queue → 推理 → 发布结果
 *   Thread 3+ (ROS):             grasp_request 订阅回调
 */
class PerceptionNode
{
public:
  /**
   * @param nh             ROS NodeHandle
   * @param model_path     深度学习模型权重路径
   * @param image_topic    输入相机topic (/camera/rgb/image_raw)
   * @param grasp_pub_topic 输出抓取位姿topic (/baxter_pnp/grasp_pose)
   * @param max_queue_size  图像队列最大长度（防内存溢出）
   */
  PerceptionNode(ros::NodeHandle& nh,
                 const std::string& model_path,
                 const std::string& image_topic = "/camera/rgb/image_raw",
                 const std::string& grasp_pub_topic = "/baxter_pnp/grasp_pose",
                 size_t max_queue_size = 5);

  ~PerceptionNode();

  /// 启动推理工作线程
  void start();

  /// 停止推理线程
  void stop();

  /// 获取当前推理FPS
  double getFPS() const { return fps_; }

private:
  // ── ROS Callbacks ────────────────────────────────────────────────────

  /**
   * @brief 相机图像回调。
   *
   * ⚠️ 运行在ROS AsyncSpinner线程上。
   * ⚠️ 不做任何图像处理！只push到线程安全队列。
   * ⚠️ 如果队列满，丢弃旧帧（drop策略）而不是阻塞。
   *
   * @see cameraImageQoS() — 使用 BEST_EFFORT 策略
   */
  void imageCB(const sensor_msgs::ImageConstPtr& msg);

  /**
   * @brief 抓取请求回调（来自PnP主节点）。
   */
  void requestCB(const std_msgs::BoolConstPtr& msg);

  // ── Background Worker ────────────────────────────────────────────────

  /// 推理工作线程主循环
  void workerLoop();

  // ── Members ───────────────────────────────────────────────────────────

  ros::NodeHandle nh_;

  // ROS接口
  ros::Subscriber image_sub_;
  ros::Subscriber request_sub_;
  ros::Publisher  grasp_pub_;

  // 深度学习模型接口（可插拔）
  PerceptionInterface::Ptr perception_;

  // 线程安全图像队列
  mutable std::mutex mutex_;
  std::queue<sensor_msgs::Image> image_queue_;
  size_t max_queue_size_;

  // 工作线程控制
  std::thread worker_thread_;
  std::atomic<bool> running_{false};
  std::atomic<double> fps_{0.0};

  // 请求触发
  std::atomic<bool> inference_requested_{true}; // 默认持续推理

  // 参数
  std::string model_path_;
  int inference_count_{0};
};

}  // namespace baxter_pnp

#endif  // BAXTER_PNP_PERCEPTION_NODE_H_
