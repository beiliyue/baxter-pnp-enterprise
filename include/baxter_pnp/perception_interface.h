#ifndef BAXTER_PNP_PERCEPTION_INTERFACE_H_
#define BAXTER_PNP_PERCEPTION_INTERFACE_H_

/**
 * @file perception_interface.h
 * @brief 视觉感知抽象接口 — 深度学习模型推理的封装。
 *
 * ========================================================
 * 为什么感知要和PnP控制分离？(面试高频)
 * ========================================================
 *
 * 原始Python代码中，感知是一个**独立进程**（GR-ConvNet），
 * 通过 numpy 文件 IPC 与 PnP 通信：
 *
 *   [GR-ConvNet 进程]           [PnP 进程]
 *       │                          │
 *       │ 推理出抓取位姿            │
 *       │                          │
 *       ├── write grasp_pose.npy ──┤
 *       ├── write grasp_avail=1 ───┤
 *       │                          ├── read grasp_pose.npy
 *       │                          ├── read grasp_avail==1?
 *       │                          │    → 执行抓取
 *       │                          ├── write grasp_request=0
 *       │                          │
 *       │    ←── read grasp_request ──┤
 *       │          → 推理下一帧       │
 *
 * 企业级重构后，改为 ROS Topic 通信：
 *
 *   [/baxter_pnp/perception 节点]   [/baxter_pnp_main 节点]
 *       │                          │
 *       │  推理出抓取位姿            │
 *       │                          │
 *       ├── pub /grasp_pose ───────┤
 *       │     (PoseStamped)        ├── DetectObject::onRunning()
 *       │                          │    → pose_received_ = true
 *       │                          │
 *       │    ←─ sub /grasp_request ──┤
 *       │     (Bool)                │
 *       │  收到请求→推理下一帧      │
 *
 * 进程隔离的好处:
 *   1. 深度学习模型崩溃 → PnP控制不受影响（Baxter停在安全位置）
 *   2. 可以使用不同的编程语言（Python做推理，C++做控制）
 *   3. 可以独立升级/替换感知算法
 *
 * 防阻塞设计:
 *   ⚠️ 深度学习推理很慢（100-500ms），必须在**后台线程**执行。
 *   ❌ 绝不能在ROS回调中直接调用 model.forward()！
 *   ✅ 图像回调只做 push to queue，推理在 std::thread 中执行。
 */

#include <memory>
#include <functional>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>

namespace baxter_pnp
{

/**
 * @brief 抓取检测结果。
 */
struct GraspDetection
{
  geometry_msgs::PoseStamped pose;   ///< 抓取位姿（base坐标系）
  double confidence{0.0};            ///< 置信度 [0, 1]
  double width{0.0};                 ///< 夹爪开口宽度 (m)
  std::string error_msg;
};

/**
 * @class PerceptionInterface
 * @brief 视觉感知的抽象接口。
 *
 * 实现可以是:
 *   - GR-ConvNet (PyTorch)
 *   - GG-CNN (TensorFlow)
 *   - 任何输出抓取位姿的深度学习模型
 *
 * 抽象化的目的:
 *   面试中可回答"通过依赖倒置，感知算法可插拔替换"
 */
class PerceptionInterface
{
public:
  using Ptr = std::shared_ptr<PerceptionInterface>;

  virtual ~PerceptionInterface() = default;

  /**
   * @brief 初始化深度学习模型。
   * @param model_path 模型权重文件路径
   * @return true if 加载成功
   */
  virtual bool initialize(const std::string& model_path) = 0;

  /**
   * @brief 执行推理（可能在后台线程调用）。
   *
   * @param image   输入相机图像
   * @return GraspDetection 检测到的抓取位姿
   *
   * ⚠️ 此函数可能耗时 100-500ms，绝不要在ROS回调中直接调用！
   */
  virtual GraspDetection detect(const sensor_msgs::Image& image) = 0;

  /**
   * @brief 返回模型是否已加载。
   */
  virtual bool isReady() const = 0;

  /**
   * @brief 获取支持的输入图像尺寸。
   */
  virtual void getInputSize(int& width, int& height) const = 0;
};

}  // namespace baxter_pnp

#endif  // BAXTER_PNP_PERCEPTION_INTERFACE_H_
