/**
 * @file camera_nodelet.cpp
 * @brief ROS1 Nodelet 包装 — 零拷贝相机图像传输。
 *
 * ========================================================
 * 为什么需要 Nodelet？(面试高频)
 * ========================================================
 *
 * 默认ROS节点间传输图像，需要序列化/反序列化（拷贝数据）。
 * 对于 640x480x3 的彩色图像，每帧拷贝约 900KB。
 * 在30FPS下，每秒拷贝 27MB 数据。
 *
 * Nodelet 将多个节点加载到同一个进程中，共享地址空间，
 * 图像数据通过指针传递，零拷贝。
 *
 * 进程隔离 vs 零拷贝 的权衡:
 *
 *                │ 进程隔离            │ 零拷贝
 *   ─────────────┼────────────────────┼────────────────────
 *   崩溃安全      │ ✅ 独立进程不互扰   │ ❌ 一个nodelet崩全崩
 *   传输速度      │ ❌ 序列化拷贝       │ ✅ 指针传递
 *   调试难度      │ ✅ 独立gdb          │ ❌ 混合日志
 *   本项目选择    │ perception独立进程  │ camera使用nodelet
 *                 │ pnp_main独立进程    │ 在pnp_main进程内
 *
 *  折中方案:
 *   - perception_node 使用独立进程（模型OOM不波及控制）
 *   - camera_nodelet 加载到 pnp_main 所在nodelet manager
 *   - camera数据零拷贝到pnp_main，但perception仍需序列化接收
 */

#include <memory>

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

namespace baxter_pnp
{

/**
 * @class CameraNodelet
 * @brief 零拷贝相机图像转发 nodelet。
 *
 * 从 Baxter 相机驱动接收图像，重发布到标准化 topic。
 * 使用 nodelet 机制避免图像数据的序列化拷贝。
 */
class CameraNodelet : public nodelet::Nodelet
{
public:
  CameraNodelet() = default;
  ~CameraNodelet() override = default;

private:
  void onInit() override
  {
    ros::NodeHandle& nh = getNodeHandle();
    ros::NodeHandle& nh_private = getPrivateNodeHandle();

    // 输入topic（Baxter相机驱动）
    std::string input_topic = nh_private.param<std::string>("input_topic",
      "/camera/rgb/image_raw");
    // 输出topic（标准化后）
    std::string output_topic = nh_private.param<std::string>("output_topic",
      "/baxter_pnp/camera/image");

    // 发布ZeroMQ传输的图像（nodelet间零拷贝）
    pub_ = nh.advertise<sensor_msgs::Image>(output_topic, 1);
    sub_ = nh.subscribe(input_topic, 1, &CameraNodelet::imageCB, this);

    NODELET_INFO("[CameraNodelet] Started: %s → %s (zero-copy)",
                 input_topic.c_str(), output_topic.c_str());
  }

  void imageCB(const sensor_msgs::ImageConstPtr& msg)
  {
    // nodelet中，pub和sub在同一进程，共享指针传递，无拷贝
    pub_.publish(msg);
  }

  ros::Publisher pub_;
  ros::Subscriber sub_;
};

}  // namespace baxter_pnp

// Register nodelet plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(baxter_pnp::CameraNodelet, nodelet::Nodelet)
