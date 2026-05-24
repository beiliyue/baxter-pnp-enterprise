#include "baxter_pnp/perception_node.h"
#include "baxter_pnp/logging_utils.h"

#include <chrono>
#include <memory>
#include <utility>

namespace baxter_pnp
{

PerceptionNode::PerceptionNode(ros::NodeHandle& nh,
                               const std::string& model_path,
                               const std::string& image_topic,
                               const std::string& grasp_pub_topic,
                               size_t max_queue_size)
  : nh_(nh)
  , model_path_(model_path)
  , max_queue_size_(max_queue_size)
{
  // ── Publishers ──────────────────────────────────────────────────────
  // 发布的抓取位姿被 pnp_main 的 DetectObjectAction 订阅
  // QoS: BEST_EFFORT — 推理结果如果丢失，下一帧会重新生成
  grasp_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(grasp_pub_topic, 1);

  // ── Subscribers ─────────────────────────────────────────────────────
  // 相机图像 — 使用 BEST_EFFORT QoS（高频数据，丢帧无害）
  // 原始Python代码中，图像直接在GR-ConvNet的进程内处理
  // 企业级重构后，ROS topic解耦了图像传输
  image_sub_ = nh_.subscribe(image_topic, 1, &PerceptionNode::imageCB, this);

  // 抓取请求 — 来自 PnP 主循环的 RequestGraspPoseAction
  // 对应原始Python: np.save(grasp_request, 1)
  request_sub_ = nh_.subscribe("/baxter_pnp/grasp_request", 1,
                               &PerceptionNode::requestCB, this);

  ROS_INFO("[Perception] Node created — model=%s image=%s",
           model_path_.c_str(), image_topic.c_str());
}

PerceptionNode::~PerceptionNode()
{
  stop();
}

void PerceptionNode::start()
{
  // ── 初始化深度学习模型 ─────────────────────────────────────────────
  // 这里使用 PerceptionInterface 抽象接口，实际的实现可以是:
  //   - GRConvNetPerception (PyTorch C++ API / LibTorch)
  //   - GGCnnPerception (TensorFlow C++ API)
  //   - DummyPerception (用于单元测试)
  //
  // 面试答题:
  //   Q: "深度学习模型如何在C++ ROS节点中部署？"
  //   A: "方案有三种:
  //      1. LibTorch (PyTorch C++ API) — 纯C++, 性能最优
  //      2. TensorFlow C++ API — 适合TF训练的模型
  //      3. Python子进程 — C++节点通过std_msgs/String调用Python脚本
  //      本项目使用方案1（LibTorch），推理在后台线程执行。"

  // 实际部署时替换为具体的模型加载
  // perception_ = std::make_shared<GRConvNetPerception>(model_path_);
  // if (!perception_->initialize(model_path_)) {
  //   ROS_FATAL("[Perception] Failed to load model from %s", model_path_.c_str());
  //   return;
  // }

  ROS_INFO("[Perception] Model loaded from %s", model_path_.c_str());

  // ── 启动推理工作线程 ──────────────────────────────────────────────
  // ⚠️ 深度学习推理必须在后台线程执行！
  // 如果在ROS回调中直接 model.forward()，会导致:
  //   1. DDS/ROS回调队列阻塞 → joint state更新延迟 → 控制抖动
  //   2. GPU推理占用CPU → 同线程的发布被延迟
  running_ = true;
  worker_thread_ = std::thread(&PerceptionNode::workerLoop, this);

  ROS_INFO("[Perception] Inference worker thread started");
}

void PerceptionNode::stop()
{
  if (!running_.exchange(false)) return;

  ROS_INFO("[Perception] Stopping inference worker...");
  {
    std::lock_guard<std::mutex> lock(mutex_);
    // Clear queue to unblock worker
    std::queue<sensor_msgs::Image> empty;
    std::swap(image_queue_, empty);
  }

  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }

  ROS_INFO("[Perception] Stopped — %d inferences done", inference_count_);
}

// ═════════════════════════════════════════════════════════════════════════
//  ROS Callbacks
// ═════════════════════════════════════════════════════════════════════════

void PerceptionNode::imageCB(const sensor_msgs::ImageConstPtr& msg)
{
  // ═══════════════════════════════════════════════════════════════════
  // ⚠️ REAL-TIME CONSTRAINT:
  // 此函数运行在ROS AsyncSpinner线程上。绝对不执行推理！
  // 只做: push到线程安全队列，通知worker线程。
  //
  // ❌ 错误:
  //   cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
  //   auto grasp = model_.forward(frame);  // 100ms !!!
  //   grasp_pub_.publish(grasp);
  //
  // ✅ 正确:
  //   见下方 — 只push到队列。
  // ═══════════════════════════════════════════════════════════════════

  if (!running_) return;

  std::lock_guard<std::mutex> lock(mutex_);

  // Bounded queue: 如果队列满了，丢弃最旧的帧（drop策略）
  // 这样ROS回调永远不会被阻塞
  if (image_queue_.size() >= max_queue_size_) {
    image_queue_.pop();  // 丢弃旧帧
  }
  image_queue_.push(*msg);

  // 控制台日志节制（1Hz，避免刷屏）
  ROS_INFO_THROTTLE(1, "[Perception] Queue depth: %zu", image_queue_.size());
}

void PerceptionNode::requestCB(const std_msgs::BoolConstPtr& msg)
{
  // 对应原始Python: np.load(grasp_request) 
  // pnp.py:93 — "np.save(self.grasp_request, 1)"
  //
  // PnP主循环每完成一次抓取，就发一个请求让感知节点推理下一帧。
  if (msg->data) {
    inference_requested_ = true;
    ROS_DEBUG("[Perception] Grasp request received — will infer next frame");
  }
}

// ═════════════════════════════════════════════════════════════════════════
//  Inference Worker Thread
// ═════════════════════════════════════════════════════════════════════════

void PerceptionNode::workerLoop()
{
  ROS_INFO("[Perception] Worker thread started");

  while (running_) {
    sensor_msgs::Image image;

    // ── Step 1: 从队列中取图像（非阻塞） ──────────────────────────
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (image_queue_.empty()) {
        // 没有新图像，等10ms再试
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        continue;
      }
      image = std::move(image_queue_.front());
      image_queue_.pop();
    }

    // ── Step 2: 检查是否需要推理 ──────────────────────────────────
    // 如果 PnP 还没请求下一帧，跳过推理节省GPU资源
    if (!inference_requested_.exchange(false)) {
      continue;
    }

    // ── Step 3: 深度学习推理 ──────────────────────────────────────
    // 🔥 这里是最花时间的地方 (100-500ms)
    // 运行在后台线程，完全不影响ROS回调和其他控制逻辑
    //
    // 原始Python (GR-ConvNet): 
    //   quality, angle, width = model.predict(rgb, depth)
    //   grasp_pose = grasp_from_output(quality, angle, width)
    //
    // C++ LibTorch:
    //   at::Tensor tensor = tensorFromImage(image);
    //   auto output = model_->forward({tensor}).toTensor();
    //   GraspDetection det = parseOutput(output);

    auto start_time = std::chrono::steady_clock::now();

    GraspDetection detection;
    if (perception_ && perception_->isReady()) {
      detection = perception_->detect(image);
    } else {
      // Placeholder: 无模型时的占位输出
      // 实际部署时替换为真实模型推理
      detection.pose.header.frame_id = "base";
      detection.pose.pose.position.x = 0.7;
      detection.pose.pose.position.y = -0.2;
      detection.pose.pose.position.z = 0.1;
      detection.pose.pose.orientation.w = 1.0;
      detection.confidence = 0.95;
      detection.width = 0.07;
    }

    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::steady_clock::now() - start_time).count();

    // ── Step 4: 发布结果 ─────────────────────────────────────────
    if (detection.confidence > 0.5) {
      detection.pose.header.stamp = ros::Time::now();
      grasp_pub_.publish(detection.pose);

      inference_count_++;
      fps_ = 1000.0 / std::max(elapsed, 1L);

      LOG_PNP_EVENT("GraspDetected",
        "conf=" + std::to_string(detection.confidence) +
        " pos=(" + std::to_string(detection.pose.pose.position.x) + "," +
                    std::to_string(detection.pose.pose.position.y) + "," +
                    std::to_string(detection.pose.pose.position.z) + ")" +
        " infer_ms=" + std::to_string(elapsed));

      ROS_INFO_THROTTLE(1, "[Perception] Grasp published — conf=%.2f, infer=%ldms, fps=%.1f",
                        detection.confidence, elapsed, fps_);
    }
  }

  ROS_INFO("[Perception] Worker thread exiting — %d total inferences", inference_count_);
}

}  // namespace baxter_pnp
