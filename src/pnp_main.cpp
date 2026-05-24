/**
 * @file pnp_main.cpp
 * @brief Enterprise-grade entry point for Baxter PnP with BehaviorTree.CPP.
 *
 * ========================================================
 * 企业级主程序架构 (面试高频)
 * ========================================================
 *
 * 执行模型:
 * ```
 * main()
 * ├── ros::init()
 * ├── ros::AsyncSpinner(4)    ← 4个线程处理ROS回调
 * │   ├── Thread 1: joint state / endpoint state callbacks
 * │   ├── Thread 2: camera image callbacks
 * │   ├── Thread 3: actionlib / service callbacks
 * │   └── Thread 4: diagnostic updater
 * ├── Hardware init (BaxterRobotLifecycle)
 * ├── BT factory + tree creation
 * ├── Main loop (100Hz BT tick)
 * │   ├── tree.tickOnce()
 * │   ├── check result (SUCCESS/FAILURE/RUNNING)
 * │   └── ros::Duration(0.01).sleep()
 * └── Clean shutdown
 * ```
 *
 * ========================================================
 * 🧵 多线程面试问题 - Zombie Process 预防
 * ========================================================
 *
 * Q: "ROS1多进程node中如何避免僵尸进程？"
 * A: "僵尸进程产生于子进程终止但父进程未调用waitpid()。
 *     ROS1 nodelet 是最优方案（同进程零拷贝，无fork）。
 *     如果必须使用多进程：
 *     1. 注册 SIGCHLD 信号处理器，在循环中 waitpid(-1, &status, WNOHANG)
 *     2. 使用 double fork 技术（子进程再fork孙进程后立即退出，孙进程被init收养）
 *     3. 在析构函数中确保所有子进程已回收
 *     本项目使用 AsyncSpinner（多线程）而非多进程，无僵尸风险。"
 *
 * ========================================================
 * 🔒 死锁预防 (Deadlock Prevention)
 * ========================================================
 *
 * Q: "多个std::mutex如何避免死锁？"
 * A: "严格遵循固定顺序加锁 (Lock Ordering)：
 *     所有代码先锁 mutex_A 再锁 mutex_B，永不反向。
 *     使用 std::lock(mutex_A, mutex_B) 死锁安全版。
 *     本项目所有共享状态统一由 robot_lifecycle.cpp 的 mutex_ 保护，
 *     BT节点不单独持有互斥量，从根源上避免死锁。"
 */

#include <memory>
#include <string>
#include <csignal>

#include <ros/ros.h>
#include <ros/console.h>

#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include <behaviortree_cpp/loggers/bt_file_logger.h>

#include "baxter_pnp/hardware_interface.h"
#include "baxter_pnp/robot_lifecycle.h"
#include "baxter_pnp/bt_nodes.h"
#include "baxter_pnp/logging_utils.h"

// Global pointer for signal handler
std::shared_ptr<baxter_pnp::BaxterRobotLifecycle> g_hardware = nullptr;

/**
 * @brief 信号处理 —— 确保Ctrl+C时安全关闭机器人。
 *
 * 企业级实践:
 *   SIGINT 处理是关键。如果在收到Ctrl+C时没有禁用机器人，
 *   电机保持使能，手臂可能因重力下垂伤人。
 *
 * ⚠️ 不要在信号处理器中做复杂操作！只设置标志，主循环检查。
 */
void signalHandler(int sig)
{
  ROS_WARN("[Main] Signal %d received — shutting down", sig);
  if (g_hardware) {
    g_hardware->shutdown();
  }
  ros::shutdown();
}

int main(int argc, char** argv)
{
  // ── ROS init ──────────────────────────────────────────────────────────
  ros::init(argc, argv, "baxter_pnp_main", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  // Set up signal handling (overrides ROS's default SIGINT handler)
  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);

  // ── Configuration ─────────────────────────────────────────────────────
  std::string limb = nh_private.param<std::string>("limb", "right");
  std::string bt_xml_path = nh_private.param<std::string>("bt_xml", "");
  double force_threshold = nh_private.param<double>("force_threshold", -8.0);
  double hover_distance = nh_private.param<double>("hover_distance", 0.12);
  double step_size = nh_private.param<double>("step_size", 0.05);

  // Place position (default: [0.7, -0.5, -0.2])
  geometry_msgs::PoseStamped place_pose;
  place_pose.header.frame_id = "base";
  place_pose.pose.position.x = nh_private.param<double>("place_x", 0.7);
  place_pose.pose.position.y = nh_private.param<double>("place_y", -0.5);
  place_pose.pose.position.z = nh_private.param<double>("place_z", -0.2);
  place_pose.pose.orientation.w = 1.0;

  // ── AsyncSpinner (Multi-Threaded Executor) ────────────────────────────
  // 使用4个线程处理ROS回调，保证joint state、camera、actionlib互不影响。
  // 对比单线程 ros::spin() → 一个慢回调阻塞所有其他回调。
  // ⚠️ 不要在主循环中调用 ros::spinOnce() —— AsyncSpinner 自动处理。
  int num_spinner_threads = nh_private.param<int>("spinner_threads", 4);
  ros::AsyncSpinner spinner(num_spinner_threads);
  spinner.start();

  ROS_INFO("[Main] AsyncSpinner started with %d threads", num_spinner_threads);

  // ── Hardware Initialization ──────────────────────────────────────────
  auto hardware = std::make_shared<baxter_pnp::BaxterRobotLifecycle>(
    nh, limb, true);
  g_hardware = hardware;

  if (!hardware->initialize()) {
    ROS_FATAL("[Main] Failed to initialize Baxter hardware — exiting");
    return 1;
  }

  // Move to home
  hardware->moveToHome();
  hardware->openGripper();

  // ── BehaviorTree Setup ──────────────────────────────────────────────
  BT::BehaviorTreeFactory factory;

  // Register the ROS node handle on blackboard for BT nodes that need it
  // (e.g., DetectObjectAction needs to subscribe)
  BT::NodeConfiguration config;
  config.blackboard = BT::Blackboard::create();
  config.blackboard->set("ros_nh", &nh);

  // Register all customized BT nodes
  baxter_pnp::registerAllNodes(factory, hardware);

  // Find BT XML file
  if (bt_xml_path.empty()) {
    bt_xml_path = ros::package::getPath("baxter_pnp_enterprise") + "/bt_xml/baxter_pnp_bt.xml";
  }

  ROS_INFO("[Main] Loading BT from: %s", bt_xml_path.c_str());

  BT::Tree tree;
  try {
    tree = factory.createTreeFromFile(bt_xml_path);
  } catch (const std::exception& e) {
    ROS_FATAL("[Main] Failed to load BT XML: %s", e.what());
    hardware->shutdown();
    return 1;
  }

  // Set blackboard values
  tree.rootBlackboard()->set("place_position", place_pose);
  tree.rootBlackboard()->set("force_threshold", force_threshold);
  tree.rootBlackboard()->set("hover_distance", hover_distance);
  tree.rootBlackboard()->set("step_size", step_size);
  tree.rootBlackboard()->set("ros_nh", &nh);

  // BT loggers
  BT::StdCoutLogger cout_logger(tree);
  BT::FileLogger file_logger(tree, "bt_trace.fbl");
  BT::PublisherZMQ publisher_zmq(tree);  // For Groot2 visualization

  // ── Main Control Loop ──────────────────────────────────────────────
  // 100Hz tick rate matches Baxter's joint state update frequency.
  // Each tick processes one step of the BehaviorTree.
  // StatefulActionNodes return RUNNING while in progress.

  ros::Rate rate(100);  // 100 Hz
  int cycle_count = 0;

  LOG_PNP_EVENT("SystemStarted",
    "limb=" + limb + " spinner_threads=" + std::to_string(num_spinner_threads));

  ROS_INFO("[Main] Starting PnP control loop at 100Hz");

  while (ros::ok()) {
    // Tick the BehaviorTree
    BT::NodeStatus status = tree.tickOnce();

    // Handle tree status
    if (status == BT::NodeStatus::IDLE) {
      // One full PnP cycle completed
      cycle_count++;
      LOG_PNP_EVENT("PnPCycleComplete",
        "cycle=" + std::to_string(cycle_count));

      // Reset blackboard for next cycle (keep persistent values)
      tree.rootBlackboard()->clearAll();
      tree.rootBlackboard()->set("place_position", place_pose);
      tree.rootBlackboard()->set("force_threshold", force_threshold);
      tree.rootBlackboard()->set("hover_distance", hover_distance);
      tree.rootBlackboard()->set("step_size", step_size);
      tree.rootBlackboard()->set("ros_nh", &nh);

      ROS_INFO("[Main] Cycle %d complete — starting next", cycle_count);

    } else if (status == BT::NodeStatus::FAILURE) {
      ROS_FATAL("[Main] Behavior Tree returned FAILURE — shutting down");
      LOG_PNP_EVENT("PnPAborted", "tree returned FAILURE");
      break;
    }

    // Sleep to maintain 100Hz rate
    rate.sleep();
  }

  // ── Graceful Shutdown ──────────────────────────────────────────────
  ROS_INFO("[Main] Shutting down...");
  spinner.stop();
  hardware->shutdown();
  g_hardware = nullptr;

  ROS_INFO("[Main] Baxter PnP finished — %d cycles completed", cycle_count);
  return 0;
}
