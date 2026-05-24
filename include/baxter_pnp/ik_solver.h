#ifndef BAXTER_PNP_IK_SOLVER_H_
#define BAXTER_PNP_IK_SOLVER_H_

/**
 * @file ik_solver.h
 * @brief Hybrid IK solver — synchronous service call + async callback.
 *
 * ========== 为什么需要IK线程？(面试高频) ==========
 *
 * Baxter的IK服务 (SolvePositionIK) 每次调用耗时 10-100ms。
 * 如果在 BT tick() 里同步等待10ms:
 *   - 100Hz的BT循环降级为 100Hz/(1+100×0.01) ≈ 50Hz
 *   - DDS/ROS回调队列被阻塞 → joint state更新延迟 → 控制不稳定
 *
 * 解决方案:
 *   - requestIK() 立即返回，IK计算在后台执行
 *   - 完成后通过 std::function 回调通知 BT 节点
 *   - BT 节点在 onRunning() 中检查结果是否就绪
 *
 * 多进程/多线程僵尸问题:
 *   IK求解放在同一进程的工作线程中，不fork子进程。
 *   如果使用 ros::service::call() 同步调用，ROS客户端库
 *   内部会阻塞当前线程但不会产生僵尸进程。
 *
 * 死锁预防:
 *   回调函数中永远不持有 mutex_ 再获取其他锁。
 *   严格遵循"固定顺序加锁"原则 (Lock Ordering):
 *     1. IK callback acquires IK mutex
 *     2. Then optionally acquires HW mutex
 *   违反顺序会导致死锁 (Deadlock)。
 */

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

#include <ros/ros.h>
#include <baxter_core_msgs/SolvePositionIK.h>

#include "hardware_interface.h"

namespace baxter_pnp
{

/**
 * @class IKSolver
 * @brief Thread-safe IK solver with asynchronous callback delivery.
 *
 * Enterprise Pattern: Thread Pool + Callback Queue
 *
 * Architecture:
 * ```
 * BT Node: requestIK(pose, cb) ──→ queue_ ──→ worker thread
 *                                              ↓
 * Worker:  call IK service ←─── ros::ServiceClient
 *                                              ↓
 *          result ready ──→ callback queue ──→ executed on ROS callback thread
 *                                              ↓
 * BT Node: onRunning() checks result flag  ←── callback sets flag
 * ```
 *
 * 内存模型 (Memory Order):
 *   - std::atomic<bool> pending_ 使用 memory_order_release/acquire
 *     保证回调中的数据在线程间正确同步
 */
class IKSolver
{
public:
  explicit IKSolver(ros::NodeHandle& nh, const std::string& ik_service_name);
  ~IKSolver();

  /// Submit IK request. callback is invoked on a background thread.
  void requestIK(const geometry_msgs::PoseStamped& pose,
                 std::function<void(const IKResult&)> callback);

  /// Cancel all pending IK requests
  void cancelAll();

  /// Get number of pending IK jobs
  size_t pendingCount() const { return pending_; }

private:
  /// Worker thread entry point
  void workerLoop();

  /// Struct for queued IK jobs
  struct IKJob
  {
    geometry_msgs::PoseStamped pose;
    std::function<void(const IKResult&)> callback;
  };

  ros::NodeHandle nh_;
  ros::ServiceClient ik_client_;

  std::thread worker_thread_;
  std::atomic<bool> running_{false};

  // Queue guarded by mutex
  mutable std::mutex mutex_;
  std::queue<IKJob> queue_;
  std::condition_variable cv_;

  // Atomic counter for pending jobs
  std::atomic<size_t> pending_{0};
};

}  // namespace baxter_pnp

#endif  // BAXTER_PNP_IK_SOLVER_H_
