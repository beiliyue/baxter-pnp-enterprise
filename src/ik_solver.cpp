#include "baxter_pnp/ik_solver.h"

#include <memory>
#include <utility>

namespace baxter_pnp
{

IKSolver::IKSolver(ros::NodeHandle& nh, const std::string& ik_service_name)
  : nh_(nh)
{
  // Wait for IK service to become available
  ik_client_ = nh_.serviceClient<baxter_core_msgs::SolvePositionIK>(ik_service_name);
  ROS_INFO("[IKSolver] Waiting for IK service: %s", ik_service_name.c_str());
  if (!ik_client_.waitForExistence(ros::Duration(10.0))) {
    ROS_WARN("[IKSolver] IK service not available after 10s — will retry on each call");
  } else {
    ROS_INFO("[IKSolver] IK service connected");
  }
}

IKSolver::~IKSolver()
{
  cancelAll();
}

void IKSolver::requestIK(const geometry_msgs::PoseStamped& pose,
                         std::function<void(const IKResult&)> callback)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    queue_.push({pose, std::move(callback)});
    pending_++;
  }
  cv_.notify_one();

  // Start worker thread lazily
  if (!running_.exchange(true)) {
    worker_thread_ = std::thread(&IKSolver::workerLoop, this);
  }
}

void IKSolver::cancelAll()
{
  running_ = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::queue<IKJob> empty;
    std::swap(queue_, empty);
    cv_.notify_all();
  }
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

void IKSolver::workerLoop()
{
  ROS_DEBUG("[IKSolver] Worker thread started");

  while (running_) {
    IKJob job;

    {
      std::unique_lock<std::mutex> lock(mutex_);
      cv_.wait_for(lock, std::chrono::milliseconds(100),
                   [this] { return !queue_.empty() || !running_; });

      if (!running_) break;
      if (queue_.empty()) continue;

      job = std::move(queue_.front());
      queue_.pop();
    }

    // Call IK service
    baxter_core_msgs::SolvePositionIK srv;
    srv.request.pose_stamp.push_back(job.pose);

    IKResult result;
    if (ik_client_.call(srv)) {
      result.success = srv.response.isValid[0];
      if (result.success) {
        result.joints = srv.response.joints[0].position;
        result.solution_cost = srv.response.residual[0];
      } else {
        result.error_msg = "IK returned invalid";
      }
    } else {
      result.error_msg = "Service call failed";
    }

    pending_--;

    // Deliver result via callback
    if (job.callback) {
      job.callback(result);
    }
  }

  ROS_DEBUG("[IKSolver] Worker thread exiting");
}

}  // namespace baxter_pnp
