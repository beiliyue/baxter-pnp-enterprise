# Baxter PnP Enterprise

**ROS1 (Noetic) + BehaviorTree.CPP v4** — 企业级 Baxter 机器人抓取放置系统。

## 三层架构

```
Decision Layer  (BT XML: 流程编排)
Skill Layer     (C++ BT Nodes: ApproachPose, GraspObject, …)
Hardware Layer  (BaxterRobotLifecycle: ROS1 Topics/Services)
```

## 快速开始

```bash
cd ~/catkin_ws/src
git clone https://github.com/beiliyue/baxter-pnp-enterprise.git
cd ..
catkin_make
source devel/setup.bash

# 启动（真机）
roslaunch baxter_pnp_enterprise baxter_pnp.launch

# 启动（模拟）
roslaunch baxter_pnp_enterprise baxter_pnp.launch sim:=true record:=true
```

## 企业级特性

- `ros::AsyncSpinner(4)` 多线程回调处理
- `std::mutex` + `std::lock_guard` 线程安全
- `diagnostic_updater` 硬件健康监控
- `dynamic_reconfigure` 运行时调参
- `rosbag record` 故障脱机分析
- BehaviorTree.CPP Fallback + Retry×3 异常处理

## 面试准备

参考 `docs/interview_qa.md` 中的17道高频面试题。
