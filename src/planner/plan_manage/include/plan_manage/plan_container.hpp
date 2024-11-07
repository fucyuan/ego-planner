#ifndef _PLAN_CONTAINER_H_
#define _PLAN_CONTAINER_H_

#include <Eigen/Eigen>
#include <vector>
#include <ros/ros.h>

#include <bspline_opt/uniform_bspline.h>
#include <traj_utils/polynomial_traj.h>

using std::vector;

namespace ego_planner
{

  // 全局轨迹数据类同时包含局部数据
  class GlobalTrajData
  {
  private:
  public:
    PolynomialTraj global_traj_; // 全局轨迹
    vector<UniformBspline> local_traj_; // 局部轨迹（包含位置、速度、加速度）

    double global_duration_; // 全局轨迹的持续时间
    ros::Time global_start_time_; // 全局轨迹的起始时间
    double local_start_time_, local_end_time_; // 局部轨迹的起止时间
    double time_increase_; // 时间增长量
    double last_time_inc_; // 上一次时间增长量
    double last_progress_time_; // 上一次的进度时间

    // 构造函数
    GlobalTrajData(/* args */) {}

    // 析构函数
    ~GlobalTrajData() {}

    // 判断局部轨迹是否到达目标点
    bool localTrajReachTarget() { return fabs(local_end_time_ - global_duration_) < 0.1; }

    // 设置全局轨迹
    void setGlobalTraj(const PolynomialTraj &traj, const ros::Time &time)
    {
      global_traj_ = traj;
      global_traj_.init();
      global_duration_ = global_traj_.getTimeSum(); // 获取全局轨迹的总时间
      global_start_time_ = time;

      local_traj_.clear(); // 清空局部轨迹
      local_start_time_ = -1;
      local_end_time_ = -1;
      time_increase_ = 0.0;
      last_time_inc_ = 0.0;
      last_progress_time_ = 0.0;
    }

    // 设置局部轨迹
    void setLocalTraj(UniformBspline traj, double local_ts, double local_te, double time_inc)
    {
      local_traj_.resize(3); // 局部轨迹包含位置、速度和加速度三个维度
      local_traj_[0] = traj; // 位置轨迹
      local_traj_[1] = local_traj_[0].getDerivative(); // 速度轨迹（位置轨迹的一阶导）
      local_traj_[2] = local_traj_[1].getDerivative(); // 加速度轨迹（速度轨迹的一阶导）

      local_start_time_ = local_ts; // 局部轨迹的开始时间
      local_end_time_ = local_te; // 局部轨迹的结束时间
      global_duration_ += time_inc; // 更新全局持续时间
      time_increase_ += time_inc; // 更新时间增长量
      last_time_inc_ = time_inc; // 更新上一次时间增长量
    }

    // 获取指定时间的位置信息
    Eigen::Vector3d getPosition(double t)
    {
      if (t >= -1e-3 && t <= local_start_time_)
      {
        return global_traj_.evaluate(t - time_increase_ + last_time_inc_); // 全局轨迹前段
      }
      else if (t >= local_end_time_ && t <= global_duration_ + 1e-3)
      {
        return global_traj_.evaluate(t - time_increase_); // 全局轨迹后段
      }
      else
      {
        double tm, tmp;
        local_traj_[0].getTimeSpan(tm, tmp);
        return local_traj_[0].evaluateDeBoor(tm + t - local_start_time_); // 局部轨迹
      }
    }

    // 获取指定时间的速度信息
    Eigen::Vector3d getVelocity(double t)
    {
      if (t >= -1e-3 && t <= local_start_time_)
      {
        return global_traj_.evaluateVel(t); // 全局轨迹前段
      }
      else if (t >= local_end_time_ && t <= global_duration_ + 1e-3)
      {
        return global_traj_.evaluateVel(t - time_increase_); // 全局轨迹后段
      }
      else
      {
        double tm, tmp;
        local_traj_[0].getTimeSpan(tm, tmp);
        return local_traj_[1].evaluateDeBoor(tm + t - local_start_time_); // 局部轨迹
      }
    }

    // 获取指定时间的加速度信息
    Eigen::Vector3d getAcceleration(double t)
    {
      if (t >= -1e-3 && t <= local_start_time_)
      {
        return global_traj_.evaluateAcc(t); // 全局轨迹前段
      }
      else if (t >= local_end_time_ && t <= global_duration_ + 1e-3)
      {
        return global_traj_.evaluateAcc(t - time_increase_); // 全局轨迹后段
      }
      else
      {
        double tm, tmp;
        local_traj_[0].getTimeSpan(tm, tmp);
        return local_traj_[2].evaluateDeBoor(tm + t - local_start_time_); // 局部轨迹
      }
    }

    // 获取在指定半径内的局部轨迹的B样条参数化数据
    // start_t: 轨迹的起始时间
    // des_radius: 期望的半径
    // dist_pt: 离散点之间的距离
    void getTrajByRadius(const double &start_t, const double &des_radius, const double &dist_pt,
                         vector<Eigen::Vector3d> &point_set, vector<Eigen::Vector3d> &start_end_derivative,
                         double &dt, double &seg_duration)
    {
      double seg_length = 0.0; // 截断段的长度
      double seg_time = 0.0;   // 截断段的持续时间
      double radius = 0.0;     // 当前点到段起点的距离

      double delt = 0.2;
      Eigen::Vector3d first_pt = getPosition(start_t); // 段的第一个点
      Eigen::Vector3d prev_pt = first_pt;              // 上一个点
      Eigen::Vector3d cur_pt;                          // 当前点

      // 前进直到轨迹超出半径或全局时间
      while (radius < des_radius && seg_time < global_duration_ - start_t - 1e-3)
      {
        seg_time += delt;
        seg_time = min(seg_time, global_duration_ - start_t);

        cur_pt = getPosition(start_t + seg_time);
        seg_length += (cur_pt - prev_pt).norm(); // 计算段的长度
        prev_pt = cur_pt;
        radius = (cur_pt - first_pt).norm(); // 更新半径
      }

      // 通过期望的点密度获取参数化dt
      int seg_num = floor(seg_length / dist_pt);

      // 获取输出数据
      seg_duration = seg_time; // 截断段的持续时间
      dt = seg_time / seg_num; // 两个点之间的时间差

      for (double tp = 0.0; tp <= seg_time + 1e-4; tp += dt)
      {
        cur_pt = getPosition(start_t + tp);
        point_set.push_back(cur_pt); // 将当前点加入点集
      }

      start_end_derivative.push_back(getVelocity(start_t)); // 起点速度
      start_end_derivative.push_back(getVelocity(start_t + seg_time)); // 终点速度
      start_end_derivative.push_back(getAcceleration(start_t)); // 起点加速度
      start_end_derivative.push_back(getAcceleration(start_t + seg_time)); // 终点加速度
    }

    // 获取固定持续时间内的局部轨迹的B样条参数化数据
    // start_t: 轨迹的起始时间
    // duration: 段的时间长度
    // seg_num: 将段分成的部分数量
    void getTrajByDuration(double start_t, double duration, int seg_num,
                           vector<Eigen::Vector3d> &point_set,
                           vector<Eigen::Vector3d> &start_end_derivative, double &dt)
    {
      dt = duration / seg_num; // 计算时间差
      Eigen::Vector3d cur_pt;
      for (double tp = 0.0; tp <= duration + 1e-4; tp += dt)
      {
        cur_pt = getPosition(start_t + tp);
        point_set.push_back(cur_pt); // 将当前点加入点集
      }

      start_end_derivative.push_back(getVelocity(start_t)); // 起点速度
      start_end_derivative.push_back(getVelocity(start_t + duration)); // 终点速度
      start_end_derivative.push_back(getAcceleration(start_t)); // 起点加速度
      start_end_derivative.push_back(getAcceleration(start_t + duration)); // 终点加速度
    }
  };

  // 规划参数结构体
  struct PlanParameters
  {
    /* 规划算法的参数 */
    double max_vel_, max_acc_, max_jerk_; // 物理限制：最大速度、加速度、加加速度
    double ctrl_pt_dist;                  // B样条控制点之间的距离
    double feasibility_tolerance_;        // 速度/加速度超过限制的容许比例
    double planning_horizen_;             // 规划视野

    /* 处理时间 */
    double time_search_ = 0.0;
    double time_optimize_ = 0.0;
    double time_adjust_ = 0.0;
  };

  // 局部轨迹数据结构体
  struct LocalTrajData
  {
    /* 生成的轨迹信息 */

    int traj_id_; // 轨迹ID
    double duration_; // 持续时间
    double global_time_offset; // 因为当局部轨迹完成并切换回全局轨迹时，全局轨迹时间不再与世界时间匹配，因此需要偏移量
    ros::Time start_time_; // 轨迹的起始时间
    Eigen::Vector3d start_pos_; // 轨迹的起始位置
    UniformBspline position_traj_, velocity_traj_, acceleration_traj_; // 位置、速度和加速度轨迹
  };

} // namespace ego_planner

#endif
