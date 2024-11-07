#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "ego_planner/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>

ros::Publisher pos_cmd_pub;

quadrotor_msgs::PositionCommand cmd;
double pos_gain[3] = {0, 0, 0}; // 位置增益参数
double vel_gain[3] = {0, 0, 0}; // 速度增益参数

using ego_planner::UniformBspline;

bool receive_traj_ = false; // 标志是否接收到轨迹
vector<UniformBspline> traj_; // 存储轨迹的向量
double traj_duration_; // 轨迹持续时间
ros::Time start_time_; // 轨迹开始时间
int traj_id_; // 轨迹ID

// 偏航控制
double last_yaw_, last_yaw_dot_; // 上一次的偏航角和偏航角速度
double time_forward_; // 向前预测的时间

// B样条轨迹回调函数
void bsplineCallback(ego_planner::BsplineConstPtr msg)
{
  // 解析位置轨迹

  Eigen::MatrixXd pos_pts(3, msg->pos_pts.size()); // 存储位置点的矩阵

  Eigen::VectorXd knots(msg->knots.size()); // 存储节点向量
  for (size_t i = 0; i < msg->knots.size(); ++i)
  {
    knots(i) = msg->knots[i];
  }

  for (size_t i = 0; i < msg->pos_pts.size(); ++i)
  {
    pos_pts(0, i) = msg->pos_pts[i].x;
    pos_pts(1, i) = msg->pos_pts[i].y;
    pos_pts(2, i) = msg->pos_pts[i].z;
  }

  // 创建位置轨迹的B样条对象
  UniformBspline pos_traj(pos_pts, msg->order, 0.1);
  pos_traj.setKnot(knots);

  // 解析偏航轨迹（代码注释掉了，暂时不使用）
  // Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
  // for (int i = 0; i < msg->yaw_pts.size(); ++i) {
  //   yaw_pts(i, 0) = msg->yaw_pts[i];
  // }
  // UniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

  // 设置轨迹的起始时间和ID
  start_time_ = msg->start_time;
  traj_id_ = msg->traj_id;

  // 清空之前的轨迹并添加新的轨迹
  traj_.clear();
  traj_.push_back(pos_traj); // 添加位置轨迹
  traj_.push_back(traj_[0].getDerivative()); // 添加速度轨迹
  traj_.push_back(traj_[1].getDerivative()); // 添加加速度轨迹

  traj_duration_ = traj_[0].getTimeSum(); // 获取轨迹的总持续时间

  receive_traj_ = true; // 标志为已接收到轨迹
}

// 计算偏航角和偏航角速度的函数
std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, ros::Time &time_now, ros::Time &time_last)
{
  constexpr double PI = 3.1415926;
  constexpr double YAW_DOT_MAX_PER_SEC = PI; // 每秒最大偏航角速度
  // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;
  std::pair<double, double> yaw_yawdot(0, 0); // 存储偏航角和偏航角速度
  double yaw = 0;
  double yawdot = 0;

  // 计算当前时间向前预测的方向向量
  Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_ ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos : traj_[0].evaluateDeBoorT(traj_duration_) - pos;
  double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_; // 如果方向向量的模大于0.1，则计算偏航角，否则使用上一次的偏航角
  double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).toSec(); // 计算允许的最大偏航角变化

  // 根据偏航角的变化范围进行限制
  if (yaw_temp - last_yaw_ > PI)
  {
    if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }
  else if (yaw_temp - last_yaw_ < -PI)
  {
    if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }
  else
  {
    if (yaw_temp - last_yaw_ < -max_yaw_change)
    {
      yaw = last_yaw_ - max_yaw_change;
      if (yaw < -PI)
        yaw += 2 * PI;

      yawdot = -YAW_DOT_MAX_PER_SEC;
    }
    else if (yaw_temp - last_yaw_ > max_yaw_change)
    {
      yaw = last_yaw_ + max_yaw_change;
      if (yaw > PI)
        yaw -= 2 * PI;

      yawdot = YAW_DOT_MAX_PER_SEC;
    }
    else
    {
      yaw = yaw_temp;
      if (yaw - last_yaw_ > PI)
        yawdot = -YAW_DOT_MAX_PER_SEC;
      else if (yaw - last_yaw_ < -PI)
        yawdot = YAW_DOT_MAX_PER_SEC;
      else
        yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
    }
  }

  // 低通滤波
  if (fabs(yaw - last_yaw_) <= max_yaw_change)
    yaw = 0.5 * last_yaw_ + 0.5 * yaw; // 简单的低通滤波
  yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;
  last_yaw_ = yaw;
  last_yaw_dot_ = yawdot;

  yaw_yawdot.first = yaw;
  yaw_yawdot.second = yawdot;

  return yaw_yawdot;
}

// 定时器回调函数，用于发布位置指令
void cmdCallback(const ros::TimerEvent &e)
{
  /* 在接收到轨迹之前不发布指令 */
  if (!receive_traj_)
    return;

  ros::Time time_now = ros::Time::now();
  double t_cur = (time_now - start_time_).toSec(); // 计算当前时间相对于轨迹开始时间的时间差

  Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;
  std::pair<double, double> yaw_yawdot(0, 0);

  static ros::Time time_last = ros::Time::now();
  if (t_cur < traj_duration_ && t_cur >= 0.0)
  {
    // 计算当前位置、速度和加速度
    pos = traj_[0].evaluateDeBoorT(t_cur);
    vel = traj_[1].evaluateDeBoorT(t_cur);
    acc = traj_[2].evaluateDeBoorT(t_cur);

    /*** 计算偏航角 ***/
    yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last);
    /*** 计算偏航角 ***/

    double tf = min(traj_duration_, t_cur + 2.0);
    pos_f = traj_[0].evaluateDeBoorT(tf); // 计算未来位置
  }
  else if (t_cur >= traj_duration_)
  {
    /* 轨迹完成后悬停 */
    pos = traj_[0].evaluateDeBoorT(traj_duration_);
    vel.setZero();
    acc.setZero();

    yaw_yawdot.first = last_yaw_;
    yaw_yawdot.second = 0;

    pos_f = pos;
  }
  else
  {
    cout << "[Traj server]: invalid time." << endl;
  }
  time_last = time_now;

  // 填充位置指令消息
  cmd.header.stamp = time_now;
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id_;

  cmd.position.x = pos(0);
  cmd.position.y = pos(1);
  cmd.position.z = pos(2);

  cmd.velocity.x = vel(0);
  cmd.velocity.y = vel(1);
  cmd.velocity.z = vel(2);

  cmd.acceleration.x = acc(0);
  cmd.acceleration.y = acc(1);
  cmd.acceleration.z = acc(2);

  cmd.yaw = yaw_yawdot.first;
  cmd.yaw_dot = yaw_yawdot.second;

  last_yaw_ = cmd.yaw;

  pos_cmd_pub.publish(cmd); // 发布位置指令
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "traj_server"); // 初始化ROS节点
  ros::NodeHandle node;
  ros::NodeHandle nh("~");

  // 订阅B样条轨迹
  ros::Subscriber bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);

  // 发布位置指令
  pos_cmd_pub = node.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);

  // 创建定时器，用于周期性发布位置指令
  ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);

  /* 控制参数 */
  cmd.kx[0] = pos_gain[0];
  cmd.kx[1] = pos_gain[1];
  cmd.kx[2] = pos_gain[2];

  cmd.kv[0] = vel_gain[0];
  cmd.kv[1] = vel_gain[1];
  cmd.kv[2] = vel_gain[2];

  nh.param("traj_server/time_forward", time_forward_, -1.0); // 获取参数 time_forward
  last_yaw_ = 0.0;
  last_yaw_dot_ = 0.0;

  ros::Duration(1.0).sleep(); // 延时1秒

  ROS_WARN("[Traj server]: ready."); // 打印准备就绪的警告信息

  ros::spin(); // 循环等待回调

  return 0;
}
