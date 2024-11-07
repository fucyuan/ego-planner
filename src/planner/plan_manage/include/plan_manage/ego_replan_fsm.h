#ifndef _REBO_REPLAN_FSM_H_
#define _REBO_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>
#include <nav_msgs/Path.h>
#include <sensor_msgs/Imu.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <vector>
#include <visualization_msgs/Marker.h>

#include <bspline_opt/bspline_optimizer.h>
#include <plan_env/grid_map.h>
#include <ego_planner/Bspline.h>
#include <ego_planner/DataDisp.h>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>

using std::vector;

namespace ego_planner
{

  class EGOReplanFSM
  {

  private:
    /* ---------- 状态机执行状态标志 ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,           // 初始化状态
      WAIT_TARGET,    // 等待目标状态
      GEN_NEW_TRAJ,   // 生成新轨迹状态
      REPLAN_TRAJ,    // 重新规划轨迹状态
      EXEC_TRAJ,      // 执行轨迹状态
      EMERGENCY_STOP  // 紧急停止状态
    };
    enum TARGET_TYPE
    {
      MANUAL_TARGET = 1,  // 手动选择目标
      PRESET_TARGET = 2,  // 预设目标
      REFENCE_PATH = 3    // 参考路径
    };

    /* 规划工具对象 */
    EGOPlannerManager::Ptr planner_manager_;           // 规划管理器
    PlanningVisualization::Ptr visualization_;         // 规划可视化工具
    ego_planner::DataDisp data_disp_;                  // 数据可视化对象

    /* 参数 */
    int target_type_;                                  // 目标类型，1为手动选择，2为硬编码
    double no_replan_thresh_, replan_thresh_;          // 重新规划阈值参数
    double waypoints_[50][3];                          // 预设航路点数组
    int waypoint_num_;                                 // 航路点数量
    double planning_horizen_, planning_horizen_time_;  // 规划视野范围和时间
    double emergency_time_;                            // 紧急情况的时间

    /* 规划数据 */
    bool trigger_, have_target_, have_odom_, have_new_target_; // 状态标志位
    FSM_EXEC_STATE exec_state_;                                 // 当前状态机执行状态
    int continously_called_times_{0};                           // 连续调用次数

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_;            // 里程计状态（位置、速度、加速度）
    Eigen::Quaterniond odom_orient_;                            // 里程计的姿态

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // 起始状态（位置、速度、加速度、偏航角）
    Eigen::Vector3d end_pt_, end_vel_;                                      // 目标状态（位置、速度）
    Eigen::Vector3d local_target_pt_, local_target_vel_;                    // 局部目标状态（位置、速度）
    int current_wp_;                                                        // 当前航路点索引

    bool flag_escape_emergency_;                                            // 紧急逃逸标志

    /* ROS 工具对象 */
    ros::NodeHandle node_;                                  // ROS 节点句柄
    ros::Timer exec_timer_, safety_timer_;                  // ROS 定时器，用于执行状态机和安全检查
    ros::Subscriber waypoint_sub_, odom_sub_;               // ROS 订阅者，用于订阅航路点和里程计信息
    ros::Publisher replan_pub_, new_pub_, bspline_pub_, data_disp_pub_; // ROS 发布者，用于发布重新规划、新轨迹、B样条和数据可视化信息

    /* 辅助函数 */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj); // 调用重新规划（包括前端和后端）
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // 调用紧急停止（包括前端和后端）
    bool planFromCurrentTraj();                                                // 从当前轨迹进行重新规划

    /* 返回值：std::pair< 连续调用相同状态的次数, 当前连续调用的状态 > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);         // 改变状态机执行状态
    std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls(); // 获取连续调用相同状态的次数和状态
    void printFSMExecState();                                                   // 打印状态机执行状态

    void planGlobalTrajbyGivenWps();  // 根据给定的航路点规划全局轨迹
    void getLocalTarget();            // 获取局部目标

    /* ROS 回调函数 */
    void execFSMCallback(const ros::TimerEvent &e);         // 状态机执行的回调函数
    void checkCollisionCallback(const ros::TimerEvent &e);  // 碰撞检测的回调函数
    void waypointCallback(const nav_msgs::PathConstPtr &msg); // 航路点的回调函数
    void odometryCallback(const nav_msgs::OdometryConstPtr &msg); // 里程计的回调函数

    bool checkCollision(); // 碰撞检查函数

  public:
    EGOReplanFSM(/* args */)
    {
    }
    ~EGOReplanFSM()
    {
    }

    void init(ros::NodeHandle &nh); // 初始化函数

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace ego_planner

#endif