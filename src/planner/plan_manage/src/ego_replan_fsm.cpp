#include <plan_manage/ego_replan_fsm.h>

namespace ego_planner
{

  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    current_wp_ = 0; // 当前的航路点索引
    exec_state_ = FSM_EXEC_STATE::INIT; // 初始状态设置为INIT
    have_target_ = false; // 是否有目标标志
    have_odom_ = false; // 是否有里程计数据标志

    /*  fsm 参数初始化  */
    nh.param("fsm/flight_type", target_type_, -1); // 飞行类型参数
    nh.param("fsm/thresh_replan", replan_thresh_, -1.0); // 重新规划的阈值
    nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0); // 不需要重新规划的阈值
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0); // 规划视野范围
    nh.param("fsm/planning_horizen_time", planning_horizen_time_, -1.0); // 规划视野时间
    nh.param("fsm/emergency_time_", emergency_time_, 1.0); // 紧急情况的时间

    nh.param("fsm/waypoint_num", waypoint_num_, -1); // 预设航路点数量
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }

    /* 初始化主要模块 */
    visualization_.reset(new PlanningVisualization(nh)); // 初始化可视化模块
    planner_manager_.reset(new EGOPlannerManager); // 初始化规划管理器
    planner_manager_->initPlanModules(nh, visualization_); // 初始化规划模块

    /* 回调函数 */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this); // 执行状态机的定时器
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this); // 碰撞检测的定时器

    odom_sub_ = nh.subscribe("/odom_world", 1, &EGOReplanFSM::odometryCallback, this); // 订阅里程计话题

    bspline_pub_ = nh.advertise<ego_planner::Bspline>("/planning/bspline", 10); // 发布B样条轨迹
    data_disp_pub_ = nh.advertise<ego_planner::DataDisp>("/planning/data_display", 100); // 发布数据可视化信息

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
      waypoint_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1, &EGOReplanFSM::waypointCallback, this); // 手动目标订阅航路点话题
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
    {
      ros::Duration(1.0).sleep(); // 延时1秒等待其他模块准备就绪
      while (ros::ok() && !have_odom_)
      ros::spinOnce(); // 等待获取到里程计数据
      planGlobalTrajbyGivenWps(); // 通过预设的航路点规划全局轨迹
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl; // 错误的目标类型
  }

  // 根据预设的航路点规划全局轨迹
  void EGOReplanFSM::planGlobalTrajbyGivenWps()
  {
    std::vector<Eigen::Vector3d> wps(waypoint_num_); // 存储航路点
    for (int i = 0; i < waypoint_num_; i++)
    {
       wps[i](0) = waypoints_[i][0];
       wps[i](1) = waypoints_[i][1];
       wps[i](2) = waypoints_[i][2];
   
      end_pt_ = wps.back(); // 终点为最后一个航路点
    }
    // 调用规划管理器进行全局轨迹规划
    bool success = planner_manager_->planGlobalTrajWaypoints(odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    // 可视化航路点
    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    if (success)
    {

      /*** 可视化全局轨迹 ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      std::vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true; // 标志为已获取目标
      have_new_target_ = true; // 标志为有新目标

      /*** 状态机切换 ***/
      // if (exec_state_ == WAIT_TARGET)
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG"); // 生成新轨迹
      // else if (exec_state_ == EXEC_TRAJ)
      //   changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // 可视化全局路径
      ros::Duration(0.001).sleep();
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
      ros::Duration(0.001).sleep();
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!"); // 生成全局轨迹失败
    }
  }

  // 航路点回调函数
  void EGOReplanFSM::waypointCallback(const nav_msgs::PathConstPtr &msg)
  {
    if (msg->poses[0].pose.position.z < -0.1)
      return; // 如果航路点的高度小于-0.1，忽略此航路点

    cout << "Triggered!" << endl; // 打印触发信息
    trigger_ = true; // 设置触发标志
    init_pt_ = odom_pos_; // 设置初始点为当前里程计位置

    bool success = false;
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0; // 设置终点为消息中的位置
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()); // 调用全局轨迹规划

    // 可视化目标点
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {

      /*** 可视化全局轨迹 ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true; // 标志为已获取目标
      have_new_target_ = true; // 标志为有新目标

      /*** 状态机切换 ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG"); // 生成新轨迹
      else if (exec_state_ == EXEC_TRAJ)
        changeFSMExecState(REPLAN_TRAJ, "TRIG"); // 重新规划轨迹

      // 可视化全局路径
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!"); // 生成全局轨迹失败
    }
  }

  // 里程计回调函数
  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg ); // 估计加速度（注释掉，暂不使用）

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true; // 标志为已获取里程计数据
  }

  // 改变状态机的执行状态
  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {
    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl; // 打印状态转换信息
  }

  // 获取连续状态调用的次数
  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  // 打印当前状态机的执行状态
  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl; // 打印当前状态
  }

  // 状态机的执行回调函数
  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState(); // 每100次调用打印一次状态
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!trigger_)
        cout << "wait for goal." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        return; // 等待获取到里程计数据
      }
      if (!trigger_)
      {
        return; // 等待触发信号
      }
      changeFSMExecState(WAIT_TARGET, "FSM"); // 改变状态为WAIT_TARGET
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        return; // 等待目标
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM"); // 生成新轨迹
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      start_pt_ = odom_pos_; // 设置起始点为当前里程计位置
      start_vel_ = odom_vel_; // 设置起始速度为当前速度
      start_acc_.setZero(); // 起始加速度为零

      bool flag_random_poly_init;
  //       // 获取连续状态调用的次数
  // std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  // {
  //   return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  // }
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      bool success = callReboundReplan(true, flag_random_poly_init); // 调用重新规划
      if (success)
      {
        changeFSMExecState(EXEC_TRAJ, "FSM"); // 改变状态为执行轨迹
        flag_escape_emergency_ = true;
      }
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM"); // 继续生成新轨迹
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      if (planFromCurrentTraj())
      {
        changeFSMExecState(EXEC_TRAJ, "FSM"); // 重新规划成功，改变状态为执行轨迹
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM"); // 重新规划失败，继续重新规划
      }
      break;
    }

    case EXEC_TRAJ:
    {
      /* 判断是否需要重新规划 */
      LocalTrajData *info = &planner_manager_->local_data_;
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - info->start_time_).toSec();
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2)
      {
        have_target_ = false; // 目标完成

        changeFSMExecState(WAIT_TARGET, "FSM"); // 改变状态为等待目标
        return;
      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)
      {
        // 如果距离终点很近，不需要重新规划
        return;
      }
      else if ((info->start_pos_ - pos).norm() < replan_thresh_)
      {
        // 如果距离起始点很近，不需要重新规划
        return;
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM"); // 需要重新规划
      }
      break;
    }

    case EMERGENCY_STOP:
    {
      if (flag_escape_emergency_) // 避免重复调用紧急停止
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM"); // 当速度足够小后，重新生成轨迹
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_); // 发布数据可视化信息
  }

  // 从当前轨迹进行重新规划
  bool EGOReplanFSM::planFromCurrentTraj()
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - info->start_time_).toSec();

    start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur); // 获取当前时间的位置信息
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur); // 获取当前时间的速度信息
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur); // 获取当前时间的加速度信息

    bool success = callReboundReplan(false, false); // 调用重新规划

    if (!success)
    {
      success = callReboundReplan(true, false);
      if (!success)
      {
        success = callReboundReplan(true, true);
        if (!success)
        {
          return false; // 重新规划失败
        }
      }
    }

    return true; // 重新规划成功
  }

  // 碰撞检测回调函数
  void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || info->start_time_.toSec() < 1e-5)
      return; // 如果处于等待目标状态或者起始时间太小，直接返回

    /* ---------- 检查轨迹 ---------- */
    constexpr double time_step = 0.01;
    double t_cur = (ros::Time::now() - info->start_time_).toSec();
    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // 如果当前时间小于2/3的轨迹时间，只检查前2/3的轨迹
        break;

      if (map->getInflateOccupancy(info->position_traj_.evaluateDeBoorT(t))) // 检查轨迹点是否与障碍物发生碰撞
      {
        if (planFromCurrentTraj()) // 如果重新规划成功
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY"); // 改变状态为执行轨迹
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_) // 如果在紧急时间内
          {
            ROS_WARN("Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur); // 紧急停止
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            changeFSMExecState(REPLAN_TRAJ, "SAFETY"); // 重新规划
          }
          return;
        }
        break;
      }
    }
  }

// 调用重新规划
bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
{
    getLocalTarget(); // 获取局部目标
    // flag_randomPolyTraj=true
  
    // 调用重新规划函数 reboundReplan，传入起始点、速度、加速度、局部目标点、局部目标速度以及标志位
    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false; // 规划完成后重置 have_new_target_ 为 false

    // cout << "final_plan_success=" << plan_success << endl; // 输出最终规划是否成功
    ROS_WARN("final_plan_success=%s", plan_success ? "true" : "false"); // 输出最终规划是否成功


    if (plan_success) // 如果规划成功
    {
        auto info = &planner_manager_->local_data_; // 获取规划器的本地数据

        /* 发布轨迹 */
        ego_planner::Bspline bspline;//自定义消息
        bspline.order = 3; // 设置B样条的阶数为3
        bspline.start_time = info->start_time_; // 设置B样条的开始时间
        bspline.traj_id = info->traj_id_; // 设置B样条的轨迹ID

        // 获取控制点并将其转换为ROS消息格式
        Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
        bspline.pos_pts.reserve(pos_pts.cols());
        for (int i = 0; i < pos_pts.cols(); ++i)
        {
            geometry_msgs::Point pt;
            pt.x = pos_pts(0, i);
            pt.y = pos_pts(1, i);
            pt.z = pos_pts(2, i);
            bspline.pos_pts.push_back(pt); // 添加每个控制点到B样条消息的控制点数组中
        }

        // 获取节点向量并将其转换为ROS消息格式
        Eigen::VectorXd knots = info->position_traj_.getKnot();
        bspline.knots.reserve(knots.rows());
        for (int i = 0; i < knots.rows(); ++i)
        {
            bspline.knots.push_back(knots(i)); // 添加每个节点到B样条消息的节点数组中
        }

        bspline_pub_.publish(bspline); // 发布B样条轨迹

        // 可视化最优轨迹，将控制点传递给可视化模块
        visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
    }

    return plan_success; // 返回规划结果
}

  // 调用紧急停止
  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {
    planner_manager_->EmergencyStop(stop_pos); // 调用规划管理器的紧急停止函数

    auto info = &planner_manager_->local_data_;

    /* 发布轨迹 */
    ego_planner::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_.publish(bspline); // 发布紧急停止后的B样条轨迹

    return true; // 返回紧急停止成功
  }

  // 获取局部目标
  void EGOReplanFSM::getLocalTarget()
  {
    double t;

    double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_; // 计算时间步长
    double dist_min = 9999, dist_min_t = 0.0;
    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      double dist = (pos_t - start_pt_).norm();

      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizen_)
      {
        // 如果上次进度时间点的距离大于规划视野，报错
        ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
        return;
      }
      if (dist < dist_min)
      {
        dist_min = dist;
        dist_min_t = t;
      }
      if (dist >= planning_horizen_)
      {
        local_target_pt_ = pos_t; // 设置局部目标点
        planner_manager_->global_data_.last_progress_time_ = dist_min_t; // 更新上次进度时间点
        break;
      }
    }
    if (t > planner_manager_->global_data_.global_duration_) // 最后一个全局点
    {
      local_target_pt_ = end_pt_;
    }

    // 如果距离终点的距离小于动态减速范围，则设置局部目标速度为零
    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      local_target_vel_ = planner_manager_->global_data_.getVelocity(t); // 获取局部目标速度
    }
  }

} // namespace ego_planner
