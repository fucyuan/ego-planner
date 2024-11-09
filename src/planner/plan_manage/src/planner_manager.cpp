// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>

namespace ego_planner
{

  // SECTION interfaces for setup and query

  EGOPlannerManager::EGOPlannerManager() {}

  EGOPlannerManager::~EGOPlannerManager() { std::cout << "des manager" << std::endl; }

  void EGOPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
  {
    /* read algorithm parameters */

    nh.param("manager/max_vel", pp_.max_vel_, -1.0);
    nh.param("manager/max_acc", pp_.max_acc_, -1.0);
    nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
    nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
    nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);
    nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);

    local_data_.traj_id_ = 0;
    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);

    bspline_optimizer_rebound_.reset(new BsplineOptimizer);
    bspline_optimizer_rebound_->setParam(nh);
    bspline_optimizer_rebound_->setEnvironment(grid_map_);
    bspline_optimizer_rebound_->a_star_.reset(new AStar);
    bspline_optimizer_rebound_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));

    visualization_ = vis;
  }

  // !SECTION

  // SECTION rebond replanning

  bool EGOPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
  {

    static int count = 0;
    std::cout << endl
              << "[rebo replan]: -------------------------------------" << count++ << std::endl;
    cout.precision(3);
    cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
         << endl;

    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      cout << "Close to goal" << endl;
      continous_failures_count_++;
      return false;
    }

    ros::Time t_start = ros::Time::now();
    ros::Duration t_init, t_opt, t_refine;

    /*** STEP 1: INIT ***/
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    static bool flag_first_call = true, flag_force_polynomial = false;
    bool flag_regenerate = false;
do
{
  // 清除之前的点集和导数集，并初始化重新生成标志
  point_set.clear();
  start_end_derivatives.clear();
  flag_regenerate = false;

  // 如果这是第一次调用，或者需要强制生成多项式轨迹（或其他条件满足时），生成初始路径
  if (flag_first_call || flag_polyInit || flag_force_polynomial /* || (start_pt - local_target_pt).norm() < 1.0 */) // 初始路径通过多项式轨迹生成
  {
    flag_first_call = false;
    flag_force_polynomial = false;

    PolynomialTraj gl_traj; // 声明一个多项式轨迹对象

    // 计算起点到局部目标点的距离
    double dist = (start_pt - local_target_pt).norm();

    // 计算轨迹时间，根据距离和最大速度、加速度确定时间
    double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist 
                  ? sqrt(dist / pp_.max_acc_) 
                  : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;

    // 如果不需要随机生成多项式轨迹，使用单段轨迹生成函数
    if (!flag_randomPolyTraj)
    {
      // 使用起点和局部目标点生成单段多项式轨迹
      gl_traj = PolynomialTraj::one_segment_traj_gen(
                    start_pt, start_vel, start_acc, 
                    local_target_pt, local_target_vel, 
                    Eigen::Vector3d::Zero(), time);
    }
    else // 否则，生成随机插入点以创建更复杂的轨迹
    {
      // 水平和垂直方向的随机偏移
      Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
      Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
      
      // 生成随机的插入点，增加轨迹的随机性
      Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                           (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizen_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
                                           (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);

      // 设置位置矩阵，其中包括起点、随机插入点和目标点
      Eigen::MatrixXd pos(3, 3);
      pos.col(0) = start_pt;
      pos.col(1) = random_inserted_pt;
      pos.col(2) = local_target_pt;

      // 设置时间向量，每段时间相等
      Eigen::VectorXd t(2);
      t(0) = t(1) = time / 2;

      // 生成多段多项式最小快照轨迹
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
    }

    double t; // 轨迹时间
    bool flag_too_far; // 标记是否点之间距离过远

    ts *= 1.5; // 增加时间步长

    do
    {
      ts /= 1.5; // 逐渐减少时间步长
      point_set.clear(); // 清除之前生成的点集
      flag_too_far = false; // 初始化标记

      Eigen::Vector3d last_pt = gl_traj.evaluate(0); // 计算初始点

      // 逐个时间步长生成轨迹点
      for (t = 0; t < time; t += ts)
      {
        Eigen::Vector3d pt = gl_traj.evaluate(t);
        if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5) // 如果两点之间距离过大，则标记并跳出循环
        {
          flag_too_far = true;
          break;
        }
        last_pt = pt;
        point_set.push_back(pt); // 将点加入点集
      }
    } while (flag_too_far || point_set.size() < 7); // 确保点集足够多

    t -= ts; // 调整最后一个点的时间

    // 存储轨迹的导数信息
    start_end_derivatives.push_back(gl_traj.evaluateVel(0));
    start_end_derivatives.push_back(local_target_vel);
    start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
    start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
  }
  else // 如果不是第一次调用，则从之前的轨迹生成初始路径
  {
    double t;
    double t_cur = (ros::Time::now() - local_data_.start_time_).toSec(); // 获取当前时间

    vector<double> pseudo_arc_length; // 存储伪弧长
    vector<Eigen::Vector3d> segment_point; // 存储轨迹分段点

    pseudo_arc_length.push_back(0.0); // 初始化伪弧长
    for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
    {
      segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t)); // 计算分段点
      if (t > t_cur)
      {
        pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
      }
    }
    t -= ts;

    // 计算多项式轨迹时间
    double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;
    if (poly_time > ts)
    {
      // 生成从当前点到目标点的多项式轨迹
      PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),
                                                                    local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                    local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                    local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

      // 添加多项式轨迹的点到分段点中
      for (t = ts; t < poly_time; t += ts)
      {
        if (!pseudo_arc_length.empty())
        {
          segment_point.push_back(gl_traj.evaluate(t));
          pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
        }
        else
        {
          ROS_ERROR("pseudo_arc_length is empty, return!"); // 错误处理
          continous_failures_count_++;
          return false;
        }
      }
    }

    double sample_length = 0;
    double cps_dist = pp_.ctrl_pt_dist * 1.5; // 初始控制点间距
    size_t id = 0;

    // 通过伪弧长采样轨迹点，确保点集足够多
    do
    {
      cps_dist /= 1.5;
      point_set.clear();
      sample_length = 0;
      id = 0;

      // 采样轨迹点，确保采样的点数足够
      while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
      {
        if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
        {
          point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                              (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
          sample_length += cps_dist;
        }
        else
          id++;
      }
      point_set.push_back(local_target_pt); // 添加最终目标点
    } while (point_set.size() < 7); // 确保点集大小足够

    // 存储导数信息
    start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
    start_end_derivatives.push_back(local_target_vel);
    start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
    start_end_derivatives.push_back(Eigen::Vector3d::Zero());

    // 如果路径长度异常长，重新生成多项式轨迹
    if (point_set.size() > pp_.planning_horizen_ / pp_.ctrl_pt_dist * 3)
    {
      flag_force_polynomial = true;
      flag_regenerate = true;
    }
  }
} while (flag_regenerate); // 如果需要重新生成轨迹，继续循环


    Eigen::MatrixXd ctrl_pts;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

    vector<vector<Eigen::Vector3d>> a_star_pathes;
    a_star_pathes = bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);

    t_init = ros::Time::now() - t_start;

    static int vis_id = 0;
    visualization_->displayInitPathList(point_set, 0.2, 0);
    visualization_->displayAStarList(a_star_pathes, vis_id);

    t_start = ros::Time::now();

   /*** STEP 2: 优化 ***/
    bool flag_step_1_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts); 
    // 调用B样条优化器的函数 BsplineOptimizeTrajRebound 对控制点 (ctrl_pts) 和时间 (ts) 进行优化，并返回优化是否成功的标志

    cout << "first_optimize_step_success=" << flag_step_1_success << endl; 
    // 输出第一次优化步骤是否成功的标志

    if (!flag_step_1_success) // 如果第一次优化失败
    {
      // visualization_->displayOptimalList( ctrl_pts, vis_id );
      continous_failures_count_++; // 连续失败次数计数加1
      return false; // 返回 false，退出函数
    }

    // visualization_->displayOptimalList( ctrl_pts, vis_id );

    t_opt = ros::Time::now() - t_start; // 计算当前优化步骤的耗时，并将结果存入 t_opt
    t_start = ros::Time::now(); // 更新 t_start 为当前时间，为下一次计时做准备


    /*** STEP 3: 精细化（重新分配时间）如果有必要 ***/
    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts); 
    // 创建一个 B 样条轨迹对象 pos，使用控制点 ctrl_pts，3 阶样条，时间间隔 ts

    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_); 
    // 为样条轨迹设置物理限制，包括最大速度、最大加速度和可行性容忍度

    double ratio; // 定义一个比率变量，用于调整时间分配
    bool flag_step_2_success = true; // 标记步骤2是否成功

    if (!pos.checkFeasibility(ratio, false)) // 检查轨迹的可行性，如果不满足限制条件
    {
        cout << "Need to reallocate time." << endl; // 输出需要重新分配时间的提示信息

        Eigen::MatrixXd optimal_control_points; // 定义一个矩阵，用于存储优化后的控制点
        flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points); 
        // 调用 refineTrajAlgo 函数优化轨迹时间分配，输出新的控制点并返回优化成功标志

        if (flag_step_2_success) // 如果优化成功
            pos = UniformBspline(optimal_control_points, 3, ts); // 更新 B 样条轨迹对象为优化后的轨迹
    }

    if (!flag_step_2_success) // 如果优化失败
    {
        // printf("\033[34mThis refined trajectory hits obstacles. It doesn't matter if appears occasionally. But if continuously appearing, increase parameter \"lambda_fitness\".\n\033[0m");
        ROS_WARN("This refined trajectory hits obstacles. It doesn't matter if it appears occasionally. But if it continuously appears, increase parameter \"lambda_fitness\".");
        // 提示信息：优化轨迹偶尔与障碍物相交并不重要，但如果经常出现，建议增加参数 "lambda_fitness"

        continous_failures_count_++; // 增加连续失败计数
        return false; // 返回 false，表示精细化优化失败
    }

    t_refine = ros::Time::now() - t_start; // 计算精细化步骤的耗时

    // 保存规划结果
    updateTrajInfo(pos, ros::Time::now()); // 调用 updateTrajInfo 函数保存规划好的轨迹信息

    cout << "total time:\033[42m" << (t_init + t_opt + t_refine).toSec() << "\033[0m,optimize:" << (t_init + t_opt).toSec() << ",refine:" << t_refine.toSec() << endl;
    // 输出总耗时、优化耗时和精细化耗时

    // 成功完成优化，重置连续失败计数
    continous_failures_count_ = 0;
    return true; // 返回 true，表示精细化步骤成功

  }

  bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }

    updateTrajInfo(UniformBspline(control_points, 3, 1.0), ros::Time::now());

    return true;
  }

  bool EGOPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);

    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      points.push_back(waypoints[wp_i]);
    }

    double total_len = 0;
    total_len += (start_pos - waypoints[0]).norm();
    for (size_t i = 0; i < waypoints.size() - 1; i++)
    {
      total_len += (waypoints[i + 1] - waypoints[i]).norm();
    }

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    double dist_thresh = max(total_len / 8, 4.0);

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // for ( int i=0; i<inter_points.size(); i++ )
    // {
    //   cout << inter_points[i].transpose() << endl;
    // }

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = ros::Time::now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool EGOPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
{
  // 生成全局参考轨迹

  vector<Eigen::Vector3d> points;
  points.push_back(start_pos); // 添加起始点 
  points.push_back(end_pos);   // 添加终点 

  // 如果距离过远，插入中间点
  vector<Eigen::Vector3d> inter_points;
  const double dist_thresh = 4.0; // 距离阈值 

  for (size_t i = 0; i < points.size() - 1; ++i)
  {
    inter_points.push_back(points.at(i)); // 添加当前点 
    double dist = (points.at(i + 1) - points.at(i)).norm(); // 计算两点之间的距离

    if (dist > dist_thresh)
    {
      int id_num = floor(dist / dist_thresh) + 1; // 计算需要插入的中间点数量

      for (int j = 1; j < id_num; ++j)
      {
        Eigen::Vector3d inter_pt =
            points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num; // 计算中间点的位置 
        inter_points.push_back(inter_pt); // 添加中间点 
      }
    }
  }

  inter_points.push_back(points.back()); // 添加终点 

  // 写入位置矩阵
  int pt_num = inter_points.size();
  Eigen::MatrixXd pos(3, pt_num); // 创建位置矩阵，大小为3行pt_num列
  for (int i = 0; i < pt_num; ++i)
    pos.col(i) = inter_points[i]; // 将中间点写入位置矩阵

  Eigen::Vector3d zero(0, 0, 0); // 零向量
  Eigen::VectorXd time(pt_num - 1); // 创建时间向量
  for (int i = 0; i < pt_num - 1; ++i)
  {
    time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_); // 计算每段的时间 
  }

  time(0) *= 2.0; // 增加起始段的时间
  time(time.rows() - 1) *= 2.0; // 增加终止段的时间

  PolynomialTraj gl_traj;
  if (pos.cols() >= 3)
    gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time); // 生成最小Snap轨迹
  else if (pos.cols() == 2)
    gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0)); // 生成单段轨迹
  else
    return false; // 如果点数小于2，返回false

  auto time_now = ros::Time::now(); // 获取当前时间
  global_data_.setGlobalTraj(gl_traj, time_now); // 设置全局轨迹

  return true; // 规划成功，返回true
}


bool EGOPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
{
    double t_inc;  // 定义一个时间增量变量，用于在重参数化过程中计算时间增加（与原来所需要的时间）

    Eigen::MatrixXd ctrl_pts; // 定义控制点矩阵，将用于存储轨迹的控制点

    // std::cout << "ratio: " << ratio << std::endl; // 打印 ratio 的值（调试用，已注释掉）
    
    // 调用 reparamBspline 函数，对轨迹进行重参数化
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);//ts===dt时间间隔

    // 使用重参数化后的控制点矩阵和时间间隔重新生成 B样条曲线对象
    traj = UniformBspline(ctrl_pts, 3, ts);

    // 计算时间步长 t_step，用于在轨迹上等间隔采样点
    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);

    // 清空 bspline_optimizer_rebound_ 对象中的参考点列表
    bspline_optimizer_rebound_->ref_pts_.clear();

    // 遍历轨迹上的每一个时间步长，使用去Boor算法计算该时刻的轨迹点，并加入参考点列表
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
        bspline_optimizer_rebound_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

    // 调用 BsplineOptimizeTrajRefine 函数，进行轨迹优化，结果存储在 optimal_control_points
    bool success = bspline_optimizer_rebound_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success; // 返回优化是否成功的布尔值
}


void EGOPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now)
{
    // 更新轨迹的起始时间，将当前时间赋值给 local_data_ 的 start_time_
    local_data_.start_time_ = time_now;

    // 更新位置轨迹，将输入的 position_traj 赋值给 local_data_ 的 position_traj_
    local_data_.position_traj_ = position_traj;

    // 计算速度轨迹，通过位置轨迹的导数获取速度轨迹
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();

    // 计算加速度轨迹，通过速度轨迹的导数获取加速度轨迹
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();

    // 获取起始位置，在 De Boor 参数 t = 0.0 处评估位置轨迹，赋值给 start_pos_
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);

    // 获取轨迹总时长，通过 position_traj 调用 getTimeSum 方法获取
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();

    // 轨迹编号自增，更新 traj_id_，以区分不同的轨迹更新
    local_data_.traj_id_ += 1;
}


  void EGOPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                        Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
      // 获取原始轨迹的总时长
      double time_origin = bspline.getTimeSum();

      // 计算轨迹的分段数量，通常为控制点数量减3（去掉两端的起始和终止节点）
      int seg_num = bspline.getControlPoint().cols() - 3;

      // 通过长度扩展时间，根据给定的比例 ratio 来调整时间间隔
      bspline.lengthenTime(ratio);

      // 获取调整后的轨迹总时长
      double duration = bspline.getTimeSum();

      // 计算新的时间间隔 dt，每段所占用的时间
      dt = duration / double(seg_num);

      // 计算时间增量 time_inc，即重参数化后的总时长与原始时长的差值
      time_inc = duration - time_origin;

      // 定义一个向量 point_set，用于存储采样点
      vector<Eigen::Vector3d> point_set;

      // 遍历轨迹的时间区间，以 dt 为步长采样轨迹上的点
      for (double time = 0.0; time <= duration + 1e-4; time += dt)
      {
          // 使用去Boor算法计算给定时间点的轨迹位置，并存入 point_set
          point_set.push_back(bspline.evaluateDeBoorT(time));
      }

      // 重新参数化轨迹，将采样点 point_set 和导数信息 start_end_derivative 转换为 B样条控制点
      UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  }


} // namespace ego_planner
