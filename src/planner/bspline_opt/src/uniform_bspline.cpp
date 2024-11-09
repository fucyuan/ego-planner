#include "bspline_opt/uniform_bspline.h"
#include <ros/ros.h>

namespace ego_planner
{

UniformBspline::UniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval)
{
    setUniformBspline(points, order, interval); 
    // 调用 setUniformBspline 函数来初始化 B 样条的控制点、阶数和时间间隔
}

UniformBspline::~UniformBspline() {} 
// 析构函数，默认析构（无任何特殊操作）

void UniformBspline::setUniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval)
{
    control_points_ = points; // 将输入的控制点矩阵 points 赋值给类的成员变量 control_points_
    p_ = order; // 将阶数 order 赋值给成员变量 p_
    interval_ = interval; // 将时间间隔 interval 赋值给成员变量 interval_

    n_ = points.cols() - 1; // 计算控制点数量减一，并赋值给成员变量 n_
    m_ = n_ + p_ + 1; // 计算节点向量的最后一个索引 m，赋值给成员变量 m_

    u_ = Eigen::VectorXd::Zero(m_ + 1); // 初始化节点向量 u_ 为零向量，长度为 m_ + 1
    for (int i = 0; i <= m_; ++i) // 遍历节点向量的所有元素
    {
        if (i <= p_) // 如果当前索引 i 小于等于阶数 p
        {
            u_(i) = double(-p_ + i) * interval_; 
            // 为前 p+1 个节点赋值为负数，表示从负方向逐步增加的间隔（用于平滑起始部分）
        }
        else if (i > p_ && i <= m_ - p_) // 如果当前索引在中间范围
        {
            u_(i) = u_(i - 1) + interval_; 
            // 按照固定时间间隔逐渐增加节点值
        }
        else if (i > m_ - p_) // 如果当前索引大于 m - p，即位于末尾部分
        {
            u_(i) = u_(i - 1) + interval_; 
            // 为后 p+1 个节点赋值，使得末尾的节点值保持一致（用于平滑终点部分）
        }
    }
}


  void UniformBspline::setKnot(const Eigen::VectorXd &knot) { this->u_ = knot; }

  Eigen::VectorXd UniformBspline::getKnot() { return this->u_; }

 bool UniformBspline::getTimeSpan(double &um, double &um_p)
{
    // 检查条件是否满足：如果 p_ 超过了 u_ 的行数，或者 m_ - p_ 超过了 u_ 的行数
    // 如果条件不满足，返回 false，表示时间跨度无法获取
    if (p_ > u_.rows() || m_ - p_ > u_.rows())
        return false;

    // 将 u_ 向量中索引为 p_ 和 m_ - p_ 的值分别赋给 um 和 um_p
    um = u_(p_);
    um_p = u_(m_ - p_);

    // 如果条件满足，返回 true，表示成功获取了时间跨度
    return true;
}


  Eigen::MatrixXd UniformBspline::getControlPoint() { return control_points_; }

  Eigen::VectorXd UniformBspline::evaluateDeBoor(const double &u)
  {

    double ub = min(max(u_(p_), u), u_(m_ - p_));

    // determine which [ui,ui+1] lay in
    int k = p_;
    while (true)
    {
      if (u_(k + 1) >= ub)
        break;
      ++k;
    }

    /* deBoor's alg */
    vector<Eigen::VectorXd> d;
    for (int i = 0; i <= p_; ++i)
    {
      d.push_back(control_points_.col(k - p_ + i));
      // cout << d[i].transpose() << endl;
    }

    for (int r = 1; r <= p_; ++r)
    {
      for (int i = p_; i >= r; --i)
      {
        double alpha = (ub - u_[i + k - p_]) / (u_[i + 1 + k - r] - u_[i + k - p_]);
        // cout << "alpha: " << alpha << endl;
        d[i] = (1 - alpha) * d[i - 1] + alpha * d[i];
      }
    }

    return d[p_];
  }

  // Eigen::VectorXd UniformBspline::evaluateDeBoorT(const double& t) {
  //   return evaluateDeBoor(t + u_(p_));
  // }

  Eigen::MatrixXd UniformBspline::getDerivativeControlPoints()
  {
    // The derivative of a b-spline is also a b-spline, its order become p_-1
    // control point Qi = p_*(Pi+1-Pi)/(ui+p_+1-ui+1)
    Eigen::MatrixXd ctp(control_points_.rows(), control_points_.cols() - 1);
    for (int i = 0; i < ctp.cols(); ++i)
    {
      ctp.col(i) =
          p_ * (control_points_.col(i + 1) - control_points_.col(i)) / (u_(i + p_ + 1) - u_(i + 1));
    }
    return ctp;
  }

  UniformBspline UniformBspline::getDerivative()
  {
    Eigen::MatrixXd ctp = getDerivativeControlPoints();
    UniformBspline derivative(ctp, p_ - 1, interval_);

    /* cut the first and last knot */
    Eigen::VectorXd knot(u_.rows() - 2);
    knot = u_.segment(1, u_.rows() - 2);
    derivative.setKnot(knot);

    return derivative;
  }

  double UniformBspline::getInterval() { return interval_; }

void UniformBspline::setPhysicalLimits(const double &vel, const double &acc, const double &tolerance)
{
    limit_vel_ = vel;                  // 设置速度限制，将输入参数vel赋值给成员变量limit_vel_
    limit_acc_ = acc;                  // 设置加速度限制，将输入参数acc赋值给成员变量limit_acc_
    limit_ratio_ = 1.1;                // 设置限制比例，固定为1.1
    feasibility_tolerance_ = tolerance; // 设置可行性容差，将输入参数tolerance赋值给成员变量feasibility_tolerance_
}


bool UniformBspline::checkFeasibility(double &ratio, bool show)
{
    bool fea = true; // 初始化可行性标记，假设轨迹可行

    Eigen::MatrixXd P = control_points_; // 将控制点矩阵赋值给局部变量 P
    int dimension = control_points_.rows(); // 获取控制点的维度（通常是 2D 或 3D）

    /* 检查速度可行性并插入点 */
    double max_vel = -1.0; // 最大速度初始值
    double enlarged_vel_lim = limit_vel_ * (1.0 + feasibility_tolerance_) + 1e-4; 
    // 计算放宽后的速度限制，增加一个微小值 1e-4 以避免边界问题

    for (int i = 0; i < P.cols() - 1; ++i) // 遍历所有控制点之间的段
    {
        // 计算第 i 段的速度向量
        Eigen::VectorXd vel = p_ * (P.col(i + 1) - P.col(i)) / (u_(i + p_ + 1) - u_(i + 1));

        // 检查速度在各维度是否超过限制
        if (fabs(vel(0)) > enlarged_vel_lim || fabs(vel(1)) > enlarged_vel_lim || fabs(vel(2)) > enlarged_vel_lim)
        {
            if (show) // 如果 `show` 为真，输出不可行的速度信息
                cout << "[Check]: Infeasible vel " << i << " :" << vel.transpose() << endl;
            fea = false; // 将可行性标记设置为 false

            for (int j = 0; j < dimension; ++j)
            {
                max_vel = max(max_vel, fabs(vel(j))); // 更新最大速度
            }
        }
    }

    /* 检查加速度可行性 */
    double max_acc = -1.0; // 最大加速度初始值
    double enlarged_acc_lim = limit_acc_ * (1.0 + feasibility_tolerance_) + 1e-4; 
    // 计算放宽后的加速度限制

    for (int i = 0; i < P.cols() - 2; ++i) // 遍历所有控制点之间的加速度段
    {
        // 计算第 i 段的加速度向量
        Eigen::VectorXd acc = p_ * (p_ - 1) *
                            ((P.col(i + 2) - P.col(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
                             (P.col(i + 1) - P.col(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
                            (u_(i + p_ + 1) - u_(i + 2));

        // 检查加速度在各维度是否超过限制
        if (fabs(acc(0)) > enlarged_acc_lim || fabs(acc(1)) > enlarged_acc_lim || fabs(acc(2)) > enlarged_acc_lim)
        {
            if (show) // 如果 `show` 为真，输出不可行的加速度信息
                cout << "[Check]: Infeasible acc " << i << " :" << acc.transpose() << endl;
            fea = false; // 将可行性标记设置为 false

            for (int j = 0; j < dimension; ++j)
            {
                max_acc = max(max_acc, fabs(acc(j))); // 更新最大加速度
            }
        }
    }

    // 计算速度和加速度超限的比率，取最大值
    ratio = max(max_vel / limit_vel_, sqrt(fabs(max_acc) / limit_acc_));

    return fea; // 返回是否可行的标记
}


void UniformBspline::lengthenTime(const double &ratio)
{
    // 定义第一个固定节点位置
    int num1 = 5;

    // 定义最后一个固定节点位置，等于结点数组的行数减1再减去5
    int num2 = getKnot().rows() - 1 - 5;

    // 计算总时间增量 delta_t，根据比例 ratio 计算出扩展后的时间增量
    double delta_t = (ratio - 1.0) * (u_(num2) - u_(num1));

    // 计算每一段时间增量 t_inc
    double t_inc = delta_t / double(num2 - num1);

    // 遍历中间的节点，从 num1 + 1 到 num2，将这些节点的时间按照 t_inc 递增
    for (int i = num1 + 1; i <= num2; ++i)
        u_(i) += double(i - num1) * t_inc;

    // 遍历 num2 之后的节点，直接增加总的 delta_t 时间增量
    for (int i = num2 + 1; i < u_.rows(); ++i)
        u_(i) += delta_t;
}



void UniformBspline::parameterizeToBspline(const double &ts, const vector<Eigen::Vector3d> &point_set,
                                           const vector<Eigen::Vector3d> &start_end_derivative,
                                           Eigen::MatrixXd &ctrl_pts)
{
    // 如果时间步长ts小于等于0，输出时间步长错误提示并返回
    if (ts <= 0)
    {
        cout << "[B-spline]:time step error." << endl;
        return;
    }

    // 如果点集的大小小于等于3，输出点集大小错误提示并返回
    if (point_set.size() <= 3)
    {
        cout << "[B-spline]:point set have only " << point_set.size() << " points." << endl;
        return;
    }

    // 如果起点和终点导数的数量不等于4，输出导数错误提示
    if (start_end_derivative.size() != 4)
    {
        cout << "[B-spline]:derivatives error." << endl;
    }

    int K = point_set.size(); // K为点集的大小

    // 定义矩阵A中的行向量，用于描述B样条的系数
    Eigen::Vector3d prow(3), vrow(3), arow(3);
    prow << 1, 4, 1;    // 用于位置的系数行
    vrow << -1, 0, 1;   // 用于速度的系数行
    arow << 1, -2, 1;   // 用于加速度的系数行

    // 初始化矩阵A为K+4行K+2列的零矩阵，表示B样条的系数矩阵
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 4, K + 2);

    // 填充矩阵A的前K行，逐行插入B样条位置的系数
    for (int i = 0; i < K; ++i)
        A.block(i, i, 1, 3) = (1 / 6.0) * prow.transpose();

    // 填充矩阵A的第K行和第K+1行，用于起始点和终点的速度
    A.block(K, 0, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();
    A.block(K + 1, K - 1, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();

    // 填充矩阵A的第K+2行和第K+3行，用于起始点和终点的加速度
    A.block(K + 2, 0, 1, 3) = (1 / ts / ts) * arow.transpose();
    A.block(K + 3, K - 1, 1, 3) = (1 / ts / ts) * arow.transpose();

    // 输出调试信息，显示矩阵A
    //cout << "A" << endl << A << endl << endl;

    // 定义b向量，存储x, y, z三个方向上的位移、速度和加速度的约束
    Eigen::VectorXd bx(K + 4), by(K + 4), bz(K + 4);

    // 将点集中的每个点的x, y, z坐标分别赋值到bx, by, bz中
    for (int i = 0; i < K; ++i)
    {
      bx(i) = point_set[i](0);
      by(i) = point_set[i](1);
      bz(i) = point_set[i](2);
    }

 //将起点和终点导数的x, y, z值分别赋值到bx, by, bz的末尾部分
    for (int i = 0; i < 4; ++i)
    {
      bx(K + i) = start_end_derivative[i](0);
      by(K + i) = start_end_derivative[i](1);
      bz(K + i) = start_end_derivative[i](2);
    }

    // 使用QR分解求解，分别解出x, y, z方向的控制点
    Eigen::VectorXd px = A.colPivHouseholderQr().solve(bx);
    Eigen::VectorXd py = A.colPivHouseholderQr().solve(by);
    Eigen::VectorXd pz = A.colPivHouseholderQr().solve(bz);

    // 将求解出的x, y, z方向的控制点转置后存储到控制点矩阵ctrl_pts中
    ctrl_pts.resize(3, K + 2);
    ctrl_pts.row(0) = px.transpose();
    ctrl_pts.row(1) = py.transpose();
    ctrl_pts.row(2) = pz.transpose();

    // 输出提示，表明B样条参数化完成
    cout << "[B-spline]: parameterization ok." << endl;
}

  double UniformBspline::getTimeSum()
  {
    double tm, tmp;
    if (getTimeSpan(tm, tmp))
      return tmp - tm;
    else
      return -1.0;
  }

  double UniformBspline::getLength(const double &res)
  {
    double length = 0.0;
    double dur = getTimeSum();
    Eigen::VectorXd p_l = evaluateDeBoorT(0.0), p_n;
    for (double t = res; t <= dur + 1e-4; t += res)
    {
      p_n = evaluateDeBoorT(t);
      length += (p_n - p_l).norm();
      p_l = p_n;
    }
    return length;
  }

  double UniformBspline::getJerk()
  {
    UniformBspline jerk_traj = getDerivative().getDerivative().getDerivative();

    Eigen::VectorXd times = jerk_traj.getKnot();
    Eigen::MatrixXd ctrl_pts = jerk_traj.getControlPoint();
    int dimension = ctrl_pts.rows();

    double jerk = 0.0;
    for (int i = 0; i < ctrl_pts.cols(); ++i)
    {
      for (int j = 0; j < dimension; ++j)
      {
        jerk += (times(i + 1) - times(i)) * ctrl_pts(j, i) * ctrl_pts(j, i);
      }
    }

    return jerk;
  }

  void UniformBspline::getMeanAndMaxVel(double &mean_v, double &max_v)
  {
    UniformBspline vel = getDerivative();
    double tm, tmp;
    vel.getTimeSpan(tm, tmp);

    double max_vel = -1.0, mean_vel = 0.0;
    int num = 0;
    for (double t = tm; t <= tmp; t += 0.01)
    {
      Eigen::VectorXd vxd = vel.evaluateDeBoor(t);
      double vn = vxd.norm();

      mean_vel += vn;
      ++num;
      if (vn > max_vel)
      {
        max_vel = vn;
      }
    }

    mean_vel = mean_vel / double(num);
    mean_v = mean_vel;
    max_v = max_vel;
  }

  void UniformBspline::getMeanAndMaxAcc(double &mean_a, double &max_a)
  {
    UniformBspline acc = getDerivative().getDerivative();
    double tm, tmp;
    acc.getTimeSpan(tm, tmp);

    double max_acc = -1.0, mean_acc = 0.0;
    int num = 0;
    for (double t = tm; t <= tmp; t += 0.01)
    {
      Eigen::VectorXd axd = acc.evaluateDeBoor(t);
      double an = axd.norm();

      mean_acc += an;
      ++num;
      if (an > max_acc)
      {
        max_acc = an;
      }
    }

    mean_acc = mean_acc / double(num);
    mean_a = mean_acc;
    max_a = max_acc;
  }
} // namespace ego_planner
