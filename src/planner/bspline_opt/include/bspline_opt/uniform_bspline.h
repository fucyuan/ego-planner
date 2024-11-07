#ifndef _UNIFORM_BSPLINE_H_ 
#define _UNIFORM_BSPLINE_H_ 

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>

using namespace std;

namespace ego_planner
{
  // 实现非均匀 B 样条，并支持不同维度
  // 同时也可以表示均匀 B 样条，作为非均匀 B 样条的特殊情况
  class UniformBspline
  {
  private:
    // B 样条的控制点，支持不同维度。
    // 每行代表一个控制点
    // 维度由列数决定
    // 例如，3D 空间中 N 个点的 B 样条 -> Nx3 矩阵
    Eigen::MatrixXd control_points_;

    int p_, n_, m_;     // p 阶次，n+1 个控制点，m = n+p+1
    Eigen::VectorXd u_; // 节点向量
    double interval_;   // 节点间隔 \delta t 

    // 获取导数控制点
    Eigen::MatrixXd getDerivativeControlPoints();

    // 物理限制和时间调整系数
    double limit_vel_, limit_acc_, limit_ratio_, feasibility_tolerance_;

  public:
    UniformBspline() {}
    UniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval);
    ~UniformBspline();

    // 获取控制点
    Eigen::MatrixXd get_control_points(void) { return control_points_; }

    // 初始化为均匀 B 样条
    void setUniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval);

    // 获取/设置基本的 B 样条信息

    // 设置节点向量
    void setKnot(const Eigen::VectorXd &knot);
    // 获取节点向量
    Eigen::VectorXd getKnot();
    // 获取控制点
    Eigen::MatrixXd getControlPoint();
    // 获取节点间隔
    double getInterval();
    // 获取时间范围
    bool getTimeSpan(double &um, double &um_p);

    // 计算位置/导数

    // 使用 De Boor 算法计算位置，输入 u \in [up, u_mp]
    Eigen::VectorXd evaluateDeBoor(const double &u);
    // 使用 t \in [0, duration] 计算位置
    inline Eigen::VectorXd evaluateDeBoorT(const double &t) { return evaluateDeBoor(t + u_(p_)); }
    // 获取导数 B 样条
    UniformBspline getDerivative();

    // 3D B 样条插值点集，具有边界速度和加速度约束
    // 输入: (K+2) 个点及边界速度/加速度；时间间隔 ts
    // 输出: (K+6) 个控制点
    static void parameterizeToBspline(const double &ts, const vector<Eigen::Vector3d> &point_set,
                                      const vector<Eigen::Vector3d> &start_end_derivative,
                                      Eigen::MatrixXd &ctrl_pts);

    /* 检查可行性，调整时间 */

    // 设置物理限制（速度，加速度，容忍度）
    void setPhysicalLimits(const double &vel, const double &acc, const double &tolerance);
    // 检查轨迹的可行性
    bool checkFeasibility(double &ratio, bool show = false);
    // 延长时间
    void lengthenTime(const double &ratio);

    /* 用于性能评估 */

    // 获取轨迹的总时间
    double getTimeSum();
    // 获取轨迹的长度，分辨率为 res
    double getLength(const double &res = 0.01);
    // 获取轨迹的加加速度（jerk）
    double getJerk();
    // 获取平均和最大速度
    void getMeanAndMaxVel(double &mean_v, double &max_v);
    // 获取平均和最大加速度
    void getMeanAndMaxAcc(double &mean_a, double &max_a);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace ego_planner 
#endif
