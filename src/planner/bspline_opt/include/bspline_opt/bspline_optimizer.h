#ifndef _BSPLINE_OPTIMIZER_H_
#define _BSPLINE_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <path_searching/dyn_a_star.h>
#include <bspline_opt/uniform_bspline.h>
#include <plan_env/grid_map.h>
#include <ros/ros.h>
#include "bspline_opt/lbfgs.hpp"

// 梯度和弹性带优化

// 输入: 签名距离场和一系列点
// 输出: 优化后的点序列
// 点的格式: N x 3 矩阵，每行是一个点
namespace ego_planner
{

  class ControlPoints
  {
  public:
    double clearance; // 清除距离
    int size; // 控制点的数量
    Eigen::MatrixXd points; // 控制点矩阵
    std::vector<std::vector<Eigen::Vector3d>> base_point; // 方向向量起点（碰撞点）
    std::vector<std::vector<Eigen::Vector3d>> direction;  // 方向向量，必须归一化
    std::vector<bool> flag_temp;                          // 在很多地方使用的标志，每次使用前初始化
    // std::vector<bool> occupancy;

    void resize(const int size_set)
    {
      size = size_set;

      base_point.clear();
      direction.clear();
      flag_temp.clear();
      // occupancy.clear();

      points.resize(3, size_set);
      base_point.resize(size);
      direction.resize(size);
      flag_temp.resize(size);
      // occupancy.resize(size);
    }
  };

  class BsplineOptimizer
  {

  public:
    BsplineOptimizer() {}
    ~BsplineOptimizer() {}

    /* 主API */
    void setEnvironment(const GridMap::Ptr &env); // 设置环境
    void setParam(ros::NodeHandle &nh); // 设置参数
    Eigen::MatrixXd BsplineOptimizeTraj(const Eigen::MatrixXd &points, const double &ts,
                                        const int &cost_function, int max_num_id, int max_time_id); // B样条轨迹优化

    /* 辅助函数 */

    // 必要输入
    void setControlPoints(const Eigen::MatrixXd &points); // 设置控制点
    void setBsplineInterval(const double &ts); // 设置B样条间隔
    void setCostFunction(const int &cost_function); // 设置代价函数
    void setTerminateCond(const int &max_num_id, const int &max_time_id); // 设置终止条件

    // 可选输入
    void setGuidePath(const vector<Eigen::Vector3d> &guide_pt); // 设置引导路径
    void setWaypoints(const vector<Eigen::Vector3d> &waypts,
                      const vector<int> &waypt_idx); // 设置航路点，最多N-2个约束

    void optimize(); // 优化函数

    Eigen::MatrixXd getControlPoints(); // 获取控制点

    AStar::Ptr a_star_; // A*算法指针
    std::vector<Eigen::Vector3d> ref_pts_; // 参考点

    std::vector<std::vector<Eigen::Vector3d>> initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init = true); // 初始化控制点
    bool BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts); // 必须在initControlPoints()后调用
    bool BsplineOptimizeTrajRefine(const Eigen::MatrixXd &init_points, const double ts, Eigen::MatrixXd &optimal_points); // 轨迹优化细化

    inline int getOrder(void) { return order_; } // 获取B样条阶数

  private:
    GridMap::Ptr grid_map_; // 网格地图指针

    enum FORCE_STOP_OPTIMIZE_TYPE
    {
      DONT_STOP, // 不停止
      STOP_FOR_REBOUND, // 因反弹停止
      STOP_FOR_ERROR // 因错误停止
    } force_stop_type_;

    // 主要输入
    // Eigen::MatrixXd control_points_;     // B样条控制点，N x dim
    double bspline_interval_; // B样条节点间隔
    Eigen::Vector3d end_pt_;  // 轨迹终点
    // int             dim_;                // B样条的维度
    //
    vector<Eigen::Vector3d> guide_pts_; // 几何引导路径点，N-6
    vector<Eigen::Vector3d> waypoints_; // 航路点约束
    vector<int> waypt_idx_;             // 航路点约束索引
                                        //
    int max_num_id_, max_time_id_;      // 停止条件
    int cost_function_;                 // 用于确定目标函数
    double start_time_;                 // 动态障碍物的全局时间

    /* 优化参数 */
    int order_;                    // B样条的阶数
    double lambda1_;               // jerk 平滑权重
    double lambda2_, new_lambda2_; // 距离权重
    double lambda3_;               // 可行性权重
    double lambda4_;               // 曲线拟合

    int a;
    //
    double dist0_;             // 安全距离
    double max_vel_, max_acc_; // 动力学限制

    int variable_num_;              // 优化变量数量
    int iter_num_;                  // 求解器迭代次数
    Eigen::VectorXd best_variable_; // 最优变量
    double min_cost_;               // 最小代价

    ControlPoints cps_; // 控制点对象

    /* 代价函数 */
    /* 以控制点 q 作为输入计算代价函数的每个部分 */

    static double costFunction(const std::vector<double> &x, std::vector<double> &grad, void *func_data); // 静态代价函数
    void combineCost(const std::vector<double> &x, vector<double> &grad, double &cost); // 组合代价

    // q 包含所有控制点
    void calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                            Eigen::MatrixXd &gradient, bool falg_use_jerk = true); // 计算平滑代价
    void calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                             Eigen::MatrixXd &gradient); // 计算可行性代价
    void calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost); // 计算反弹距离代价
    void calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient); // 计算拟合代价
    bool check_collision_and_rebound(void); // 检查碰撞并反弹

    static int earlyExit(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls); // 提前退出
    static double costFunctionRebound(void *func_data, const double *x, double *grad, const int n); // 反弹代价函数
    static double costFunctionRefine(void *func_data, const double *x, double *grad, const int n); // 细化代价函数

    bool rebound_optimize(); // 反弹优化
    bool refine_optimize(); // 细化优化
    void combineCostRebound(const double *x, double *grad, double &f_combine, const int n); // 组合反弹代价
    void combineCostRefine(const double *x, double *grad, double &f_combine, const int n); // 组合细化代价

    /* 仅用于基准评估 */
  public:
    typedef unique_ptr<BsplineOptimizer> Ptr;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace ego_planner
#endif