#ifndef EKF_FUSER_3D_H
#define EKF_FUSER_3D_H

#include <Eigen/Dense>
#include <string>
#include <eigen_conversions/eigen_msg.h>
#include <optional> // 用于存储上一个里程计状态
// #include "loop_closing.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
#include <tf/tf.h>

/**
 * @struct OdomState
 * @brief 里程计系统的输出状态结构
 */
struct OdomState {
    Eigen::Matrix3d R_q;      // 旋转矩阵 (世界坐标系下的姿态)
    Eigen::Vector3d p;        // 位置 (世界坐标系下的位置)
    // 其他字段 (v, ba, bg等) 在此融合器中不直接使用，但保留结构完整性
    Eigen::Vector3d v;
    Eigen::Vector3d ba;
    Eigen::Vector3d bg;
    long double time;
};

/**
 * @struct GlobalPose
 * @brief 全局文字定位结果的输出状态结构
 */
struct GlobalPose {
    Eigen::Vector3d position;       // 位置 p = [px, py, pz]
    Eigen::Quaterniond orientation; // 旋转 q = [qw, qx, qy, qz]
    double timestamp;
};


/**
 * @class EKFLocalizationFuser3D
 * @brief 适配新数据结构的3D扩展卡尔曼滤波器
 */
class EKFLocalizationFuser3D {
public:
    // 使用 Eigen 定义状态向量和矩阵的类型别名
    using StateVector = Eigen::Matrix<double, 6, 1>;
    using StateMatrix = Eigen::Matrix<double, 6, 6>;

    /**
     * @brief 构造函数
     * @param initial_pose 车辆的初始3D位姿
     */
    explicit EKFLocalizationFuser3D(const StateVector& initial_state);

    /**
     * @brief EKF 预测步骤，由里程计系统的完整状态驱动
     * @param current_odom_state 里程计系统当前时刻的输出状态
     */
    void predict(const OdomState& current_odom_state);

    /**
     * @brief EKF 更新步骤，使用全局位姿测量值进行校正
     * @param global_pose 通过车位号匹配获得的全局位姿测量值
     */
    void update(const GlobalPose& global_pose);

    /**
     * @brief 获取当前融合后的最优估计位姿 (以四元数表示姿态)
     * @return 当前的全局位姿 (GlobalPose格式)
     */
    GlobalPose getFusedPose() const;

    /**
     * @brief 获取当前的状态向量
     * @return 当前的6x1状态向量 (Eigen格式)
     */
    StateVector getStateVector() const;

private:
    // 内部帮助函数
    double normalizeAngle(double angle);
    bool initialized_;
    StateVector poseToVector(const Eigen::Vector3d& position, const Eigen::Vector3d& euler_angles);
    Eigen::Vector3d rotationToEuler(const Eigen::Matrix3d& R);
    Eigen::Matrix3d eulerToRotation(const Eigen::Vector3d& euler) const;


    // EKF 状态和协方差
    StateVector state_;         // 状态向量 [x, y, z, roll, pitch, yaw]^T
    StateMatrix P_;             // 6x6 状态协方差矩阵
    StateMatrix Q_;             // 6x6 过程噪声协方差矩阵
    StateMatrix R_;             // 6x6 测量噪声协方差矩阵
    StateMatrix H_;             // 6x6 测量矩阵
    StateMatrix I_;             // 6x6 单位矩阵

    // 用于存储上一个里程计状态以计算增量
    std::optional<OdomState> last_odom_state_;

    // 一致性检查的门控阈值
    double gating_threshold_;
};

#endif // EKF_FUSER_3D_H