#ifndef EKF_LOCALIZATION_3D_H
#define EKF_LOCALIZATION_3D_H

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

// --- 辅助函数 ---
// 将角度归一化到 [-pi, pi]
inline double normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle <= -M_PI) angle += 2.0 * M_PI;
    return angle;
}

// 表示一个 3D 位姿 (x, y, z, roll, pitch, yaw)
struct Pose3D {
    double x, y, z;
    double roll, pitch, yaw;

    // 转换为 Eigen 向量
    Eigen::Matrix<double, 6, 1> toVector() const {
        Eigen::Matrix<double, 6, 1> vec;
        vec << x, y, z, roll, pitch, yaw;
        return vec;
    }

    // 从 Eigen 向量创建
    static Pose3D fromVector(const Eigen::Matrix<double, 6, 1>& vec) {
        return {vec(0), vec(1), vec(2), normalizeAngle(vec(3)), normalizeAngle(vec(4)), normalizeAngle(vec(5))};
    }
};

// 表示 3D 里程计增量 (在机器人坐标系下)
struct OdometryDelta3D {
    double delta_x;
    double delta_y;
    double delta_z;
    double delta_roll;
    double delta_pitch;
    double delta_yaw;
};


class EKFLocalization3D {
public:
    using StateVector = Eigen::Matrix<double, 6, 1>;
    using StateMatrix = Eigen::Matrix<double, 6, 6>;

    EKFLocalization3D(const StateMatrix& process_noise_q,
                        const StateMatrix& measurement_noise_r,
                        double outlier_mahalanobis_threshold);

    void initialize(const Pose3D& initial_pose, const StateMatrix& initial_covariance);
    bool isInitialized() const;
    void predict(const OdometryDelta3D& odom); // 预测总是执行
    void update(const Pose3D& global_pose_measurement);
    Pose3D getCurrentPose() const;
    StateVector getStateVector() const;
    StateMatrix getCovariance() const;
    void setUninitialized();

private:
    StateVector state_;
    StateMatrix covariance_;
    StateMatrix Q_;
    StateMatrix R_;
    double mahalanobis_threshold_;
    bool initialized_;

    Eigen::Matrix3d getRotationMatrix() const;
};

#endif // EKF_LOCALIZATION_3D_H
