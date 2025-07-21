#include "ekf_fuser_3d.h"
#include <cmath>
#include <iostream>

// 定义 PI
const double PI = 3.14159265358979323846;

EKFLocalizationFuser3D::EKFLocalizationFuser3D(const StateVector& initial_state)
    : state_(initial_state),
      gating_threshold_(12.592) ,initialized_(false){ // 卡方分布，6个自由度，95%置信度

    // 初始化单位矩阵
    I_.setIdentity();

    // 初始化协方差矩阵 P，表示初始状态的不确定性
    P_.setIdentity(); // 初始不确定性设为1

    // 设置过程噪声协方差 Q，代表里程计模型的不确定性
    Q_.setIdentity();
    Q_(0, 0) = 0.05 * 0.05; Q_(1, 1) = 0.05 * 0.05; Q_(2, 2) = 0.05 * 0.05; // 位置增量噪声
    Q_(3, 3) = 0.01 * 0.01; Q_(4, 4) = 0.01 * 0.01; Q_(5, 5) = 0.01 * 0.01; // 姿态增量噪声

    // 设置测量噪声协方差 R，代表全局定位结果的不确定性
    R_.setIdentity();
    R_(0, 0) = 0.5 * 0.5; R_(1, 1) = 0.5 * 0.5; R_(2, 2) = 0.5 * 0.5; // 位置测量噪声
    R_(3, 3) = 0.2 * 0.2; R_(4, 4) = 0.2 * 0.2; R_(5, 5) = 0.2 * 0.2; // 姿态测量噪声

    // 设置测量矩阵 H，因为我们直接测量状态，所以 H 是单位矩阵
    H_.setIdentity();

}

// bool EKFLocalizationFuser3D::isInitialized() const {
//     return initialized_;
// }

void EKFLocalizationFuser3D::predict(const OdomState& current_odom_state) {
    Eigen::Matrix3d R_correction = Eigen::AngleAxisd(PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    // 如果是第一次接收里程计数据，则只保存状态，不进行预测
    if (!last_odom_state_.has_value()) {
        // std::cout<<"predict curstate_------------:"<<current_odom_state.p<<std::endl;
        last_odom_state_ =current_odom_state;
        return;
    }
    //  std::cout<<"predict state_------------:"<<current_odom_state.p<<std::endl;
    // --- 1. 计算里程计位姿增量 ---
    // 计算旋转增量
    Eigen::Matrix3d delta_R = last_odom_state_->R_q.transpose() * current_odom_state.R_q;
    // 计算位置增量 (转换到上一帧的 body 坐标系下)
    Eigen::Vector3d delta_p = last_odom_state_->R_q.transpose() * (current_odom_state.p - last_odom_state_->p);



    // --- 2. EKF 状态预测 ---
    const double r = state_(3); // 当前EKF状态的 roll
    const double p = state_(4); // 当前EKF状态的 pitch
    const double y = state_(5); // 当前EKF状态的 yaw

    const double sr = sin(r), cr = cos(r);
    const double sp = sin(p), cp = cos(p);
    const double sy = sin(y), cy = cos(y);

    // 将计算出的位移增量从当前EKF的body系转换到世界系
        
    Eigen::Matrix3d R_ekf_body_to_world = eulerToRotation({r, p, y});
    
    R_ekf_body_to_world=R_ekf_body_to_world;
     Eigen::Vector3d delta_euler = rotationToEuler( delta_R);

   
    // std::cout<<"rotation---------:"<<R_ekf_body_to_world<<std::endl;
    state_.head<3>() += R_ekf_body_to_world * delta_p;
    // 直接累加姿态增量
    state_.tail<3>() += delta_euler;

    // 规范化角度
    state_(3) = normalizeAngle(state_(3));
    state_(4) = normalizeAngle(state_(4));
    state_(5) = normalizeAngle(state_(5));

    // --- 3. EKF 协方差预测 ---
    StateMatrix F = I_;
    // 计算雅可比矩阵 F (位置对姿态的偏导数)
    F(0, 3) = delta_p.y() * (cy*sp*cr + sy*sr) + delta_p.z() * (-cy*sp*sr + sy*cr);
    F(1, 3) = delta_p.y() * (sy*sp*cr - cy*sr) + delta_p.z() * (-sy*sp*sr - cy*cr);
    F(2, 3) = delta_p.y() * (cp*cr) + delta_p.z() * (-cp*sr);
    F(0, 4) = delta_p.x() * (-cy*sp) + delta_p.y() * (cy*cp*sr) + delta_p.z() * (cy*cp*cr);
    F(1, 4) = delta_p.x() * (-sy*sp) + delta_p.y() * (sy*cp*sr) + delta_p.z() * (sy*cp*cr);
    F(2, 4) = delta_p.x() * (-cp) + delta_p.y() * (-sp*sr) + delta_p.z() * (-sp*cr);
    F(0, 5) = delta_p.x() * (-sy*cp) + delta_p.y() * (-sy*sp*sr - cy*cr) + delta_p.z() * (-sy*sp*cr + cy*sr);
    F(1, 5) = delta_p.x() * (cy*cp) + delta_p.y() * (cy*sp*sr - sy*cr) + delta_p.z() * (cy*sp*cr + sy*sr);

    P_ = F * P_ * F.transpose() + Q_;
    // std::cout<<"predict state_------------:"<<state_<<std::endl;

    // 更新上一个里程计状态
    last_odom_state_ = current_odom_state;
}

void EKFLocalizationFuser3D::update(const GlobalPose& global_pose) {
    // 将全局定位结果转换为 EKF 的状态向量格式 z
    Eigen::Vector3d euler_angles = rotationToEuler(global_pose.orientation.toRotationMatrix());
    StateVector z = poseToVector(global_pose.position, euler_angles);

    // 计算新息 y = z - H * x
    StateVector y = z - H_ * state_;
    y(3) = normalizeAngle(y(3)); y(4) = normalizeAngle(y(4)); y(5) = normalizeAngle(y(5));

    // 计算新息协方差 S = H * P * H^T + R
    StateMatrix S = H_ * P_ * H_.transpose() + R_;

    // 一致性检查 (Gating)
    double mahalanobis_sq = y.transpose() * S.inverse() * y;
    if (mahalanobis_sq > gating_threshold_) {
        std::cout << "  -> [Warning] Global pose failed consistency check. Skipping update. (dist_sq=" << mahalanobis_sq << ")" << std::endl;
        return;
    }

    // 计算卡尔曼增益 K
    StateMatrix K = P_ * H_.transpose() * S.inverse();

    // 更新状态
    state_ = state_ + K * y;
    state_(3) = normalizeAngle(state_(3)); state_(4) = normalizeAngle(state_(4)); state_(5) = normalizeAngle(state_(5));

    // 更新协方差
    P_ = (I_ - K * H_) * P_;
}

GlobalPose EKFLocalizationFuser3D::getFusedPose() const {
    GlobalPose pose;
    pose.position = state_.head<3>();
    Eigen::Matrix3d R = eulerToRotation(state_.tail<3>());
    pose.orientation = Eigen::Quaterniond(R);
    return pose;
}

EKFLocalizationFuser3D::StateVector EKFLocalizationFuser3D::getStateVector() const {
    return state_;
}

double EKFLocalizationFuser3D::normalizeAngle(double angle) {
    while (angle > PI) angle -= 2.0 * PI;
    while (angle < -PI) angle += 2.0 * PI;
    return angle;
}

EKFLocalizationFuser3D::StateVector EKFLocalizationFuser3D::poseToVector(const Eigen::Vector3d& position, const Eigen::Vector3d& euler_angles) {
    StateVector vec;
    vec << position(0), position(1), position(2), euler_angles(0), euler_angles(1), euler_angles(2);
    return vec;
}

Eigen::Vector3d EKFLocalizationFuser3D::rotationToEuler(const Eigen::Matrix3d& R) {
    return R.eulerAngles(0, 1, 2); // Roll, Pitch, Yaw
}

Eigen::Matrix3d EKFLocalizationFuser3D::eulerToRotation(const Eigen::Vector3d& euler) const {
    Eigen::AngleAxisd rollAngle(euler(0), Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(euler(1), Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(euler(2), Eigen::Vector3d::UnitZ());
    // Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return q.matrix();
}