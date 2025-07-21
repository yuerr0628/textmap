#include "ekf_localization_3d.h"
#include <Eigen/Geometry>
#include <limits>

// --- EKFLocalization3D 类实现 ---

EKFLocalization3D::EKFLocalization3D(const StateMatrix& process_noise_q,
                                     const StateMatrix& measurement_noise_r,
                                     double outlier_mahalanobis_threshold)
    : Q_(process_noise_q),
      R_(measurement_noise_r),
      mahalanobis_threshold_(outlier_mahalanobis_threshold),
      initialized_(false) // 初始时未初始化
{
      state_.setZero(); // 将状态向量重置为零
    covariance_.setIdentity() * 1e6; // 将协方差重置为表示高度不确定性的单位矩阵
    // state_ 和 covariance_ 在 setUninitialized 中或首次构造时被赋予初始值
    // setUninitialized(); // 确保构造后处于明确的未初始化状态
    std::cout << "3D EKF instance created. Defaulting to uninitialized state." << std::endl;
}

void EKFLocalization3D::initialize(const Pose3D& initial_pose, const StateMatrix& initial_covariance) {
    state_ = initial_pose.toVector();
    state_(3) = normalizeAngle(state_(3)); // Roll
    state_(4) = normalizeAngle(state_(4)); // Pitch
    state_(5) = normalizeAngle(state_(5)); // Yaw
    covariance_ = initial_covariance;
    initialized_ = true; // 标记为已初始化
    std::cout << "3D EKF Initialized. Initial state: x=" << state_(0) << ", y=" << state_(1) << ", z=" << state_(2)
              << ", roll=" << state_(3) << ", pitch=" << state_(4) << ", yaw=" << state_(5) << std::endl;
    // std::cout << "Initial Covariance:\n" << covariance_ << std::endl;
}

bool EKFLocalization3D::isInitialized() const {
    return initialized_;
}

void EKFLocalization3D::setUninitialized() {
    initialized_ = false; // 标记为未初始化
    // state_.setZero(); // 将状态向量重置为零
    covariance_.setIdentity() * 1e6; // 将协方差重置为表示高度不确定性的单位矩阵
    std::cout << "3D EKF has been set to uninitialized state. Ready for (re-)initialization." << std::endl;
}

Eigen::Matrix3d EKFLocalization3D::getRotationMatrix() const {
    double r = state_(3); // roll from current state
    double p = state_(4); // pitch from current state
    double y = state_(5); // yaw from current state

    double cy = cos(y); double sy = sin(y);
    double cp = cos(p); double sp = sin(p);
    double cr = cos(r); double sr = sin(r);

    Eigen::Matrix3d R_mat;
    R_mat(0,0) = cy*cp;
    R_mat(0,1) = cy*sp*sr - sy*cr;
    R_mat(0,2) = cy*sp*cr + sy*sr;
    R_mat(1,0) = sy*cp;
    R_mat(1,1) = sy*sp*sr + cy*cr;
    R_mat(1,2) = sy*sp*cr - cy*sr;
    R_mat(2,0) = -sp;
    R_mat(2,1) = cp*sr;
    R_mat(2,2) = cp*cr;
    return R_mat;
}

void EKFLocalization3D::predict(const OdometryDelta3D& odom) {
    // 预测步骤总是执行，无论是否已初始化。
    // 如果未初始化 (initialized_ == false)，state_ 可能是零（来自setUninitialized）
    // 或上一次有效状态（如果setUninitialized保留了状态）。
    // 当前实现中，setUninitialized会将state_清零。
    // 这意味着如果EKF从未被初始化，它将从(0,0,0,0,0,0)开始预测。
    // 如果EKF曾被初始化然后被setUninitialized，它也将从(0,0,0,0,0,0)开始预测。
    // 如果希望从上一个已知位姿继续推算，setUninitialized不应清零state_，但这会增加复杂性。

    const StateVector current_state = state_; // 使用当前的 state_ 进行预测
    const double x = current_state(0);
    const double y = current_state(1);
    const double z = current_state(2);
    const double roll = current_state(3);
    const double pitch = current_state(4);
    const double yaw = current_state(5);

    const double dx_body = odom.delta_x;
    const double dy_body = odom.delta_y;
    const double dz_body = odom.delta_z;
    const double droll = odom.delta_roll;
    const double dpitch = odom.delta_pitch;
    const double dyaw = odom.delta_yaw;

    StateVector predicted_state;
    Eigen::Vector3d delta_pos_body(dx_body, dy_body, dz_body);
    Eigen::Matrix3d R_world_body = getRotationMatrix(); // 使用当前state_的姿态计算旋转矩阵
    Eigen::Vector3d delta_pos_world = R_world_body * delta_pos_body;

    predicted_state(0) = x + delta_pos_world(0);
    predicted_state(1) = y + delta_pos_world(1);
    predicted_state(2) = z + delta_pos_world(2);
    predicted_state(3) = normalizeAngle(roll + droll);
    predicted_state(4) = normalizeAngle(pitch + dpitch);
    predicted_state(5) = normalizeAngle(yaw + dyaw);

    StateMatrix F = StateMatrix::Identity();
    // 雅可比矩阵 F 的计算 (简化版，主要考虑yaw对xy的影响)
    double cy_pred = cos(predicted_state(5)); // 使用预测后的yaw角，或当前yaw角，取决于模型细节
    double sy_pred = sin(predicted_state(5));
    double cp_pred = cos(predicted_state(4));
    // double sp_pred = sin(predicted_state(4));
    double cr_pred = cos(predicted_state(3));
    double sr_pred = sin(predicted_state(3));


    // 使用当前姿态（预测前）的yaw角来计算雅可比矩阵F中姿态对位置的影响更常见
    double cy_curr = cos(yaw); double sy_curr = sin(yaw);
    double cp_curr = cos(pitch); //double sp_curr = sin(pitch);
    double sr_curr = sin(roll); double cr_curr = cos(roll);


    // d(R*delta_pos_body)/dyaw (基于当前姿态的偏导)
    double dR11_dy = -sy_curr*cp_curr;
    double dR12_dy = -sy_curr*sin(pitch)*sr_curr - cy_curr*cr_curr;
    double dR13_dy = -sy_curr*sin(pitch)*cr_curr + cy_curr*sr_curr;
    double dR21_dy = cy_curr*cp_curr;
    double dR22_dy = cy_curr*sin(pitch)*sr_curr - sy_curr*cr_curr;
    double dR23_dy = cy_curr*sin(pitch)*cr_curr + sy_curr*sr_curr;

    F(0, 5) = dR11_dy * dx_body + dR12_dy * dy_body + dR13_dy * dz_body;
    F(1, 5) = dR21_dy * dx_body + dR22_dy * dy_body + dR23_dy * dz_body;
    // F(2,5) 假设为0，因 ZYX 欧拉角定义下，世界坐标Z主要受pitch和roll影响，而非yaw

    // 预测协方差： P_k|k-1 = F * P_k-1|k-1 * F^T + Q
    // covariance_ 在这里是 P_k-1|k-1
    StateMatrix predicted_covariance = F * covariance_ * F.transpose() + Q_;

    state_ = predicted_state; // 更新状态为预测状态
    covariance_ = 0.5 * (predicted_covariance + predicted_covariance.transpose()); // 更新协方差并确保对称
}

void EKFLocalization3D::update(const Pose3D& global_pose_measurement) {
    if (!initialized_) { // 更新步骤只在已初始化后执行
        std::cerr << "Error: EKF Update called but EKF is not initialized. Skipping." << std::endl;
        return;
    }

    StateVector z = global_pose_measurement.toVector();
    z(3) = normalizeAngle(z(3));
    z(4) = normalizeAngle(z(4));
    z(5) = normalizeAngle(z(5));

    StateMatrix H = StateMatrix::Identity(); // 观测矩阵H，因为直接观测状态
    StateVector y = z - state_; // 测量残差 (innovation) state_是预测后的状态 x_k|k-1

    // 归一化角度误差
    y(3) = normalizeAngle(y(3));
    y(4) = normalizeAngle(y(4));
    y(5) = normalizeAngle(y(5));

    // 残差协方差 S = H * P_k|k-1 * H^T + R
    StateMatrix S = H * covariance_ * H.transpose() + R_;
    Eigen::LDLT<StateMatrix> ldltOfS(S); // Cholesky分解用于求解

    if(ldltOfS.info() != Eigen::Success) {
         std::cerr << "Error: Innovation covariance matrix S is not positive definite in Update. Skipping update." << std::endl;
         // std::cerr << "S matrix:\n" << S << std::endl; // 调试时可以取消注释
         return;
    }

    // 马氏距离检查，用于异常值剔除
    double mahalanobis_dist_sq = y.transpose() * ldltOfS.solve(StateMatrix::Identity()) * y;

    // if (mahalanobis_dist_sq > mahalanobis_threshold_ || !std::isfinite(mahalanobis_dist_sq)) {
    //     if (!std::isfinite(mahalanobis_dist_sq)) {
    //          std::cout << "Outlier rejected! Mahalanobis distance squared is non-finite." << std::endl;
    //     } else {
    //          std::cout << "Outlier rejected! Mahalanobis distance squared: "
    //                   << mahalanobis_dist_sq << " > threshold: " << mahalanobis_threshold_ << std::endl;
    //     }
    //     //  std::cout << "  Measurement: x=" << z(0) << ", y=" << z(1) << ", z=" << z(2)
    //     //           << ", roll=" << z(3) << ", pitch=" << z(4) << ", yaw=" << z(5) << std::endl;
    //     //  std::cout << "  Predicted:   x=" << state_(0) << ", y=" << state_(1) << ", z=" << state_(2)
    //     //           << ", roll=" << state_(3) << ", pitch=" << state_(4) << ", yaw=" << state_(5) << std::endl;
    //     return; // 跳过此次更新
    // }

    // 卡尔曼增益 K = P_k|k-1 * H^T * S^-1
    StateMatrix K = covariance_ * H.transpose() * ldltOfS.solve(StateMatrix::Identity());

    // 更新状态估计 x_k|k = x_k|k-1 + K * y
    state_ = state_ + K * y;

    // 归一化更新后的角度
    state_(3) = normalizeAngle(state_(3));
    state_(4) = normalizeAngle(state_(4));
    state_(5) = normalizeAngle(state_(5));

    // 更新协方差 P_k|k = (I - K * H) * P_k|k-1
    StateMatrix I = StateMatrix::Identity();
    covariance_ = (I - K * H) * covariance_;
    covariance_ = 0.5 * (covariance_ + covariance_.transpose()); // 确保对称性
}

Pose3D EKFLocalization3D::getCurrentPose() const {
    return Pose3D::fromVector(state_);
}

EKFLocalization3D::StateVector EKFLocalization3D::getStateVector() const {
    return state_;
}

EKFLocalization3D::StateMatrix EKFLocalization3D::getCovariance() const {
    return covariance_;
}
