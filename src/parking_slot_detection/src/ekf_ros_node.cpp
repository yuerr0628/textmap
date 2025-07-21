#include "ekf_ros_node.h" 

#include <cmath> // For M_PI, though <math.h> might be more standard C for M_PI


// --- ROS 节点类实现 ---

EKFNode::EKFNode(ros::NodeHandle& nh) : nh_(nh), ekf_(nullptr), first_odom_received_(false) { // 初始化成员变量
    loadParams();
    ekf_ = new EKFLocalization3D(Q_, R_, mahalanobis_threshold_);

    odom_sub_ = nh_.subscribe("ekf_odom", 10, &EKFNode::odomCallback, this);
    global_pose_sub_ = nh_.subscribe("global_pose", 5, &EKFNode::globalPoseCallback, this);
    // initial_pose_sub_ = nh_.subscribe("initial_pose", 1, &EKFNode::initialPoseCallback, this);
    ekf_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("ekf_pose", 10);

    if (allow_reinitialization_by_global_pose_timeout_) {
        double timer_check_interval = 1.0;
        nh_.param("global_pose_timer_check_interval", timer_check_interval, 1.0);
        check_global_pose_timer_ = nh_.createTimer(ros::Duration(timer_check_interval), &EKFNode::checkGlobalPoseTimeoutCallback, this);
        ROS_INFO("Global pose timeout check enabled for EKF re-initialization (Interval: %.1f s, Timeout: %.1f s).", timer_check_interval, global_pose_timeout_sec_);
    } else {
        ROS_INFO("Global pose timeout check for EKF re-initialization is disabled.");
    }
    ROS_INFO("EKF ROS Node constructed. Waiting for initial pose or first global pose...");
}

EKFNode::~EKFNode() {
    if (ekf_) {
        delete ekf_;
        ekf_ = nullptr;
    }
}

void EKFNode::loadParams() {
    Q_ = EKFLocalization3D::StateMatrix::Identity() * 0.01;
    R_ = EKFLocalization3D::StateMatrix::Identity() * 0.1;
    mahalanobis_threshold_ = 12.592;
    global_pose_timeout_sec_ = 10.0;
    allow_reinitialization_by_global_pose_timeout_ = true;

    nh_.param("allow_reinitialization_by_global_pose_timeout", allow_reinitialization_by_global_pose_timeout_, true);
    nh_.param("global_pose_timeout_seconds", global_pose_timeout_sec_, 10.0);
    nh_.param<std::string>("ekf_output_frame_id", output_frame_id_, "world");


    std::vector<double> q_list;
    // 注意：从 YAML 加载 Eigen Matrix 的更健壮方法是逐元素或使用专门的库。
    // 这里的代码假定参数服务器中的列表顺序与矩阵元素对应。
    if (nh_.getParam("ekf_process_noise_q", q_list) && q_list.size() == 36) {
        for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j) Q_(i, j) = q_list[i * 6 + j];
        ROS_INFO("Loaded process noise Q from parameter server.");
    } else {
        ROS_WARN("Failed to load 'ekf_process_noise_q'. Using default Q.");
        Q_(0,0)=0.05*0.05; Q_(1,1)=0.05*0.05; Q_(2,2)=0.02*0.02;
        Q_(3,3)=(0.3*M_PI/180.0)*(0.3*M_PI/180.0); Q_(4,4)=(0.3*M_PI/180.0)*(0.3*M_PI/180.0); Q_(5,5)=(0.5*M_PI/180.0)*(0.5*M_PI/180.0);
    }
    // std::cout << "Using Process Noise Q:\n" << Q_ << std::endl;

    std::vector<double> r_list;
    if (nh_.getParam("ekf_measurement_noise_r", r_list) && r_list.size() == 36) {
        for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j) R_(i, j) = r_list[i * 6 + j];
        ROS_INFO("Loaded measurement noise R from parameter server.");
    } else {
        ROS_WARN("Failed to load 'ekf_measurement_noise_r'. Using default R.");
        R_(0,0)=0.1*0.1; R_(1,1)=0.1*0.1; R_(2,2)=0.1*0.1;
        R_(3,3)=(1.0*M_PI/180.0)*(1.0*M_PI/180.0); R_(4,4)=(1.0*M_PI/180.0)*(1.0*M_PI/180.0); R_(5,5)=(2.0*M_PI/180.0)*(2.0*M_PI/180.0);
    }
    // std::cout << "Using Measurement Noise R:\n" << R_ << std::endl;

    nh_.param("ekf_mahalanobis_threshold", mahalanobis_threshold_, 12.592);
    ROS_INFO("Using Mahalanobis Threshold (squared): %f", mahalanobis_threshold_);
    ROS_INFO("EKF output frame ID: %s", output_frame_id_.c_str());
}

void EKFNode::checkGlobalPoseTimeoutCallback(const ros::TimerEvent& event) {
    if (!ekf_ || !allow_reinitialization_by_global_pose_timeout_) return;

    // 只有当EKF当前是“已初始化”状态时，才检查是否因为超时而需要将其设为“未初始化”
    if (ekf_->isInitialized()) {
        if (!last_global_pose_received_time_.isZero() &&
            (ros::Time::now() - last_global_pose_received_time_).toSec() > global_pose_timeout_sec_) {
            ROS_WARN("Global pose timeout of %.1f seconds exceeded. EKF is being marked as uninitialized for re-initialization.", global_pose_timeout_sec_);
            ekf_->setUninitialized();
            last_global_pose_received_time_ = ros::Time(0); // 重置，避免立即再次触发
        }
    }
}

// 如果你使用了 initialPoseCallback，将它移到这里并加上 EKFNode:: 前缀
// void EKFNode::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
//     if (!ekf_) return;
//     // ... implementation ...
// }


void EKFNode::odomCallback(const nav_msgs::OdometryConstPtr& msg) {
    if (!ekf_) return;

    ros::Time current_time = msg->header.stamp;

    if (!first_odom_received_) {
        last_odom_time_ = current_time;
        first_odom_received_ = true;
        // 首次收到里程计，不进行预测，只更新时间戳
        return; // 在第一次收到里程计时，不进行predict，避免使用0的dt
    }

    double dt = (current_time - last_odom_time_).toSec();
    // 只有在非首次接收时，dt才可能有效。首次接收时dt可能是0或基于未初始化的last_odom_time_。
    if (dt <= 1e-6) { // Use a very small threshold to account for floating point issues
         // ROS_WARN_THROTTLE(1.0, "Odometry dt is too small or non-positive (%.4f). Skipping prediction. Current: %.3f, Last: %.3f",
         //                  dt, current_time.toSec(), last_odom_time_.toSec());
        last_odom_time_ = current_time; // 仍然更新时间戳
        return;
    }

    OdometryDelta3D odom_delta;
    odom_delta.delta_x = msg->twist.twist.linear.x * dt;
    odom_delta.delta_y = msg->twist.twist.linear.y * dt;
    odom_delta.delta_z = msg->twist.twist.linear.z * dt;
    odom_delta.delta_roll = msg->twist.twist.angular.x * dt;
    odom_delta.delta_pitch = msg->twist.twist.angular.y * dt;
    odom_delta.delta_yaw = msg->twist.twist.angular.z * dt;

    ekf_->predict(odom_delta);
    last_odom_time_ = current_time;


    publishEKFPose(msg->header.stamp, output_frame_id_, !ekf_->isInitialized() /*is_odometry_only_prediction*/);

}

void EKFNode::globalPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
    if (!ekf_) return;

    last_global_pose_received_time_ = msg->header.stamp; // 总是更新此时间戳
    

    Pose3D global_pose_measurement;
    global_pose_measurement.x = msg->pose.pose.position.x;
    global_pose_measurement.y = msg->pose.pose.position.y;
    global_pose_measurement.z = msg->pose.pose.position.z;
    tf2::Quaternion q_tf2; tf2::fromMsg(msg->pose.pose.orientation, q_tf2); tf2::Matrix3x3 m_tf2(q_tf2);
    m_tf2.getRPY(global_pose_measurement.roll, global_pose_measurement.pitch, global_pose_measurement.yaw);

    // 获取测量值的协方差
    EKFLocalization3D::StateMatrix measurement_covariance = EKFLocalization3D::StateMatrix::Zero();
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            measurement_covariance(i, j) = msg->pose.covariance[i * 6 + j];
        }
    }
    // 检查对角线元素是否接近零，并替换为小值以避免数学问题
    for(int i=0; i<6; ++i) {
        if(std::abs(measurement_covariance(i,i)) < 1e-9) {
             measurement_covariance(i,i) = 1e-3; // Using a small value instead of 1e-9
             ROS_WARN_THROTTLE(1.0, "Global pose covariance diagonal element %d from measurement was near zero. Using 1e-3.", i);
        }
    }


    if (!ekf_->isInitialized()) {
        ROS_INFO("EKF not initialized. Attempting initialization with /global_pose message.");
        // 使用测量值的协方差作为初始协方差
        ekf_->initialize(global_pose_measurement, measurement_covariance);
        ROS_INFO("EKF initialized/re-initialized with pose from /global_pose.");
        first_odom_received_ = false; // 重置里程计，以便dt从新起点计算
        // 初始化后，通常会立即发布一次这个初始化的位姿
        publishEKFPose(msg->header.stamp, output_frame_id_, false); // 初始化通常是融合结果
        return; // 此消息用于初始化，不需要进行 update
    }

    // EKF已初始化，执行更新
    // 在进行 update 之前，你可以选择根据 Mahalanobis 距离进行门控，以拒绝异常测量。
    // EKFLocalization3D::StateVector innovation = ekf_->getInnovation(global_pose_measurement);
    // EKFLocalization3D::StateMatrix innovation_covariance = ekf_->getInnovationCovariance();
    // double mahalanobis_sq = innovation.transpose() * innovation_covariance.inverse() * innovation;

    // if (mahalanobis_sq < mahalanobis_threshold_) {
        ekf_->update(global_pose_measurement);
        publishEKFPose(msg->header.stamp, output_frame_id_, false /*is_odometry_only_prediction=false*/);
    // } else {
    //     ROS_WARN_THROTTLE(1.0, "Global pose measurement rejected by Mahalanobis threshold (%.2f > %.2f).", mahalanobis_sq, mahalanobis_threshold_);
    //     // 测量被拒绝时，只发布当前的预测结果（这是在globalPoseCallback之前收到的最新odomCallback中进行的预测）
    //     // publishEKFPose(msg->header.stamp, output_frame_id_, !ekf_->isInitialized()); // EKF 此时是 Initialized 的，所以这里是 false
    //     // 注意：如果测量被拒绝，最新的发布应该是上一个 OdomCallback 的预测。
    //     // 如果你想在拒绝测量时也发布一下 EKF 的状态（尽管没有更新），你可以在这里调用一次 publishEKFPose，
    //     // 但要注意避免重复发布（因为 OdomCallback 可能也发布了）。
    //     // 一个简单的做法是：只在接收并使用测量进行 update 后才在这里发布。如果拒绝了，就不在这里发布。
    // }
}

void EKFNode::publishEKFPose(const ros::Time& stamp, const std::string& frame_id, bool is_odometry_only_prediction) {
    if (!ekf_) return;

    // 检查是否可以发布融合结果。如果是纯里程计预测，则不受此限制。
    if (!is_odometry_only_prediction && !ekf_->isInitialized()){
        ROS_WARN_THROTTLE(1.0, "Attempting to publish fused EKF pose, but EKF is not initialized. Skipping.");
        return;
    }

    // 根据是否是纯里程计预测来确定发布的坐标系。
    // 如果 EKF 未初始化 (即 is_odometry_only_prediction 为 true)，通常发布在里程计坐标系 (如 "odom")。
    // 如果 EKF 已初始化并融合了全局定位 (即 is_odometry_only_prediction 为 false)，通常发布在全局坐标系 (如 "map")。
    std::string publish_frame_id;
    if (is_odometry_only_prediction) {
        // 你需要获取里程计消息的 frame_id。如果你的 odomCallback 不保存 frame_id，
        // 你可能需要修改 odomCallback 来保存它，或者约定一个默认的里程计 frame_id。
        // 为了简单，这里暂时仍然使用 output_frame_id_，但这可能不准确。
        // 正确的做法是：如果 !ekf_->isInitialized()，使用里程计消息的 frame_id；如果 ekf_->isInitialized()，使用 output_frame_id_。
         // 假设我们约定未初始化时发布到 "odom" frame (或者从里程计消息中获取)。
        // publish_frame_id = "odom"; // 假设里程计 frame 是 "odom"
        publish_frame_id = frame_id; // 保持原逻辑，使用传入的frame_id (通常是 output_frame_id_)
         ROS_WARN_THROTTLE(1.0, "Publishing EKF state based SOLELY on odometry prediction (global pose unavailable/lost or EKF not yet initialized with global pose). Covariance will grow. Frame ID: %s", publish_frame_id.c_str());

    } else {
        publish_frame_id = output_frame_id_; // 发布融合结果到全局坐标系
        // ROS_INFO_THROTTLE(1.0, "Publishing fused EKF pose. Frame ID: %s", publish_frame_id.c_str());
    }


    geometry_msgs::PoseWithCovarianceStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = publish_frame_id;

    EKFLocalization3D::StateVector current_state = ekf_->getStateVector();
    EKFLocalization3D::StateMatrix current_covariance = ekf_->getCovariance();

    pose_msg.pose.pose.position.x = current_state(0);
    pose_msg.pose.pose.position.y = current_state(1);
    pose_msg.pose.pose.position.z = current_state(2);
    tf2::Quaternion q_tf2_pub; q_tf2_pub.setRPY(current_state(3), current_state(4), current_state(5));
    pose_msg.pose.pose.orientation = tf2::toMsg(q_tf2_pub);
    for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j) pose_msg.pose.covariance[i * 6 + j] = current_covariance(i, j);

    ekf_pose_pub_.publish(pose_msg);
}