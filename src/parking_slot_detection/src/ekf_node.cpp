#include "ekf_localization_3d.h" // 包含 EKF 类定义
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // 用于转换
#include <Eigen/Dense>

// --- ROS 节点类 ---
class EKFNode {
public:
    EKFNode(ros::NodeHandle& nh) : nh_(nh), ekf_(nullptr) {
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

    ~EKFNode() {
        if (ekf_) {
            delete ekf_;
            ekf_ = nullptr;
        }
    }

private:
    ros::NodeHandle nh_;
    EKFLocalization3D* ekf_;

    ros::Subscriber odom_sub_;
    ros::Subscriber global_pose_sub_;
    ros::Subscriber initial_pose_sub_;
    ros::Publisher ekf_pose_pub_;

    EKFLocalization3D::StateMatrix Q_;
    EKFLocalization3D::StateMatrix R_;
    double mahalanobis_threshold_;

    ros::Time last_odom_time_;
    bool first_odom_received_ = false; // 用于正确计算第一个dt
    ros::Time last_global_pose_received_time_;
    double global_pose_timeout_sec_;
    bool allow_reinitialization_by_global_pose_timeout_;
    std::string output_frame_id_ = "map"; // EKF输出的坐标系ID

    ros::Timer check_global_pose_timer_;

    void loadParams() {
        Q_ = EKFLocalization3D::StateMatrix::Identity() * 0.01;
        R_ = EKFLocalization3D::StateMatrix::Identity() * 0.1;
        mahalanobis_threshold_ = 12.592;
        global_pose_timeout_sec_ = 10.0;
        allow_reinitialization_by_global_pose_timeout_ = true;

        nh_.param("allow_reinitialization_by_global_pose_timeout", allow_reinitialization_by_global_pose_timeout_, true);
        nh_.param("global_pose_timeout_seconds", global_pose_timeout_sec_, 10.0);
        nh_.param<std::string>("ekf_output_frame_id", output_frame_id_, "map");


        std::vector<double> q_list;
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

    void checkGlobalPoseTimeoutCallback(const ros::TimerEvent& event) {
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

    //  void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
    //     if (!ekf_) return;
    //     if (ekf_->isInitialized()) {
    //         ROS_WARN("EKF already initialized. Ignoring message on /initial_pose.");
    //         return;
    //     }

    //     Pose3D initial_pose_val; // 使用不同的变量名以避免与成员变量混淆
    //     initial_pose_val.x = msg->pose.pose.position.x;
    //     initial_pose_val.y = msg->pose.pose.position.y;
    //     initial_pose_val.z = msg->pose.pose.position.z;
    //     tf2::Quaternion q; tf2::fromMsg(msg->pose.pose.orientation, q); tf2::Matrix3x3 m(q);
    //     m.getRPY(initial_pose_val.roll, initial_pose_val.pitch, initial_pose_val.yaw);

    //     EKFLocalization3D::StateMatrix initial_covariance_val = EKFLocalization3D::StateMatrix::Zero();
    //     for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j) initial_covariance_val(i, j) = msg->pose.covariance[i * 6 + j];
    //     for(int i=0; i<6; ++i) if(initial_covariance_val(i,i) < 1e-9) {
    //         initial_covariance_val(i,i) = 1e-9;
    //         ROS_WARN("Initial covariance diagonal element %d from /initial_pose was near zero, setting to 1e-9", i);
    //     }

    //     ekf_->initialize(initial_pose_val, initial_covariance_val);
    //     ROS_INFO("EKF initialized with pose from /initial_pose.");
    //     first_odom_received_ = false;
    //     last_global_pose_received_time_ = msg->header.stamp;
    //     // 初始化后发布一次位姿
    //     publishEKFPose(msg->header.stamp, msg->header.frame_id, false);
    // }


    void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
        if (!ekf_) return;

        ros::Time current_time = msg->header.stamp;

        if (!first_odom_received_) {
            last_odom_time_ = current_time;
            first_odom_received_ = true;
            // 即使是第一次收到里程计，如果EKF未初始化（例如，刚启动，state_为零），
            // predict会从零开始。如果希望避免发布从零开始的无效预测，
            // 可以在publishEKFPose中增加一个检查，如果state_接近零且是纯里程计，则不发布。
            // 但当前逻辑是：只要predict了，如果!isInitialized，就发布。
        }

        double dt = (current_time - last_odom_time_).toSec();
        // 只有在非首次接收时，dt才可能有效。首次接收时dt可能是0或基于未初始化的last_odom_time_。
        if (first_odom_received_ && dt <= 1e-3 && last_odom_time_ != current_time) {
             ROS_WARN_THROTTLE(1.0, "Odometry dt is too small or non-positive (%.4f). Skipping prediction. Current: %.3f, Last: %.3f",
                              dt, current_time.toSec(), last_odom_time_.toSec());
            last_odom_time_ = current_time; // 仍然更新时间戳
            return;
        }
         // 如果是首次里程计消息，dt的计算是相对于它自己的，所以dt将是0，除非我们在这里处理它
         // 上面的 first_odom_received_ 逻辑应该已经处理了首次情况，使 dt 在第二次调用时才有效。
         // 所以，如果 dt > 1e-3，则认为是有效的后续里程计。

        OdometryDelta3D odom_delta;
        odom_delta.delta_x = msg->twist.twist.linear.x * dt;
        odom_delta.delta_y = msg->twist.twist.linear.y * dt;
        odom_delta.delta_z = msg->twist.twist.linear.z * dt;
        odom_delta.delta_roll = msg->twist.twist.angular.x * dt;
        odom_delta.delta_pitch = msg->twist.twist.angular.y * dt;
        odom_delta.delta_yaw = msg->twist.twist.angular.z * dt;

        ekf_->predict(odom_delta);
        last_odom_time_ = current_time;

        if (!ekf_->isInitialized()) {
            // 只有在EKF未初始化（例如超时后，或从未被全局定位初始化过）时，才发布纯里程计预测
            publishEKFPose(msg->header.stamp, output_frame_id_, true /*is_odometry_only_prediction*/);
        }
    }

    void globalPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg) {
        if (!ekf_) return;

        last_global_pose_received_time_ = msg->header.stamp; // 总是更新此时间戳

        Pose3D global_pose_measurement;
        global_pose_measurement.x = msg->pose.pose.position.x;
        global_pose_measurement.y = msg->pose.pose.position.y;
        global_pose_measurement.z = msg->pose.pose.position.z;
        tf2::Quaternion q_tf2; tf2::fromMsg(msg->pose.pose.orientation, q_tf2); tf2::Matrix3x3 m_tf2(q_tf2);
        m_tf2.getRPY(global_pose_measurement.roll, global_pose_measurement.pitch, global_pose_measurement.yaw);

        if (!ekf_->isInitialized()) {
            ROS_INFO("EKF not initialized. Attempting initialization with /global_pose message.");
            EKFLocalization3D::StateMatrix initial_covariance_from_global = EKFLocalization3D::StateMatrix::Zero();
            for (int i = 0; i < 6; ++i) for (int j = 0; j < 6; ++j) initial_covariance_from_global(i, j) = msg->pose.covariance[i * 6 + j];
            for(int i=0; i<6; ++i) if(initial_covariance_from_global(i,i) < 1e-9) {
                initial_covariance_from_global(i,i) = 1e-3;
                ROS_WARN("Initial covariance diagonal element %d from /global_pose for init was near zero. Using 1e-3.", i);
            }
            ekf_->initialize(global_pose_measurement, initial_covariance_from_global);
            ROS_INFO("EKF initialized/re-initialized with pose from /global_pose.");
            first_odom_received_ = false; // 重置里程计，以便dt从新起点计算
            // 初始化后，通常会立即发布一次这个初始化的位姿
            publishEKFPose(msg->header.stamp, msg->header.frame_id, false);
            return; // 此消息用于初始化
        }

        // EKF已初始化，执行更新
        ekf_->update(global_pose_measurement);
        publishEKFPose(msg->header.stamp, msg->header.frame_id, false /*is_odometry_only_prediction=false*/);
    }

   void publishEKFPose(const ros::Time& stamp, const std::string& frame_id, bool is_odometry_only_prediction = false) {
        if (!ekf_) return;

        // 检查是否可以发布。如果期望融合结果但EKF未初始化，则不应发布。
        if (!is_odometry_only_prediction && !ekf_->isInitialized()){
            ROS_WARN_THROTTLE(1.0, "Attempting to publish fused EKF pose, but EKF is not initialized. Skipping.");
            return;
        }
        // 如果是纯里程计预测（因为 is_odometry_only_prediction 为 true），则总是可以发布 EKF 的当前状态，
        // 即使 ekf_->isInitialized() 为 false（表示全局定位丢失）。

        if (is_odometry_only_prediction) { // is_odometry_only_prediction implies !ekf_->isInitialized() or we are in a special mode
            ROS_WARN_THROTTLE(1.0, "Publishing EKF state based SOLELY on odometry prediction (global pose unavailable/lost or EKF not yet initialized with global pose). Covariance will grow.");
        }

        geometry_msgs::PoseWithCovarianceStamped pose_msg;
        pose_msg.header.stamp = stamp;
        pose_msg.header.frame_id = frame_id; // 通常是 output_frame_id_ ("map")

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
};

// int main(int argc, char **argv) {
//     ros::init(argc, argv, "ekf_localization_node");
//     ros::NodeHandle nh("~");
//     EKFNode ekf_node(nh);
//     ros::spin();
//     return 0;
// }
