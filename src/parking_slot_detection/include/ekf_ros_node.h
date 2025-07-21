#ifndef EKF_ROS_NODE_H
#define EKF_ROS_NODE_H

// 包含 EKF 类定义 (你需要确保这个头文件存在并包含 EKFLocalization3D 的定义)
#include "ekf_localization_3d.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h> // 如果在类定义或成员变量中需要，保留
#include <nav_msgs/Odometry.h>
// tf2 和 Eigen 通常只在实现中需要，但如果成员变量使用了 Eigen 类型，则 Eigen 头文件必须包含
#include <Eigen/Dense>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // 用于转换




// --- ROS 节点类定义 ---
class EKFNode {
public:
    EKFNode(ros::NodeHandle& nh); // 构造函数声明
    ~EKFNode(); // 析构函数声明

private:
    // 成员变量声明
    ros::NodeHandle nh_;
    EKFLocalization3D* ekf_;

    ros::Subscriber odom_sub_;
    ros::Subscriber global_pose_sub_;
    // ros::Subscriber initial_pose_sub_; // 如果不使用，可以移除
    ros::Publisher ekf_pose_pub_;

    EKFLocalization3D::StateMatrix Q_;
    EKFLocalization3D::StateMatrix R_;
    double mahalanobis_threshold_;

    ros::Time last_odom_time_;
    bool first_odom_received_;
    ros::Time last_global_pose_received_time_;
    double global_pose_timeout_sec_;
    bool allow_reinitialization_by_global_pose_timeout_;
    std::string output_frame_id_="world";

    ros::Timer check_global_pose_timer_;

    // 成员函数声明
    void loadParams();
    void checkGlobalPoseTimeoutCallback(const ros::TimerEvent& event);
    // void initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg); // 如果不使用，可以移除
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);
    void globalPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    void publishEKFPose(const ros::Time& stamp, const std::string& frame_id, bool is_odometry_only_prediction = false);
};

#endif // EKF_ROS_NODE_H