#include "ekf_fusion_node.h"

EKFFusionNode::EKFFusionNode(ros::NodeHandle& nh) : nh_(nh) {
    // --- 参数初始化 ---
    // 从参数服务器获取话题名称和坐标系名称，如果未设置则使用默认值
    // std::string odom_topic, global_pose_topic, fused_odom_topic;
    // nh_.param<std::string>("viodom_topic", odom_topic, "/viodom");
    // nh_.param<std::string>("global_pose_topic", global_pose_topic, "/global_pose");
    // nh_.param<std::string>("fused_odom_topic", fused_odom_topic, "/odom/fused");
    // nh_.param<std::string>("odom_frame_id", odom_frame_id_, "odom");
    // nh_.param<std::string>("base_frame_id", base_frame_id_, "base_link");

    // --- 初始化 EKF 融合器 ---
    // 使用一个零位姿进行初始化。EKF 会在收到第一个里程计数据时获取其实际初始位姿。
    EKFLocalizationFuser3D::StateVector initial_state;
   initial_state << 0, 0, 0, 0, 0, 0.96;
    fuser_ = std::make_unique<EKFLocalizationFuser3D>(initial_state);
    ROS_INFO("EKF Fusion core initialized.");

    // --- 设置 ROS 订阅者和发布者 ---
    sub_odom_ = nh_.subscribe("ekf_odom", 10, &EKFFusionNode::viodomCallback, this);
    sub_global_pose_ = nh_.subscribe("global_pose", 5, &EKFFusionNode::globalPoseCallback, this);
    pub_fused_odom_ = nh_.advertise<nav_msgs::Odometry>("fused_odom_topic", 10);

    ROS_INFO("EKF Fusion Node is ready.");
}

void EKFFusionNode::viodomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // 将 ROS Odometry 消息转换为 OdomState 结构体
  // **FIXED**: 直接将 ROS 的 Quaternion 消息转换为 Eigen 的旋转矩阵
  
  OdomState current_odom;
   
     current_odom.p.x() = msg->pose.pose.position.x;
    current_odom.p.y() = msg->pose.pose.position.y;
    current_odom.p.z() = msg->pose.pose.position.z;
    //  std::cout<<"predict curstate_------------:"<<current_odom.p<<std::endl;
    Eigen::Quaterniond q_eigen(msg->pose.pose.orientation.w,
                               msg->pose.pose.orientation.x,
                               msg->pose.pose.orientation.y,
                               msg->pose.pose.orientation.z);
    current_odom.R_q = q_eigen.toRotationMatrix();
    Eigen::Vector3d euler = q_eigen.toRotationMatrix().eulerAngles(2, 1, 0); // ZYX顺序: yaw, pitch, roll
double yaw = euler[0];  // 弧度制
//  std::cout<<"yaw------------:"<<yaw<<std::endl;

    // 调用 EKF 的 predict 步骤
    fuser_->predict(current_odom);

    // 每次预测后都发布当前最优估计
    publishFusedPose();
}

void EKFFusionNode::globalPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    // 将 ROS PoseWithCovarianceStamped 消息转换为 GlobalPose 结构体
    GlobalPose global_pose;
    global_pose.position.x() = msg->pose.pose.position.x;
    global_pose.position.y() = msg->pose.pose.position.y;
    global_pose.position.z() = msg->pose.pose.position.z;

    // **FIXED**: 直接将 ROS 的 Quaternion 消息转换为 Eigen 的 Quaternion
    global_pose.orientation = Eigen::Quaterniond(msg->pose.pose.orientation.w,
                                                 msg->pose.pose.orientation.x,
                                                 msg->pose.pose.orientation.y,
                                                 msg->pose.pose.orientation.z);
    std::cout<<"global curstate_------------:"<<global_pose.position<<std::endl;
    // 调用 EKF 的 update 步骤
    fuser_->update(global_pose);
    
    // 更新后发布最优估计
    // (注意：在实际应用中，您可能只想在 odomCallback 中发布以保持固定频率)
    publishFusedPose();
}

void EKFFusionNode::publishFusedPose() {
    // 获取融合后的位姿
    GlobalPose fused_pose = fuser_->getFusedPose();

    // 创建要发布的 Odometry 消息
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = ros::Time::now();
    odom_msg.header.frame_id = "world";
    odom_msg.child_frame_id = "world";

    // 填充位姿信息
    odom_msg.pose.pose.position.x = fused_pose.position.x();
    odom_msg.pose.pose.position.y = fused_pose.position.y();
    odom_msg.pose.pose.position.z = fused_pose.position.z();
    // **FIXED**: 直接将 Eigen 的 Quaternion 转换为 ROS 的 Quaternion 消息
    odom_msg.pose.pose.orientation.w = fused_pose.orientation.w();
    odom_msg.pose.pose.orientation.x = fused_pose.orientation.x();
    odom_msg.pose.pose.orientation.y = fused_pose.orientation.y();
    odom_msg.pose.pose.orientation.z = fused_pose.orientation.z();
    
    // (可选) 填充协方差
    // EKFLocalizationFuser3D::StateMatrix P = fuser_->getCovariance();
    // for (int i = 0; i < 6; ++i) {
    //     for (int j = 0; j < 6; ++j) {
    //         odom_msg.pose.covariance[i * 6 + j] = P(i, j);
    //     }
    // }

    pub_fused_odom_.publish(odom_msg);
}
