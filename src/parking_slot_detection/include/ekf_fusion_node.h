//20250701修改
#ifndef EKF_FUSION_NODE_H
#define EKF_FUSION_NODE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> 
#include <Eigen/Dense>
#include <tf/tf.h>
#include "ekf_fuser_3d.h" // 引入您的核心 EKF 融合器
#include <Eigen/Geometry>  

/**
 * @class EKFFusionNode
 * @brief 封装了 EKF 融合器的 ROS 节点。
 * 订阅里程计和全局位姿话题，发布融合后的位姿。
 */
class EKFFusionNode {
public:
    /**
     * @brief 构造函数
     * @param nh ROS 节点句柄
     */
    explicit EKFFusionNode(ros::NodeHandle& nh);

private:
    /**
     * @brief 里程计话题的回调函数
     * @param msg 从话题接收到的 Odometry 消息
     */
    void viodomCallback(const nav_msgs::Odometry::ConstPtr& msg);

    /**
     * @brief 全局位姿话题的回调函数
     * @param msg 从话题接收到的 PoseWithCovarianceStamped 消息
     */
    void globalPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

    /**
     * @brief 发布融合后的位姿和 TF 变换
     */
    void publishFusedPose();

    // ROS 相关句柄
    ros::NodeHandle nh_;
    ros::Subscriber sub_odom_;
    ros::Subscriber sub_global_pose_;
    ros::Publisher pub_fused_odom_;

    // 核心 EKF 融合器实例
    std::unique_ptr<EKFLocalizationFuser3D> fuser_;

    // 坐标系框架 ID
    std::string odom_frame_id_;
    std::string base_frame_id_;
};

#endif // EKF_FUSION_NODE_H
