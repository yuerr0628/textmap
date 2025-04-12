#ifndef POSE_H
#define POSE_H

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <fstream>

struct PoseData {
    double x;  // 纬度
    double y; // 经度
    double z;  // 海拔
    double yaw;       // 偏航角（度）
    double timestamp;
};

class VehiclePose {
public:
    VehiclePose(ros::NodeHandle& nh);
    PoseData pose; 
    std::vector<PoseData> pose_path;
    
    ros::Subscriber gps_sub;
    ros::Subscriber imu_sub;
    // ros::Publisher path_pub;

    // 回调函数
    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg);
    void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg);
    // void odomCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg, const sensor_msgs::Imu::ConstPtr& imu_msg);

    // 存储车辆位置和朝向
    geometry_msgs::PoseStamped vehicle_pose_;
};
void odomCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg, const sensor_msgs::Imu::ConstPtr& imu_msg);

#endif // VEHICLE_POSE_H