#ifndef RVIZSHOW_H
#define RVIZSHOW_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Polygon.h>
#include <visualization_msgs/Marker.h>
#include <tf/tf.h>
#include "pose.h"
#include "associate.h"
#include <Eigen/Dense>
#include"ekfodom.h"
#include <eigen_conversions/eigen_msg.h>
#include <eigen3/Eigen/Geometry>
#include <queue>
#include <thread>
#include <mutex>
using namespace Eigen;

class RvizDisplay {
public:
    ros::Publisher path_pub;
    ros::Publisher slots_pub;
    ros::Publisher plates_pub;
    ros::Publisher map_pub;
     ros::Publisher odom_pub;
    ros::Publisher camera_pub;
     ros::Publisher match_pub;
    ros::Publisher ekfodom_pub;
    ros::Publisher ekfodom_new_pub;
    ros::Publisher ekfpath_new_pub;
    ros::Publisher localizationmap;
    // 存储车辆位姿的路径
    std::vector<geometry_msgs::PoseStamped> pose_path;
    
    nav_msgs::Path path_msg;
    nav_msgs::Path odompath_msg;
    nav_msgs::Path ekfodom_msg;
    nav_msgs::Path ekfodom_new_msg;
    // nav_msgs::Odometry odom_msg;
    geometry_msgs::PoseStamped pose_stamped;
    geometry_msgs::PoseStamped odompose_stamped;
    geometry_msgs::PoseStamped ekfodom_stamped;
    geometry_msgs::PoseStamped ekfodom_new_stamped;
    void RvizDisplay_init(ros::NodeHandle& nh);


    void displayParkingSpots(const std::vector<AssociatedPair>& worldassociatedPairs);
   void displayPlate(const std::vector<LicensePlate>& plates);
    void displaymap(const AssociatedPair& mapPairs);
    void publishPosePath(const PoseData& new_pose);
    void publishodomPath(const Eigen::Vector3d & position);
    void publishekfodom(const Eigen::VectorXd &state);
    void publishekfodom_new(const State &state_);
    void visualizeMapPoints(const std::vector<AssociatedPair>& mapPoints);
    void matchdisplay(const std::vector<Eigen::Vector3d>  matched_points);
};

#endif // RVIZSHOW_H