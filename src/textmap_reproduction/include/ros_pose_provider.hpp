#pragma once

#include "textmap_types.hpp"

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

namespace textmap {

class RosPoseProvider {
 public:
  explicit RosPoseProvider(ros::NodeHandle& nh);

  bool hasPose() const;
  Pose2 currentPose() const;
  double lastTimestamp() const;

 private:
  ros::Subscriber gps_sub_;
  ros::Subscriber imu_sub_;

  bool initialized_origin_ = false;
  bool has_pose_ = false;
  double origin_lat_ = 0.0;
  double origin_lon_ = 0.0;
  double yaw_ = 0.0;
  double timestamp_ = 0.0;
  Pose2 pose_;

  void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
  void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
};

}  // namespace textmap
