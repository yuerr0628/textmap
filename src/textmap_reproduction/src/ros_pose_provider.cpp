#include "ros_pose_provider.hpp"

#include <tf/tf.h>

#include <cmath>

namespace textmap {
namespace {

constexpr double kEarthRadiusMeters = 6378137.0;

double deg2rad(double deg) { return deg * M_PI / 180.0; }

}  // namespace

RosPoseProvider::RosPoseProvider(ros::NodeHandle& nh) {
  std::string gps_topic = "/Inertial/gps/fix";
  std::string imu_topic = "/Inertial/imu/data";
  nh.param<std::string>("gps_topic", gps_topic, gps_topic);
  nh.param<std::string>("imu_topic", imu_topic, imu_topic);

  gps_sub_ = nh.subscribe(gps_topic, 50, &RosPoseProvider::gpsCallback, this);
  imu_sub_ = nh.subscribe(imu_topic, 100, &RosPoseProvider::imuCallback, this);
}

bool RosPoseProvider::hasPose() const { return has_pose_; }

Pose2 RosPoseProvider::currentPose() const { return pose_; }

double RosPoseProvider::lastTimestamp() const { return timestamp_; }

void RosPoseProvider::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
  if (!initialized_origin_) {
    origin_lat_ = msg->latitude;
    origin_lon_ = msg->longitude;
    initialized_origin_ = true;
    return;
  }

  const double dlat = deg2rad(msg->latitude - origin_lat_);
  const double dlon = deg2rad(msg->longitude - origin_lon_);
  const double mean_lat = deg2rad((msg->latitude + origin_lat_) * 0.5);

  const double north = dlat * kEarthRadiusMeters;
  const double east = dlon * kEarthRadiusMeters * std::cos(mean_lat);

  pose_.x = east;
  pose_.y = north;
  pose_.yaw = yaw_;
  timestamp_ = msg->header.stamp.toSec();
  has_pose_ = true;
}

void RosPoseProvider::imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
  if (msg->orientation_covariance[0] < 0.0) return;
  tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
  double roll = 0.0;
  double pitch = 0.0;
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw_);
  pose_.yaw = yaw_;
}

}  // namespace textmap
