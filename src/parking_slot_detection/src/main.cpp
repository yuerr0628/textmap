#include "pose.h"
#include "drawmap.h"
#include "associate.h"
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <opencv2/opencv.hpp>
#include "rvizshow.h"
#include "ekfodom.h"
#include "loop_closing.h"
#include "p3p.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "vehicle_pose");
    ros::NodeHandle nh;

    // VehiclePose vehiclePose; 
    // LoopClosing loopClosing(nh,"/data/yhy/map0723.yaml");
    // 订阅GPS和IMU话题
    // Odometry Odometry(nh);
    // ros::Subscriber gps_sub = nh.subscribe("/Inertial/gps/fix", 10, VehiclePose::gpsCallback); 
      // Affine parameters

    // VehiclePose VehiclePose(nh);
    // VehiclePose VehiclePose(nh);
    
    AssociatedParkingInfo AssociatedParkingInfo(nh);
    // mydisplay.RvizDisplay(nh);

    
    ros::spin();
    return 0;
}