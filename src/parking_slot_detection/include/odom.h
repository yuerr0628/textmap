#ifndef ODOM_H
#define ODOM_H

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include "parking_slot_detection/gcn_parking.h" // 假设这是包含srv定义的头文件
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <string> 
#include"pose.h"
#include <Eigen/Dense>

#include"../include/Hungarian.h"
using std::string;
using namespace Eigen;

struct Spot {
    std::vector<cv::Point2d> corners; // 使用cv::Point2d存储双精度浮点坐标
};

struct OCRtext {
    std::string text; // 车位号ID
    std::vector<cv::Point2d> textcorners; // 使用cv::Point2d存储双精度浮点坐标
};

struct Spotwithtext {
    Spot spot; // 关联的车位信息
    OCRtext ocrPoint; // 关联的车位号信息
    int ID; // 帧id
};

struct Pose {
    Eigen::Matrix3d R; // 3x3 旋转矩阵
    Eigen::Vector3d t; // 3x1 平移向量
};

class Odometry {
public:
    Odometry(ros::NodeHandle& nh);
    ros::Subscriber image_sub;
    ros::Subscriber imu_sub_odom;
    ros::Subscriber wheel_speed_sub;
    parking_slot_detection::gcn_parking srv;
    ros::ServiceClient client;
    Pose vpose;
    Eigen::VectorXd state_;  // 状态向量 [x, y, z, q_w, q_x, q_y, q_z, vx, vy, vz]
    Eigen::MatrixXd P_;      // 状态协方差矩阵
    Eigen::MatrixXd Q_;      // 过程噪声
    Eigen::MatrixXd R_;      // 观测噪声
    Eigen::MatrixXd F_;      // 状态转移矩阵
    Eigen::MatrixXd H_;      // 观测矩阵
        // State imuOdometryUpdate(const Vector3d& accel, const Vector3d& gyro, double dt, const State& prev_state);
    void computeStateTransitionJacobian(const Eigen::VectorXd &u, double dt);
    void predict(const Eigen::VectorXd &u, double dt);
    void update(const Matrix4d& icp_transformation);
    void ekfodometry(const VectorXd &state);

    void matchpoint(const std::vector<cv::Point2f> & preFrame,const std::vector<cv::Point2f> & currentFrame);
    void piextocamera(const parking_slot_detection::gcn_parking &srv);
    void drawslot(const cv::Mat& image);
    void drawline(const Eigen::Matrix4d&relative_pose);
    void matchframes(const std::vector<cv::Point2f> & currentFrameSpots,const std::vector<cv::Point2f> & currentFrameSpots_ocr);
    void imageCallback_odo(const sensor_msgs::CompressedImageConstPtr& msg);
    void imuCallback_odom(const sensor_msgs::Imu::ConstPtr& msg);
    void wheelSpeedCallback(const nav_msgs::Odometry::ConstPtr &msg);
        // 从两帧车位角点估计相机的位姿
    Eigen::Matrix4d estimatePoseICP(const std::vector<Eigen::Vector3d>& prevPoints,const std::vector<Eigen::Vector3d>& currPoints);

    // 计算 ICP 旋转和平移
    void computeTransformation(const std::vector<Eigen::Vector3d>& prevPoints,
                                const std::vector<Eigen::Vector3d>& currPoints,
                               Eigen::Matrix3d& R, Eigen::Vector3d& t);

    // 计算质心
    Eigen::Vector3d computeCentroid(const std::vector<Eigen::Vector3d>& points);
    // void worldlocationspots(const std::vector<Spotwithtext>& pairspots );
    std::vector<cv::Point2f>  preFramespot;
    std::vector<cv::Point2f>  preFramespot_ocr;
    // 存储关联结果的向量
    std::vector<Spotwithtext> pixelPairs;//像素坐标系的车位位置
    std::vector<Spotwithtext> cameraPairs; //车辆坐标系下的车位位置
    std::vector<Spotwithtext> mapspotPairs; //停车场车位
    bool isFirstFrame; 
};

#endif // ODOM_H