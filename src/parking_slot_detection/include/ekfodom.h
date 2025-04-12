#ifndef EKFODOM_H
#define EKFODOM_H

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include "parking_slot_detection/gcn_parking.h" // 假设这是包含srv定义的头文件
#include "parking_slot_detection/PlateRecognition.h" 
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <string> 
#include"pose.h"
#include <Eigen/Dense>
#include <parking_slot_detection/SpeedFeedback.h>
#include <eigen_conversions/eigen_msg.h>
#include "utility.h"
#include <queue>
#include <thread>
#include <mutex>
#include <sensor_msgs/NavSatFix.h>
#include"pose.h"
#include"p3p.h"
// #include"loop_closing.h"
// #include <GeographicLib/LocalCartesian.hpp>

#include"../include/Hungarian.h"
// #include <message_filters/sync_policies/ApproximateTime.h>
using std::string;
using namespace Eigen;
using namespace std;
#define IF_USE_GPS 1
#define IF_USE_WHL 1


// #define IF_USE_ODOM 

// typedef nav_msgs::OdometryConstPtr WheelDataPtr;
// typedef geometry_msgs::PoseStampedConstPtr GNSSDataPtr;
// typedef cyber_msgs::SpeedFeedbackConstPtr WheelDataPtr;
// typedef sensor_msgs::NavSatFixConstPtr GNSSDataPtr;

struct Spot {
    vector<Eigen::Vector3d> corners; // 使用cv::Point2d存储双精度浮点坐标
};

struct OCRtext {
    std::string text; // 车位号ID
    std::vector<Eigen::Vector3d> textcorners; // 使用cv::Point2d存储双精度浮点坐标
    std::vector<std::pair<std::string, Eigen::Vector3d>> ocrdata; 
};

struct TEXTDATA {
    vector<Spot> spots; // 关联的车位信息
    vector<OCRtext> ocrPoints; // 关联的车位号信息
    vector< LicensePlate>licplate;
    double timestamp;//时间戳
};

struct Pose {
    Eigen::Matrix3d R; // 3x3 旋转矩阵
    Eigen::Vector3d t; // 3x1 平移向量
};

struct IMUData{
    double timestamp;
    Eigen::Matrix3d q;
    Eigen::Vector3d acc;
    Eigen::Vector3d gyro;
};
using IMUDataPtr = std::shared_ptr<IMUData>;


struct VIOODOMData
{
    double timestamp;
    geometry_msgs::Pose pose;
};
using VIOODOMDataPtr = std::shared_ptr<VIOODOMData>;

struct WHLData
{
    double timestamp;

    Eigen::Vector3d speed;
};

using WheelDataPtr = std::shared_ptr<WHLData>;

struct GNSSData
{
    double timestamp;

    Eigen::Vector3d lla;
    Eigen::Matrix3d cov;
};
using GNSSDataPtr = std::shared_ptr<GNSSData>;

enum StateIndex : uint {
    R = 0,                   // (3 dimention) rotation in world frame
    P = 3,                   // (3 dimention) position in world frame
    V = 6,                   // (3 dimention) velocity in world frame
    BA = 9,                 // (3 dimention) IMU acceleration bias
    BG = 12,                 // (3 dimention) IMU gyroscope bias
    IMU_INSTALL_ANGLE = 15,  // (3 dimention) imu install error IMU_INSTALL_ANGLE
    WS = 18,                 // (1 dimention) wheel speed ratio
    STATE_TOTAL = 19
};
    // IMU_INSTALL_ANGLE = 15,  // (3 dimention) imu install error imu_q_veh
    // WS = 18,                 // (1 dimention) wheel speed ratio

enum StateNoiseIndex : uint {
    ACC_NOISE = 0,         // linear acceleration change
    GYRO_NOISE = 3,          // angular velocity change
    ACC_RANDOM_WALK = 6,   // IMU aceleration bias random walk
    GYRO_RANDOM_WALK = 9,  // IMU gyroscope bias random walk
    NOISE_TOTAL = 12
};

struct State {
    Eigen::Matrix3d R_q;
    Eigen::Vector3d p;
    Eigen::Vector3d v;
    Eigen::Vector3d ba;
    Eigen::Vector3d bg;
    Eigen::Matrix3d R_imu;
    long double time;
    double ws;
    Eigen::MatrixXd P;
};
using StatePtr = std::shared_ptr<State>;

class Odometry {
public:
    VehiclePose pose1;
    P3P p3p;
    Pose vpose;
    
        std::unordered_map<uint32_t, sensor_msgs::CompressedImageConstPtr> avm_buffer1;
    std::unordered_map<uint32_t, sensor_msgs::CompressedImageConstPtr> front_buffer1;
    Odometry(ros::NodeHandle& nh);

    // Odometry();
    // ~Odometry();

    ros::Subscriber avmimage_sub;
    ros::Subscriber frontimage_sub;
    ros::Subscriber imu_sub_odom;
    ros::Subscriber gps_sub_odom;
    ros::Subscriber wheel_speed_sub;
    parking_slot_detection::gcn_parking srv;
    parking_slot_detection::PlateRecognition srv_plate;
    ros::ServiceClient client;
    ros::ServiceClient client_plate;
    State state_;
    Eigen::Vector3d gravity_;
    bool stateInit_ = false;
    bool stateInit_gps = false;
    long double init_time_ = 0.;
     Eigen::MatrixXd Q_; // variance matrix of imu
    Eigen::MatrixXd whl_Rm_; // variance matrix of wheel odometer
    Eigen::MatrixXd gps_Rm_; // variance matrix of gps
    Eigen::MatrixXd odom_Rm_;// variance matrix of odometer
       bool shutdown_ = false;
    std::deque<IMUDataPtr> imu_buf_;
    IMUDataPtr last_imu_ptr_;
    bool parameter_lock = false;
    // long double init_time_ = 0.;
    const double IMU_Std = 3.0;
    const int IMU_BUF_SIZE = 100;
    Eigen::Vector3d p_I_GNSS_;
    Eigen::Vector3d init_lla_;
    nav_msgs::Path gnss_path_;
    VIOODOMData viodata;
    bool IF_USE_ODOM=true;
    // bool IF_USE_GPS=true;

    double acc_n_var, gyr_n_var, acc_rw_var, gyr_rw_var,imu_mount_rw_var;
    // double whl_odom_var,gnss_noise_var,odom_var;
    double whl_odom_var,gnss_noise_var,odom_rot_var,odom_tran_var;
    bool IF_GPS_DENY = false;
    bool InitState();
   void imuCallback_odom(const sensor_msgs::Imu::ConstPtr& imu_msg);
    // void WheelCallback(const diankong::VehicleFeedbackConstPtr &wheel_msg);
    void WheelCallback(const parking_slot_detection::SpeedFeedbackConstPtr &wheel_msg);
    void GpsCallback(const sensor_msgs::NavSatFixConstPtr &gps_msg);
    // void GpsCallback(const sensor_msgs::NavSatFixConstPtr &gps_msg) ;
    // void LidarOdomCallback(const cyber_msgs::LocalizationEstimateConstPtr &lidar_odom_msg);
    bool process_IMU_Data(IMUDataPtr imu_data_ptr);
    void PredictByImu(IMUDataPtr last_imu_ptr, IMUDataPtr cur_imu_ptr);
    void vio_process(const VIOODOMData& viodata);
    void UpdateByWheel(WheelDataPtr wheel_data_ptr);
    void UpdateByOdom(VIOODOMDataPtr odom_data_ptr);
    void UpdateByGps(GNSSDataPtr gnss_data_ptr);
    // void imageCallback_odo(const sensor_msgs::CompressedImageConstPtr& msg);
    void avm_callback(const sensor_msgs::CompressedImageConstPtr& msg);
    void front_callback(const sensor_msgs::CompressedImageConstPtr& msg);
    void cleanup_buffers(uint32_t current_seq);
    void process_synced_images(const sensor_msgs::CompressedImageConstPtr&avm_msg,const sensor_msgs::CompressedImageConstPtr& front_msg);
    
    // void UpdateByGps(GNSSDataPtr gnss_data_ptr);
    // void UpdateByOdom(ODOMDataPtr odom_data_ptr);
    void publish();



    void matchpoint(const std::vector<cv::Point2f> & preFrame,const std::vector<cv::Point2f> & currentFrame);
    void piextocamera(const parking_slot_detection::gcn_parking &srv);
    void drawslot(const cv::Mat& image);
    void drawline( const Eigen::Matrix4d&relative_pose);
    void matchframes(const TEXTDATA &ctextdata);
    
        // 从两帧车位角点估计相机的位姿
    Eigen::Matrix4d estimatePoseICP(const TEXTDATA&ptdata,const TEXTDATA&cdata);

    // 计算 ICP 旋转和平移
    void computeTransformation(const std::vector<Eigen::Vector3d>& prevPoints,
                                const std::vector<Eigen::Vector3d>& currPoints,
                               Eigen::Matrix3d& R, Eigen::Vector3d& t);
                               

    // 计算质心
    Eigen::Vector3d computeCentroid(const std::vector<Eigen::Vector3d>& points);
    // Eigen::Matrix4d computepose(const std::vector<Eigen::Vector3d>  prevPoints,const std::vector<Eigen::Vector3d>  currPoints);
    // void worldlocationspots(const std::vector<Spotwithtext>& pairspots );
    // std::vector<cv::Point2f>  preFramespot;
    // std::vector<cv::Point2f>  preFramespot_ocr;
    TEXTDATA lasttextdata;
    // 存储关联结果的向量
    std::vector<TEXTDATA> pixelPairs;//像素坐标系的车位位置
    std::vector<TEXTDATA> cameraPairs; //车辆坐标系下的车位位置
    std::vector<TEXTDATA> mapspotPairs; //停车场车位
    bool isFirstFrame=true; 
    
        // 假设手动指定初始位置
    Eigen::Matrix4d initial_pose ;

    // 当前帧的位姿
    Eigen::Matrix4d current_pose;
};

#endif // EKFODOM_H