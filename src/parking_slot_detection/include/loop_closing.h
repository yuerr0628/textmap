#ifndef LOOP_CLOSING_H
#define LOOP_CLOSING_H

#include <string>
#include <vector>
#include <unordered_map>
#include <yaml-cpp/yaml.h>

#include <iostream>
#include <algorithm>

// #include <levenshtein.h>  // 确保你有一个 Levenshtein 距离的实现
#include"associate.h"
#include"rvizshow.h"
#include"ekfodom.h"


struct EKFState {
    Eigen::Vector3d position;            // 相机位置 p = [px, py, pz]
    Eigen::Quaterniond orientation;      // 相机旋转 q = [qw, qx, qy, qz]
    Eigen::MatrixXd P;                   // 状态协方差
    double timestamp;
};


// struct ParkingSpot {
//     double x1, y1, x2, y2, x3, y3, x4, y4;
//     int vacant;
//     int vacant_update;
// };

// struct OCRPoint {
//     std::string text;
//     double confidence;
//     double x1, y1, x2, y2;
// };

// struct AssociatedPair {
//     int ID;
//     int time_since_update;
//     int age;
//     double distanceocr;
//     bool IF_TEXT_DET;
//     bool IF_SPOT_DET;
//     ParkingSpot spot;
//     OCRPoint ocrPoint;
// };

class LoopClosing {
public:
    EKFState ekf;
       ros::Subscriber avmimage_sub1;
    ros::Subscriber frontimage_sub1;
            std::unordered_map<uint32_t, sensor_msgs::CompressedImageConstPtr> avm_buffer2;
    std::unordered_map<uint32_t, sensor_msgs::CompressedImageConstPtr> front_buffer2;
    // Odometry odom;
    ros::Subscriber image_sub;
    ros::ServiceClient client;
    bool intialloop;
    bool vio_intial=false;
    parking_slot_detection::gcn_parking srv;
    LoopClosing(ros::NodeHandle& nh,const std::string& filename);
    void loadMap(const std::string& filename);
    std::vector<AssociatedPair> getMapPoints() const;
    void detectLoop(const std::vector<AssociatedPair>& latestPair);


    std::vector<AssociatedPair> mapPoints;
    double computeSimilarity(const std::string& si, const std::string& sj);
    void selectCandidates(const std::unordered_map<std::string, int>& matchedWords);
    void saveTrajectoryToTUM(const std::string& filename, const EKFState& pose);
    void locateFrame(const AssociatedPair& latestPair);
    int levenshtein(const std::string& s1, const std::string& s2);
    void piexl_to_3d(const std::vector<AssociatedPair>&map_point,const std::vector<AssociatedPair>&cur_point);
    // void imageCallback_map(const sensor_msgs::CompressedImageConstPtr& msg,const sensor_msgs::CompressedImageConstPtr& front_msg);
    void imageCallback_map(const sensor_msgs::CompressedImageConstPtr& msg,const sensor_msgs::CompressedImageConstPtr&front_msg);
    std::string  removeA(const std::string& str);
    Eigen::Matrix4d icp_pose(const std::vector<Eigen::Vector3d>  prevPoints,const std::vector<Eigen::Vector3d>  currPoints);
    void front_callback1(const sensor_msgs::CompressedImageConstPtr& msg);
    void avm_callback1(const sensor_msgs::CompressedImageConstPtr& msg); 

    std::vector<AssociatedPair> associatedPairs_map;

    Odometry ekfodom;
    std::ofstream file;
    Pose pose_loop;
    Eigen::Matrix4d intial_pose = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d curr_pose =intial_pose;

    void initializeEKF(EKFState &ekf);
    void predict( const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, 
             const Eigen::MatrixXd &Q);
             void update(const Eigen::Vector3d &measured_p, const Eigen::Quaterniond &measured_q, 
            const Eigen::MatrixXd &R);

};

#endif // LOOP_CLOSING_H
