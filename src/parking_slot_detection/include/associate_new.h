#ifndef ASSOCIATE_H
#define ASSOCIATE_H

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
#include"p3p.h"

#include"../include/Hungarian.h"
#include <fstream>
#include <yaml-cpp/yaml.h>
// using json = nlohmann::json;
using std::string;
using namespace std;


struct ParkingSpot {
    double x1, y1, x2, y2, x3, y3, x4, y4;
    int vacant; //1为占用
    int vacant_update=0;
    int noccupy=0;
    int nfree=0;
};

struct OCRPoint {
    std::string text; // 车位号ID
    double confidence;
    double x1, y1, x2, y2;
};

struct AssociatedPair {
    ParkingSpot spot; // 关联的车位信息
    OCRPoint ocrPoint; // 关联的车位号信息
    int ID; // 帧id
    int unmatch_ID = 0;   //未匹配上的次数
    int age=0;         //匹配上的寿命
    double distanceocr;
    bool IF_TEXT_DET=true;
    bool IF_SPOT_DET=true;
};



class AssociatedParkingInfo {
public:
    P3P p3p1;
    std::unordered_map<uint32_t, sensor_msgs::CompressedImageConstPtr> avm_buffer;
    std::unordered_map<uint32_t, sensor_msgs::CompressedImageConstPtr> front_buffer;
    AssociatedParkingInfo(ros::NodeHandle& nh);
     ros::Subscriber avmimage_sub;
    ros::Subscriber frontimage_sub;
    parking_slot_detection::gcn_parking srv;
    parking_slot_detection::PlateRecognition srv_plate;
    ros::ServiceClient client;
    ros::ServiceClient client_plate;
    VehiclePose vehiclepose;
    void associateSpotsAndNumbers(const parking_slot_detection::gcn_parking &srv);
    void drawslotwithnumber(const cv::Mat& image);
    void addFrameAssociatedPairs(const std::vector<AssociatedPair>& framePairs);
    // void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg);
    void process_synced_images(const sensor_msgs::CompressedImageConstPtr&avm_msg,const sensor_msgs::CompressedImageConstPtr& front_msg);
    void avm_callback(const sensor_msgs::CompressedImageConstPtr& msg);
    void front_callback(const sensor_msgs::CompressedImageConstPtr& msg);
    void matchParkingSpots(const std::vector<AssociatedPair>& currentFrame);
    void worldlocationspots(const std::vector<AssociatedPair>& pairspots );
    void HandleDeath(const std::vector<AssociatedPair>& cFrame,const  std::vector<AssociatedPair>& death);
    void selectocr(const std::vector<AssociatedPair>& pairspots );
    void saveToYAML(const std::vector<AssociatedPair>& pairs, const std::string& filename);
    // void Datafuse(std::vector<int> Assignment);
    std::vector<AssociatedPair> preFrame;
    // 存储所有帧的关联结果
    std::vector<std::vector<AssociatedPair>> AllFramesAssociatedPairs;
    // 存储关联结果的向量
    std::vector<AssociatedPair> associatedPairs;//像素坐标系的车位位置
    std::vector<AssociatedPair> worldassociatedPairs; //世界坐标系下的车位位置
    std::vector<AssociatedPair> updates;
    std::vector<AssociatedPair> mapassociatedPairs; //停车场车位
    std::vector<AssociatedPair> birth; // 新车位
    std::vector<AssociatedPair> death; // 消失车位
    std::map<int, int> disappearanceCount; // 消失计数
    std::vector<LicensePlate>plates;
};

#endif // ASSOCIATED_PARKING_INFO_H