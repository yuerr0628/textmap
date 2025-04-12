#include <showtrajectory/OCRResult.h>
#include "ros/ros.h"
#include <showtrajectory/Slots.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <showtrajectory/slotocr.h>
#include <iostream>

// 全局变量，存储检测到的车位和车位号
// 假设我们有以下全局变量
std::map<int, std::vector<showtrajectory::Slots::ConstPtr>> parking_spots; // 按帧ID存储车位
std::map<int, std::vector<showtrajectory::OCRResult::ConstPtr>> parking_numbers; // 
std::vector<geometry_msgs::Point32> slotcenter_points;
std::vector<geometry_msgs::Point32> numbercenter_points;
ros::Publisher pub ;
showtrajectory::slotocr associated_msg;
// std::vector<showtrajectory::Slots::ConstPtr>
struct my_pose
{
    double latitude;
    double longitude;
    double altitude;
};
//角度制转弧度制
double rad(double d) 
{
	return d * 3.1415926 / 180.0;
}
//全局变量
static double EARTH_RADIUS = 6378.137;//地球半径
 geometry_msgs::PoseStamped pose;
double roll;
double pitch;
double yaw;
geometry_msgs::Quaternion imu_quaternion;

// 关联函数
void associateAndPublish(int frame_id) {
    // 简单的关联逻辑：基于位置的距离
    double association_distance = 0.5; // 距离阈值，根据实际情况调整
    auto it_spot = parking_spots.find(frame_id);
    auto it_number = parking_numbers.find(frame_id);
    if (it_spot != parking_spots.end() && it_number != parking_numbers.end()) {
        const std::vector<showtrajectory::Slots::ConstPtr>& spots = it_spot->second;
        const std::vector<showtrajectory::OCRResult::ConstPtr>& numbers = it_number->second;
        for (const auto& number_ptr : numbers) {
            geometry_msgs::Point32 p1;
            const std::vector<geometry_msgs::Point32>& vertices = number_ptr->polygon.points;
            p1.x=(vertices[0].x+vertices[2].x)/2;
            p1.y=(vertices[0].y+vertices[2].y)/2;
            for (const auto& spot_ptr : spots) {
                geometry_msgs::Point32 p2;
                const std::vector<geometry_msgs::Point32>& vertices1 = spot_ptr->polygon.points;
                p2.x=(vertices1[0].x+vertices1[1].x)/2;
                p2.y=(vertices1[0].y+vertices1[1].y)/2;
                double distance = sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
                if (distance < association_distance) {
                // 关联成功，打包为一个整体
                    
                    associated_msg.polygon.points = spot_ptr->polygon.points;
                    associated_msg.text = number_ptr->text;
                    associated_msg.ID = number_ptr->ID;
                     std::cout<<associated_msg.ID;
                     std::cout<<associated_msg.ID<<","<<associated_msg.text<<std::endl;

                // 发布关联结果
               
                
                pub.publish(associated_msg);

                break;
            }
        }
        }
    }
}

// 回调函数：车位检测结果
void parkingSpotCallback(const showtrajectory::Slots::ConstPtr& slotsmsg) {
    int current_id = slotsmsg->ID; 
    parking_spots[current_id].push_back(slotsmsg);
    // if(!slotsmsg->polygon.points.empty()){
        // std::cout<<"slotsid:"<<current_id<<std::endl;
    // 尝试关联当前帧的车位和车位号
    associateAndPublish(current_id);
    // }
   
   
}

// 回调函数：车位号检测结果
void parkingNumberCallback(const showtrajectory::OCRResult::ConstPtr& ocrmsg) {
    int current_id = ocrmsg->ID; 
    parking_numbers[current_id].push_back(ocrmsg);
    // if(!ocrmsg->text.empty()){
    // 尝试关联当前帧的车位和车位号
    //  std::cout<<"ocrid:"<<current_id<<std::endl;
        associateAndPublish(current_id);\
        // }
       
}



int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "parking_spot_associator");
    ros::NodeHandle nh;

    // 订阅车位检测和车位号检测的话题
    ros::Subscriber parking_slot = nh.subscribe("slots", 14, parkingSpotCallback);
    ros::Subscriber parking_number = nh.subscribe("ocr_text_with_box", 14, parkingNumberCallback);
    pub = nh.advertise<showtrajectory::OCRResult>("parking_slot_with_number", 10);

    // 进入ROS循环
    ros::spin();

    return 0;
}