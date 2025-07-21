#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <nlohmann/json.hpp>
#include <fstream>
#include <cmath>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "parking_types.h"
#include "parking_lot_mapper.h"
#include "parking_slot_detection/gcn_parking.h" // 替换为您的服务头文件

uint32_t avmseq=-1;
uint32_t frontseq=-1;

class ParkingMappingNode {
public:
    ParkingMappingNode(ros::NodeHandle& nh);

private:
    // --- 配置 ---
    struct TransformConfig {
        double spot_center_x, spot_center_y, spot_pixel_to_meter_x, spot_pixel_to_meter_y;
        double ocr_center_x, ocr_center_y, ocr_pixel_to_meter_x, ocr_pixel_to_meter_y;
        double association_dist_pixels;
    } m_tf_config;
    
    // --- 成员 ---
    ros::NodeHandle m_nh;
    ros::Subscriber m_avm_sub;
    ros::Subscriber m_front_sub;
    ros::ServiceClient m_gcn_client;
    ros::Timer m_save_map_timer;
    std::unique_ptr<message_filters::Subscriber<sensor_msgs::CompressedImage>> m_front_cam_sub;
    std::unique_ptr<message_filters::Subscriber<nav_msgs::Odometry>> m_odom_sub;
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::CompressedImage, 
        nav_msgs::Odometry
    > OdomFrontSyncPolicy;
    std::unique_ptr<message_filters::Synchronizer<OdomFrontSyncPolicy>> m_synchronizer;
    ParkingLotMapper::Config m_config_mapper;
    ParkingLotMapper m_mapper;
    std::unordered_map<uint32_t, sensor_msgs::CompressedImageConstPtr> avm_buffer;
    std::unordered_map<uint32_t, sensor_msgs::CompressedImageConstPtr> front_buffer;
    
    // --- 方法 ---
    void loadParams();
    // void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg);
    void saveMapCallback(const ros::TimerEvent&);
    void process_synced_images(const sensor_msgs::CompressedImageConstPtr&avm_msg,const sensor_msgs::CompressedImageConstPtr& front_msg);
    void avm_callback(const sensor_msgs::CompressedImageConstPtr& msg);
    void front_callback(const sensor_msgs::CompressedImageConstPtr& msg);

    std::vector<RawDetection> associateInFrame(const parking_slot_detection::gcn_parking::Response& res);
    MapSpot transformToWorld(const RawDetection& detection, const VehiclePose& pose);
    void syncedCallback(const sensor_msgs::CompressedImageConstPtr& front_cam_msg, const nav_msgs::OdometryConstPtr& odom_msg);
};

ParkingMappingNode::ParkingMappingNode(ros::NodeHandle& nh) : m_nh(nh), m_mapper({}) {
    loadParams();
    m_mapper = ParkingLotMapper(m_config_mapper);
    m_avm_sub = m_nh.subscribe("/driver/fisheye/avm/compressed", 10, &ParkingMappingNode::avm_callback, this);
    m_front_sub = m_nh.subscribe("/driver/fisheye/front/compressed", 10, &ParkingMappingNode::front_callback, this);
    m_front_cam_sub = std::make_unique<message_filters::Subscriber<sensor_msgs::CompressedImage>>(m_nh, "/driver/fisheye/left/compressed", 10);
    m_odom_sub = std::make_unique<message_filters::Subscriber<nav_msgs::Odometry>>(m_nh, "/your_odometry_topic", 50);
    m_gcn_client = m_nh.serviceClient<parking_slot_detection::gcn_parking>("gcn_service");
    m_save_map_timer = m_nh.createTimer(ros::Duration(10.0), &ParkingMappingNode::saveMapCallback, this);
       m_synchronizer = std::make_unique<message_filters::Synchronizer<OdomFrontSyncPolicy>>(OdomFrontSyncPolicy(10), *m_front_cam_sub, *m_odom_sub);
    m_synchronizer->registerCallback(boost::bind(&ParkingMappingNode::syncedCallback, this, _1, _2));

    ROS_INFO("Parking Mapping Node Initialized.");
}

void ParkingMappingNode::loadParams() {
    m_nh.param("mapper/association_threshold", m_config_mapper.association_threshold, 0.8);
    // ... load other mapper configs ...
    m_nh.param("transform/spot_pixel_to_meter_x", m_tf_config.spot_pixel_to_meter_x, 50.08);
    m_nh.param("transform/association_dist_pixels", m_tf_config.association_dist_pixels, 40.0);
    // ... load other transform configs ...
}

void AssociatedParkingInfo::avm_callback(const sensor_msgs::CompressedImageConstPtr& msg) {
    // cout<<"enteravm"<<endl;
    avmseq = ++avmseq; // 获取序列号
    //  cout<<avmseq<<endl;
    avm_buffer[avmseq] = msg;          // 将消息存入缓冲区
}

void AssociatedParkingInfo::front_callback(const sensor_msgs::CompressedImageConstPtr& msg) {
    // cout<<"enterfront"<<endl;
    // uint32_t seq = msg->header.seq; // 获取序列号
    frontseq = ++frontseq;
    //  cout<<frontseq<<endl;
    front_buffer[frontseq] = msg;        // 将消息存入缓冲区
}

void ParkingMappingNode::syncedCallback(const sensor_msgs::CompressedImageConstPtr& front_cam_msg, const nav_msgs::OdometryConstPtr& odom_msg) {
       // 检查是否有匹配的 AVM 图像
    if (avm_buffer.count(frontseq)) {
        // 找到匹配的 AVM 图像
        auto avm_msg = avm_buffer[frontseq];

        // 移除已匹配的消息
        avm_buffer.erase(frontseq);
        front_buffer.erase(frontseq);

        // 触发后续处理逻辑
        process_synced_images(avm_msg, msg);
    }
    else {
        // 未找到匹配的AVM，可能AVM消息还没到或者丢失了
        ROS_WARN("No matching AVM image for front_cam_seq %lu", current_front_seq);
    }
}

// void ParkingMappingNode::imageCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
//     sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(msg->header, "bgr8", cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR)).toImageMsg();
//     parking_slot_detection::gcn_parking srv;
//     srv.request.image_data = *image_msg;

//     if (!m_gcn_client.call(srv)) {
//         ROS_ERROR("Failed to call GCN service.");
//         return;
//     }



//     std::vector<RawDetection> raw_detections = associateInFrame(srv.response);
    
//     // TODO: 从您的定位模块获取真实位姿
//     VehiclePose current_vehicle_pose = {10.0, 5.0, 0.0, 0.5}; // 示例位姿 {x, y, z, yaw}

//     std::vector<MapSpot> world_spots;
//     for (const auto& det : raw_detections) {
//         world_spots.push_back(transformToWorld(det, current_vehicle_pose));
//     }
//     m_mapper.update(world_spots, current_vehicle_pose);
// }

void AssociatedParkingInfo::process_synced_images(const sensor_msgs::CompressedImageConstPtr&avm_msg,const sensor_msgs::CompressedImageConstPtr& front_msg)
{
    // cout<<"enterprocess_synced_images"<<endl;
     try
    {
        // 解压缩图像
        cv::Mat image = cv::imdecode(cv::Mat(avm_msg->data),cv::IMREAD_COLOR);
        cv::Mat image_front = cv::imdecode(cv::Mat(front_msg->data),cv::IMREAD_COLOR);
    //     // 确保图像尺寸符合预期
        int original_width = image.cols;
     int original_height = image.rows;
        sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        sensor_msgs::ImagePtr frontimage_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_front).toImageMsg();
        srv.request.image_data = *image_msg;

        if (client.call(srv))
        {
            // std::cout<<"enter"<<std::endl;
            // std::cout<<srv.response.point0_x.size()<<std::endl;
           double time_image=front_msg->header.stamp.toSec();
           double time_odom=ekf_odom.state_.time;
           double time_gps=vehiclepose.pose.timestamp;
           cout<<"time_gps-time_image: "<<time_gps-time_image<<endl;
           cout<<"time_odom-time_image: "<<time_odom-time_image<<endl;
            //  cout<<"gps.pose-ekf.pose: "<<vehiclepose.pose.x-ekf_odom.state_.p(0)<<","<<vehiclepose.pose.y-ekf_odom.state_.p(1)<<endl;
            // associateSpotsAndNumbers(srv);
            // drawslotwithnumber(image);
            // 成功接收响应
            std::vector<RawDetection> raw_detections = associateInFrame(srv.response);
            std::vector<MapSpot> world_spots;
            for (const auto& det : raw_detections) {
                world_spots.push_back(transformToWorld(det, current_vehicle_pose));
            }
            m_mapper.update(world_spots, current_vehicle_pose);

        }
        else {
            ROS_ERROR("Failed to call service");
        }
    }
    // }

    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", avm_msg->format.c_str());
    }


}

// 该函数实现了您 associateSpotsAndNumbers 的逻辑
std::vector<RawDetection> ParkingMappingNode::associateInFrame(const parking_slot_detection::gcn_parking::Response& res) {
    std::vector<RawDetection> detections;
    std::vector<bool> matched_spots(res.point0_x.size(), false);
    std::vector<bool> matched_ocrs(res.texts.size(), false);

    // 1. 寻找“车位+车位号”的完美匹配
    for (size_t i = 0; i < res.point0_x.size(); ++i) {
        for (size_t j = 0; j < res.texts.size(); ++j) {
            if (matched_ocrs[j]) continue;
            
            double spot_cx = (res.point0_x[i] + res.point1_x[i] + res.point2_x[i] + res.point3_x[i]) / 4.0;
            double spot_cy = (res.point0_y[i] + res.point1_y[i] + res.point2_y[i] + res.point3_y[i]) / 4.0;
            double ocr_cx = (res.ocrpointx1[j] + res.ocrpointx2[j]) / 2.0;
            double ocr_cy = (res.ocrpointy1[j] + res.ocrpointy2[j]) / 2.0;

            if (std::hypot(spot_cx - ocr_cx, spot_cy - ocr_cy) < m_tf_config.association_dist_pixels) {
                RawDetection det;
                det.is_spot_detected = true;
                det.is_text_detected = true;
                det.spot_geom_vehicle = {{res.point0_x[i], res.point0_y[i], 0}, {res.point1_x[i], res.point1_y[i], 0}, {res.point2_x[i], res.point2_y[i], 0}, {res.point3_x[i], res.point3_y[i], 0}};
                det.ocr_res_vehicle = {res.texts[j], res.confidence[j], {res.ocrpointx1[j], res.ocrpointy1[j], 0}, {res.ocrpointx2[j], res.ocrpointy2[j], 0}};
                detections.push_back(det);
                matched_spots[i] = true;
                matched_ocrs[j] = true;
                break; // 一个车位只匹配一个最近的车位号
            }
        }
    }

    // 2. 添加未匹配的“仅车位”
    for (size_t i = 0; i < res.point0_x.size(); ++i) {
        if (!matched_spots[i]) {
            RawDetection det;
            det.is_spot_detected = true;
            det.is_text_detected = false;
            det.spot_geom_vehicle = {{res.point0_x[i], res.point0_y[i], 0}, {res.point1_x[i], res.point1_y[i], 0}, {res.point2_x[i], res.point2_y[i], 0}, {res.point3_x[i], res.point3_y[i], 0}};
            detections.push_back(det);
        }
    }

    // 3. 添加未匹配的“仅车位号”
    for (size_t j = 0; j < res.texts.size(); ++j) {
        if (!matched_ocrs[j]) {
            RawDetection det;
            det.is_spot_detected = false;
            det.is_text_detected = true;
            det.ocr_res_vehicle = {res.texts[j], res.confidence[j], {res.ocrpointx1[j], res.ocrpointy1[j], 0}, {res.ocrpointx2[j], res.ocrpointy2[j], 0}};
            detections.push_back(det);
        }
    }

    return detections;
}

// 辅助函数，用于转换单个3D点
static Point3D transform_point(double in_x, double in_y, double center_x, double center_y, double scale_x, double scale_y, const VehiclePose& pose) {
    double px_veh = (in_x - center_x) / scale_x;
    double py_veh = (in_y - center_y) / scale_y;
    
    Point3D world_p;
    world_p.x = pose.x + px_veh * std::cos(pose.yaw) - py_veh * std::sin(pose.yaw);
    world_p.y = pose.y + px_veh * std::sin(pose.yaw) + py_veh * std::cos(pose.yaw);
    world_p.z = pose.z;
    return world_p;
}

// 该函数实现了您 worldlocationspots 的逻辑
MapSpot ParkingMappingNode::transformToWorld(const RawDetection& detection, const VehiclePose& pose) {
    MapSpot spot;
    if (detection.is_spot_detected) {
        const auto& geom_v = detection.spot_geom_vehicle;
        spot.spot_geom_world.p1 = transform_point(geom_v.p1.x, geom_v.p1.y, m_tf_config.spot_center_x, m_tf_config.spot_center_y, m_tf_config.spot_pixel_to_meter_x, m_tf_config.spot_pixel_to_meter_y, pose);
        spot.spot_geom_world.p2 = transform_point(geom_v.p2.x, geom_v.p2.y, m_tf_config.spot_center_x, m_tf_config.spot_center_y, m_tf_config.spot_pixel_to_meter_x, m_tf_config.spot_pixel_to_meter_y, pose);
        spot.spot_geom_world.p3 = transform_point(geom_v.p3.x, geom_v.p3.y, m_tf_config.spot_center_x, m_tf_config.spot_center_y, m_tf_config.spot_pixel_to_meter_x, m_tf_config.spot_pixel_to_meter_y, pose);
        spot.spot_geom_world.p4 = transform_point(geom_v.p4.x, geom_v.p4.y, m_tf_config.spot_center_x, m_tf_config.spot_center_y, m_tf_config.spot_pixel_to_meter_x, m_tf_config.spot_pixel_to_meter_y, pose);
    }
    if (detection.is_text_detected) {
        const auto& ocr_v = detection.ocr_res_vehicle;
        spot.ocr_res_world.p1 = transform_point(ocr_v.p1.x, ocr_v.p1.y, m_tf_config.ocr_center_x, m_tf_config.ocr_center_y, m_tf_config.ocr_pixel_to_meter_x, m_tf_config.ocr_pixel_to_meter_y, pose);
        spot.ocr_res_world.p2 = transform_point(ocr_v.p2.x, ocr_v.p2.y, m_tf_config.ocr_center_x, m_tf_config.ocr_center_y, m_tf_config.ocr_pixel_to_meter_x, m_tf_config.ocr_pixel_to_meter_y, pose);
        spot.ocr_res_world.text = ocr_v.text;
        spot.ocr_res_world.confidence = ocr_v.confidence;
    }
    return spot;
}

void ParkingMappingNode::saveMapCallback(const ros::TimerEvent&) {
    const auto& final_map = m_mapper.getFinalMap();
    if (final_map.empty()) return;

    using json = nlohmann::json;
    json j_array = json::array();
    for (const auto& spot : final_map) {
        json j_spot;
        // ... 序列化地图 ...
        j_array.push_back(j_spot);
    }
    std::string filename = "/data/yhy/final_map.json";
    std::ofstream fout(filename);
    fout << j_array.dump(4);
    ROS_INFO("Map saved to %s", filename.c_str());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "parking_mapping_node");
    ros::NodeHandle nh("~");
    ParkingMappingNode node(nh);
    ros::spin();
    return 0;
}
