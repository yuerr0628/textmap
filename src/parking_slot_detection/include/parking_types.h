#ifndef PARKING_TYPES_H
#define PARKING_TYPES_H

#include <vector>
#include <string>

// --- 已更新为3D坐标 ---
struct Point3D {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

// --- 已更新为3D位姿 (平面假设，z和roll, pitch为0) ---
struct VehiclePose {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double yaw = 0.0; // 航向角, 弧度
};

// 车位几何形状 --- 已更新为Point3D
struct ParkingSpotGeometry {
    Point3D p1, p2, p3, p4;
    int vacant_status = 0;
};

// OCR识别结果 --- 已更新为Point3D
struct OcrResult {
    std::string text;
    double confidence = 0.0;
    Point3D p1, p2;
};

// 经过帧内关联后的原始检测对象 (车辆坐标系) --- 已更新
struct RawDetection {
    double timestamp;
    ParkingSpotGeometry spot_geom_vehicle;
    OcrResult ocr_res_vehicle;
    bool is_spot_detected = false;
    bool is_text_detected = false;
};

// 地图中的车位对象，也是追踪的基本单元 --- 已更新
struct MapSpot {
    int unique_id = -1;
    ParkingSpotGeometry spot_geom_world;
    OcrResult ocr_res_world;
    double best_ocr_cost = 1e9;

    // 动态追踪信息
    int C_match = 0;
    int C_frame = 0;
    int C_unobserved = 0;
    bool is_stable = false;
};

#endif // PARKING_TYPES_H