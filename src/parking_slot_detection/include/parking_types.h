#ifndef PARKING_TYPES_H
#define PARKING_TYPES_H

#include <map>
#include <string>
#include <vector>

struct Point3D {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

struct VehiclePose {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double yaw = 0.0;
};

struct ParkingSpotGeometry {
    Point3D p1, p2, p3, p4;
    int vacant_status = 0;
};

struct OcrResult {
    std::string text;
    double confidence = 0.0;
    Point3D p1, p2;
};

struct RawDetection {
    double timestamp = 0.0;
    ParkingSpotGeometry spot_geom_vehicle;
    OcrResult ocr_res_vehicle;
    bool is_spot_detected = false;
    bool is_text_detected = false;
};

struct MapSpot {
    int unique_id = -1;
    ParkingSpotGeometry spot_geom_world;
    OcrResult ocr_res_world;
    double best_ocr_cost = 1e9;

    int C_match = 0;
    int C_frame = 0;
    int C_unobserved = 0;
    bool is_stable = false;

    std::map<std::string, double> text_votes;
    double geometry_quality = 0.0;
};

#endif
