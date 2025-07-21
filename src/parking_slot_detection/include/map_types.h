#ifndef MAP_TYPES_H
#define MAP_TYPES_H

#include <vector>
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>

// Define Point2D using Eigen
using Point2D = Eigen::Vector2d;

// Structure to represent a single parking space
struct ParkingSpace {
    std::string local_id; // Use OCR text as ID (string)
    std::vector<Point2D> corners; // Corner coordinates parsed from x1,y1..x4,y4
    std::string ocr_text;         // Store the original OCR text here as well
    double ocr_confidence = 0.0;  // Optional: OCR confidence
    bool is_vacant = true;        // Optional: Vacancy status

    // --- Fields added after fusion ---
    std::string global_id = ""; // Use string for global ID as well
    std::vector<std::pair<std::string, std::string>> source_maps; // Track origin (map_id, parking_space_id)
};

// Structure to represent a local map segment
struct LocalMap {
    std::string map_id;
    std::vector<ParkingSpace> spaces;

    // --- Fields added after optimization ---
    Eigen::Isometry2d optimized_pose = Eigen::Isometry2d::Identity();
    int g2o_vertex_id = -1;
};

#endif // MAP_TYPES_H