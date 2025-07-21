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
#include "map_stitcher.h"   // The stitching logic
#include "json_loader.h"    // The JSON loading functions
#include "map_types.h"      // Common data types
#include <iostream>
#include <vector>
#include <string>
#include <fstream> // For creating/checking dummy files
#include <filesystem> // Requires C++17 for checking file existence easily
#include "ekf_fusion_node.h"

// void createDummyJsonFile(const std::string& filename, const std::string& content) {
//     // Optional: Check if file already exists to avoid overwriting
//     // if (std::filesystem::exists(filename)) {
//     //     std::cout << "Dummy file already exists: " << filename << std::endl;
//     //     return;
//     // }
//     std::ofstream ofs(filename);
//     if (ofs.is_open()) {
//         ofs << content;
//         ofs.close();
//         std::cout << "Created/Updated dummy file: " << filename << std::endl;
//     } else {
//         std::cerr << "Error: Could not create dummy file: " << filename << std::endl;
//     }
// }

// // --- Sample JSON Data Strings (Used to create dummy files) ---
// const std::string json_map1_ocr = R"([{"ID":101,"OCRPoint":{"confidence":0.99,"text":"A01","x1":0.5,"x2":1.5,"y1":5.5,"y2":5.1},"ParkingSpot":{"vacant":1,"x1":0,"y1":0,"x2":2,"y2":0,"x3":2,"y3":5,"x4":0,"y4":5}},{"ID":102,"OCRPoint":{"confidence":0.98,"text":"A02","x1":3.5,"x2":4.5,"y1":5.5,"y2":5.1},"ParkingSpot":{"vacant":0,"x1":3,"y1":0,"x2":5,"y2":0,"x3":5,"y3":5,"x4":3,"y4":5}},{"ID":103,"OCRPoint":{"confidence":0.97,"text":"A03","x1":0.5,"x2":1.5,"y1":11.5,"y2":11.1},"ParkingSpot":{"vacant":1,"x1":0,"y1":6,"x2":2,"y2":6,"x3":2,"y3":11,"x4":0,"y4":11}}])";
// const std::string json_map2_ocr = R"([{"ID":202,"OCRPoint":{"confidence":0.98,"text":"A02","x1":-0.48,"x2":1.51,"y1":5.22,"y2":4.83},"ParkingSpot":{"vacant":0,"x1":-1.046,"y1":-0.087,"x2":0.949,"y2":-0.192,"x3":1.212,"y3":4.786,"x4":-0.783,"y4":4.891}},{"ID":204,"OCRPoint":{"confidence":0.96,"text":"B01","x1":2.5,"x2":3.5,"y1":4.5,"y2":4.1},"ParkingSpot":{"vacant":1,"x1":2,"y1":-1,"x2":4,"y2":-1,"x3":4,"y3":4,"x4":2,"y4":4}}])";
// const std::string json_map3_ocr = R"([{"ID":304,"OCRPoint":{"confidence":0.96,"text":"B01","x1":-1.49,"x2":0.51,"y1":4.32,"y2":3.92},"ParkingSpot":{"vacant":1,"x1":-1.526,"y1":-0.947,"x2":0.474,"y2":-1.000,"x3":0.605,"y3":3.974,"x4":-1.395,"y4":4.026}},{"ID":305,"OCRPoint":{"confidence":0.95,"text":"C01","x1":1.5,"x2":2.5,"y1":10.5,"y2":10.1},"ParkingSpot":{"vacant":1,"x1":1,"y1":5,"x2":3,"y2":5,"x3":3,"y3":10,"x4":1,"y4":10}}])";


int main(int argc, char **argv) {
    ros::init(argc, argv, "vehicle_pose");
    ros::NodeHandle nh;

    // VehiclePose vehiclePose; 
    // LoopClosing loopClosing(nh,"/data/yhy/map250701_start0.yaml");
    // 订阅GPS和IMU话题
    // Odometry Odometry(nh);
    // ros::Subscriber gps_sub = nh.subscribe("/Inertial/gps/fix", 10, VehiclePose::gpsCallback); 
      // Affine parameters

    // VehiclePose VehiclePose(nh);
    // VehiclePose VehiclePose(nh);
    
    AssociatedParkingInfo AssociatedParkingInfo(nh);
    // EKFFusionNode node(nh);
    // mydisplay.RvizDisplay(nh);
    //  std::cout << "=== C++ Map Stitching with OCR Text ID and g2o (File Loading & Saving) ===" << std::endl;

    // // 1. Define input file paths and output file path
    // std::vector<std::string> json_input_file_paths;
    // std::string output_file_path = "global_map_output.json"; // Default output file name

    // // --- Argument Parsing (Simple Example) ---
    // if (argc <= 1) {
    //     // No arguments provided, use default dummy files for testing
    //     std::cout << "Usage: " << argv[0] << " <output_file.json> <input1.json> [input2.json] ..." << std::endl;
    //     std::cout << "No input files specified. Using default dummy files for testing." << std::endl;
    //     json_input_file_paths = {"/data/yhy/ocr/ocrmap/map0723_testseg_5.json", "/data/yhy/ocr/ocrmap/map0723_testseg_6.json", "/data/yhy/ocr/ocrmap/map0723_testseg_loop1.json"};
    //     output_file_path = "/data/yhy/ocr/ocrmap/global_map_testloop.json";

    //     // Create dummy files if using defaults
    //     std::cout << "\n--- Creating dummy JSON files for testing ---" << std::endl;
    //     createDummyJsonFile(json_input_file_paths[0], json_map1_ocr);
    //     createDummyJsonFile(json_input_file_paths[1], json_map2_ocr);
    //     createDummyJsonFile(json_input_file_paths[2], json_map3_ocr);
    //     std::cout << "--- End of dummy file creation ---" << std::endl;

    // } else {
    //     // Use command line arguments: output_file input1 [input2 ...]
    //     output_file_path = argv[1];
    //     for (int i = 2; i < argc; ++i) {
    //         json_input_file_paths.push_back(argv[i]);
    //     }
    //     if (json_input_file_paths.empty()) {
    //          std::cerr << "Error: No input JSON files provided after output file path." << std::endl;
    //          std::cout << "Usage: " << argv[0] << " <output_file.json> <input1.json> [input2.json] ..." << std::endl;
    //          return 1;
    //     }
    //      std::cout << "Output file set to: " << output_file_path << std::endl;
    //      std::cout << "Input files:" << std::endl;
    //      for(const auto& path : json_input_file_paths) std::cout << " - " << path << std::endl;
    // }


    // // 2. Load local maps from JSON files
    // std::cout << "\n--- Loading Local Maps from JSON Files ---" << std::endl;
    // std::vector<LocalMap> local_maps = loadLocalMapsFromJsonFiles(json_input_file_paths);


    // if (local_maps.empty()) {
    //     std::cerr << "Error: No local maps were loaded successfully. Exiting." << std::endl;
    //     std::cerr << "Please ensure the JSON files exist and are readable." << std::endl;
    //     return 1;
    // }

    // // 3. Perform Stitching
    // MapStitcher stitcher;
    // for(const auto& map : local_maps) {
    //     stitcher.addLocalMap(map); // Add the loaded maps to the stitcher
    // }

    // std::cout << "\n--- Starting Stitching Process ---" << std::endl;
    // bool success = stitcher.stitchMaps(20); // Run optimization

    // // 4. Display Results and Save Output
    // if (success) {
    //     std::cout << "\n--- Stitching Successful ---" << std::endl;

    //     // Display optimized poses
    //     std::map<std::string, Eigen::Isometry2d> poses = stitcher.getOptimizedPoses();
    //     std::cout << "\nOptimized Poses (in Global Frame):" << std::endl;
    //     for(const auto& pair : poses) {
    //         const auto& pose = pair.second;
    //         Eigen::Rotation2Dd rot; rot.fromRotationMatrix(pose.linear());
    //         double angle_deg = rot.angle() * 180.0 / M_PI;
    //         std::cout << " - Map '" << pair.first << "': Translation(" << pose.translation().x() << ", " << pose.translation().y()
    //                   << "), Rotation(" << angle_deg << " deg)" << std::endl;
    //     }

    //     // Get the final fused map
    //     std::vector<ParkingSpace> final_map = stitcher.getGlobalMap();
    //     std::cout << "\nFinal Global Map (" << final_map.size() << " spaces):" << std::endl;
    //     // (Optional: Print details of final map to console)
    //     /*
    //     for (const auto& space : final_map) {
    //         std::cout << " - Global ID: '" << space.global_id << "' (Parking Space ID: '" << space.parking_space_id << "')" << std::endl;
    //         std::cout << "   OCR Conf: " << space.ocr_confidence << ", Vacant: " << (space.is_vacant ? "Yes" : "No") << std::endl;
    //         Point2D centroid(0,0);
    //         if (!space.corners.empty()) { for(const auto& c : space.corners) centroid += c; centroid /= static_cast<double>(space.corners.size()); }
    //         std::cout << "   Centroid (approx): (" << centroid.x() << ", " << centroid.y() << ")" << std::endl;
    //         std::cout << "   Sources: ";
    //          for(const auto& src : space.source_maps) { std::cout << src.first << "(id:'" << src.second << "') "; }
    //          std::cout << std::endl;
    //     }
    //     */

    //     // Save the final map to the specified JSON file
    //     std::cout << "\n--- Saving Global Map to JSON File ---" << std::endl;
    //     if (!saveGlobalMapToJsonFile(final_map, output_file_path)) {
    //          std::cerr << "Error: Failed to save the global map to " << output_file_path << std::endl;
    //          // Decide if this should be a fatal error for the program's return code
    //     }

    // } else {
    //     std::cerr << "\n--- Stitching Failed ---" << std::endl;
    //      return 1; // Indicate failure
    // }

    // std::cout << "\n=== Execution Finished ===" << std::endl;




    
    ros::spin();
    return 0;
}