#ifndef MAP_STITCHER_H
#define MAP_STITCHER_H

#include <vector>
#include "map_types.h" // Include the common types
#include <string>
#include <map>
#include <optional>
#include <Eigen/Core>
#include <Eigen/Geometry> // For Isometry2d

// Forward declarations for g2o types if needed, or include directly
namespace g2o {
    class SparseOptimizer;
    class VertexSE2;
    class EdgeSE2;
}

using Point2D = Eigen::Vector2d;

// struct ParkingSpace {
//     int local_id;
//     std::vector<Point2D> corners; // Assumed order (e.g., clockwise)
//     // --- Optional fields after fusion ---
//     int global_id = -1; // Unique ID in the final map
//     std::vector<std::pair<std::string, int>> source_maps; // Track origin (map_id, local_id)
// };

// struct LocalMap {
//     std::string map_id;
//     std::vector<ParkingSpace> spaces;
//     // --- Optional fields after optimization ---
//     Eigen::Isometry2d optimized_pose = Eigen::Isometry2d::Identity(); // Pose in global frame
//     int g2o_vertex_id = -1;
// };

struct PairwiseMatchResult {
    int map_idx_i;
    int map_idx_j;
    Eigen::Isometry2d transform_j_to_i; // Transformation from map j's frame to map i's frame
    Eigen::Matrix3d information_matrix; // Confidence in the transform
    std::vector<std::pair<std::string, std::string>> matched_ids; // List of (local_id_i, local_id_j) pairs used
};

class MapStitcher {
public:
    MapStitcher();
    ~MapStitcher(); // Ensure optimizer is cleaned up if dynamically allocated

    // Add local maps to be processed
    void addLocalMap(const LocalMap& map);

    // Perform pairwise matching and global optimization
    bool stitchMaps(int optimization_iterations = 10);

    // Get the final fused map
    std::vector<ParkingSpace> getGlobalMap() const;

    // Get the optimized poses of local maps
    std::map<std::string, Eigen::Isometry2d> getOptimizedPoses() const;


private:
    std::vector<LocalMap> local_maps_;
    std::vector<PairwiseMatchResult> pairwise_matches_;
    std::map<std::string, const ParkingSpace*> map_i_spaces_by_id_; // Helper map for current map_i
    std::map<std::string, const ParkingSpace*> map_j_spaces_by_id_; // Helper map for current map_j

    // Finds shared IDs and estimates relative transform
    std::optional<PairwiseMatchResult> findMatchAndEstimateTransform(int map_idx_i, int map_idx_j);

    // Estimates SE(2) transform from points_from to points_to using SVD
    std::optional<Eigen::Isometry2d> estimateRigidTransform(
        const std::vector<Point2D>& points_from,
        const std::vector<Point2D>& points_to);

    // Builds and runs the g2o pose graph
    bool runGlobalOptimization(g2o::SparseOptimizer& optimizer, int iterations);

    // Fuses maps using optimized poses
    void fuseMaps();

    std::vector<ParkingSpace> global_map_; // The final result
    bool optimization_done_ = false;

     // --- Helper: Build temporary lookup maps for matching ---
    void buildIdLookup(int map_idx, std::map<std::string, const ParkingSpace*>& lookup);
};


#endif // MAP_STITCHER_H