#include "map_stitcher.h"
#include <iostream>
#include <map>
#include <vector>
#include <cmath> // For std::sqrt

#include <Eigen/SVD> // For SVD

// --- g2o Includes ---
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h" // Or other solvers
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/edge_se2.h"

using  namespace std;


MapStitcher::MapStitcher() = default;
MapStitcher::~MapStitcher() = default;

//add a new local map to the list,将外部加载的地图添加到内部的localmap向量中
void MapStitcher::addLocalMap(const LocalMap& map) {
    local_maps_.push_back(map);
    optimization_done_ = false; // Adding maps invalidates previous optimization
    global_map_.clear();       // Clear previous fusion result
    cout<< "Added local map "<<map.map_id<<" with "<<map.spaces.size()<<" space "<<endl;

}

// Helper to build ID lookup for a map，创建一个从local_id到车位指针的map，用于快速查找
void MapStitcher::buildIdLookup(int map_idx, std::map<std::string, const ParkingSpace*>& lookup) {
    lookup.clear();
    if (map_idx < 0 || map_idx >= local_maps_.size()) return;
    for (const auto& space : local_maps_[map_idx].spaces) {
        lookup[space.local_id] = &space;
    }
}


std::optional<PairwiseMatchResult> MapStitcher::findMatchAndEstimateTransform(int map_idx_i, int map_idx_j) {
    if (map_idx_i < 0 || map_idx_i >= local_maps_.size() ||
        map_idx_j < 0 || map_idx_j >= local_maps_.size() ||
        map_idx_i == map_idx_j) {
        return std::nullopt;
    }

    const auto& map_i = local_maps_[map_idx_i];
    const auto& map_j = local_maps_[map_idx_j];

    // Build quick ID lookups
    buildIdLookup(map_idx_i, map_i_spaces_by_id_);
    buildIdLookup(map_idx_j, map_j_spaces_by_id_);


    std::vector<Point2D> points_i; // Points in map i's frame (destination)
    std::vector<Point2D> points_j; // Corresponding points in map j's frame (source)
    std::vector<std::pair<string, string>> matched_ids;

    // Find spaces with matching IDs
    for (const auto& [id_i, space_ptr_i] : map_i_spaces_by_id_) {
        auto it = map_j_spaces_by_id_.find(id_i); // Find same ID in map j
        if (it != map_j_spaces_by_id_.end()) {
            const auto& space_j = *(it->second);
            // Ensure corner counts match (basic sanity check)
            if (space_ptr_i->corners.size() == space_j.corners.size() && !space_ptr_i->corners.empty()) {
                matched_ids.push_back({space_ptr_i->local_id, space_j.local_id});
                // Add all corresponding corners
                for (size_t k = 0; k < space_ptr_i->corners.size(); ++k) {
                    points_i.push_back(space_ptr_i->corners[k]);
                    points_j.push_back(space_j.corners[k]);
                }
            }
        }
    }

    if (points_i.size() < 3) { // Need at least 3 non-collinear points for SE(2)
        std::cerr << "Warning: Not enough corresponding corner points (" << points_i.size()
                  << ") found between map " << map_i.map_id << " and " << map_j.map_id << " based on matching IDs." << std::endl;
        return std::nullopt;
    }

    // Estimate transformation T_j_to_i (transforms points from j's frame to i's frame)
    auto transform_opt = estimateRigidTransform(points_j, points_i);

    if (!transform_opt) {
        std::cerr << "Warning: Failed to estimate transform between map "
                  << map_i.map_id << " and " << map_j.map_id << std::endl;
        return std::nullopt;
    }

    PairwiseMatchResult result;
    result.map_idx_i = map_idx_i;
    result.map_idx_j = map_idx_j;
    result.transform_j_to_i = *transform_opt;
    result.matched_ids = matched_ids;

    // --- Define Information Matrix (Confidence) ---
    // Simple example: higher confidence for more points/matches
    // This needs tuning based on sensor noise, etc.
    double confidence_factor = static_cast<double>(matched_ids.size()); // Or points_i.size()
    result.information_matrix = Eigen::Matrix3d::Identity() * confidence_factor;
    // You might want to anisotropically scale confidence (e.g., less confident in rotation vs translation)

    std::cout << "Found match: " << map_i.map_id << " <-> " << map_j.map_id
              << " with " << matched_ids.size() << " matching IDs (" << points_i.size() << " points)." << std::endl;

    return result;
}

std::optional<Eigen::Isometry2d> MapStitcher::estimateRigidTransform(
    const std::vector<Point2D>& points_from,
    const std::vector<Point2D>& points_to)
{
    if (points_from.size() != points_to.size() || points_from.size() < 2) {
        std::cerr << "Error: Need at least 2 corresponding points for rigid transform." << std::endl;
        return std::nullopt;
    }

    // Convert std::vector<Eigen::Vector2d> to Eigen::MatrixXd (Nx2)
    Eigen::MatrixXd P_from(points_from.size(), 2);
    Eigen::MatrixXd P_to(points_to.size(), 2);
    for (size_t i = 0; i < points_from.size(); ++i) {
        P_from.row(i) = points_from[i];
        P_to.row(i) = points_to[i];
    }

    // Calculate centroids
    Point2D centroid_from = P_from.colwise().mean();
    Point2D centroid_to = P_to.colwise().mean();

    // Center the points
    Eigen::MatrixXd Q_from = P_from.rowwise() - centroid_from.transpose();
    Eigen::MatrixXd Q_to = P_to.rowwise() - centroid_to.transpose();

    // Calculate covariance matrix H
    Eigen::MatrixXd H = Q_from.transpose() * Q_to;

    // SVD
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(H, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::MatrixXd U = svd.matrixU();
    Eigen::MatrixXd V = svd.matrixV();

    // Calculate rotation R
    Eigen::Matrix2d R = V * U.transpose();

    // Handle potential reflection
    if (R.determinant() < 0) {
        // std::cout << "Reflection detected, correcting R." << std::endl;
        V.col(1) *= -1; // Flip the sign of the last column of V
        R = V * U.transpose();
    }

    // Calculate translation t
    Point2D t = centroid_to - R * centroid_from;

    // Create SE(2) transformation
    Eigen::Isometry2d transform = Eigen::Isometry2d::Identity();
    transform.linear() = R;
    transform.translation() = t;

    return transform;
}


bool MapStitcher::runGlobalOptimization(g2o::SparseOptimizer& optimizer, int iterations) {
     // --- Setup g2o ---
    // Define block solver type (SE2 pose, 3x3 matrix)
    using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<3, 3>>;
    // Define linear solver type (Eigen based)
    using LinearSolverType = g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;

    // Create solver
    auto linearSolver = std::make_unique<LinearSolverType>();
    linearSolver->setBlockOrdering(false); // Optional: depends on solver/problem structure
    auto blockSolver = std::make_unique<BlockSolverType>(std::move(linearSolver));
    auto algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));

    optimizer.setAlgorithm(algorithm);
    optimizer.setVerbose(true); // Print optimization progress


    std::cout << "Running g2o Global Optimization..." << std::endl;

    // --- Add Vertices (Poses) ---
    int max_vertex_id = -1;
    for (size_t i = 0; i < local_maps_.size(); ++i) {
        auto& map = local_maps_[i];
        g2o::VertexSE2* v = new g2o::VertexSE2();
        v->setId(i); // Use map index as vertex ID
        map.g2o_vertex_id = i; // Store the vertex ID
        // Initialize pose (e.g., origin for first, identity or pairwise for others)
        // For simplicity, initialize all but first to Identity. A better init helps convergence.
         v->setEstimate(Eigen::Isometry2d::Identity());
        if (i == 0) {
            v->setFixed(true); // Fix the first map's pose as the global origin
        }
        optimizer.addVertex(v);
        if (i > max_vertex_id) max_vertex_id = i;
    }
     std::cout << "Added " << optimizer.vertices().size() << " vertices." << std::endl;

    // --- Add Edges (Constraints from Pairwise Matches) ---
    int edge_count = 0;
    for (const auto& match : pairwise_matches_) {
         if (match.map_idx_i < 0 || match.map_idx_i >= local_maps_.size() ||
             match.map_idx_j < 0 || match.map_idx_j >= local_maps_.size()) continue;

        g2o::EdgeSE2* edge = new g2o::EdgeSE2();
        edge->setVertex(0, optimizer.vertex(local_maps_[match.map_idx_i].g2o_vertex_id)); // From vertex i
        edge->setVertex(1, optimizer.vertex(local_maps_[match.map_idx_j].g2o_vertex_id)); // To vertex j

        // Measurement is T_j_to_i (transform from j to i)
        edge->setMeasurement(match.transform_j_to_i);
        edge->setInformation(match.information_matrix);

        optimizer.addEdge(edge);
        edge_count++;
    }
    std::cout << "Added " << edge_count << " edges." << std::endl;

    if (edge_count == 0 && local_maps_.size() > 1) {
        std::cerr << "Warning: No pairwise constraints found. Optimization might not be meaningful." << std::endl;
        // We can still proceed, but only the first map will be placed.
    }


    // --- Run Optimization ---
    optimizer.initializeOptimization();
    // optimizer.computeActiveErrors(); // Optional: check initial error
    // std::cout << "Initial chi2 = " << optimizer.chi2() << std::endl;

    int result = optimizer.optimize(iterations); // Number of iterations

    std::cout << "Optimization finished after " << result << " iterations. Final chi2 = " << optimizer.chi2() << std::endl;

    if (result <= 0) {
         std::cerr << "Error: g2o optimization failed or did not converge properly." << std::endl;
         // You might want to check optimizer.forceStopFlag() or other status indicators
         return false;
    }


    // --- Update Map Poses from Optimizer Results ---
    for (size_t i = 0; i < local_maps_.size(); ++i) {
        g2o::VertexSE2* v = static_cast<g2o::VertexSE2*>(optimizer.vertex(local_maps_[i].g2o_vertex_id));
        if (v) {
            // local_maps_[i].optimized_pose = v->estimate();
            g2o::SE2 pose = v->estimate();
            double x = pose.translation().x();
            double y = pose.translation().y();
            double theta = pose.rotation().angle();

            // Construct Eigen::Isometry2d
            Eigen::Isometry2d isometry = Eigen::Isometry2d::Identity();
            isometry.rotate(Eigen::Rotation2Dd(theta)); // Set rotation
            isometry.pretranslate(Eigen::Vector2d(x, y)); // Set translation
            local_maps_[i].optimized_pose = isometry;
        } else {
             std::cerr << "Error: Could not retrieve optimized pose for map " << local_maps_[i].map_id << std::endl;
             // Fallback? Keep identity?
        }
    }

    return true;
}


void MapStitcher::fuseMaps() {
    global_map_.clear();
    // Use a map to handle merging based on the *original local ID*, assuming it's globally meaningful
    std::map<string, ParkingSpace> merged_spaces_by_id;
    int next_global_id_counter = 1001; // Starting ID for potentially new global IDs

    for (const auto& map : local_maps_) {
        const Eigen::Isometry2d& global_pose = map.optimized_pose;

        for (const auto& local_space : map.spaces) {
            const string& current_local_id = local_space.local_id;

            // Transform corners to global frame
            std::vector<Point2D> global_corners;
            global_corners.reserve(local_space.corners.size());
            for (const auto& local_corner : local_space.corners) {
                global_corners.push_back(global_pose * local_corner); // Apply SE(2) transform
            }

            auto it = merged_spaces_by_id.find(current_local_id);

            if (it == merged_spaces_by_id.end()) {
                // --- First time seeing this ID ---
                ParkingSpace global_space;
                global_space.local_id = current_local_id; // Store original ID for reference
                global_space.global_id = current_local_id; // ASSUME local ID acts as global ID
                                                           // OR: global_space.global_id = next_global_id_counter++;
                global_space.corners = global_corners;
                global_space.source_maps.push_back({map.map_id, local_space.local_id});
                merged_spaces_by_id[current_local_id] = global_space;
            } else {
                // --- Already saw this ID - Merge Strategy ---
                // Strategy 1: Keep First (do nothing here)

                // Strategy 2: Average Corners (simplistic, can distort shape)
                
                ParkingSpace& existing_space = it->second;
                if (existing_space.corners.size() == global_corners.size()&& !existing_space.corners.empty()) {
                    for(size_t k=0; k < existing_space.corners.size(); ++k) {
                        existing_space.corners[k] = (existing_space.corners[k] * existing_space.source_maps.size() + global_corners[k]) / (existing_space.source_maps.size() + 1.0);
                    }
                }
                // else
                // Strategy 3: Select based on confidence (needs confidence metric)

                // --- Always update source map list ---
                //  ParkingSpace& existing_space = it->second;
                 existing_space.source_maps.push_back({map.map_id, local_space.local_id});
            }
        }
    }

    // Transfer merged spaces to the final global_map_ vector
    global_map_.reserve(merged_spaces_by_id.size());
    for (auto const& [id, space] : merged_spaces_by_id) {
        global_map_.push_back(space);
    }
     std::cout << "Fused map contains " << global_map_.size() << " unique spaces." << std::endl;
}


bool MapStitcher::stitchMaps(int optimization_iterations) {
    if (local_maps_.size() < 2) {
        std::cout << "Only one map provided, no stitching needed. Using it as the global map." << std::endl;
         if (!local_maps_.empty()) {
             // Simple fusion for one map
             global_map_.clear();
             int gid = 1001;
             for(const auto& space : local_maps_[0].spaces) {
                 ParkingSpace gs = space;
                 gs.global_id = gid++; // Assign some global ID
                 gs.source_maps.push_back({local_maps_[0].map_id, space.local_id});
                 global_map_.push_back(gs);
             }
             local_maps_[0].optimized_pose = Eigen::Isometry2d::Identity(); // Set pose for consistency
             optimization_done_ = true;
         }
        return true; // Technically successful, nothing to stitch.
    }

    pairwise_matches_.clear();
    std::cout << "Finding pairwise matches based on shared local IDs..." << std::endl;
    for (size_t i = 0; i < local_maps_.size(); ++i) {
        for (size_t j = i + 1; j < local_maps_.size(); ++j) {
            auto match_opt = findMatchAndEstimateTransform(i, j);
            if (match_opt) {
                pairwise_matches_.push_back(*match_opt);
            }
        }
    }

     if (pairwise_matches_.empty() && local_maps_.size() > 1) {
         std::cerr << "Warning: No pairwise matches found between any maps based on shared IDs." << std::endl;
         // Cannot proceed with graph optimization without constraints.
         // We could still output individual maps at origin, or return false.
         // Let's fuse them all at origin for now.
         local_maps_[0].optimized_pose = Eigen::Isometry2d::Identity();
          for (size_t i = 1; i < local_maps_.size(); ++i) {
               local_maps_[i].optimized_pose = Eigen::Isometry2d::Identity(); // Place others at origin too
          }
         fuseMaps(); // Fuse based on these (likely incorrect) poses
         optimization_done_ = true; // Mark as 'done' even if no optimization ran
         return false; // Indicate that stitching didn't really happen
     }


    // --- Setup and Run g2o ---
    g2o::SparseOptimizer optimizer; // Allocate optimizer on the stack or heap
    bool optim_success = runGlobalOptimization(optimizer, optimization_iterations);


    if (optim_success) {
        // --- Fuse Maps using Optimized Poses ---
        fuseMaps();
        optimization_done_ = true;
        return true;
    } else {
        std::cerr << "Global optimization failed. Map fusion might be inaccurate." << std::endl;
         // Optionally, fuse based on initial poses or return false
        optimization_done_ = false; // Indicate optimization failed
        return false;
    }
}

std::vector<ParkingSpace> MapStitcher::getGlobalMap() const {
     if (!optimization_done_) {
        std::cerr << "Warning: Requesting global map, but stitching/optimization has not been successfully run or is outdated." << std::endl;
    }
    return global_map_;
}


std::map<std::string, Eigen::Isometry2d> MapStitcher::getOptimizedPoses() const {
     if (!optimization_done_) {
        std::cerr << "Warning: Requesting optimized poses, but stitching/optimization has not been successfully run or is outdated." << std::endl;
    }
    std::map<std::string, Eigen::Isometry2d> poses;
     for(const auto& map : local_maps_) {
         poses[map.map_id] = map.optimized_pose;
     }
     return poses;
}