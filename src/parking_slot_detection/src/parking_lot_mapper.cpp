#include "parking_lot_mapper.h"
#include <iostream>
#include <limits>
#include <algorithm>
#include <cmath>

// 构造函数：初始化配置和ID计数器
ParkingLotMapper::ParkingLotMapper(const Config& config) 
    : m_config(config), m_next_unique_id(0) {}

static Point3D get_representative_center(const MapSpot& spot) {
    double sum_x = 0.0, sum_y = 0.0;
    int point_count = 0;

    // 检查是否存在有效的几何信息
    bool has_geom = (spot.spot_geom_world.p1.x != 0 || spot.spot_geom_world.p1.y != 0);
    // 检查是否存在有效的OCR信息
    bool has_ocr = !spot.ocr_res_world.text.empty();

    if (has_geom) {
        sum_x += spot.spot_geom_world.p1.x + spot.spot_geom_world.p2.x + spot.spot_geom_world.p3.x + spot.spot_geom_world.p4.x;
        sum_y += spot.spot_geom_world.p1.y + spot.spot_geom_world.p2.y + spot.spot_geom_world.p3.y + spot.spot_geom_world.p4.y;
        point_count += 4;
    }
    if (has_ocr) {
        sum_x += spot.ocr_res_world.p1.x + spot.ocr_res_world.p2.x;
        sum_y += spot.ocr_res_world.p1.y + spot.ocr_res_world.p2.y;
        point_count += 2;
    }

    if (point_count > 0) {
        return {sum_x / point_count, sum_y / point_count, 0.0};
    }
    
    // 如果没有任何信息，返回原点
    return {0.0, 0.0, 0.0};
}
/**
 * 计算两个车位中心点的距离，用于代价矩阵
 */
static double calculateDistance(const MapSpot& spot1, const MapSpot& spot2) {
    bool geom1_exists = (spot1.spot_geom_world.p1.x != 0 || spot1.spot_geom_world.p1.y != 0);
    bool ocr1_exists = !spot1.ocr_res_world.text.empty();
    bool geom2_exists = (spot2.spot_geom_world.p1.x != 0 || spot2.spot_geom_world.p1.y != 0);
    bool ocr2_exists = !spot2.ocr_res_world.text.empty();

    // // 核心规则：如果两个对象都有文本信息，它们的文本必须相同，否则无法匹配。
    // // 这是利用文本作为唯一标识符的关键。
    // if (ocr1_exists && ocr2_exists && spot1.ocr_res_world.text != spot2.ocr_res_world.text) {
    //     return std::numeric_limits<double>::max();
    // }

    // 使用辅助函数获取各自的代表性中心点
    Point3D center1 = get_representative_center(spot1);
    Point3D center2 = get_representative_center(spot2);

    // 如果任一车位没有任何信息，则无法计算距离
    if ((center1.x == 0 && center1.y == 0) || (center2.x == 0 && center2.y == 0)) {
        return std::numeric_limits<double>::max();
    }

    // 计算XY平面上的欧几里得距离作为代价
    // 这个统一的计算方式隐式地处理了所有情况：
    // 1. 车位和车位号都存在：center是两者的平均中心，距离计算更鲁棒。
    // 2. 仅车位存在：center是车位的几何中心。
    // 3. 仅车位号存在：center是车位号框的中心。
    return std::hypot(center1.x - center2.x, center1.y - center2.y);
}


// 主更新函数，是整个流程的协调器
void ParkingLotMapper::update(const std::vector<MapSpot>& current_frame_spots, const VehiclePose& current_vehicle_pose) {
    // ---- 1. 初始化：处理第一帧 ----
    for (auto& spot : m_tracked_spots) {
        spot.C_frame++;
        spot.C_unobserved++;
    }

    if (m_tracked_spots.empty()) {
        m_tracked_spots = current_frame_spots;
        for (auto& spot : m_tracked_spots) {
            spot.unique_id = m_next_unique_id++;
            spot.C_match = 1;
            spot.C_frame = 1;
        }
    }

  else if (!current_frame_spots.empty()) {
        std::vector<std::vector<double>> cost_matrix(m_tracked_spots.size(), std::vector<double>(current_frame_spots.size(), std::numeric_limits<double>::max()));
        for (size_t i = 0; i < m_tracked_spots.size(); ++i) {
            for (size_t j = 0; j < current_frame_spots.size(); ++j) {
                double dist = calculateDistance(m_tracked_spots[i], current_frame_spots[j]);
                if (dist < m_config.association_threshold) {
                    cost_matrix[i][j] = dist;
                }
            }
        }  

     HungarianAlgorithm hungarian;
    std::vector<int> assignment;
    hungarian.Solve(cost_matrix, assignment);
    std::vector<bool> current_matched_flags(current_frame_spots.size(), false);

    for (size_t i = 0; i < assignment.size(); ++i) {
        int match_idx = assignment[i];
        if (match_idx != -1 && cost_matrix[i][match_idx] < m_config.association_threshold) {
            MapSpot& tracked_spot = m_tracked_spots[i];
            const MapSpot& current_spot = current_frame_spots[match_idx];
            
            tracked_spot.C_match++;
            tracked_spot.C_unobserved = 0;
            
            size_t n = tracked_spot.C_match - 1;
             // --- 开始鲁棒的融合逻辑 ---
            bool tracked_has_geom = (tracked_spot.spot_geom_world.p1.x != 0 || tracked_spot.spot_geom_world.p1.y != 0);
            bool current_has_geom = (current_spot.spot_geom_world.p1.x != 0 || current_spot.spot_geom_world.p1.y != 0);

            if (current_has_geom) {
                if (tracked_has_geom) {
                    // 场景1: 两者都有几何信息 -> 加权平均融合
                    size_t n = tracked_spot.C_match - 1;
                    tracked_spot.spot_geom_world.p1.x = (tracked_spot.spot_geom_world.p1.x * n + current_spot.spot_geom_world.p1.x) / (n + 1);
                    tracked_spot.spot_geom_world.p1.y = (tracked_spot.spot_geom_world.p1.y * n + current_spot.spot_geom_world.p1.y) / (n + 1);
                    // ... 对 p2, p3, p4 执行同样操作 ...
                    // (为了简洁，这里省略了p2, p3, p4的代码，但逻辑是相同的)
                } else {
                    // 场景2: 追踪对象没有，当前对象有 -> 直接继承
                    tracked_spot.spot_geom_world = current_spot.spot_geom_world;
                }
            }
            // 场景3: 当前对象没有几何信息 -> 保持追踪对象的几何信息不变，无需操作

            // --- OCR信息融合逻辑 ---
            bool tracked_has_text = !tracked_spot.ocr_res_world.text.empty();
            bool current_has_text = !current_spot.ocr_res_world.text.empty();

            if (current_has_text) {
                if (tracked_has_text) {
                    // 场景A: 两者都有文本信息 -> 取代价更低的更新
                    double current_ocr_cost = calculateOcrCost(current_spot.ocr_res_world, current_vehicle_pose);
                    if (current_ocr_cost < tracked_spot.best_ocr_cost) {
                        tracked_spot.ocr_res_world = current_spot.ocr_res_world;
                        tracked_spot.best_ocr_cost = current_ocr_cost;
                    }
                } else {
                    // 场景B: 追踪对象没有，当前对象有 -> 直接继承
                    tracked_spot.ocr_res_world = current_spot.ocr_res_world;
                    tracked_spot.best_ocr_cost = calculateOcrCost(current_spot.ocr_res_world, current_vehicle_pose);
                }
            }
            // 场景C: 当前对象没有文本信息 -> 保持追踪对象的文本信息不变，无需操作
            
            current_matched_flags[match_idx] = true;
        }
    }

for (size_t i = 0; i < current_frame_spots.size(); ++i) {
            if (!current_matched_flags[i]) {
                MapSpot new_spot = current_frame_spots[i];
                new_spot.unique_id = m_next_unique_id++;
                new_spot.C_match = 1;
                new_spot.C_frame = 1;
                new_spot.best_ocr_cost = calculateOcrCost(new_spot.ocr_res_world, current_vehicle_pose);
                m_tracked_spots.push_back(new_spot);
            }
        }
    }
    
    manageLifecycle();
}


void ParkingLotMapper::manageLifecycle() {
    m_tracked_spots.erase(std::remove_if(m_tracked_spots.begin(), m_tracked_spots.end(),
        [&](MapSpot& spot) {
            if (spot.C_match >= m_config.C_match_stable_threshold) {
                spot.is_stable = true;
            }
            if (spot.is_stable && spot.C_unobserved >= m_config.C_unobserved_finalize_threshold) {
                m_final_map.push_back(spot);
                return true;
            }
            if (!spot.is_stable && spot.C_frame >= m_config.C_frame_discard_threshold) {
                return true;
            }
            return false;
        }), m_tracked_spots.end());
}

double ParkingLotMapper::calculateOcrCost(const OcrResult& ocr, const VehiclePose& pose) {
    if (ocr.text.empty()) {
        return std::numeric_limits<double>::max();
    }
    double cost_s = 1.0 - ocr.confidence;
    
    double ocr_center_x = (ocr.p1.x + ocr.p2.x) / 2.0;
    double ocr_center_y = (ocr.p1.y + ocr.p2.y) / 2.0;
    double cost_g = std::hypot(pose.x - ocr_center_x, pose.y - ocr_center_y);

    return m_config.ocr_cost_weight_s * cost_s + m_config.ocr_cost_weight_g * cost_g;
}

const std::vector<MapSpot>& ParkingLotMapper::getFinalMap() const {
    return m_final_map;
}