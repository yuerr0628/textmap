#ifndef PARKING_LOT_MAPPER_H
#define PARKING_LOT_MAPPER_H

#include "parking_types.h"
#include "Hungarian.h"
#include <vector>
#include <map>


class ParkingLotMapper {
public:
    // 配置参数
    struct Config {
        double association_threshold = 0.8;
        int C_frame_discard_threshold = 20;
        int C_match_stable_threshold = 5;
        int C_unobserved_finalize_threshold = 10;
        double ocr_cost_weight_s = 0.5; // 文本代价权重
        double ocr_cost_weight_g = 0.5; // 几何代价权重
    };

    ParkingLotMapper(const Config& config);
    void update(const std::vector<MapSpot>& current_frame_spots, const VehiclePose& current_vehicle_pose);
    const std::vector<MapSpot>& getFinalMap() const;

private:
    Config m_config;
    int m_next_unique_id = 0;

    // 内部状态：正在追踪的所有车位
    std::vector<MapSpot> m_tracked_spots;
    // 最终生成的稳定地图
    std::vector<MapSpot> m_final_map;

    void manageLifecycle();
    double calculateOcrCost(const OcrResult& ocr, const VehiclePose& pose);
};

#endif // PARKING_LOT_MAPPER_H