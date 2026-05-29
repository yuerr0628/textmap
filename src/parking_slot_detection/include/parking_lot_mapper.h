#ifndef PARKING_LOT_MAPPER_H
#define PARKING_LOT_MAPPER_H

#include "Hungarian.h"
#include "parking_types.h"
#include <vector>

class ParkingLotMapper {
public:
    struct Config {
        double association_threshold = 2.0;
        double text_gate_cost = 0.35;
        double geometry_gate = 2.5;
        double text_cost_weight = 0.55;
        double geometry_cost_weight = 0.45;
        double view_distance_weight = 0.08;
        int C_frame_discard_threshold = 20;
        int C_match_stable_threshold = 4;
        int C_unobserved_finalize_threshold = 8;
    };

    explicit ParkingLotMapper(const Config& config = Config());

    void update(const std::vector<MapSpot>& current_frame_spots,
                const VehiclePose& current_vehicle_pose);
    const std::vector<MapSpot>& getFinalMap() const;
    const std::vector<MapSpot>& getTrackedMap() const;

private:
    Config m_config;
    int m_next_unique_id = 0;
    std::vector<MapSpot> m_tracked_spots;
    std::vector<MapSpot> m_final_map;

    void mergeSpot(MapSpot& tracked, const MapSpot& observed,
                   const VehiclePose& current_vehicle_pose);
    void updateTextVotes(MapSpot& tracked, const OcrResult& ocr,
                         const VehiclePose& current_vehicle_pose);
    void refreshBestText(MapSpot& spot);
    void manageLifecycle();
    double calculateAssociationCost(const MapSpot& tracked,
                                    const MapSpot& observed) const;
    double calculateOcrCost(const OcrResult& ocr,
                            const VehiclePose& pose) const;
};

#endif
