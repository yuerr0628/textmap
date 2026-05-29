#pragma once

#include "textmap_types.hpp"

namespace textmap {

struct MapperConfig {
  double association_distance = 0.5;
  double text_weight = 0.7;
  double geometry_weight = 0.3;
  double max_text_distance = 8.0;
  double min_confidence = 0.2;
  int stable_match_threshold = 5;
  int noise_frame_threshold = 20;
  int finalize_unseen_threshold = 10;
  double decay_lambda = 0.25;
  double min_update_weight = 0.15;
};

class SingleVehicleMapper {
 public:
  explicit SingleVehicleMapper(MapperConfig config = {});

  void addFrame(const FrameObservation& frame);
  TextMap exportMap(const std::string& name = "single_vehicle_map") const;

 private:
  MapperConfig config_;
  int next_track_id_ = 0;
  std::vector<MapSlot> active_;
  std::vector<MapSlot> finalized_;

  void associateAndFuse(const SlotObservation& obs_world,
                        const Pose2& vehicle_pose,
                        std::vector<bool>& matched);
  void updateLifecycle(const std::vector<bool>& matched);
  void addNewTrack(const SlotObservation& obs_world, const Pose2& vehicle_pose);
  void fuseInto(MapSlot& track, const SlotObservation& obs, const Pose2& vehicle_pose);
  double textCost(const SlotObservation& obs, const Pose2& vehicle_pose) const;
};

}  // namespace textmap
