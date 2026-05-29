#include "single_vehicle_mapper.hpp"

#include "geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace textmap {

SingleVehicleMapper::SingleVehicleMapper(MapperConfig config) : config_(config) {}

void SingleVehicleMapper::addFrame(const FrameObservation& frame) {
  std::vector<bool> matched(active_.size(), false);
  const Transform2 tf = transformFromPose(frame.vehicle_pose);

  for (auto obs : frame.slots) {
    obs.corners = transformCorners(tf, obs.corners);
    obs.text_bbox[0] = transformPoint(tf, obs.text_bbox[0]);
    obs.text_bbox[1] = transformPoint(tf, obs.text_bbox[1]);
    associateAndFuse(obs, frame.vehicle_pose, matched);
  }
  updateLifecycle(matched);
}

TextMap SingleVehicleMapper::exportMap(const std::string& name) const {
  TextMap out;
  out.name = name;
  out.slots = finalized_;
  for (const auto& s : active_) {
    if (s.stable) out.slots.push_back(s);
  }
  return out;
}

void SingleVehicleMapper::associateAndFuse(const SlotObservation& obs_world,
                                           const Pose2& vehicle_pose,
                                           std::vector<bool>& matched) {
  int best = -1;
  double best_cost = std::numeric_limits<double>::infinity();
  const Point2 obs_center = centerOf(obs_world.corners);

  for (size_t i = 0; i < active_.size(); ++i) {
    const double d = distance(obs_center, centerOf(active_[i].corners));
    const bool text_ok = obs_world.has_text && !active_[i].slot_id.empty() &&
                         obs_world.slot_id == active_[i].slot_id;
    const bool geometry_ok = d < config_.association_distance;
    if (!text_ok && !geometry_ok) continue;

    const double text_penalty = text_ok ? 0.0 : 1.0;
    const double cost = 0.65 * d + 0.35 * text_penalty;
    if (cost < best_cost) {
      best = static_cast<int>(i);
      best_cost = cost;
    }
  }

  if (best >= 0) {
    fuseInto(active_[best], obs_world, vehicle_pose);
    if (static_cast<size_t>(best) < matched.size()) matched[best] = true;
  } else {
    addNewTrack(obs_world, vehicle_pose);
  }
}

void SingleVehicleMapper::updateLifecycle(const std::vector<bool>& matched) {
  std::vector<MapSlot> keep;
  for (size_t i = 0; i < active_.size(); ++i) {
    if (i >= matched.size()) {
      keep.push_back(active_[i]);
      continue;
    }
    auto slot = active_[i];
    const bool seen = matched[i];
    slot.frame_count += 1;
    slot.unseen_count = seen ? 0 : slot.unseen_count + 1;
    if (slot.match_count >= config_.stable_match_threshold) slot.stable = true;

    const bool noise = slot.frame_count >= config_.noise_frame_threshold &&
                       slot.match_count < config_.stable_match_threshold;
    if (noise) continue;

    if (slot.stable && slot.unseen_count >= config_.finalize_unseen_threshold) {
      finalized_.push_back(slot);
    } else {
      keep.push_back(slot);
    }
  }
  active_ = std::move(keep);
}

void SingleVehicleMapper::addNewTrack(const SlotObservation& obs_world, const Pose2& vehicle_pose) {
  MapSlot slot;
  slot.track_id = next_track_id_++;
  slot.slot_id = obs_world.slot_id;
  slot.confidence = obs_world.confidence;
  slot.best_text_cost = textCost(obs_world, vehicle_pose);
  slot.text_bbox = obs_world.text_bbox;
  slot.corners = obs_world.corners;
  slot.vacant = obs_world.vacant;
  slot.match_count = 1;
  slot.frame_count = 1;
  slot.unseen_count = 0;
  slot.stable = false;
  if (!slot.slot_id.empty()) slot.text_votes[slot.slot_id] = obs_world.confidence;
  active_.push_back(slot);
}

void SingleVehicleMapper::fuseInto(MapSlot& track,
                                   const SlotObservation& obs,
                                   const Pose2& vehicle_pose) {
  const double n = static_cast<double>(std::max(1, track.match_count));
  const double w = std::max(config_.min_update_weight, std::exp(-config_.decay_lambda * (n - 1.0)));
  for (size_t i = 0; i < 4; ++i) {
    track.corners[i].x = (n * track.corners[i].x + w * obs.corners[i].x) / (n + w);
    track.corners[i].y = (n * track.corners[i].y + w * obs.corners[i].y) / (n + w);
  }
  for (size_t i = 0; i < 2; ++i) {
    track.text_bbox[i].x = (n * track.text_bbox[i].x + w * obs.text_bbox[i].x) / (n + w);
    track.text_bbox[i].y = (n * track.text_bbox[i].y + w * obs.text_bbox[i].y) / (n + w);
  }

  if (obs.has_text && obs.confidence >= config_.min_confidence) {
    track.text_votes[obs.slot_id] += obs.confidence;
    const double c = textCost(obs, vehicle_pose);
    if (c < track.best_text_cost) {
      track.best_text_cost = c;
      track.slot_id = obs.slot_id;
      track.confidence = obs.confidence;
    }
  }
  track.vacant = obs.vacant;
  track.match_count += 1;
  track.unseen_count = 0;
}

double SingleVehicleMapper::textCost(const SlotObservation& obs, const Pose2& vehicle_pose) const {
  if (!obs.has_text || obs.confidence < config_.min_confidence) return 1e9;
  const double semantic_cost = 1.0 - std::clamp(obs.confidence, 0.0, 1.0);
  const Point2 vehicle{vehicle_pose.x, vehicle_pose.y};
  const double geo_cost = std::min(1.0, distance(vehicle, centerOf(obs.text_bbox)) / config_.max_text_distance);
  return config_.text_weight * semantic_cost + config_.geometry_weight * geo_cost;
}

}  // namespace textmap
