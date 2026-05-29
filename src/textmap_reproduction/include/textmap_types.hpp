#pragma once

#include <array>
#include <map>
#include <string>
#include <vector>

namespace textmap {

struct Point2 {
  double x = 0.0;
  double y = 0.0;
};

struct Pose2 {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
};

struct Transform2 {
  double c = 1.0;
  double s = 0.0;
  double tx = 0.0;
  double ty = 0.0;
};

struct SlotObservation {
  std::string slot_id;
  double confidence = 0.0;
  std::array<Point2, 2> text_bbox{};
  std::array<Point2, 4> corners{};
  bool has_text = false;
  bool has_geometry = true;
  int vacant = 0;
};

struct FrameObservation {
  double timestamp = 0.0;
  Pose2 vehicle_pose;
  std::vector<SlotObservation> slots;
  std::string image_path;  // optional: path to frame image for VLM-based repair
};

struct MapSlot {
  int track_id = -1;
  std::string slot_id;
  double confidence = 0.0;
  double best_text_cost = 1e9;
  std::array<Point2, 2> text_bbox{};
  std::array<Point2, 4> corners{};
  int vacant = 0;
  int match_count = 0;
  int frame_count = 0;
  int unseen_count = 0;
  bool stable = false;
  std::map<std::string, double> text_votes;
};

struct TextMap {
  std::string name;
  std::vector<MapSlot> slots;
};

struct MatchPair {
  int source_index = -1;
  int target_index = -1;
  std::string slot_id;
  double error = 0.0;
};

struct RansacResult {
  Transform2 transform;
  std::vector<MatchPair> inliers;
  double inlier_ratio = 0.0;
  double rmse = 0.0;
};

struct LocalizationResult {
  bool success = false;
  Pose2 pose;
  Transform2 transform;
  std::vector<MatchPair> inliers;
  double inlier_ratio = 0.0;
  double mean_error = 0.0;
  bool vlm_repair_triggered = false;
  std::string status;
};

}  // namespace textmap
