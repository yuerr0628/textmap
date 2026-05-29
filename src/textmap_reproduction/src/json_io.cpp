#include "json_io.hpp"

#include <nlohmann/json.hpp>

#include <fstream>
#include <stdexcept>

using json = nlohmann::json;

namespace textmap {
namespace {

Point2 pointFromJson(const json& j) {
  if (j.is_array()) return {j.at(0).get<double>(), j.at(1).get<double>()};
  return {j.value("x", 0.0), j.value("y", 0.0)};
}

std::array<Point2, 4> cornersFromJson(const json& j) {
  std::array<Point2, 4> c{};
  if (j.contains("corners")) {
    for (size_t i = 0; i < 4; ++i) c[i] = pointFromJson(j.at("corners").at(i));
  } else if (j.contains("ParkingSpot")) {
    const auto& s = j.at("ParkingSpot");
    c = {{{s.value("x1", 0.0), s.value("y1", 0.0)},
          {s.value("x2", 0.0), s.value("y2", 0.0)},
          {s.value("x3", 0.0), s.value("y3", 0.0)},
          {s.value("x4", 0.0), s.value("y4", 0.0)}}};
  }
  return c;
}

std::array<Point2, 2> bboxFromJson(const json& j) {
  if (j.contains("bbox")) return {pointFromJson(j.at("bbox").at(0)), pointFromJson(j.at("bbox").at(1))};
  if (j.contains("OCRPoint")) {
    const auto& o = j.at("OCRPoint");
    return {{{o.value("x1", 0.0), o.value("y1", 0.0)}, {o.value("x2", 0.0), o.value("y2", 0.0)}}};
  }
  return {};
}

SlotObservation observationFromJson(const json& j) {
  SlotObservation obs;
  if (j.contains("OCRPoint")) {
    const auto& o = j.at("OCRPoint");
    obs.slot_id = o.value("text", "");
    obs.confidence = o.value("confidence", 0.0);
  } else {
    obs.slot_id = j.value("slot_id", j.value("id", ""));
    obs.confidence = j.value("confidence", 0.0);
  }
  obs.has_text = !obs.slot_id.empty();
  obs.text_bbox = bboxFromJson(j);
  obs.corners = cornersFromJson(j);
  if (j.contains("ParkingSpot")) obs.vacant = j.at("ParkingSpot").value("vacant", 0);
  obs.vacant = j.value("vacant", obs.vacant);
  return obs;
}

json pointToJson(const Point2& p) {
  return json::array({p.x, p.y});
}

json slotToJson(const MapSlot& s) {
  json j;
  j["track_id"] = s.track_id;
  j["slot_id"] = s.slot_id;
  j["confidence"] = s.confidence;
  j["bbox"] = json::array({pointToJson(s.text_bbox[0]), pointToJson(s.text_bbox[1])});
  j["corners"] = json::array();
  for (const auto& p : s.corners) j["corners"].push_back(pointToJson(p));
  j["vacant"] = s.vacant;
  j["match_count"] = s.match_count;
  j["frame_count"] = s.frame_count;
  j["stable"] = s.stable;
  j["text_votes"] = s.text_votes;
  return j;
}

}  // namespace

std::vector<FrameObservation> loadFrameObservations(const std::string& path) {
  std::ifstream fin(path);
  if (!fin) throw std::runtime_error("Cannot open input json: " + path);
  json root;
  fin >> root;

  std::vector<FrameObservation> frames;
  const json* frame_array = nullptr;
  if (root.is_array()) {
    const bool first_is_object = root.empty() || root.front().is_object();
    const bool looks_like_slot_array =
        first_is_object &&
        (root.empty() || root.front().contains("ParkingSpot") || root.front().contains("OCRPoint") ||
         root.front().contains("slot_id") || root.front().contains("corners"));
    if (!looks_like_slot_array) frame_array = &root;
  }
  if (root.is_object() && root.contains("frames")) frame_array = &root.at("frames");

  if (!frame_array) {
    FrameObservation f;
    const auto& slots = root.is_object() && root.contains("spots") ? root.at("spots") : root;
    for (const auto& item : slots) f.slots.push_back(observationFromJson(item));
    frames.push_back(f);
    return frames;
  }

  for (const auto& fj : *frame_array) {
    FrameObservation f;
    f.timestamp = fj.value("timestamp", 0.0);
    if (fj.contains("pose")) {
      const auto& p = fj.at("pose");
      f.vehicle_pose = {p.value("x", 0.0), p.value("y", 0.0), p.value("yaw", 0.0)};
    }
    f.image_path = fj.value("image_path", "");
    const auto& slots = fj.contains("slots") ? fj.at("slots") : fj.at("detections");
    for (const auto& item : slots) f.slots.push_back(observationFromJson(item));
    frames.push_back(f);
  }
  return frames;
}

TextMap loadTextMap(const std::string& path) {
  TextMap map;
  map.name = path;
  const auto frames = loadFrameObservations(path);
  int id = 0;
  for (const auto& frame : frames) {
    for (const auto& obs : frame.slots) {
      MapSlot s;
      s.track_id = id++;
      s.slot_id = obs.slot_id;
      s.confidence = obs.confidence;
      s.best_text_cost = 1.0 - obs.confidence;
      s.text_bbox = obs.text_bbox;
      s.corners = obs.corners;
      s.vacant = obs.vacant;
      s.match_count = 1;
      s.frame_count = 1;
      s.stable = true;
      if (!s.slot_id.empty()) s.text_votes[s.slot_id] = obs.confidence;
      map.slots.push_back(s);
    }
  }
  return map;
}

void saveTextMap(const TextMap& map, const std::string& path) {
  json root;
  root["schema"] = "textmap_reproduction/v1";
  root["name"] = map.name;
  root["spots"] = json::array();
  for (const auto& s : map.slots) root["spots"].push_back(slotToJson(s));
  std::ofstream fout(path);
  if (!fout) throw std::runtime_error("Cannot write output json: " + path);
  fout << root.dump(2);
}

void saveLocalizationResult(const LocalizationResult& result, const std::string& path) {
  json root;
  root["success"] = result.success;
  root["status"] = result.status;
  root["pose"] = {{"x", result.pose.x}, {"y", result.pose.y}, {"yaw", result.pose.yaw}};
  root["inlier_ratio"] = result.inlier_ratio;
  root["mean_error"] = result.mean_error;
  root["vlm_repair_triggered"] = result.vlm_repair_triggered;
  root["inliers"] = json::array();
  for (const auto& m : result.inliers) {
    root["inliers"].push_back({{"slot_id", m.slot_id}, {"source_index", m.source_index},
                               {"target_index", m.target_index}, {"error", m.error}});
  }
  std::ofstream fout(path);
  if (!fout) throw std::runtime_error("Cannot write localization json: " + path);
  fout << root.dump(2);
}

}  // namespace textmap
