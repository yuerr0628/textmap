#include "localization.hpp"

#include "geometry.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <sstream>
#include <unordered_map>

using json = nlohmann::json;

namespace textmap {
namespace {

std::string encodeImageToBase64(const std::string& image_path) {
  std::ifstream file(image_path, std::ios::binary);
  if (!file) return "";
  std::ostringstream oss;
  oss << file.rdbuf();
  const std::string raw = oss.str();
  if (raw.empty()) return "";

  static const char table[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  std::string encoded;
  encoded.reserve(((raw.size() + 2) / 3) * 4);
  for (size_t i = 0; i < raw.size(); i += 3) {
    unsigned int n = (static_cast<unsigned char>(raw[i]) << 16);
    if (i + 1 < raw.size()) n |= (static_cast<unsigned char>(raw[i + 1]) << 8);
    if (i + 2 < raw.size()) n |= static_cast<unsigned char>(raw[i + 2]);
    encoded.push_back(table[(n >> 18) & 0x3F]);
    encoded.push_back(table[(n >> 12) & 0x3F]);
    encoded.push_back((i + 1 < raw.size()) ? table[(n >> 6) & 0x3F] : '=');
    encoded.push_back((i + 2 < raw.size()) ? table[n & 0x3F] : '=');
  }
  return encoded;
}

}  // namespace

TextGeometryLocalizer::TextGeometryLocalizer(LocalizationConfig config) : config_(std::move(config)) {
  if (config_.enable_vlm_repair) {
    api_client_ = std::make_unique<QwenApiClient>(config_.api_config);
  }
}

LocalizationResult TextGeometryLocalizer::localize(const TextMap& global_map,
                                                   const FrameObservation& frame) const {
  const auto matches = idMatches(global_map, frame);
  auto ransac = estimateInitialPose(global_map, frame, matches);
  LocalizationResult result;
  result.transform = ransac.transform;
  result.inliers = ransac.inliers;
  result.inlier_ratio = ransac.inlier_ratio;
  result.mean_error = ransac.rmse;

  if (!ransac.inliers.empty()) {
    result.transform = refineTextGeometryIcp(global_map, frame, ransac.transform, ransac.inliers);
    double sum = 0.0;
    for (auto& m : result.inliers) {
      m.error = cornerRmse(frame.slots[m.source_index].corners,
                           global_map.slots[m.target_index].corners,
                           result.transform);
      sum += m.error;
    }
    result.mean_error = result.inliers.empty() ? 0.0 : sum / result.inliers.size();
  }
  result.pose = poseFromTransform(result.transform);
  result.success = !isAbnormal(result, static_cast<int>(matches.size()));
  result.status = result.success ? "ok" : "abnormal";

  if (!result.success && config_.enable_vlm_repair) {
    FrameObservation repaired = repairWithVlm(global_map, frame);
    auto retry_config = config_;
    retry_config.enable_vlm_repair = false;
    TextGeometryLocalizer retry_localizer(retry_config);
    auto retry = retry_localizer.localize(global_map, repaired);
    retry.vlm_repair_triggered = true;
    if (retry.success) return retry;
    result.vlm_repair_triggered = true;
  }
  return result;
}

FrameObservation TextGeometryLocalizer::repairWithVlm(const TextMap& map,
                                                      const FrameObservation& frame) const {
  FrameObservation repaired = frame;

  // Count how many slots need repair
  int missing_count = 0;
  for (const auto& obs : repaired.slots) {
    if (obs.slot_id.empty()) ++missing_count;
  }
  if (missing_count == 0) return repaired;

  // If we have an image and API client, call Qwen-VL
  if (api_client_ && !frame.image_path.empty()) {
    std::string image_b64 = encodeImageToBase64(frame.image_path);
    if (!image_b64.empty()) {
      try {
        // Build a prompt describing what we need
        std::ostringstream prompt;
        prompt << "This is a bird's-eye view (BEV) image of a parking area. "
               << "I can see " << repaired.slots.size() << " parking slots. "
               << "Some slots have unreadable text IDs. "
               << "Please identify ALL parking slot numbers visible in this image. "
               << "Output ONLY a JSON array in order from left to right, top to bottom: "
               << "[{\"idx\":0,\"id\":\"slot_number\"}, ...]\n\n"
               << "Known slot IDs (from OCR, may have errors):\n[";
        for (size_t i = 0; i < repaired.slots.size(); ++i) {
          prompt << "{\"idx\":" << i
                 << ",\"id\":\"" << repaired.slots[i].slot_id << "\""
                 << ",\"conf\":" << repaired.slots[i].confidence << "}";
          if (i + 1 != repaired.slots.size()) prompt << ",";
        }
        prompt << "]\n\nFill in the empty IDs and correct low-confidence ones.";

        const std::string response = api_client_->chatVlm(prompt.str(), image_b64);

        // Parse VLM response
        std::string json_str = response;
        auto arr_start = json_str.find('[');
        auto arr_end = json_str.rfind(']');
        if (arr_start != std::string::npos && arr_end != std::string::npos && arr_end > arr_start) {
          json_str = json_str.substr(arr_start, arr_end - arr_start + 1);
          json corrections = json::parse(json_str);
          for (const auto& item : corrections) {
            int idx = item.value("idx", -1);
            std::string new_id = item.value("id", "");
            if (idx >= 0 && idx < static_cast<int>(repaired.slots.size()) && !new_id.empty()) {
              if (repaired.slots[idx].slot_id.empty() || repaired.slots[idx].confidence < 0.5) {
                repaired.slots[idx].slot_id = new_id;
                repaired.slots[idx].confidence = 0.6;
                repaired.slots[idx].has_text = true;
              }
            }
          }
        }
        return repaired;
      } catch (const std::exception& e) {
        std::cerr << "[TextGeometryLocalizer] VLM API call failed: " << e.what() << "\n";
      }
    }
  }

  // Fallback if no image or API fails: use map neighbor heuristic with QwQ-32B text reasoning
  if (api_client_) {
    try {
      std::ostringstream prompt;
      prompt << "Given a parking map and current frame observations, "
             << "some parking slot IDs are missing. Based on the known IDs and their positions, "
             << "infer the missing IDs.\n\nMap slots nearby:\n[";
      for (size_t i = 0; i < std::min<size_t>(map.slots.size(), 30); ++i) {
        const auto c = centerOf(map.slots[i].corners);
        prompt << "{\"id\":\"" << map.slots[i].slot_id << "\",\"x\":" << c.x << ",\"y\":" << c.y << "}";
        if (i + 1 < std::min<size_t>(map.slots.size(), 30)) prompt << ",";
      }
      prompt << "]\n\nCurrent frame observations:\n[";
      for (size_t i = 0; i < repaired.slots.size(); ++i) {
        const auto c = centerOf(repaired.slots[i].corners);
        prompt << "{\"idx\":" << i << ",\"id\":\"" << repaired.slots[i].slot_id << "\""
               << ",\"x\":" << c.x << ",\"y\":" << c.y << "}";
        if (i + 1 != repaired.slots.size()) prompt << ",";
      }
      prompt << "]\n\nOutput ONLY a JSON array of corrections: [{\"idx\":N,\"id\":\"inferred_id\"}, ...]";

      std::vector<ChatMessage> messages;
      messages.push_back({"system",
          "You are a parking slot ID inference engine. "
          "Based on spatial patterns and known IDs from the map, infer missing slot IDs.", ""});
      messages.push_back({"user", prompt.str(), ""});

      const std::string response = api_client_->chatLlm(messages);

      std::string json_str = response;
      auto arr_start = json_str.find('[');
      auto arr_end = json_str.rfind(']');
      if (arr_start != std::string::npos && arr_end != std::string::npos && arr_end > arr_start) {
        json_str = json_str.substr(arr_start, arr_end - arr_start + 1);
        json corrections = json::parse(json_str);
        for (const auto& item : corrections) {
          int idx = item.value("idx", -1);
          std::string new_id = item.value("id", "");
          if (idx >= 0 && idx < static_cast<int>(repaired.slots.size()) && !new_id.empty()) {
            if (repaired.slots[idx].slot_id.empty()) {
              repaired.slots[idx].slot_id = new_id;
              repaired.slots[idx].confidence = 0.45;
              repaired.slots[idx].has_text = true;
            }
          }
        }
      }
    } catch (const std::exception& e) {
      std::cerr << "[TextGeometryLocalizer] LLM fallback for VLM also failed: " << e.what() << "\n";
    }
  }
  return repaired;
}

std::vector<MatchPair> TextGeometryLocalizer::idMatches(const TextMap& map,
                                                        const FrameObservation& frame) const {
  std::unordered_map<std::string, int> by_id;
  for (size_t i = 0; i < map.slots.size(); ++i) {
    if (!map.slots[i].slot_id.empty()) by_id[map.slots[i].slot_id] = static_cast<int>(i);
  }
  std::vector<MatchPair> matches;
  for (size_t i = 0; i < frame.slots.size(); ++i) {
    const auto it = by_id.find(frame.slots[i].slot_id);
    if (it != by_id.end()) matches.push_back({static_cast<int>(i), it->second, frame.slots[i].slot_id, 0.0});
  }
  return matches;
}

RansacResult TextGeometryLocalizer::estimateInitialPose(const TextMap& map,
                                                        const FrameObservation& frame,
                                                        const std::vector<MatchPair>& matches) const {
  RansacResult best;
  if (matches.size() < 2) return best;

  std::mt19937 rng(13);
  std::uniform_int_distribution<size_t> pick(0, matches.size() - 1);
  for (int iter = 0; iter < config_.ransac_iterations; ++iter) {
    size_t a = pick(rng), b = pick(rng);
    if (a == b) continue;
    std::vector<Point2> src, dst;
    for (const size_t idx : {a, b}) {
      const auto& m = matches[idx];
      for (size_t k = 0; k < 4; ++k) {
        src.push_back(frame.slots[m.source_index].corners[k]);
        dst.push_back(map.slots[m.target_index].corners[k]);
      }
    }
    const Transform2 tf = estimateRigidTransform(src, dst);

    std::vector<MatchPair> inliers;
    double err_sum = 0.0;
    for (auto m : matches) {
      m.error = cornerRmse(frame.slots[m.source_index].corners, map.slots[m.target_index].corners, tf);
      if (m.error < config_.ransac_threshold) {
        inliers.push_back(m);
        err_sum += m.error * m.error;
      }
    }
    if (inliers.size() > best.inliers.size()) {
      best.transform = tf;
      best.inliers = inliers;
      best.inlier_ratio = static_cast<double>(inliers.size()) / static_cast<double>(matches.size());
      best.rmse = std::sqrt(err_sum / std::max<size_t>(1, inliers.size()));
    }
  }
  if (!best.inliers.empty()) {
    std::vector<Point2> src, dst;
    for (const auto& m : best.inliers) {
      for (size_t k = 0; k < 4; ++k) {
        src.push_back(frame.slots[m.source_index].corners[k]);
        dst.push_back(map.slots[m.target_index].corners[k]);
      }
    }
    best.transform = estimateRigidTransform(src, dst);
  }
  return best;
}

Transform2 TextGeometryLocalizer::refineTextGeometryIcp(const TextMap& map,
                                                        const FrameObservation& frame,
                                                        const Transform2& initial,
                                                        const std::vector<MatchPair>& text_matches) const {
  Transform2 current = initial;
  const double avg_conf = [&]() {
    double sum = 0.0;
    for (const auto& m : text_matches) sum += frame.slots[m.source_index].confidence;
    return text_matches.empty() ? 0.0 : sum / text_matches.size();
  }();
  const double alpha = config_.lambda_max *
                       (1.0 - std::exp(-static_cast<double>(text_matches.size()) / config_.gamma)) *
                       std::clamp(avg_conf, 0.0, 1.0);

  for (int iter = 0; iter < config_.icp_iterations; ++iter) {
    std::vector<Point2> src, dst;
    for (const auto& m : text_matches) {
      const double copies = std::max(1.0, std::round(1.0 + alpha * 4.0));
      for (int rep = 0; rep < static_cast<int>(copies); ++rep) {
        for (size_t k = 0; k < 4; ++k) {
          src.push_back(frame.slots[m.source_index].corners[k]);
          dst.push_back(map.slots[m.target_index].corners[k]);
        }
      }
    }

    for (size_t i = 0; i < frame.slots.size(); ++i) {
      if (frame.slots[i].has_text) continue;
      for (const auto& p : frame.slots[i].corners) {
        const Point2 tp = transformPoint(current, p);
        double best_d = std::numeric_limits<double>::infinity();
        Point2 best_q;
        for (const auto& slot : map.slots) {
          for (const auto& q : slot.corners) {
            const double d = distance(tp, q);
            if (d < best_d) {
              best_d = d;
              best_q = q;
            }
          }
        }
        if (best_d < config_.nearest_neighbor_gate) {
          src.push_back(p);
          dst.push_back(best_q);
        }
      }
    }
    if (src.size() >= 2) current = estimateRigidTransform(src, dst);
  }
  return current;
}

bool TextGeometryLocalizer::isAbnormal(const LocalizationResult& result, int text_match_count) const {
  if (text_match_count < config_.min_text_matches) return true;
  if (result.inlier_ratio < config_.min_inlier_ratio) return true;
  if (result.mean_error > config_.max_mean_error) return true;
  return false;
}

}  // namespace textmap
