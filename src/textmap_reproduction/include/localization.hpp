#pragma once

#include "qwen_api.hpp"
#include "textmap_types.hpp"

#include <memory>

namespace textmap {

struct LocalizationConfig {
  int ransac_iterations = 300;
  double ransac_threshold = 0.8;
  int min_text_matches = 3;
  double min_inlier_ratio = 0.6;
  double max_mean_error = 0.8;
  double lambda_max = 0.9;
  double gamma = 4.0;
  int icp_iterations = 8;
  double nearest_neighbor_gate = 2.0;
  bool enable_vlm_repair = true;
  QwenApiConfig api_config;
};

class TextGeometryLocalizer {
 public:
  explicit TextGeometryLocalizer(LocalizationConfig config = {});

  LocalizationResult localize(const TextMap& global_map, const FrameObservation& frame) const;

 private:
  LocalizationConfig config_;
  std::unique_ptr<QwenApiClient> api_client_;

  std::vector<MatchPair> idMatches(const TextMap& map, const FrameObservation& frame) const;
  RansacResult estimateInitialPose(const TextMap& map,
                                   const FrameObservation& frame,
                                   const std::vector<MatchPair>& matches) const;
  Transform2 refineTextGeometryIcp(const TextMap& map,
                                   const FrameObservation& frame,
                                   const Transform2& initial,
                                   const std::vector<MatchPair>& text_matches) const;
  bool isAbnormal(const LocalizationResult& result, int text_match_count) const;
  FrameObservation repairWithVlm(const TextMap& map, const FrameObservation& frame) const;
};

}  // namespace textmap
