#pragma once

#include "qwen_api.hpp"
#include "textmap_types.hpp"

#include <memory>

namespace textmap {

struct LlmOptimizerConfig {
  int sliding_window = 9;
  double max_neighbor_gap = 2.5;
  double min_confidence_for_anchor = 0.55;
  bool enable_api = true;  // true: call QwQ-32B; false: offline rule fallback
  QwenApiConfig api_config;
};

class LlmMapOptimizer {
 public:
  explicit LlmMapOptimizer(LlmOptimizerConfig config = {});

  TextMap refine(const TextMap& input) const;
  std::string buildPrompt(const std::vector<MapSlot>& window) const;

 private:
  LlmOptimizerConfig config_;
  std::unique_ptr<QwenApiClient> api_client_;

  /// API-based refinement: sliding window + QwQ-32B
  void refineWithLlm(TextMap& map) const;

  /// Parse QwQ-32B JSON response and apply corrections to the window slots
  void applyLlmResponse(std::vector<MapSlot>& window, const std::string& response) const;

  /// Legacy offline rules (kept as fallback if API is disabled or fails)
  void normalizeText(MapSlot& slot) const;
  void repairSequence(TextMap& map) const;
  void fillMissingGeometry(TextMap& map) const;
};

}  // namespace textmap
