#include "llm_map_optimizer.hpp"

#include "geometry.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <iostream>
#include <sstream>

using json = nlohmann::json;

namespace textmap {

LlmMapOptimizer::LlmMapOptimizer(LlmOptimizerConfig config) : config_(std::move(config)) {
  if (config_.enable_api) {
    api_client_ = std::make_unique<QwenApiClient>(config_.api_config);
  }
}

TextMap LlmMapOptimizer::refine(const TextMap& input) const {
  TextMap out = input;

  if (config_.enable_api && api_client_) {
    refineWithLlm(out);
  } else {
    for (auto& slot : out.slots) normalizeText(slot);
    repairSequence(out);
    fillMissingGeometry(out);
  }

  out.name = input.name + "_llm_refined";
  return out;
}

void LlmMapOptimizer::refineWithLlm(TextMap& map) const {
  // Sort slots spatially for coherent sliding windows
  std::sort(map.slots.begin(), map.slots.end(), [](const MapSlot& a, const MapSlot& b) {
    const auto ca = centerOf(a.corners);
    const auto cb = centerOf(b.corners);
    return ca.x == cb.x ? ca.y < cb.y : ca.x < cb.x;
  });

  const int n = static_cast<int>(map.slots.size());
  const int win = config_.sliding_window;
  const int stride = std::max(1, win / 2);

  for (int start = 0; start < n; start += stride) {
    const int end = std::min(start + win, n);
    std::vector<MapSlot> window(map.slots.begin() + start, map.slots.begin() + end);

    const std::string prompt = buildPrompt(window);

    try {
      std::vector<ChatMessage> messages;
      messages.push_back({"system",
          "You are a parking-slot text map data corrector. "
          "Given a list of parking slot observations with OCR-detected IDs, positions, and confidence scores, "
          "correct OCR-confused characters (e.g. O/0, I/1/L, S/5, B/8), "
          "complete missing IDs when the local sequence and geometry are consistent, "
          "and output ONLY a JSON array of corrected IDs in the same order. "
          "Output format: [{\"idx\":0,\"id\":\"corrected_id\"}, ...]. "
          "Only include slots whose ID you changed or filled. "
          "If no correction is needed, output an empty array [].",
          ""});
      messages.push_back({"user", prompt, ""});

      const std::string response = api_client_->chatLlm(messages);
      applyLlmResponse(window, response);

      // Write corrected window back
      for (int i = start; i < end; ++i) {
        map.slots[i] = window[i - start];
      }
    } catch (const std::exception& e) {
      std::cerr << "[LlmMapOptimizer] API call failed for window at " << start
                << ", falling back to rules: " << e.what() << "\n";
      for (auto& slot : window) normalizeText(slot);
      for (int i = start; i < end; ++i) {
        map.slots[i] = window[i - start];
      }
    }
  }

  fillMissingGeometry(map);
}

void LlmMapOptimizer::applyLlmResponse(std::vector<MapSlot>& window,
                                        const std::string& response) const {
  // Extract JSON from response (LLM may wrap in markdown code block)
  std::string json_str = response;
  auto code_start = json_str.find('[');
  auto code_end = json_str.rfind(']');
  if (code_start != std::string::npos && code_end != std::string::npos && code_end > code_start) {
    json_str = json_str.substr(code_start, code_end - code_start + 1);
  }

  try {
    json corrections = json::parse(json_str);
    for (const auto& item : corrections) {
      int idx = item.value("idx", -1);
      std::string new_id = item.value("id", "");
      if (idx >= 0 && idx < static_cast<int>(window.size()) && !new_id.empty()) {
        window[idx].slot_id = new_id;
        if (window[idx].confidence < config_.min_confidence_for_anchor) {
          window[idx].confidence = config_.min_confidence_for_anchor;
        }
        window[idx].text_votes[new_id] += 0.5;
      }
    }
  } catch (const json::exception& e) {
    std::cerr << "[LlmMapOptimizer] Failed to parse LLM response JSON: " << e.what() << "\n";
  }
}

std::string LlmMapOptimizer::buildPrompt(const std::vector<MapSlot>& window) const {
  std::ostringstream oss;
  oss << "Parking slot observations in spatial order:\n[";
  for (size_t i = 0; i < window.size(); ++i) {
    const auto c = centerOf(window[i].corners);
    oss << "{\"idx\":" << i
        << ",\"id\":\"" << window[i].slot_id << "\""
        << ",\"conf\":" << window[i].confidence
        << ",\"x\":" << c.x << ",\"y\":" << c.y << "}";
    if (i + 1 != window.size()) oss << ",";
  }
  oss << "]\n\nCorrect OCR errors and fill missing IDs based on spatial sequence patterns.";
  return oss.str();
}

// --- Legacy offline rules (kept for fallback) ---

void LlmMapOptimizer::normalizeText(MapSlot& slot) const {
  std::string out;
  for (char ch : slot.slot_id) {
    if (std::isalnum(static_cast<unsigned char>(ch))) out.push_back(static_cast<char>(std::toupper(ch)));
  }
  for (char& ch : out) {
    if (ch == 'O') ch = '0';
    if (ch == 'I' || ch == 'L') ch = '1';
    if (ch == 'S') ch = '5';
    if (ch == 'B') ch = '8';
  }
  slot.slot_id = out;
}

void LlmMapOptimizer::repairSequence(TextMap& map) const {
  std::sort(map.slots.begin(), map.slots.end(), [](const MapSlot& a, const MapSlot& b) {
    const auto ca = centerOf(a.corners);
    const auto cb = centerOf(b.corners);
    return ca.x == cb.x ? ca.y < cb.y : ca.x < cb.x;
  });

  for (size_t i = 1; i + 1 < map.slots.size(); ++i) {
    auto& prev = map.slots[i - 1];
    auto& curr = map.slots[i];
    auto& next = map.slots[i + 1];
    if (prev.slot_id.empty() || next.slot_id.empty()) continue;
    try {
      const int a = std::stoi(prev.slot_id);
      const int b = std::stoi(next.slot_id);
      if (std::abs(b - a) == 2) {
        const std::string inferred = std::to_string((a + b) / 2);
        if (curr.slot_id.empty() || curr.confidence < config_.min_confidence_for_anchor) {
          curr.slot_id = inferred;
          curr.confidence = std::max(curr.confidence, 0.5);
          curr.text_votes[inferred] += 0.5;
        }
      }
    } catch (...) {
    }
  }
}

void LlmMapOptimizer::fillMissingGeometry(TextMap& map) const {
  for (size_t i = 1; i + 1 < map.slots.size(); ++i) {
    auto& curr = map.slots[i];
    if (slotSize(curr.corners) > 0.1) continue;
    const auto& prev = map.slots[i - 1];
    const auto& next = map.slots[i + 1];
    if (distance(centerOf(prev.corners), centerOf(next.corners)) > config_.max_neighbor_gap * 2.0) continue;
    for (size_t k = 0; k < 4; ++k) {
      curr.corners[k].x = 0.5 * (prev.corners[k].x + next.corners[k].x);
      curr.corners[k].y = 0.5 * (prev.corners[k].y + next.corners[k].y);
    }
  }
}

}  // namespace textmap
