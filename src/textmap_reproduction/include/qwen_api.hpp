#pragma once

#include <string>
#include <vector>

namespace textmap {

struct QwenApiConfig {
  std::string api_key;
  std::string base_url = "https://dashscope.aliyuncs.com/compatible-mode/v1/chat/completions";
  std::string llm_model = "qwq-32b";
  std::string vlm_model = "qwen-vl-max";
  double temperature = 0.3;
  int max_tokens = 2048;
  int timeout_seconds = 60;
};

struct ChatMessage {
  std::string role;
  std::string text_content;
  std::string image_base64;  // non-empty for VLM image input
};

class QwenApiClient {
 public:
  explicit QwenApiClient(QwenApiConfig config = {});

  /// Call QwQ-32B with text-only messages, return assistant response text.
  std::string chatLlm(const std::vector<ChatMessage>& messages) const;

  /// Call Qwen-VL with text + image, return assistant response text.
  std::string chatVlm(const std::string& prompt, const std::string& image_base64) const;

  /// Load API key from environment variable DASHSCOPE_API_KEY if not set in config.
  static QwenApiConfig configFromEnv();

 private:
  QwenApiConfig config_;

  std::string httpPost(const std::string& url, const std::string& json_body) const;
  std::string buildLlmRequestBody(const std::vector<ChatMessage>& messages) const;
  std::string buildVlmRequestBody(const std::string& prompt, const std::string& image_base64) const;
  std::string extractContent(const std::string& response_json) const;
};

}  // namespace textmap
