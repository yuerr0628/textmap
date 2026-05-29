#include "qwen_api.hpp"

#include <nlohmann/json.hpp>

#include <cstdlib>
#include <curl/curl.h>
#include <stdexcept>
#include <sstream>

using json = nlohmann::json;

namespace textmap {
namespace {

size_t writeCallback(char* ptr, size_t size, size_t nmemb, std::string* data) {
  data->append(ptr, size * nmemb);
  return size * nmemb;
}

}  // namespace

QwenApiClient::QwenApiClient(QwenApiConfig config) : config_(std::move(config)) {
  if (config_.api_key.empty()) {
    config_ = configFromEnv();
  }
}

QwenApiConfig QwenApiClient::configFromEnv() {
  QwenApiConfig cfg;
  const char* key = std::getenv("DASHSCOPE_API_KEY");
  if (key) cfg.api_key = key;
  const char* url = std::getenv("DASHSCOPE_BASE_URL");
  if (url) cfg.base_url = url;
  return cfg;
}

std::string QwenApiClient::chatLlm(const std::vector<ChatMessage>& messages) const {
  if (config_.api_key.empty()) {
    throw std::runtime_error("QwenApiClient: DASHSCOPE_API_KEY not set");
  }
  const std::string body = buildLlmRequestBody(messages);
  const std::string response = httpPost(config_.base_url, body);
  return extractContent(response);
}

std::string QwenApiClient::chatVlm(const std::string& prompt, const std::string& image_base64) const {
  if (config_.api_key.empty()) {
    throw std::runtime_error("QwenApiClient: DASHSCOPE_API_KEY not set");
  }
  const std::string body = buildVlmRequestBody(prompt, image_base64);
  const std::string response = httpPost(config_.base_url, body);
  return extractContent(response);
}

std::string QwenApiClient::buildLlmRequestBody(const std::vector<ChatMessage>& messages) const {
  json msgs = json::array();
  for (const auto& m : messages) {
    msgs.push_back({{"role", m.role}, {"content", m.text_content}});
  }
  json body = {
      {"model", config_.llm_model},
      {"messages", msgs},
      {"temperature", config_.temperature},
      {"max_tokens", config_.max_tokens},
  };
  return body.dump();
}

std::string QwenApiClient::buildVlmRequestBody(const std::string& prompt,
                                                const std::string& image_base64) const {
  json content = json::array();
  if (!image_base64.empty()) {
    std::string data_url = "data:image/jpeg;base64," + image_base64;
    content.push_back({{"type", "image_url"}, {"image_url", {{"url", data_url}}}});
  }
  content.push_back({{"type", "text"}, {"text", prompt}});

  json msgs = json::array();
  msgs.push_back({{"role", "user"}, {"content", content}});

  json body = {
      {"model", config_.vlm_model},
      {"messages", msgs},
      {"temperature", config_.temperature},
      {"max_tokens", config_.max_tokens},
  };
  return body.dump();
}

std::string QwenApiClient::httpPost(const std::string& url, const std::string& json_body) const {
  CURL* curl = curl_easy_init();
  if (!curl) throw std::runtime_error("QwenApiClient: failed to init curl");

  std::string response_data;
  struct curl_slist* headers = nullptr;
  headers = curl_slist_append(headers, "Content-Type: application/json");
  std::string auth_header = "Authorization: Bearer " + config_.api_key;
  headers = curl_slist_append(headers, auth_header.c_str());

  curl_easy_setopt(curl, CURLOPT_URL, url.c_str());
  curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
  curl_easy_setopt(curl, CURLOPT_POSTFIELDS, json_body.c_str());
  curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, static_cast<long>(json_body.size()));
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_data);
  curl_easy_setopt(curl, CURLOPT_TIMEOUT, static_cast<long>(config_.timeout_seconds));
  curl_easy_setopt(curl, CURLOPT_SSL_VERIFYPEER, 1L);

  CURLcode res = curl_easy_perform(curl);
  curl_slist_free_all(headers);
  curl_easy_cleanup(curl);

  if (res != CURLE_OK) {
    throw std::runtime_error(std::string("QwenApiClient HTTP error: ") + curl_easy_strerror(res));
  }
  return response_data;
}

std::string QwenApiClient::extractContent(const std::string& response_json) const {
  try {
    json resp = json::parse(response_json);
    if (resp.contains("error")) {
      std::string err_msg = resp["error"].value("message", "unknown API error");
      throw std::runtime_error("QwenApiClient API error: " + err_msg);
    }
    return resp.at("choices").at(0).at("message").at("content").get<std::string>();
  } catch (const json::exception& e) {
    throw std::runtime_error(std::string("QwenApiClient: failed to parse response: ") + e.what() +
                             "\nRaw response: " + response_json.substr(0, 500));
  }
}

}  // namespace textmap
