#pragma once

#include "textmap_types.hpp"

#include <string>
#include <vector>

namespace textmap {

std::vector<FrameObservation> loadFrameObservations(const std::string& path);
TextMap loadTextMap(const std::string& path);
void saveTextMap(const TextMap& map, const std::string& path);
void saveLocalizationResult(const LocalizationResult& result, const std::string& path);

}  // namespace textmap
