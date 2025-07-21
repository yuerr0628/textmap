#ifndef JSON_LOADER_H
#define JSON_LOADER_H

#include "map_types.h"
#include <string>
#include <vector>
#include <optional>

// Function to load local maps from a vector of JSON strings
std::vector<LocalMap> loadLocalMapsFromJsonStrings(const std::vector<std::pair<std::string, std::string>>& map_data);

// Function to parse a single JSON string representing a list of parking spaces
std::optional<LocalMap> parseLocalMap(const std::string& map_id, const std::string& json_content);

// Function to load local maps from a vector of JSON file paths
std::vector<LocalMap> loadLocalMapsFromJsonFiles(const std::vector<std::string>& file_paths);

// Function to save the fused global map to a JSON file
bool saveGlobalMapToJsonFile(const std::vector<ParkingSpace>& global_map, const std::string& output_file_path);


#endif // JSON_LOADER_H