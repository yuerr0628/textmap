#include "json_loader.h"
#include <iostream>
#include <fstream> // Required for std::ifstream and std::ofstream
#include <streambuf> // Required for std::istreambuf_iterator
#include <iomanip> // Required for std::setw (pretty printing JSON)
#include "nlohmann/json.hpp" // Include the nlohmann/json header

using json = nlohmann::json;

// Helper function to safely get JSON values with default (no changes needed)
template<typename T>
T get_json_value(const json& j, const std::string& key, const T& default_value) {
    if (j.contains(key) && !j.at(key).is_null()) {
        try {
            return j.at(key).get<T>();
        } catch (const json::exception& e) {
            std::cerr << "Warning: JSON parsing error for key '" << key << "': " << e.what() << ". Using default value." << std::endl;
            return default_value;
        }
    }
    return default_value;
}


// Parse a single JSON string containing an array of parking space objects
std::optional<LocalMap> parseLocalMap(const std::string& map_id, const std::string& json_content) {
    LocalMap local_map;
    local_map.map_id = map_id;

    try {
        json j_array = json::parse(json_content);

        if (!j_array.is_array()) {
            std::cerr << "Error: Expected a JSON array for map " << map_id << ", but got something else." << std::endl;
            return std::nullopt;
        }

        for (const auto& j_space : j_array) {
            ParkingSpace space;

            // --- Get Parking Space ID from OCRPoint.text ---
            if (!j_space.contains("OCRPoint") || !j_space["OCRPoint"].is_object()) {
                std::cerr << "Warning: Skipping space in map " << map_id << " due to missing or invalid 'OCRPoint' object." << std::endl;
                continue;
            }
            const auto& ocr = j_space["OCRPoint"];
            if (!ocr.contains("text") || !ocr["text"].is_string() || ocr["text"].get<std::string>().empty()) {
                //  std::cerr << "Warning: Skipping space in map " << map_id << " due to missing, null, non-string, or empty 'text' in 'OCRPoint'." << std::endl;
                //  continue;
                 space.local_id=map_id+"0000";
            }
            space.local_id = ocr["text"].get<std::string>(); // Use OCR text as the primary ID
            space.ocr_text = space.local_id; // Also store it in the ocr_text field
            space.ocr_confidence = get_json_value(ocr, "confidence", 0.0);


            // --- ParkingSpot Corners ---
            if (!j_space.contains("ParkingSpot") || !j_space["ParkingSpot"].is_object()) {
                 std::cerr << "Warning: Skipping space ID '" << space.local_id << "' in map " << map_id << " due to missing or invalid 'ParkingSpot' object." << std::endl;
                 continue;
            }
            const auto& ps = j_space["ParkingSpot"];
            // IMPORTANT: Ensure corners exist before attempting to access them.
            if (!(ps.contains("x1") && ps.contains("y1") && ps.contains("x2") && ps.contains("y2") &&
                  ps.contains("x3") && ps.contains("y3") && ps.contains("x4") && ps.contains("y4"))) {
                 std::cerr << "Warning: Skipping space ID '" << space.local_id << "' in map " << map_id << " due to missing corner coordinates." << std::endl;
                 continue;
            }
            // Check if corners vector has the expected size (e.g., 4 for quadrilateral)
            // This check is now implicitly handled by the structure, but ensure parsing logic matches.
             if (space.corners.size() != 0) { // Should be 0 before parsing
                  std::cerr << "Internal Warning: Corners vector not empty before parsing space ID '" << space.local_id << "'." << std::endl;
                  space.corners.clear();
             }
            space.corners.push_back(Point2D(get_json_value(ps, "x1", 0.0), get_json_value(ps, "y1", 0.0)));
            space.corners.push_back(Point2D(get_json_value(ps, "x2", 0.0), get_json_value(ps, "y2", 0.0)));
            space.corners.push_back(Point2D(get_json_value(ps, "x3", 0.0), get_json_value(ps, "y3", 0.0)));
            space.corners.push_back(Point2D(get_json_value(ps, "x4", 0.0), get_json_value(ps, "y4", 0.0)));
             if (space.corners.size() != 4) { // Example check if expecting 4 corners
                 std::cerr << "Warning: Parsed " << space.corners.size() << " corners for space ID '" << space.local_id << "' in map " << map_id << ". Expected 4." << std::endl;
                 // Decide how to handle: skip space, use available corners? Skipping for safety.
                 continue;
             }


            // Optional: Vacancy status
            space.is_vacant = (get_json_value(ps, "vacant", 1) != 0);

            // --- We ignore the top-level "ID" field as requested ---

            local_map.spaces.push_back(space);
        }

    } catch (const json::parse_error& e) {
        std::cerr << "Error parsing JSON for map " << map_id << ": " << e.what() << std::endl;
        return std::nullopt;
    } catch (const std::exception& e) {
        std::cerr << "Error processing map " << map_id << ": " << e.what() << std::endl;
        return std::nullopt;
    }

    // std::cout << "Successfully parsed map " << map_id << " with " << local_map.spaces.size() << " valid spaces (using OCR text as ID)." << std::endl; // Moved message to caller
    return local_map;
}


// Load maps from a list of (map_id, json_string) pairs
std::vector<LocalMap> loadLocalMapsFromJsonStrings(const std::vector<std::pair<std::string, std::string>>& map_data) {
    std::vector<LocalMap> loaded_maps;
    for (const auto& pair : map_data) {
        auto map_opt = parseLocalMap(pair.first, pair.second);
        if (map_opt) {
             std::cout << "Successfully parsed map " << pair.first << " from string with " << map_opt->spaces.size() << " valid spaces." << std::endl;
            loaded_maps.push_back(*map_opt);
        } else {
            std::cerr << "Failed to load map from string: " << pair.first << std::endl;
        }
    }
    return loaded_maps;
}

// --- Function to load from file paths ---
std::vector<LocalMap> loadLocalMapsFromJsonFiles(const std::vector<std::string>& file_paths) {
    std::vector<LocalMap> loaded_maps;
    for (const std::string& file_path : file_paths) {
        std::ifstream ifs(file_path); // Open the file
        if (!ifs.is_open()) {
            std::cerr << "Error: Could not open JSON file: " << file_path << std::endl;
            continue; // Skip this file
        }
        try {
            // Read entire file content into a string
             std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
             ifs.close(); // Close file immediately after reading

            // Extract map ID from filename (example: "path/to/segment_1.json" -> "segment_1")
            std::string map_id = file_path;
            size_t last_slash = map_id.find_last_of("/\\"); // Find last directory separator
            if (last_slash != std::string::npos) {
                map_id = map_id.substr(last_slash + 1); // Get filename part
            }
            size_t dot_pos = map_id.rfind('.'); // Find last dot
            if (dot_pos != std::string::npos) {
                map_id = map_id.substr(0, dot_pos); // Remove extension
            }


            // Parse the JSON content using the existing function
            auto map_opt = parseLocalMap(map_id, content);
            if (map_opt) {
                 std::cout << "Successfully parsed map " << map_id << " from file '" << file_path << "' with " << map_opt->spaces.size() << " valid spaces." << std::endl;
                loaded_maps.push_back(*map_opt); // Add the successfully parsed map
            } else {
                std::cerr << "Failed to parse map content from file: " << file_path << std::endl;
            }
        } catch (const std::exception& e) {
             // Catch potential errors during file reading or other standard exceptions
             std::cerr << "Error reading or processing file " << file_path << ": " << e.what() << std::endl;
             if(ifs.is_open()) ifs.close(); // Ensure file is closed on error
        }
    }
    return loaded_maps;
}


// --- Function to save the fused global map to a JSON file ---
bool saveGlobalMapToJsonFile(const std::vector<ParkingSpace>& global_map, const std::string& output_file_path) {
    json j_output_array = json::array(); // Create an empty JSON array

    for (const auto& space : global_map) {
        json j_space; // JSON object for a single space

        // Use global_id as the top-level "ID" in the output, or local_id if global_id is empty
        j_space["ID"] = !space.global_id.empty() ? space.global_id : space.local_id;

        // Create OCRPoint object
        json j_ocr;
        j_ocr["text"] = !space.global_id.empty() ? space.global_id : space.local_id;; // Use the stored ocr_text
        j_ocr["confidence"] = space.ocr_confidence;
             j_ocr["x1"] = 0;
            j_ocr["y1"] = 0;
            j_ocr["x2"] = 0;
            j_ocr["y2"] = 0;
        // Note: We don't have the original x1,y1,x2,y2 for OCR box in global frame,
        // so we only output text and confidence. Add calculation if needed.
        j_space["OCRPoint"] = j_ocr;

        // Create ParkingSpot object
        json j_ps;
        j_ps["vacant"] = space.is_vacant ? 1 : 0; // Output 0 for occupied, 1 for vacant

        // Add corners - ensure there are 4 corners
        if (space.corners.size() == 4) {
            j_ps["x1"] = space.corners[0].x();
            j_ps["y1"] = space.corners[0].y();
            j_ps["x2"] = space.corners[1].x();
            j_ps["y2"] = space.corners[1].y();
            j_ps["x3"] = space.corners[2].x();
            j_ps["y3"] = space.corners[2].y();
            j_ps["x4"] = space.corners[3].x();
            j_ps["y4"] = space.corners[3].y();
        } else {
            std::cerr << "Warning: Skipping corners for space ID '" << space.local_id
                      << "' during saving, as it has " << space.corners.size() << " corners instead of 4." << std::endl;
            // Optionally add null or empty values for corners
             j_ps["x1"] = nullptr; j_ps["y1"] = nullptr;
             j_ps["x2"] = nullptr; j_ps["y2"] = nullptr;
             j_ps["x3"] = nullptr; j_ps["y3"] = nullptr;
             j_ps["x4"] = nullptr; j_ps["y4"] = nullptr;
        }
        j_space["ParkingSpot"] = j_ps;

        // Optional: Add source map information if desired
        /*
        json j_sources = json::array();
        for(const auto& src : space.source_maps) {
            j_sources.push_back({{"map_id", src.first}, {"original_id", src.second}});
        }
        j_space["Sources"] = j_sources;
        */

        j_output_array.push_back(j_space); // Add the space object to the array
    }

    // Write the JSON array to the output file
    std::ofstream ofs(output_file_path);
    if (!ofs.is_open()) {
        std::cerr << "Error: Could not open output file for writing: " << output_file_path << std::endl;
        return false;
    }

    try {
        ofs << std::setw(4) << j_output_array << std::endl; // Write pretty-printed JSON
        ofs.close();
        std::cout << "Successfully saved global map to: " << output_file_path << std::endl;
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error writing JSON to file " << output_file_path << ": " << e.what() << std::endl;
        if (ofs.is_open()) ofs.close();
        return false;
    }
}
