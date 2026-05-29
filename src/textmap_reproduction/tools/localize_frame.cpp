#include "json_io.hpp"
#include "localization.hpp"

#include <iostream>

int main(int argc, char** argv) {
  if (argc < 4) {
    std::cerr << "Usage: localize_frame <global_map.json> <frame.json> <output_pose.json> [frame_image]\n";
    return 1;
  }
  try {
    const auto map = textmap::loadTextMap(argv[1]);
    auto frames = textmap::loadFrameObservations(argv[2]);
    if (frames.empty()) {
      std::cerr << "No frame observation in " << argv[2] << "\n";
      return 2;
    }

    // Attach optional image path for VLM repair
    if (argc >= 5) {
      frames.front().image_path = argv[4];
    }

    textmap::TextGeometryLocalizer localizer;
    const auto result = localizer.localize(map, frames.front());
    textmap::saveLocalizationResult(result, argv[3]);
    std::cout << "Saved localization result to " << argv[3] << "\n";
    if (result.vlm_repair_triggered) {
      std::cout << "  (VLM repair was triggered)\n";
    }
    return result.success ? 0 : 3;
  } catch (const std::exception& e) {
    std::cerr << e.what() << "\n";
    return 2;
  }
}
