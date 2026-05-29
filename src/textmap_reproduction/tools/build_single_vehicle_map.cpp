#include "json_io.hpp"
#include "single_vehicle_mapper.hpp"

#include <iostream>

int main(int argc, char** argv) {
  if (argc < 3) {
    std::cerr << "Usage: build_single_vehicle_map <frames_or_map.json> <output_map.json>\n";
    return 1;
  }
  try {
    textmap::SingleVehicleMapper mapper;
    for (const auto& frame : textmap::loadFrameObservations(argv[1])) mapper.addFrame(frame);
    textmap::saveTextMap(mapper.exportMap("single_vehicle_text_map"), argv[2]);
    std::cout << "Saved single-vehicle text map to " << argv[2] << "\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << "\n";
    return 2;
  }
  return 0;
}
