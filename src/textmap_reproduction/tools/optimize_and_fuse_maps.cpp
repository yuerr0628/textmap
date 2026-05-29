#include "crowd_fusion.hpp"
#include "json_io.hpp"
#include "llm_map_optimizer.hpp"

#include <iostream>
#include <vector>

int main(int argc, char** argv) {
  if (argc < 4) {
    std::cerr << "Usage: optimize_and_fuse_maps <output_map.json> <local_map_1.json> <local_map_2.json> [...]\n";
    return 1;
  }
  try {
    textmap::LlmMapOptimizer optimizer;
    std::vector<textmap::TextMap> maps;
    for (int i = 2; i < argc; ++i) maps.push_back(optimizer.refine(textmap::loadTextMap(argv[i])));
    textmap::CrowdFusion fusion;
    textmap::saveTextMap(fusion.fuse(maps), argv[1]);
    std::cout << "Saved fused crowdsourced text map to " << argv[1] << "\n";
  } catch (const std::exception& e) {
    std::cerr << e.what() << "\n";
    return 2;
  }
  return 0;
}
