#pragma once

#include "textmap_types.hpp"

namespace textmap {

struct FusionConfig {
  int ransac_iterations = 300;
  double ransac_threshold = 0.8;
  double strict_match_threshold = 0.3;
  double topology_neighbor_radius = 3.2;
  double topology_similarity_threshold = 0.35;
  int pose_graph_iterations = 25;
  double huber_delta = 0.75;
  double damping = 1e-4;
};

class CrowdFusion {
 public:
  explicit CrowdFusion(FusionConfig config = {});

  TextMap fuse(const std::vector<TextMap>& local_maps) const;
  RansacResult alignByTextAnchors(const TextMap& source, const TextMap& target) const;

 private:
  struct PoseGraphEdge {
    int source_map = -1;
    int target_map = -1;
    Pose2 measurement;
    double weight = 1.0;
    std::vector<MatchPair> matches;
  };

  struct SlotGraphNode {
    int map_index = -1;
    int slot_index = -1;
    MapSlot slot;
    Pose2 pose;
    Pose2 initial_pose;
  };

  struct SlotGraphEdge {
    int a = -1;
    int b = -1;
    Pose2 measurement;
    double weight = 1.0;
    bool zero_relative = false;
  };

  FusionConfig config_;

  std::vector<MatchPair> buildIdMatches(const TextMap& source, const TextMap& target) const;
  std::vector<MatchPair> filterByTopology(const TextMap& source,
                                          const TextMap& target,
                                          const std::vector<MatchPair>& matches,
                                          const Transform2& tf) const;
  std::vector<PoseGraphEdge> buildPoseGraphEdges(const std::vector<TextMap>& local_maps) const;
  std::vector<Pose2> optimizePoseGraph(size_t map_count,
                                       const std::vector<PoseGraphEdge>& edges) const;
  TextMap optimizeSlotFactorGraph(const std::vector<TextMap>& local_maps,
                                  const std::vector<Pose2>& map_poses) const;
  std::vector<SlotGraphEdge> buildSlotGraphEdges(const std::vector<SlotGraphNode>& nodes) const;
  void solveSlotGraph(std::vector<SlotGraphNode>& nodes,
                      const std::vector<SlotGraphEdge>& edges) const;
  void mergeOptimizedSlots(TextMap& global, const std::vector<SlotGraphNode>& nodes) const;
  Pose2 slotPose(const MapSlot& slot) const;
  MapSlot applyOptimizedPose(const SlotGraphNode& node) const;
  void mergeAligned(TextMap& global, const TextMap& local, const Transform2& tf) const;
};

}  // namespace textmap
