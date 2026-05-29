#include "crowd_fusion.hpp"

#include "geometry.hpp"

#include <Eigen/Dense>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam2d/types_slam2d.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <random>
#include <set>
#include <unordered_map>

namespace textmap {

CrowdFusion::CrowdFusion(FusionConfig config) : config_(config) {}

TextMap CrowdFusion::fuse(const std::vector<TextMap>& local_maps) const {
  TextMap global;
  global.name = "crowdsourced_text_map";
  if (local_maps.empty()) return global;

  const auto edges = buildPoseGraphEdges(local_maps);
  const auto optimized_poses = optimizePoseGraph(local_maps.size(), edges);

  return optimizeSlotFactorGraph(local_maps, optimized_poses);
}

RansacResult CrowdFusion::alignByTextAnchors(const TextMap& source, const TextMap& target) const {
  const auto matches = buildIdMatches(source, target);
  RansacResult best;
  if (matches.size() < 2) return best;

  std::mt19937 rng(7);
  std::uniform_int_distribution<size_t> pick(0, matches.size() - 1);

  for (int iter = 0; iter < config_.ransac_iterations; ++iter) {
    size_t a = pick(rng), b = pick(rng);
    if (a == b) continue;
    std::vector<Point2> src = {centerOf(source.slots[matches[a].source_index].corners),
                               centerOf(source.slots[matches[b].source_index].corners)};
    std::vector<Point2> dst = {centerOf(target.slots[matches[a].target_index].corners),
                               centerOf(target.slots[matches[b].target_index].corners)};
    if (distance(src[0], src[1]) < 0.2) continue;
    const Transform2 tf = estimateRigidTransform(src, dst);

    std::vector<MatchPair> inliers;
    double err_sum = 0.0;
    for (auto m : matches) {
      const auto ps = centerOf(source.slots[m.source_index].corners);
      const auto pt = centerOf(target.slots[m.target_index].corners);
      m.error = distance(transformPoint(tf, ps), pt);
      if (m.error < config_.ransac_threshold) {
        inliers.push_back(m);
        err_sum += m.error * m.error;
      }
    }

    if (inliers.size() > best.inliers.size()) {
      best.transform = tf;
      best.inliers = inliers;
      best.rmse = inliers.empty() ? 0.0 : std::sqrt(err_sum / inliers.size());
      best.inlier_ratio = static_cast<double>(inliers.size()) / static_cast<double>(matches.size());
    }
  }

  if (best.inliers.size() >= 2) {
    std::vector<Point2> src, dst;
    for (const auto& m : best.inliers) {
      src.push_back(centerOf(source.slots[m.source_index].corners));
      dst.push_back(centerOf(target.slots[m.target_index].corners));
    }
    best.transform = estimateRigidTransform(src, dst);
    best.inliers = filterByTopology(source, target, best.inliers, best.transform);
  }
  return best;
}

std::vector<MatchPair> CrowdFusion::buildIdMatches(const TextMap& source, const TextMap& target) const {
  std::unordered_map<std::string, int> target_by_id;
  for (size_t i = 0; i < target.slots.size(); ++i) {
    if (!target.slots[i].slot_id.empty()) target_by_id[target.slots[i].slot_id] = static_cast<int>(i);
  }

  std::vector<MatchPair> matches;
  for (size_t i = 0; i < source.slots.size(); ++i) {
    const auto it = target_by_id.find(source.slots[i].slot_id);
    if (it != target_by_id.end()) {
      matches.push_back({static_cast<int>(i), it->second, source.slots[i].slot_id, 0.0});
    }
  }
  return matches;
}

std::vector<MatchPair> CrowdFusion::filterByTopology(const TextMap& source,
                                                     const TextMap& target,
                                                     const std::vector<MatchPair>& matches,
                                                     const Transform2& tf) const {
  std::set<std::string> matched_ids;
  for (const auto& m : matches) matched_ids.insert(m.slot_id);

  std::vector<MatchPair> out;
  for (auto m : matches) {
    const auto sc = transformPoint(tf, centerOf(source.slots[m.source_index].corners));
    const auto tc = centerOf(target.slots[m.target_index].corners);
    m.error = distance(sc, tc);
    if (m.error > config_.strict_match_threshold && matches.size() > 3) continue;

    std::set<std::string> ns, nt;
    for (const auto& s : source.slots) {
      if (distance(centerOf(s.corners), centerOf(source.slots[m.source_index].corners)) <
          config_.topology_neighbor_radius) ns.insert(s.slot_id);
    }
    for (const auto& t : target.slots) {
      if (distance(centerOf(t.corners), centerOf(target.slots[m.target_index].corners)) <
          config_.topology_neighbor_radius) nt.insert(t.slot_id);
    }

    int inter = 0;
    for (const auto& id : ns) {
      if (nt.count(id)) ++inter;
    }
    const int uni = static_cast<int>(ns.size() + nt.size() - inter);
    const double sim = uni == 0 ? 1.0 : static_cast<double>(inter) / static_cast<double>(uni);
    if (sim >= config_.topology_similarity_threshold || matches.size() <= 3) out.push_back(m);
  }
  return out;
}

void CrowdFusion::mergeAligned(TextMap& global, const TextMap& local, const Transform2& tf) const {
  for (const auto& raw : local.slots) {
    MapSlot slot = raw;
    slot.corners = transformCorners(tf, raw.corners);
    slot.text_bbox[0] = transformPoint(tf, raw.text_bbox[0]);
    slot.text_bbox[1] = transformPoint(tf, raw.text_bbox[1]);

    auto it = std::find_if(global.slots.begin(), global.slots.end(), [&](const MapSlot& g) {
      return !slot.slot_id.empty() && g.slot_id == slot.slot_id &&
             distance(centerOf(g.corners), centerOf(slot.corners)) < config_.ransac_threshold;
    });
    if (it == global.slots.end()) {
      slot.track_id = static_cast<int>(global.slots.size());
      global.slots.push_back(slot);
      continue;
    }
    const double n = static_cast<double>(std::max(1, it->match_count));
    for (size_t k = 0; k < 4; ++k) {
      it->corners[k].x = (n * it->corners[k].x + slot.corners[k].x) / (n + 1.0);
      it->corners[k].y = (n * it->corners[k].y + slot.corners[k].y) / (n + 1.0);
    }
    it->confidence = std::max(it->confidence, slot.confidence);
    it->match_count += std::max(1, slot.match_count);
    it->stable = true;
  }
}

std::vector<CrowdFusion::PoseGraphEdge> CrowdFusion::buildPoseGraphEdges(
    const std::vector<TextMap>& local_maps) const {
  std::vector<PoseGraphEdge> edges;
  for (size_t i = 0; i < local_maps.size(); ++i) {
    for (size_t j = i + 1; j < local_maps.size(); ++j) {
      const RansacResult alignment = alignByTextAnchors(local_maps[i], local_maps[j]);
      if (alignment.inliers.size() < 2) continue;

      PoseGraphEdge edge;
      edge.source_map = static_cast<int>(i);
      edge.target_map = static_cast<int>(j);
      edge.measurement = poseFromTransform(alignment.transform);
      edge.weight = std::max(1.0, static_cast<double>(alignment.inliers.size()));
      edge.matches = alignment.inliers;
      edges.push_back(edge);
    }
  }
  return edges;
}

std::vector<Pose2> CrowdFusion::optimizePoseGraph(
    size_t map_count, const std::vector<PoseGraphEdge>& edges) const {
  std::vector<Pose2> poses(map_count);
  if (map_count == 0) return poses;

  std::vector<bool> initialized(map_count, false);
  initialized[0] = true;
  bool changed = true;
  while (changed) {
    changed = false;
    for (const auto& edge : edges) {
      if (initialized[edge.source_map] && !initialized[edge.target_map]) {
        poses[edge.target_map] = poseFromTransform(
            composeTransform(transformFromPose(poses[edge.source_map]), transformFromPose(edge.measurement)));
        initialized[edge.target_map] = true;
        changed = true;
      } else if (initialized[edge.target_map] && !initialized[edge.source_map]) {
        poses[edge.source_map] = poseFromTransform(
            composeTransform(transformFromPose(poses[edge.target_map]), inverseTransform(transformFromPose(edge.measurement))));
        initialized[edge.source_map] = true;
        changed = true;
      }
    }
  }

  if (map_count == 1 || edges.empty()) return poses;

  const int variable_nodes = static_cast<int>(map_count) - 1;
  const int dim = variable_nodes * 3;

  for (int iter = 0; iter < config_.pose_graph_iterations; ++iter) {
    Eigen::MatrixXd hessian = Eigen::MatrixXd::Zero(dim, dim);
    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(dim);

    auto addBlock = [&](int map_idx, const Eigen::Matrix3d& h, const Eigen::Vector3d& g) {
      if (map_idx == 0) return;
      const int offset = (map_idx - 1) * 3;
      hessian.block<3, 3>(offset, offset) += h;
      gradient.segment<3>(offset) += g;
    };
    auto addCross = [&](int a, int b, const Eigen::Matrix3d& h) {
      if (a == 0 || b == 0) return;
      const int oa = (a - 1) * 3;
      const int ob = (b - 1) * 3;
      hessian.block<3, 3>(oa, ob) += h;
      hessian.block<3, 3>(ob, oa) += h.transpose();
    };

    for (const auto& edge : edges) {
      const Pose2 predicted = relativePose(poses[edge.source_map], poses[edge.target_map]);
      Eigen::Vector3d residual;
      residual << predicted.x - edge.measurement.x,
          predicted.y - edge.measurement.y,
          normalizeAngle(predicted.yaw - edge.measurement.yaw);

      double weight = edge.weight;
      const double norm = residual.norm();
      if (norm > config_.huber_delta && norm > 1e-9) weight *= config_.huber_delta / norm;

      Eigen::Matrix3d Ji = -Eigen::Matrix3d::Identity();
      Eigen::Matrix3d Jj = Eigen::Matrix3d::Identity();

      addBlock(edge.source_map, weight * Ji.transpose() * Ji, weight * Ji.transpose() * residual);
      addBlock(edge.target_map, weight * Jj.transpose() * Jj, weight * Jj.transpose() * residual);
      addCross(edge.source_map, edge.target_map, weight * Ji.transpose() * Jj);
    }

    hessian += config_.damping * Eigen::MatrixXd::Identity(dim, dim);
    const Eigen::VectorXd dx = -hessian.ldlt().solve(gradient);
    if (dx.norm() < 1e-6) break;

    for (int map_idx = 1; map_idx < static_cast<int>(map_count); ++map_idx) {
      const int offset = (map_idx - 1) * 3;
      poses[map_idx].x += dx[offset + 0];
      poses[map_idx].y += dx[offset + 1];
      poses[map_idx].yaw = normalizeAngle(poses[map_idx].yaw + dx[offset + 2]);
    }
  }
  return poses;
}

TextMap CrowdFusion::optimizeSlotFactorGraph(const std::vector<TextMap>& local_maps,
                                             const std::vector<Pose2>& map_poses) const {
  std::vector<SlotGraphNode> nodes;
  for (size_t map_idx = 0; map_idx < local_maps.size(); ++map_idx) {
    const Transform2 tf = transformFromPose(map_poses[map_idx]);
    for (size_t slot_idx = 0; slot_idx < local_maps[map_idx].slots.size(); ++slot_idx) {
      MapSlot slot = local_maps[map_idx].slots[slot_idx];
      slot.corners = transformCorners(tf, slot.corners);
      slot.text_bbox[0] = transformPoint(tf, slot.text_bbox[0]);
      slot.text_bbox[1] = transformPoint(tf, slot.text_bbox[1]);

      SlotGraphNode node;
      node.map_index = static_cast<int>(map_idx);
      node.slot_index = static_cast<int>(slot_idx);
      node.slot = slot;
      node.pose = slotPose(slot);
      node.initial_pose = node.pose;
      nodes.push_back(node);
    }
  }

  const auto slot_edges = buildSlotGraphEdges(nodes);
  solveSlotGraph(nodes, slot_edges);

  TextMap global;
  global.name = "crowdsourced_text_map";
  mergeOptimizedSlots(global, nodes);
  return global;
}

std::vector<CrowdFusion::SlotGraphEdge> CrowdFusion::buildSlotGraphEdges(
    const std::vector<SlotGraphNode>& nodes) const {
  std::vector<SlotGraphEdge> edges;

  std::unordered_map<int, std::vector<int>> nodes_by_map;
  std::unordered_map<std::string, std::vector<int>> nodes_by_text;
  for (size_t i = 0; i < nodes.size(); ++i) {
    nodes_by_map[nodes[i].map_index].push_back(static_cast<int>(i));
    if (!nodes[i].slot.slot_id.empty()) nodes_by_text[nodes[i].slot.slot_id].push_back(static_cast<int>(i));
  }

  for (auto& group : nodes_by_map) {
    auto indices = group.second;
    std::sort(indices.begin(), indices.end(), [&](int a, int b) {
      const auto ca = centerOf(nodes[a].slot.corners);
      const auto cb = centerOf(nodes[b].slot.corners);
      return ca.x == cb.x ? ca.y < cb.y : ca.x < cb.x;
    });
    for (size_t i = 1; i < indices.size(); ++i) {
      SlotGraphEdge edge;
      edge.a = indices[i - 1];
      edge.b = indices[i];
      edge.measurement = relativePose(nodes[edge.a].initial_pose, nodes[edge.b].initial_pose);
      edge.weight = 1.0;
      edge.zero_relative = false;
      edges.push_back(edge);
    }
  }

  for (const auto& group : nodes_by_text) {
    const auto& indices = group.second;
    for (size_t i = 0; i < indices.size(); ++i) {
      for (size_t j = i + 1; j < indices.size(); ++j) {
        const double d = distance(centerOf(nodes[indices[i]].slot.corners), centerOf(nodes[indices[j]].slot.corners));
        if (d > config_.ransac_threshold) continue;
        SlotGraphEdge edge;
        edge.a = indices[i];
        edge.b = indices[j];
        edge.measurement = {};
        edge.weight = 8.0;
        edge.zero_relative = true;
        edges.push_back(edge);
      }
    }
  }
  return edges;
}

void CrowdFusion::solveSlotGraph(std::vector<SlotGraphNode>& nodes,
                                 const std::vector<SlotGraphEdge>& edges) const {
  if (nodes.empty() || edges.empty()) return;

  using BlockSolver = g2o::BlockSolver<g2o::BlockSolverTraits<3, 3>>;
  using LinearSolver = g2o::LinearSolverEigen<BlockSolver::PoseMatrixType>;

  auto linear_solver = std::make_unique<LinearSolver>();
  auto block_solver = std::make_unique<BlockSolver>(std::move(linear_solver));
  auto algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(algorithm);
  optimizer.setVerbose(false);

  std::vector<g2o::VertexSE2*> vertices(nodes.size(), nullptr);
  for (size_t i = 0; i < nodes.size(); ++i) {
    auto* vertex = new g2o::VertexSE2();
    vertex->setId(static_cast<int>(i));
    vertex->setEstimate(g2o::SE2(nodes[i].pose.x, nodes[i].pose.y, nodes[i].pose.yaw));
    if (nodes[i].map_index == 0) vertex->setFixed(true);
    optimizer.addVertex(vertex);
    vertices[i] = vertex;
  }

  int edge_id = static_cast<int>(nodes.size());
  for (const auto& constraint : edges) {
    if (constraint.a < 0 || constraint.b < 0) continue;
    if (constraint.a >= static_cast<int>(vertices.size()) || constraint.b >= static_cast<int>(vertices.size())) {
      continue;
    }

    auto* edge = new g2o::EdgeSE2();
    edge->setId(edge_id++);
    edge->vertices()[0] = vertices[constraint.a];
    edge->vertices()[1] = vertices[constraint.b];
    edge->setMeasurement(g2o::SE2(
        constraint.measurement.x, constraint.measurement.y, constraint.measurement.yaw));

    Eigen::Matrix3d information = Eigen::Matrix3d::Identity() * std::max(1e-3, constraint.weight);
    if (constraint.zero_relative) information *= 2.0;
    edge->setInformation(information);

    auto* kernel = new g2o::RobustKernelHuber();
    kernel->setDelta(config_.huber_delta);
    edge->setRobustKernel(kernel);
    optimizer.addEdge(edge);
  }

  optimizer.initializeOptimization();
  optimizer.optimize(config_.pose_graph_iterations);

  for (size_t i = 0; i < nodes.size(); ++i) {
    const Eigen::Vector3d estimate = vertices[i]->estimate().toVector();
    nodes[i].pose.x = estimate[0];
    nodes[i].pose.y = estimate[1];
    nodes[i].pose.yaw = normalizeAngle(estimate[2]);
  }
}

void CrowdFusion::mergeOptimizedSlots(TextMap& global, const std::vector<SlotGraphNode>& nodes) const {
  for (const auto& node : nodes) {
    MapSlot slot = applyOptimizedPose(node);
    auto it = std::find_if(global.slots.begin(), global.slots.end(), [&](const MapSlot& existing) {
      return !slot.slot_id.empty() && existing.slot_id == slot.slot_id &&
             distance(centerOf(existing.corners), centerOf(slot.corners)) < config_.ransac_threshold;
    });
    if (it == global.slots.end()) {
      slot.track_id = static_cast<int>(global.slots.size());
      slot.stable = true;
      global.slots.push_back(slot);
      continue;
    }
    const double n = static_cast<double>(std::max(1, it->match_count));
    for (size_t k = 0; k < 4; ++k) {
      it->corners[k].x = (n * it->corners[k].x + slot.corners[k].x) / (n + 1.0);
      it->corners[k].y = (n * it->corners[k].y + slot.corners[k].y) / (n + 1.0);
    }
    it->confidence = std::max(it->confidence, slot.confidence);
    it->match_count += std::max(1, slot.match_count);
    for (const auto& vote : slot.text_votes) it->text_votes[vote.first] += vote.second;
  }
}

Pose2 CrowdFusion::slotPose(const MapSlot& slot) const {
  const auto c = centerOf(slot.corners);
  return {c.x, c.y, headingOf(slot.corners)};
}

MapSlot CrowdFusion::applyOptimizedPose(const SlotGraphNode& node) const {
  MapSlot slot = node.slot;
  const double dc = std::cos(node.pose.yaw - node.initial_pose.yaw);
  const double ds = std::sin(node.pose.yaw - node.initial_pose.yaw);
  const Point2 initial_center{node.initial_pose.x, node.initial_pose.y};
  const Point2 optimized_center{node.pose.x, node.pose.y};

  auto movePoint = [&](const Point2& p) {
    const double x = p.x - initial_center.x;
    const double y = p.y - initial_center.y;
    return Point2{dc * x - ds * y + optimized_center.x, ds * x + dc * y + optimized_center.y};
  };

  for (auto& p : slot.corners) p = movePoint(p);
  slot.text_bbox[0] = movePoint(slot.text_bbox[0]);
  slot.text_bbox[1] = movePoint(slot.text_bbox[1]);
  return slot;
}

}  // namespace textmap
