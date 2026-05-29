#include "geometry.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <limits>

namespace textmap {

Point2 operator+(const Point2& a, const Point2& b) { return {a.x + b.x, a.y + b.y}; }
Point2 operator-(const Point2& a, const Point2& b) { return {a.x - b.x, a.y - b.y}; }
Point2 operator*(const Point2& p, double scale) { return {p.x * scale, p.y * scale}; }

double distance(const Point2& a, const Point2& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  return std::sqrt(dx * dx + dy * dy);
}

Point2 centerOf(const std::array<Point2, 4>& corners) {
  Point2 c;
  for (const auto& p : corners) {
    c.x += p.x;
    c.y += p.y;
  }
  return c * 0.25;
}

Point2 centerOf(const std::array<Point2, 2>& bbox) {
  return {(bbox[0].x + bbox[1].x) * 0.5, (bbox[0].y + bbox[1].y) * 0.5};
}

double headingOf(const std::array<Point2, 4>& corners) {
  const Point2 edge = corners[1] - corners[0];
  return std::atan2(edge.y, edge.x);
}

Point2 transformPoint(const Transform2& tf, const Point2& p) {
  return {tf.c * p.x - tf.s * p.y + tf.tx, tf.s * p.x + tf.c * p.y + tf.ty};
}

std::array<Point2, 4> transformCorners(const Transform2& tf, const std::array<Point2, 4>& corners) {
  std::array<Point2, 4> out{};
  for (size_t i = 0; i < corners.size(); ++i) out[i] = transformPoint(tf, corners[i]);
  return out;
}

Transform2 transformFromPose(const Pose2& pose) {
  return {std::cos(pose.yaw), std::sin(pose.yaw), pose.x, pose.y};
}

Pose2 poseFromTransform(const Transform2& tf) {
  return {tf.tx, tf.ty, std::atan2(tf.s, tf.c)};
}

Transform2 composeTransform(const Transform2& a, const Transform2& b) {
  Transform2 out;
  out.c = a.c * b.c - a.s * b.s;
  out.s = a.s * b.c + a.c * b.s;
  out.tx = a.c * b.tx - a.s * b.ty + a.tx;
  out.ty = a.s * b.tx + a.c * b.ty + a.ty;
  return out;
}

Transform2 inverseTransform(const Transform2& tf) {
  Transform2 out;
  out.c = tf.c;
  out.s = -tf.s;
  out.tx = -(tf.c * tf.tx + tf.s * tf.ty);
  out.ty = tf.s * tf.tx - tf.c * tf.ty;
  return out;
}

double normalizeAngle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

Pose2 relativePose(const Pose2& from, const Pose2& to) {
  const Transform2 tf_from = transformFromPose(from);
  const Transform2 tf_to = transformFromPose(to);
  return poseFromTransform(composeTransform(inverseTransform(tf_from), tf_to));
}

Transform2 estimateRigidTransform(const std::vector<Point2>& source,
                                  const std::vector<Point2>& target) {
  if (source.size() != target.size() || source.size() < 2) return {};

  Eigen::Vector2d src_mean(0.0, 0.0);
  Eigen::Vector2d dst_mean(0.0, 0.0);
  for (size_t i = 0; i < source.size(); ++i) {
    src_mean += Eigen::Vector2d(source[i].x, source[i].y);
    dst_mean += Eigen::Vector2d(target[i].x, target[i].y);
  }
  src_mean /= static_cast<double>(source.size());
  dst_mean /= static_cast<double>(target.size());

  Eigen::Matrix2d h = Eigen::Matrix2d::Zero();
  for (size_t i = 0; i < source.size(); ++i) {
    Eigen::Vector2d ps(source[i].x, source[i].y);
    Eigen::Vector2d pt(target[i].x, target[i].y);
    h += (ps - src_mean) * (pt - dst_mean).transpose();
  }

  Eigen::JacobiSVD<Eigen::Matrix2d> svd(h, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix2d r = svd.matrixV() * svd.matrixU().transpose();
  if (r.determinant() < 0.0) {
    Eigen::Matrix2d v = svd.matrixV();
    v.col(1) *= -1.0;
    r = v * svd.matrixU().transpose();
  }
  Eigen::Vector2d t = dst_mean - r * src_mean;
  return {r(0, 0), r(1, 0), t.x(), t.y()};
}

double cornerRmse(const std::array<Point2, 4>& source,
                  const std::array<Point2, 4>& target,
                  const Transform2& tf) {
  double sum = 0.0;
  for (size_t i = 0; i < 4; ++i) {
    const double d = distance(transformPoint(tf, source[i]), target[i]);
    sum += d * d;
  }
  return std::sqrt(sum / 4.0);
}

double slotSize(const std::array<Point2, 4>& corners) {
  return 0.5 * (distance(corners[0], corners[1]) + distance(corners[2], corners[3]));
}

}  // namespace textmap
