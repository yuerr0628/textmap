#pragma once

#include "textmap_types.hpp"

#include <array>
#include <vector>

namespace textmap {

Point2 operator+(const Point2& a, const Point2& b);
Point2 operator-(const Point2& a, const Point2& b);
Point2 operator*(const Point2& p, double scale);

double distance(const Point2& a, const Point2& b);
Point2 centerOf(const std::array<Point2, 4>& corners);
Point2 centerOf(const std::array<Point2, 2>& bbox);
double headingOf(const std::array<Point2, 4>& corners);

Point2 transformPoint(const Transform2& tf, const Point2& p);
std::array<Point2, 4> transformCorners(const Transform2& tf, const std::array<Point2, 4>& corners);
Transform2 transformFromPose(const Pose2& pose);
Pose2 poseFromTransform(const Transform2& tf);
Transform2 composeTransform(const Transform2& a, const Transform2& b);
Transform2 inverseTransform(const Transform2& tf);
Pose2 relativePose(const Pose2& from, const Pose2& to);
double normalizeAngle(double angle);

Transform2 estimateRigidTransform(const std::vector<Point2>& source,
                                  const std::vector<Point2>& target);
double cornerRmse(const std::array<Point2, 4>& source,
                  const std::array<Point2, 4>& target,
                  const Transform2& tf);
double slotSize(const std::array<Point2, 4>& corners);

}  // namespace textmap
