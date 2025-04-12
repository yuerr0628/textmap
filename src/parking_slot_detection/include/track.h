#ifndef PARKING_MAP_H
#define PARKING_MAP_H

#include <vector>
#include <string>
#include <utility> // for std::pair
#include"pose.h"
#include"associate.h"

// 定义ICP算法的接口
class IterativeClosestPoint {
public:
    std::vector<std::pair<int, int>> matchSpots(
    const std::vector<ParkingSpot>& spotsA,
    const std::vector<ParkingSpot>& spotsB);
};

// 定义地图类
class ParkingMap {
private:
    std::vector<std::vector<AssociatedPair>> AllFramesAssociatedPairs;
    std::vector<CameraPose> cameraPoses; // 存储每帧的相机位置和姿态

public:
    void addFrame(const std::vector<AssociatedPair>& frame_pairs, int frame_id);
    void associateConsecutiveFrames();
};

#endif // PARKING_MAP_H