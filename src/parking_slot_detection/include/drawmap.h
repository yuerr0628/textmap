#ifndef DRAWMAP_H
#define DRAWMAP_H

#include <vector>
#include "pose.h" // 确保包含VehiclePose类的头文件
#include "associate.h"

// 用于OpenCV的图像类型

// 绘制车辆轨迹的函数声明

 void drawVehicleTrajectory(const std::vector<PoseData>& path);
 void drawslot(const std::vector<AssociatedPair>& updateslots);
// void drawmapspot(const std::vector<PoseData>& path,const std::vector<AssociatedPair>& worldspots);


#endif // DRAWMAP_H