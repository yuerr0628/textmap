// frame.h
#ifndef FRAME_H
#define FRAME_H

#include <string>
#include <vector>
#include <geometry_msgs/Point.h>

using namespace std;
namespace OcrSlam {

class Frame {
public:
    Frame(const std::string& parking_slot_id,
          const std::vector<geometry_msgs::Point>& corners,
          const std::string& license_plate);

    std::string getParkingSlotID() const;
    std::vector<geometry_msgs::Point> getCorners() const;
    std::string getLicensePlate() const;

private:
    std::string parking_slot_id_;         // 停车位车位号
    std::vector<geometry_msgs::Point> corners_; // 角点
    std::string license_plate_;            // 车牌号
};

}

#endif // FRAME_H
