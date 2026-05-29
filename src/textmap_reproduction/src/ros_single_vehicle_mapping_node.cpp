#include "json_io.hpp"
#include "ros_pose_provider.hpp"
#include "single_vehicle_mapper.hpp"

#include <parking_slot_detection/gcn_parking.h>

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/imgcodecs.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/Header.h>

#include <cmath>

namespace textmap {
namespace {

constexpr double kSpotCx = 256.0;
constexpr double kSpotCy = 256.0;
constexpr double kSpotScaleX = 55.52;
constexpr double kSpotScaleY = 54.94;
constexpr double kTextCx = 512.0;
constexpr double kTextCy = 512.0;
constexpr double kTextScaleX = 111.04;
constexpr double kTextScaleY = 109.88;

struct PixelPair {
  SlotObservation obs;
  bool matched_spot = false;
  bool matched_text = false;
};

double spotMetricX(double px) { return (px - kSpotCx) / kSpotScaleX; }
double spotMetricY(double py) { return (py - kSpotCy) / kSpotScaleY; }
double textMetricX(double px) { return (px - kTextCx) / kTextScaleX; }
double textMetricY(double py) { return (py - kTextCy) / kTextScaleY; }

Point2 bevToVehicleSpot(double x_px, double y_px) {
  return {-(y_px - 319.0) / kSpotScaleY, -(x_px - 256.0) / kSpotScaleX};
}

Point2 bevToVehicleText(double x_px, double y_px) {
  return {-(y_px - 638.0) / kTextScaleY, -(x_px - 512.0) / kTextScaleX};
}

Point2 vehicleToWorld(const Point2& local, const Pose2& pose) {
  const double c = std::cos(pose.yaw);
  const double s = std::sin(pose.yaw);
  return {c * local.x - s * local.y + pose.x, s * local.x + c * local.y + pose.y};
}

SlotObservation makeObservationFromResponse(const parking_slot_detection::gcn_parking::Response& resp,
                                            size_t spot_idx,
                                            int text_idx) {
  SlotObservation obs;
  obs.has_geometry = spot_idx < resp.point0_x.size();
  obs.has_text = text_idx >= 0;
  if (obs.has_geometry) {
    obs.corners = {{
        bevToVehicleSpot(resp.point0_x[spot_idx], resp.point0_y[spot_idx]),
        bevToVehicleSpot(resp.point1_x[spot_idx], resp.point1_y[spot_idx]),
        bevToVehicleSpot(resp.point2_x[spot_idx], resp.point2_y[spot_idx]),
        bevToVehicleSpot(resp.point3_x[spot_idx], resp.point3_y[spot_idx]),
    }};
    obs.vacant = resp.label[spot_idx];
  }
  if (obs.has_text) {
    obs.text_bbox = {{
        bevToVehicleText(resp.ocrpointx1[text_idx], resp.ocrpointy1[text_idx]),
        bevToVehicleText(resp.ocrpointx2[text_idx], resp.ocrpointy2[text_idx]),
    }};
    obs.slot_id = resp.texts[text_idx];
    obs.confidence = resp.confidence[text_idx];
  }
  return obs;
}

FrameObservation buildFrameFromDetection(const parking_slot_detection::gcn_parking::Response& resp,
                                         const Pose2& pose,
                                         double timestamp) {
  FrameObservation frame;
  frame.timestamp = timestamp;
  frame.vehicle_pose = pose;

  std::vector<bool> matched_spot(resp.point0_x.size(), false);
  std::vector<bool> matched_text(resp.ocrpointx1.size(), false);

  for (size_t i = 0; i < resp.point0_x.size(); ++i) {
    for (size_t j = 0; j < resp.ocrpointx1.size(); ++j) {
      const double point_x = (resp.point0_x[i] + resp.point1_x[i]) * 0.5;
      const double point_y = (resp.point0_y[i] + resp.point1_y[i]) * 0.5;
      const double text_x = (resp.ocrpointx1[j] + resp.ocrpointx2[j]) * 0.5;
      const double text_y = (resp.ocrpointy1[j] + resp.ocrpointy2[j]) * 0.5;
      const double dx = textMetricX(text_x) - spotMetricX(point_x);
      const double dy = textMetricY(text_y) - spotMetricY(point_y);
      if (std::sqrt(dx * dx + dy * dy) < 0.6) {
        frame.slots.push_back(makeObservationFromResponse(resp, i, static_cast<int>(j)));
        matched_spot[i] = true;
        matched_text[j] = true;
        break;
      }
    }
  }

  for (size_t i = 0; i < resp.point0_x.size(); ++i) {
    if (!matched_spot[i]) frame.slots.push_back(makeObservationFromResponse(resp, i, -1));
  }
  for (size_t j = 0; j < resp.ocrpointx1.size(); ++j) {
    if (!matched_text[j]) frame.slots.push_back(makeObservationFromResponse(resp, resp.point0_x.size(), static_cast<int>(j)));
  }

  for (auto& slot : frame.slots) {
    if (slot.has_geometry) {
      for (auto& p : slot.corners) p = vehicleToWorld(p, pose);
    }
    if (slot.has_text) {
      slot.text_bbox[0] = vehicleToWorld(slot.text_bbox[0], pose);
      slot.text_bbox[1] = vehicleToWorld(slot.text_bbox[1], pose);
    }
  }
  return frame;
}

class RosSingleVehicleMappingNode {
 public:
  explicit RosSingleVehicleMappingNode(ros::NodeHandle& nh)
      : nh_(nh),
        pose_provider_(nh),
        mapper_(),
        client_(nh.serviceClient<parking_slot_detection::gcn_parking>("gcn_service")) {
    nh_.param<std::string>("avm_topic", avm_topic_, "/driver/fisheye/avm/compressed");
    nh_.param<std::string>("front_topic", front_topic_, "/driver/fisheye/front/compressed");
    nh_.param<std::string>("output_path", output_path_, std::string("/tmp/single_vehicle_text_map.json"));
    nh_.param<int>("save_every_n_frames", save_every_n_frames_, 20);

    avm_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::CompressedImage>>(nh_, avm_topic_, 20);
    front_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::CompressedImage>>(nh_, front_topic_, 20);
    sync_ = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(20), *avm_sub_, *front_sub_);
    sync_->registerCallback(boost::bind(&RosSingleVehicleMappingNode::syncedCallback, this, _1, _2));
  }

  ~RosSingleVehicleMappingNode() {
    try {
      saveTextMap(mapper_.exportMap("single_vehicle_text_map"), output_path_);
    } catch (...) {
    }
  }

 private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::CompressedImage, sensor_msgs::CompressedImage>;

  ros::NodeHandle nh_;
  RosPoseProvider pose_provider_;
  SingleVehicleMapper mapper_;
  ros::ServiceClient client_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::CompressedImage>> avm_sub_;
  std::unique_ptr<message_filters::Subscriber<sensor_msgs::CompressedImage>> front_sub_;
  std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  std::string avm_topic_;
  std::string front_topic_;
  std::string output_path_;
  int save_every_n_frames_ = 20;
  int processed_frames_ = 0;

  void syncedCallback(const sensor_msgs::CompressedImageConstPtr& avm_msg,
                      const sensor_msgs::CompressedImageConstPtr& front_msg) {
    if (!pose_provider_.hasPose()) {
      ROS_WARN_THROTTLE(2.0, "textmap_reproduction: waiting for pose source before mapping.");
      return;
    }

    cv::Mat avm = cv::imdecode(cv::Mat(avm_msg->data), cv::IMREAD_COLOR);
    if (avm.empty()) {
      ROS_WARN_THROTTLE(2.0, "textmap_reproduction: failed to decode AVM image.");
      return;
    }

    auto image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", avm).toImageMsg();
    parking_slot_detection::gcn_parking srv;
    srv.request.image_data = *image_msg;
    if (!client_.call(srv)) {
      ROS_ERROR_THROTTLE(2.0, "textmap_reproduction: failed to call gcn_service.");
      return;
    }

    const double timestamp = front_msg->header.stamp.toSec();
    FrameObservation frame = buildFrameFromDetection(srv.response, pose_provider_.currentPose(), timestamp);
    mapper_.addFrame(frame);
    ++processed_frames_;

    if (save_every_n_frames_ > 0 && processed_frames_ % save_every_n_frames_ == 0) {
      saveTextMap(mapper_.exportMap("single_vehicle_text_map"), output_path_);
      ROS_INFO_STREAM("textmap_reproduction: saved intermediate map to " << output_path_);
    }
  }
};

}  // namespace
}  // namespace textmap

int main(int argc, char** argv) {
  ros::init(argc, argv, "single_vehicle_mapping_node");
  ros::NodeHandle nh("~");
  textmap::RosSingleVehicleMappingNode node(nh);
  ros::spin();
  return 0;
}
