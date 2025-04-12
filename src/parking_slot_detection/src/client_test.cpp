#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <parking_slot_detection/gcn_parking.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <string> 
using std::string;

cv::Mat canvas(1000, 1000, CV_8UC3, cv::Scalar(255, 255, 255));
cv::String windowName = "slots";
int m=0;
struct ParkingSpot {
    double x1, y1, x2, y2, x3, y3, x4, y4;
};

struct OCRPoint {
    string text; // 车位号ID
    double x1, y1, x2, y2;

};

struct AssociatedPair {
    ParkingSpot spot; // 关联的车位信息
    OCRPoint ocrPoint; // 关联的车位号信息
    int ID; // 帧id
};

class AssociatedParkingInfo {
public:
    std::vector<AssociatedPair> associatedPairs; // 存储关联结果的向量
    std::vector<std::vector<AssociatedPair>> AllFramesAssociatedPairs;
    void associateSpotsAndNumbers(const parking_slot_detection::gcn_parking & srv);
    void drawslotwithnumber(const cv::Mat& image);
    void addFrameAssociatedPairs(const std::vector<AssociatedPair>& framePairs);
};

void AssociatedParkingInfo::addFrameAssociatedPairs(const std::vector<AssociatedPair>& framePairs)
{
    AllFramesAssociatedPairs.push_back(framePairs);
}


void AssociatedParkingInfo::associateSpotsAndNumbers(const parking_slot_detection::gcn_parking & srv)
{
    double association_distance=0.5;
    m=m+1;
    AssociatedPair pair;
    pair.ID=m;
    std::vector<bool> matched_spot_flags(srv.response.point0_x.size(), false); // 标记车位是否已匹配
    std::vector<bool> matched_number_flags(srv.response.ocrpointx1.size(), false); // 标记车位号是否已匹配
     for (size_t i = 0; i < srv.response.point0_x.size(); ++i){
         for (size_t j = 0; j < srv.response.ocrpointx1.size(); ++j){
                double point0_x = (srv.response.point0_x[i]+srv.response.point1_x[i])/216;
                double point0_y = (srv.response.point0_y[i]+srv.response.point1_y[i])/216;
                double x1 = (srv.response.ocrpointx1[j]+srv.response.ocrpointx2[j])/216;
                double y1 = (srv.response.ocrpointy1[j]+srv.response.ocrpointy2[j])/216;
                double distance = sqrt(pow(x1 - point0_x, 2) + pow(y1 - point0_y, 2));
                if (distance < association_distance) {
                    // std::cout<<"enter"<<std::endl;
                    // 将关联的车位和车位号打包
                    pair.spot.x1 = srv.response.point0_x[i];
                    pair.spot.y1 = srv.response.point0_y[i];
                    pair.spot.x2 = srv.response.point1_x[i];
                    pair.spot.y2 = srv.response.point1_y[i];
                    pair.spot.x3 = srv.response.point2_x[i];
                    pair.spot.y3 = srv.response.point2_y[i];
                    pair.spot.x4 = srv.response.point3_x[i];
                    pair.spot.y4 = srv.response.point3_y[i];
                    pair.ocrPoint.x1 = srv.response.ocrpointx1[j];
                    pair.ocrPoint.y1 = srv.response.ocrpointy1[j];
                    pair.ocrPoint.x2 = srv.response.ocrpointx2[j];
                    pair.ocrPoint.y2 = srv.response.ocrpointy2[j];
                    pair.ocrPoint.text = srv.response.texts[j];
                    associatedPairs.push_back(pair); // 存储关联对
                    matched_spot_flags[i] = true;
                    matched_number_flags[j] = true;
                    break;
                }

         }
     }
         // 添加未匹配的车位信息
    for (size_t i = 0; i < matched_spot_flags.size(); ++i) {
        if (!matched_spot_flags[i]) {
            // 填充车位信息，车位号信息留空或设置默认值
            std::cout<<pair.ocrPoint.x1;
            pair.spot.x1 = srv.response.point0_x[i];
            pair.spot.y1 = srv.response.point0_y[i];
            pair.spot.x2 = srv.response.point1_x[i];
            pair.spot.y2 = srv.response.point1_y[i];
            pair.spot.x3 = srv.response.point2_x[i];
            pair.spot.y3 = srv.response.point2_y[i];
            pair.spot.x4 = srv.response.point3_x[i];
            pair.spot.y4 = srv.response.point3_y[i];
            associatedPairs.push_back(pair);
        }
    }
    // 添加未匹配的车位号信息
    for (size_t j = 0; j < matched_number_flags.size(); ++j) {
        if (!matched_number_flags[j]) {
            // 车位信息留空或设置默认值，填充车位号信息
            // pair.spot 留空 ...
            pair.ocrPoint.x1 = srv.response.ocrpointx1[j];
            pair.ocrPoint.y1 = srv.response.ocrpointy1[j];
            pair.ocrPoint.x2 = srv.response.ocrpointx2[j];
            pair.ocrPoint.y2 = srv.response.ocrpointy2[j];
            pair.ocrPoint.text = srv.response.texts[j];
            associatedPairs.push_back(pair);
        }
    }
    addFrameAssociatedPairs(associatedPairs);
}

void AssociatedParkingInfo::drawslotwithnumber(const cv::Mat& image)
{
    cv::Scalar color1(255, 0, 0);
    cv::Scalar color(0, 255, 0);
    cv::Mat output_image = image.clone(); 
for (const auto& pair : associatedPairs) {
            // 绘制车位多边形
             if(pair.spot.x1!=0&&!pair.ocrPoint.text.empty()){
                cv::circle(output_image, cv::Point(pair.spot.x1, pair.spot.y1), 3, color1, 2);
                cv::circle(output_image, cv::Point(pair.spot.x2, pair.spot.y2), 3, color1, 2);
                cv::line(output_image, cv::Point(pair.spot.x1, pair.spot.y1), cv::Point(pair.spot.x2, pair.spot.y2), cv::Scalar(0, 0, 255), 2);
                cv::line(output_image, cv::Point(pair.spot.x1, pair.spot.y1), cv::Point(pair.spot.x3, pair.spot.y3), cv::Scalar(0, 0, 255), 2);
                cv::line(output_image, cv::Point(pair.spot.x2, pair.spot.y2), cv::Point(pair.spot.x4, pair.spot.y4), cv::Scalar(0, 0, 255), 2);
                cv::Rect ocr_rect(cv::Point(pair.ocrPoint.x1, pair.ocrPoint.y1),
                             cv::Point(pair.ocrPoint.x2, pair.ocrPoint.y2));
                cv::rectangle(output_image, ocr_rect, cv::Scalar(255, 0, 0), 2);
                cv::putText(output_image, pair.ocrPoint.text,
                        cv::Point(pair.ocrPoint.x1, pair.ocrPoint.y1 - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255));
                }
            // if(!pair.spot.x1==0&&pair.ocrPoint.text.empty()){
            //     cv::circle(output_image, cv::Point(pair.spot.x1, pair.spot.y1), 3, color, 2);
            //     cv::circle(output_image, cv::Point(pair.spot.x2, pair.spot.y2), 3, color, 2);
            //     cv::line(output_image, cv::Point(pair.spot.x1, pair.spot.y1), cv::Point(pair.spot.x2, pair.spot.y2), cv::Scalar(0, 255, 255), 2);
            //     cv::line(output_image, cv::Point(pair.spot.x1, pair.spot.y1), cv::Point(pair.spot.x3, pair.spot.y3), cv::Scalar(0, 255, 255), 2);
            //     cv::line(output_image, cv::Point(pair.spot.x2, pair.spot.y2), cv::Point(pair.spot.x4, pair.spot.y4), cv::Scalar(0, 255, 255), 2);
            //  if(!pair.ocrPoint.text.empty()&&pair.spot.x1==0){
            // // 绘制车位号的矩形框和文本
            // cv::Rect ocr_rect(cv::Point(pair.ocrPoint.x1, pair.ocrPoint.y1),
            //                  cv::Point(pair.ocrPoint.x2, pair.ocrPoint.y2));
            // cv::rectangle(output_image, ocr_rect, cv::Scalar(255, 255, 0), 2);
            // cv::putText(output_image, pair.ocrPoint.text,
            //             cv::Point(pair.ocrPoint.x1, pair.ocrPoint.y1 - 10),
            //             cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255));}
        // }

        // 显示结果图像
        cv::imshow("Associated Parking Spots and Numbers", output_image);
        cv::waitKey(1); // 等待按键继续
    }
    
}





void imageCallback(const sensor_msgs::CompressedImageConstPtr& msg)
{
    canvas.setTo(cv::Scalar(255, 255, 255));
     try
    {
        // 解压缩图像
        cv::Mat image = cv::imdecode(cv::Mat(msg->data),cv::IMREAD_COLOR);
        // 确保图像尺寸符合预期
        cv::resize(image, image, cv::Size(512, 512));
        cv::imshow(windowName, image);
        cv::waitKey(1);

        // 转换为ROS图像消息
        sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

        // 创建ROS服务客户端
        ros::NodeHandle nh;
        ros::ServiceClient client = nh.serviceClient<parking_slot_detection::gcn_parking>("gcn_service");

        // 创建请求和响应消息
        parking_slot_detection::gcn_parking srv;
        srv.request.image_data = *image_msg;
        AssociatedParkingInfo associatedInfo;
    // while(ros::ok()){
        // 发送请求
        if (client.call(srv))
        {
            
            associatedInfo.associateSpotsAndNumbers(srv);
            
            // 成功接收响应
            cv::Mat image_vis = image.clone();
            associatedInfo.drawslotwithnumber(image_vis);
            cv::Scalar color(255, 0, 0);
            for (size_t i = 0; i < srv.response.point0_x.size(); ++i)
            {
                int point0_x = srv.response.point0_x[i];
                int point0_y = srv.response.point0_y[i];
                int point1_x = srv.response.point1_x[i];
                int point1_y = srv.response.point1_y[i];
                int point2_x = srv.response.point2_x[i];
                int point2_y = srv.response.point2_y[i];
                int point3_x = srv.response.point3_x[i];
                int point3_y = srv.response.point3_y[i];
                int type = srv.response.types[i];
                cv::circle(image_vis, cv::Point(point0_x, point0_y), 3, color, 2);
                cv::circle(image_vis, cv::Point(point1_x, point1_y), 3, color, 2);
                cv::line(image_vis, cv::Point(point0_x, point0_y), cv::Point(point1_x, point1_y), cv::Scalar(0, 0, 255), 2);
                cv::line(image_vis, cv::Point(point0_x, point0_y), cv::Point(point2_x, point2_y), cv::Scalar(0, 0, 255), 2);
                cv::line(image_vis, cv::Point(point1_x, point1_y), cv::Point(point3_x, point3_y), cv::Scalar(0, 0, 255), 2);
            }
            for (size_t i = 0; i < srv.response.ocrpointx1.size(); ++i)
            {
                double x1 = srv.response.ocrpointx1[i];
                double x2 = srv.response.ocrpointx2[i];
                double y1 = srv.response.ocrpointy1[i];
                double y2 = srv.response.ocrpointy2[i];
                string text=srv.response.texts[i];
                cv::Scalar rectangleColor(0, 255, 0); // 绿色，BGR颜色空间
                cv::rectangle(image_vis, cv::Point(x1, y1), cv::Point(x2, y2), rectangleColor, 2);
                int fontFace = cv::FONT_HERSHEY_SIMPLEX;
                double fontScale = 0.5;
                cv::Scalar textColor(0, 255, 0); // 绿色
                int thickness = 2;
                // 绘制文本
                // cv::putText(image_vis, text, cv::Point(x1, y1-10), fontFace, fontScale, textColor, thickness);
            }
            cv::imshow("detected_results", image_vis);
            cv::waitKey(1);

        }
        else {
            ROS_ERROR("Failed to call service");
        }
    }
    // }

    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->format.c_str());
    }

}


int main(int argc, char** argv) {
    ros::init(argc, argv, "~");
    ros::NodeHandle mh;
    // cv::namedWindow("detected_results");
    //  cv::startWindowThread();
    // image_transport::ImageTransport it(nh);
    // // 订阅压缩图像主题
    // image_transport::ImageTransport it(nh);
    // image_transport::Subscriber sub = it.subscribe("/driver/fisheye/avm/compressed", 1, imageCallback, ros::VoidPtr(), image_transport::TransportHints("compressed"));
    ros::Subscriber sub = mh.subscribe("/driver/fisheye/avm/compressed", 10, imageCallback);

    // image_transport::Subscriber sub = it.subscribe("/driver/fisheye/avm/compressed", 1, imageCallback);
    ros::spin();
    // cv::destroyAllWindows();

 
}