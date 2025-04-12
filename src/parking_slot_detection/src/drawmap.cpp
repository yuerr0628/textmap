#include"drawmap.h"
#include <opencv2/opencv.hpp>
cv::Mat canvas(1000, 1000, CV_8UC3, cv::Scalar(255, 255, 255));
cv::Mat canva(1000, 1000, CV_8UC3, cv::Scalar(255, 255, 255));
cv::String windowName = "Vehicle Path";
 
//cv::namedWindow(windowName, cv::WINDOW_NORMAL);

void drawVehicleTrajectory(const std::vector<PoseData>& path)
{
    canvas.setTo(cv::Scalar(255, 255, 255)); // 清空画布
    cv::namedWindow(windowName, cv::WINDOW_NORMAL);

    // 绘制车辆轨迹
    for (const auto& pose : path)
    {
        cv::Point point(pose.x + canvas.cols / 2, pose.y + canvas.rows / 2);
        cv::circle(canvas, point, 2, cv::Scalar(0, 0, 255), cv::FILLED);
    }

    // 显示画布
    cv::imshow(windowName, canvas);
    cv::waitKey(1);
}

void drawslot(const std::vector<AssociatedPair> &updateslots)
{
    
       cv::Scalar color1(255, 0, 0);
    cv::Scalar color(0, 255, 0);
for (const auto& pair : updateslots) {
            // 绘制车位多边形
                cv::circle(canva, cv::Point(pair.spot.x1+ canvas.cols / 2, pair.spot.y1 + canvas.rows / 2), 3, color1, 2);
                cv::circle(canva, cv::Point(pair.spot.x2+ canvas.cols / 2, pair.spot.y2 + canvas.rows / 2), 3, color1, 2);
                cv::line(canva, cv::Point(pair.spot.x1+ canvas.cols / 2, pair.spot.y1 + canvas.rows / 2), cv::Point(pair.spot.x2+ canvas.cols / 2, pair.spot.y2 + canvas.rows / 2), cv::Scalar(0, 0, 255), 2);
                cv::line(canva, cv::Point(pair.spot.x1+ canvas.cols / 2, pair.spot.y1 + canvas.rows / 2), cv::Point(pair.spot.x3+ canvas.cols / 2, pair.spot.y3 + canvas.rows / 2), cv::Scalar(0, 0, 255), 2);
                cv::line(canva, cv::Point(pair.spot.x2+ canvas.cols / 2, pair.spot.y2 + canvas.rows / 2), cv::Point(pair.spot.x4+ canvas.cols / 2, pair.spot.y4 + canvas.rows / 2), cv::Scalar(0, 0, 255), 2);
                cv::Rect ocr_rect(cv::Point(pair.ocrPoint.x1+ canvas.cols / 2, pair.ocrPoint.y1 + canvas.rows / 2),
                             cv::Point(pair.ocrPoint.x2+ canvas.cols / 2, pair.ocrPoint.y2 + canvas.rows / 2));
                cv::rectangle(canva, ocr_rect, cv::Scalar(255, 0, 0), 2);
                cv::putText(canva, pair.ocrPoint.text,
                        cv::Point(pair.ocrPoint.x1+ canvas.cols / 2, pair.ocrPoint.y1 - 10 + canvas.rows / 2),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 0, 255));
                }
        
    
    cv::imshow("map", canva);
    cv::waitKey(1); // 等待按键继续

}

