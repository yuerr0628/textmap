#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <parking_slot_detection/gcn_parking.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

cv::Mat canvas(1000, 1000, CV_8UC3, cv::Scalar(255, 255, 255));
cv::String windowName = "slots";


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
    // while(ros::ok()){
        // 发送请求
        if (client.call(srv))
        {
            // 成功接收响应
            cv::Mat image_vis = image.clone();
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