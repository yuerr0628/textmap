#include <ros/ros.h>
#include "turtlesim/Pose.h"
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <math.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/String.h>
#include <geometry_msgs/Polygon.h>
#include <showtrajectory/OCRResult.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

struct my_pose
{
    double latitude;
    double longitude;
    double altitude;
};
//角度制转弧度制
double rad(double d) 
{
	return d * 3.1415926 / 180.0;
}
//全局变量
static double EARTH_RADIUS = 6378.137;//地球半径
ros::Publisher state_pub_;
ros::Publisher marker_pub;  // RViz中显示标记的发布器
nav_msgs::Path ros_path_;
bool init;
my_pose init_pose;
int marker_id = 0;
 geometry_msgs::PoseStamped pose;
 double time_1;
 std::string word_1;
double roll;
double pitch;
double yaw;
geometry_msgs::Quaternion imu_quaternion;
// 创建OpenCV窗口和画布
cv::Mat canvas(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));
cv::String windowName = "Vehicle Path";

void drawPathOnCV(const nav_msgs::Path& path)
{
    canvas.setTo(cv::Scalar(255, 255, 255)); // 清空画布

    // 绘制车辆轨迹
    for (const auto& pose : path.poses)
    {
        cv::Point point(pose.pose.position.x + canvas.cols / 2, pose.pose.position.y + canvas.rows / 2);
        cv::circle(canvas, point, 2, cv::Scalar(0, 0, 255), cv::FILLED);
    }

    // 显示画布
    cv::imshow(windowName, canvas);
    cv::waitKey(1);
}
void gpsCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg_ptr)
{

   // std::cout <<  " gps: "  << gps_msg_ptr->header.stamp << std::endl;
    //初始化
    double time_2=gps_msg_ptr->header.stamp.toSec();
    if(!init)
    {
        init_pose.latitude = gps_msg_ptr->latitude;
        init_pose.longitude = gps_msg_ptr->longitude;
        init_pose.altitude = gps_msg_ptr->altitude;
        init = true;
    }
    else
    {
    //计算相对位置
        double radLat1 ,radLat2, radLong1,radLong2,delta_lat,delta_long,x,y;
		radLat1 = rad(init_pose.latitude);
        radLong1 = rad(init_pose.longitude);
		radLat2 = rad(gps_msg_ptr->latitude);
		radLong2 = rad(gps_msg_ptr->longitude);
    
        //计算x
        delta_long = 0;
	delta_lat = radLat2 - radLat1;  //(radLat1,radLong1)-(radLat2,radLong1)
	if(delta_lat>0)
        x = 2*asin( sqrt( pow( sin( delta_lat/2 ),2) + cos( radLat1 )*cos( radLat2)*pow( sin( delta_long/2 ),2 ) ));
        else
	x=-2*asin( sqrt( pow( sin( delta_lat/2 ),2) + cos( radLat1 )*cos( radLat2)*pow( sin( delta_long/2 ),2 ) ));

        x = x*EARTH_RADIUS*1000;

        //计算y
	delta_lat = 0;
        delta_long = radLong2  - radLong1;   //(radLat1,radLong1)-(radLat1,radLong2)
	if(delta_long>0)
	y = 2*asin( sqrt( pow( sin( delta_lat/2 ),2) + cos( radLat2 )*cos( radLat2)*pow( sin( delta_long/2 ),2 ) ) );
	else
	y=-2*asin( sqrt( pow( sin( delta_lat/2 ),2) + cos( radLat2 )*cos( radLat2)*pow( sin( delta_long/2 ),2 ) ) );
        //double y = 2*asin( sin( delta_lat/2 ) + cos( radLat2 )*cos( radLat2)* sin( delta_long/2 )   );
        y = y*EARTH_RADIUS*1000;

        //计算z
      //  double z = gps_msg_ptr->altitude - init_pose.altitude;
            double z =0;
        //发布轨迹
        ros_path_.header.frame_id = "map";
        ros_path_.header.stamp = ros::Time::now();  
        // std::cout<<time_1<<","<<time_2<<std::endl;
    //    if((time_1-time_2<0.1&&time_1-time_2>-0.1) || word_1.length() == 0){
        pose.header = ros_path_.header;

        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;

        ros_path_.poses.push_back(pose);

      //  ROS_INFO("( x:%0.6f ,y:%0.6f ,z:%0.6f)",x ,y ,z );

        state_pub_.publish(ros_path_);
            // 在OpenCV上绘制轨迹
        drawPathOnCV(ros_path_);
    //    }

    }
}

void imuCallback(const sensor_msgs::Imu::ConstPtr&imu_msg_ptr)
{
    if(imu_msg_ptr->orientation_covariance[0] < 0)
	{
		return;
	}

    imu_quaternion = imu_msg_ptr->orientation;

  //  tf2::convert(imu_quaternion, tf_quaternion);

	tf::Quaternion quarternion(
		imu_msg_ptr->orientation.x,
		imu_msg_ptr->orientation.y,
		imu_msg_ptr->orientation.z,
		imu_msg_ptr->orientation.w
	);
 
	tf::Matrix3x3(quarternion).getRPY(roll, pitch, yaw);
	
	roll = roll*180.0f/M_PI;
	pitch = pitch*180.0f/M_PI;
     yaw = -yaw;
	// yaw = yaw*180.0f/M_PI;

//	ROS_INFO("roll = %0.3f, pitch=%0.3f, yaw=%0.3f", roll, pitch, yaw);

}


void ocrCallback(const showtrajectory::OCRResult::ConstPtr&ocr_msg)
{
  const std::vector<geometry_msgs::Point32>& vertices = ocr_msg->polygon.points;
  //time_1=ocr_msg->stamp;
  word_1=ocr_msg->text;
    visualization_msgs::Marker marker;
    marker.header.frame_id= "map";
    marker.ns = "text_boxes";
    marker.id = marker_id++;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    // marker.pose.orientation.w =  imu_quaternion.w;
    // marker.pose.orientation.z =  imu_quaternion.z;
    marker.scale.x = 0.1;

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.lifetime = ros::Duration(0);

    for (size_t i = 0; i < vertices.size(); i++)
    {
        geometry_msgs::Point p1;
    
        // p1.x = vertices[i].x+pose.pose.position.x;
        // p1.y = vertices[i].y+pose.pose.position.y;
        // p1.z = vertices[i].z+pose.pose.position.z;

        // std::cout << yaw << std::endl;
        p1.x = vertices[i].y * cos(yaw) -  vertices[i].x * sin(yaw) +pose.pose.position.x;
        p1.y = vertices[i].y* sin(yaw) +  vertices[i].x * cos(yaw)+pose.pose.position.y;
        p1.z = vertices[i].z+pose.pose.position.z;
        // p1.x = vertices[i].x * cos(yaw) -  vertices[i].y * sin(yaw) +pose.pose.position.x;
        // p1.y = vertices[i].x* sin(yaw) +  vertices[i].y * cos(yaw)+pose.pose.position.y;
        // p1.z = vertices[i].z+pose.pose.position.z;
        
        
        geometry_msgs::Point p2;
        p2.x =  vertices[(i + 1) % vertices.size()].y * cos(yaw) -  vertices[(i + 1) % vertices.size()].x * sin(yaw)+pose.pose.position.x;
        p2.y = vertices[(i + 1) % vertices.size()].y * sin(yaw) +  vertices[(i + 1) % vertices.size()].x * cos(yaw) +pose.pose.position.y;
        p2.z = vertices[(i + 1) % vertices.size()].z+pose.pose.position.z;
        // p2.x =  vertices[(i + 1) % vertices.size()].x * cos(yaw) -  vertices[(i + 1) % vertices.size()].y * sin(yaw)+pose.pose.position.x;
        // p2.y = vertices[(i + 1) % vertices.size()].x * sin(yaw) +  vertices[(i + 1) % vertices.size()].y * cos(yaw) +pose.pose.position.y;
        // p2.z = vertices[(i + 1) % vertices.size()].z+pose.pose.position.z;

        marker.points.push_back(p1);
        marker.points.push_back(p2);
    }

    marker_pub.publish(marker);

    // Calculate the center point of the bounding box
    geometry_msgs::Point center;
    for (const geometry_msgs::Point32& vertex : vertices)
    {
        center.x += vertex.y * cos(yaw) -  vertex.x * sin(yaw)+pose.pose.position.x;
        center.y += vertex.y * sin(yaw) +  vertex.x * cos(yaw)+pose.pose.position.y;
        center.z += vertex.z+pose.pose.position.z;
    }
    center.x /= vertices.size();
    center.y /= vertices.size();
    center.z /= vertices.size();

    visualization_msgs::Marker text_marker;
     text_marker.header.frame_id = "map"; ;
    text_marker.ns = "text";
    text_marker.id = marker_id++;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.action = visualization_msgs::Marker::ADD;
    text_marker.pose.position = center;
    // text_marker.pose.orientation.w =  imu_quaternion.w;
    // text_marker.pose.orientation.z =  imu_quaternion.z;

    text_marker.scale.z = 1.0;
    text_marker.color.r = 0.0;
    text_marker.color.g = 1.0;
    text_marker.color.b = 0.0;
    text_marker.color.a = 1.0;
    text_marker.lifetime = ros::Duration(0);
    text_marker.text = ocr_msg->text;

    marker_pub.publish(text_marker);
}


int main(int argc,char **argv)
{
    init = false;
    ros::init(argc,argv,"gps_to_rviz");
    ros::NodeHandle nh;
    ros::Subscriber pose_sub=nh.subscribe("/Inertial/gps/fix",10, gpsCallback);
   
    ros::Subscriber sub = nh.subscribe<showtrajectory::OCRResult>("ocr_text_with_box", 1000, ocrCallback);
    ros::Subscriber imu_sub = nh.subscribe("/Inertial/imu/data", 1, imuCallback);

   marker_pub = nh.advertise<visualization_msgs::Marker>("text_boxes", 10);
    state_pub_ = nh.advertise<nav_msgs::Path>("gps_path", 10);   


    // message_filters::Subscriber<showtrajectory::OCRResult> pc_origin1_sub(nh, "ocr_text_with_box", 200);
    // message_filters::Subscriber<sensor_msgs::Imu> pc_origin2_sub(nh, "/Inertial/imu/data", 2000);
    // message_filters::Subscriber<sensor_msgs::NavSatFix> pc_origin3_sub(nh, "/Inertial/gps/fix", 2000);

    // typedef message_filters::sync_policies::ApproximateTime<showtrajectory::OCRResult, sensor_msgs::Imu,sensor_msgs::NavSatFix> MySyncPolicy;
    // message_filters::Synchronizer<MySyncPolicy> sync1(MySyncPolicy(200), pc_origin1_sub, pc_origin2_sub,pc_origin3_sub ); // queue size=10
    // sync1.registerCallback(boost::bind(&callback_origin, _1, _2, _3));
   

    ros::spin();
    return 0;//还可以根据需要对图像进行畸变校正。您可以使用提供的畸变中心坐标和映射系数矩阵来进行校正，使用cv2.undistort()函数可以实现畸变校正。
}
