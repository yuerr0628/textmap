#include "pose.h"
#include <tf/tf.h>
#include "drawmap.h"
#include "rvizshow.h"
// #include <gps_common/conversions.h>

RvizDisplay mydisplay;
std::ofstream tum_file;

bool init=false;
static double EARTH_RADIUS = 6378.137;//地球半径
double roll;
double pitch;
double yaw;
struct my_pose
{
    double latitude;
    double longitude;
    double altitude;
};
my_pose init_pose;
double rad(double d) 
{
	return d * 3.1415926 / 180.0;
}
VehiclePose::VehiclePose(ros::NodeHandle& nh) {
    gps_sub= nh.subscribe("/Inertial/gps/fix", 10, &VehiclePose::gpsCallback, this);
    imu_sub = nh.subscribe("/Inertial/imu/data", 10, &VehiclePose::imuCallback, this);
    message_filters::Subscriber<sensor_msgs::NavSatFix> gps_sub1(nh, "/Inertial/gps/fix", 200);
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub1(nh, "/Inertial/imu/data", 200);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::NavSatFix, sensor_msgs::Imu> MySyncPolicy2;
    message_filters::Synchronizer<MySyncPolicy2> sync2(MySyncPolicy2(1000), gps_sub1, imu_sub1); // queue size=10
    sync2.registerCallback(boost::bind(&odomCallback, _1, _2));
    // path_pub = nh.advertise<nav_msgs::Path>("vehicle_trajectory", 10);

    mydisplay.RvizDisplay_init(nh);
}

void VehiclePose::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
    // 从IMU消息中获取朝向信息（例如，使用四元数转换为欧拉角）

    // std::cout << "imuCallback" << std::endl;

        if(imu_msg->orientation_covariance[0] < 0)
	{
		return;
	}
    tf::Quaternion q(imu_msg->orientation.x, imu_msg->orientation.y,
                     imu_msg->orientation.z, imu_msg->orientation.w);
     std::ofstream file("/data/yhy/imu_trajectory.txt", std::ios::out | std::ios::app);
    if (!file.is_open()) {
        // std::cerr << "Error: Unable to open file " << filename << " for writing." << std::endl;
        return;
    }

    // 遍历轨迹，写入 TUM 格式
    double timestamp=imu_msg->header.stamp.toSec();
    
        file << std::fixed << std::setprecision(6)
             << timestamp << " "
             << q.x() << " "               // 四元数 x
             << q.y() << " "               // 四元数 y
             << q.z() << " "               // 四元数 z
             << q.w()                      // 四元数 w
             << std::endl;
    

    tf::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    // 更新车辆朝向
    roll = roll*180.0f/M_PI;
	pitch = pitch*180.0f/M_PI;
    // yaw=yaw+1/2*M_PI;
    // yaw = -yaw;
    pose.yaw=yaw;
}

void VehiclePose::gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg) {
    // 从GPS消息中获取位置信息
    if(!init)
    {
        init_pose.latitude = gps_msg->latitude;
        init_pose.longitude = gps_msg->longitude;
        init_pose.altitude = gps_msg->altitude;
        init = true;
    }
    else
    {
    //计算相对位置
        double radLat1 ,radLat2, radLong1,radLong2,delta_lat,delta_long,x,y;
		radLat1 = rad(init_pose.latitude);
        radLong1 = rad(init_pose.longitude);
		radLat2 = rad(gps_msg->latitude);
		radLong2 = rad(gps_msg->longitude);
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
        pose.x=y;
        pose.y=x;
        pose.z=z;
        // std::cout<<"pose.x"<<pose.x<<"pose.y:"<<pose.y<<endl;
            double y1 = sin(delta_long) * cos(radLat2);
    double x1 = cos(radLat1) * sin(radLat2) - sin(radLat1) * cos(radLat2) * cos(delta_long);
    double heading = atan2(x1, y1);

    // 将航向转换为度
    heading = heading * 180.0 / M_PI;
    // cout<<heading<<endl;

         std::ofstream file("/data/yhy/gps_trajectory.txt", std::ios::out | std::ios::app);
    if (!file.is_open()) {
        // std::cerr << "Error: Unable to open file " << filename << " for writing." << std::endl;
        return;
    }

    // 遍历轨迹，写入 TUM 格式
    double timestamp=gps_msg->header.stamp.toSec();
    
        file << std::fixed << std::setprecision(6)
             << timestamp << " "
             << x << " "              // ENU x
             << y<< " "              // ENU y
             << z<< " "              // ENU z
             << std::endl;
        pose_path.push_back(pose);
        //  drawVehicleTrajectory(pose_path);
        // RvizDisplay display;
    //      double latitude = msg->latitude;
    // double longitude = msg->longitude;
    // double altitude = msg->altitude;

    // // 转换为UTM坐标
    // double utm_x, utm_y;
    // std::string utm_zone;
    // gps_common::LLtoUTM(latitude, longitude, utm_y, utm_x, utm_zone);

    // // 保存UTM坐标到路径点
    // PoseData pose_data;
    // pose_data.x = utm_x;
    // pose_data.y = utm_y;
    // pose_data.z = altitude; // 如果需要高度信息

    // // 设置朝向 (假设航向角由其他传感器或算法提供)
    // pose_data.roll = 0.0;   // 假设无滚转角
    // pose_data.pitch = 0.0;  // 假设无俯仰角
    // pose_data.yaw = calculateHeading(msg); // 自定义函数计算航向角
    //     mydisplay.publishPosePath(pose_data);

        mydisplay.publishPosePath(pose);

    }
    

    // 可以在这里添加处理GPS信息的其他逻辑
}

// 回调函数，处理同步的 GPS 和 IMU 数据
void odomCallback(const sensor_msgs::NavSatFix::ConstPtr& gps_msg, const sensor_msgs::Imu::ConstPtr& imu_msg) {
        // tum_file.open("/data/yhy/gps_imu_trajectory.txt", std::ios::out);
        std::cout<<"odomcallbavck"<<endl;
        std::ofstream file("/data/yhy/gps_imu_trajectory.txt", std::ios::out);
    if (!tum_file.is_open()) {
        ROS_ERROR("TUM file is not open.");
        return;
    }

     if(!init)
    {
        init_pose.latitude = gps_msg->latitude;
        init_pose.longitude = gps_msg->longitude;
        init_pose.altitude = gps_msg->altitude;
        init = true;
    }
    else
    {
    //计算相对位置
        double radLat1 ,radLat2, radLong1,radLong2,delta_lat,delta_long,x,y;
		radLat1 = rad(init_pose.latitude);
        radLong1 = rad(init_pose.longitude);
		radLat2 = rad(gps_msg->latitude);
		radLong2 = rad(gps_msg->longitude);
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
    // IMU 提供朝向（四元数）
    Eigen::Quaterniond orientation(
        imu_msg->orientation.w,
        imu_msg->orientation.x,
        imu_msg->orientation.y,
        imu_msg->orientation.z
    );

    // 写入 TUM 格式文件
    tum_file << std::fixed << std::setprecision(6)
             << gps_msg->header.stamp.toSec() << " "  // 时间戳
             << x << " "              // ENU x
             << y<< " "              // ENU y
             << z<< " "              // ENU z
             << orientation.x() << " "               // 四元数 x
             << orientation.y() << " "               // 四元数 y
             << orientation.z() << " "               // 四元数 z
             << orientation.w()                      // 四元数 w
             << std::endl;

    // ROS_INFO("Data written to TUM file: [%f, %f, %f]", enu_position.x(), enu_position.y(), enu_position.z());
}
}
