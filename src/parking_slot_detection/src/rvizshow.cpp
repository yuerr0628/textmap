#include "rvizshow.h"
#include <visualization_msgs/MarkerArray.h>



int spotMarker_id = 0;
void RvizDisplay::RvizDisplay_init(ros::NodeHandle& nh){
    // 初始化ROS发布者和订阅者
    path_pub = nh.advertise<nav_msgs::Path>("vehicle_trajectory", 10);
    odom_pub = nh.advertise<nav_msgs::Path>("odom_trajectory", 10);
    ekfodom_pub = nh.advertise<nav_msgs::Path>("ekfodom_trajectory", 10);
    slots_pub = nh.advertise<visualization_msgs::MarkerArray>("parking_spots", 10);
    plates_pub = nh.advertise<visualization_msgs::MarkerArray>("plates", 10);
    map_pub = nh.advertise<visualization_msgs::Marker>("parking_map", 10);
    ekfpath_new_pub = nh.advertise<nav_msgs::Path>("/ekf_path", 10);
    ekfodom_new_pub = nh.advertise<nav_msgs::Odometry>("/ekf_odom", 10);
    localizationmap=nh.advertise<visualization_msgs::MarkerArray>("localization_map", 1000);
    camera_pub = nh.advertise<visualization_msgs::Marker>("camera_marker", 10);
    match_pub = nh.advertise<visualization_msgs::Marker>("matched_points", 100);

    // ros::Duration(10.0).sleep();
}



void RvizDisplay::publishPosePath(const PoseData& pose_data) {
    // std::cout<<"pose"<<std::endl;
    // int marker_id = 0;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = "world";
    pose_stamped.header = path_msg.header;
    pose_stamped.pose.position.x = pose_data.x;
    pose_stamped.pose.position.y = pose_data.y;
    pose_stamped.pose.position.z = pose_data.z;
    path_msg.poses.push_back(pose_stamped);
    path_pub.publish(path_msg);
            //     // 创建相机小方块
            // visualization_msgs::Marker camera_marker;
            // camera_marker.header.frame_id = "world";
            // camera_marker.header.stamp = ros::Time::now();
            // camera_marker.ns = "camera";
            // camera_marker.id = marker_id++;
            // camera_marker.type = visualization_msgs::Marker::CUBE;
            // camera_marker.action = visualization_msgs::Marker::ADD;
            // camera_marker.pose.position.x = pose_data.x;
            // camera_marker.pose.position.y = pose_data.y;
            // camera_marker.pose.position.z = pose_data.z;
            // Eigen::Quaterniond quat(Matrix3d::Identity());
            // camera_marker.pose.orientation.x = quat.x();
            // camera_marker.pose.orientation.y = quat.y();
            // camera_marker.pose.orientation.z = quat.z();
            // camera_marker.pose.orientation.w = quat.w();
            // camera_marker.scale.x = 1;  // 小方块尺寸
            // camera_marker.scale.y = 1;
            // camera_marker.scale.z = 1;
            // camera_marker.color.r = 1.0f;
            // camera_marker.color.g = 0.0f;
            // camera_marker.color.b = 0.0f;
            // camera_marker.color.a = 1.0;
            // camera_marker.lifetime = ros::Duration(0);// 模拟实时更新
            // camera_pub.publish(camera_marker);
            // marker_pub.publish(arrow_marker);

           

            // // 创建方向箭头
            // visualization_msgs::Marker arrow_marker;
            // arrow_marker.header.frame_id = "map";
            // arrow_marker.header.stamp = ros::Time::now();
            // arrow_marker.ns = "camera_arrow";
            // arrow_marker.id = marker_id++;
            // arrow_marker.type = visualization_msgs::Marker::ARROW;
            // arrow_marker.action = visualization_msgs::Marker::ADD;
            // arrow_marker.pose.position.x = t_global.x();
            // arrow_marker.pose.position.y = t_global.y();
            // arrow_marker.pose.position.z = t_global.z();
            // arrow_marker.pose.orientation.x = quat.x();
            // arrow_marker.pose.orientation.y = quat.y();
            // arrow_marker.pose.orientation.z = quat.z();
            // arrow_marker.pose.orientation.w = quat.w();
            // arrow_marker.scale.x = 0.2;  // 箭头长度
            // arrow_marker.scale.y = 0.05; // 箭头宽度
            // arrow_marker.scale.z = 0.05;
            // arrow_marker.color.r = 1.0f;
            // arrow_marker.color.g = 0.0f;
            // arrow_marker.color.b = 0.0f;
            // arrow_marker.color.a = 1.0;

            // // 添加到MarkerArray
            // marker_array.markers.push_back(camera_marker);
            // marker_array.markers.push_back(arrow_marker);

            // // 发布MarkerArray
            // marker_pub.publish(marker_array);
}

void RvizDisplay::publishodomPath(const Eigen::Vector3d & position)
{
     int marker_id = 0;
    odompath_msg.header.stamp = ros::Time::now();
    odompath_msg.header.frame_id = "world";
    odompose_stamped.pose.position.x = position(0);
    odompose_stamped.pose.position.y = position(1);
    odompose_stamped.pose.position.z = position(2);
    odompath_msg.poses.push_back(odompose_stamped);
    odom_pub.publish(odompath_msg);

        // 创建相机小方块
            visualization_msgs::Marker camera_marker;
            camera_marker.header.frame_id = "world";
            camera_marker.header.stamp = ros::Time::now();
            camera_marker.ns = "camera";
            camera_marker.id = marker_id++;
            camera_marker.type = visualization_msgs::Marker::CUBE;
            camera_marker.action = visualization_msgs::Marker::ADD;
            camera_marker.pose.position.x = position(0);
            camera_marker.pose.position.y = position(1);
            camera_marker.pose.position.z = position(2);
            Eigen::Quaterniond quat(Matrix3d::Identity());
            camera_marker.pose.orientation.x = quat.x();
            camera_marker.pose.orientation.y = quat.y();
            camera_marker.pose.orientation.z = quat.z();
            camera_marker.pose.orientation.w = quat.w();
            camera_marker.scale.x = 1;  // 小方块尺寸
            camera_marker.scale.y = 1;
            camera_marker.scale.z = 1;
            camera_marker.color.r = 1.0f;
            camera_marker.color.g = 0.0f;
            camera_marker.color.b = 0.0f;
            camera_marker.color.a = 1.0;
            camera_marker.lifetime = ros::Duration(0);// 模拟实时更新
            camera_pub.publish(camera_marker);
            // marker_pub.publish(arrow_marker);

}

void RvizDisplay::publishekfodom(const Eigen::VectorXd &state)
{
    ekfodom_msg.header.stamp = ros::Time::now();
    ekfodom_msg.header.frame_id = "world";
    ekfodom_stamped.pose.position.x = state(0);
    ekfodom_stamped.pose.position.y = state(1);
    ekfodom_stamped.pose.position.z = state(2);
    // 填充姿态信息
    ekfodom_stamped.pose.orientation.w = state(3);
    ekfodom_stamped.pose.orientation.x = state(4);
    ekfodom_stamped.pose.orientation.y = state(5);
    ekfodom_stamped.pose.orientation.z = state(6);
    ekfodom_msg.poses.push_back(ekfodom_stamped);
    ekfodom_pub.publish(ekfodom_msg);
}

void RvizDisplay::publishekfodom_new(const State &state_)
{
        double time = state_.time;
    Eigen::Vector3d position = state_.p;
    Eigen::Vector3d velocity = state_.v;


    // publish the odometry
    std::string fixed_id = "world";
    nav_msgs::Odometry odom_msg;
    odom_msg.header.frame_id =fixed_id;
    odom_msg.header.stamp = ros::Time(time);
    Eigen::Isometry3d T_wb = Eigen::Isometry3d::Identity();
    T_wb.linear() = state_.R_q;
    T_wb.translation() = position;
    tf::poseEigenToMsg(T_wb, odom_msg.pose.pose);
    tf::vectorEigenToMsg(velocity, odom_msg.twist.twist.linear);
    Eigen::Matrix3d P_pp = state_.P.block<3, 3>(StateIndex::P, StateIndex::P); // position covariance
    Eigen::Matrix3d P_po = state_.P.block<3, 3>(StateIndex::P, StateIndex::R); // position rotation covariance
    Eigen::Matrix3d P_op = state_.P.block<3, 3>(StateIndex::R, StateIndex::P); // rotation position covariance
    Eigen::Matrix3d P_oo = state_.P.block<3, 3>(StateIndex::R, StateIndex::R); // rotation covariance
    Eigen::Matrix<double, 6, 6, Eigen::RowMajor> P_imu_pose = Eigen::Matrix<double, 6, 6>::Zero();
    P_imu_pose << P_pp, P_po, P_op, P_oo;
    for (int i = 0; i < 36; i++)
    odom_msg.pose.covariance[i] = P_imu_pose.data()[i];
    ekfodom_new_pub.publish(odom_msg);

    // std::cout<<state_.R_q<<std::endl;

    // save state p q lla
    // Eigen::Vector3d lla;
    // Utility::convert_enu_to_lla(init_lla_, state_.p, &lla);
    // file_our_lla << std::fixed << std::setprecision(15) << lla[1] << "," << lla[0] << "," << lla[2] << " ";

    Eigen::Quaterniond q_G_I(state_.R_q);
    q_G_I.normalize();
    // Eigen::AngleAxisd rotation_vector(rotation_matrix);
    // Eigen::Vector3d eulerAngle=state_.R_q.eulerAngles(2,1,0);
    // eulerAngle  = eulerAngle*180.0/M_PI;
    Eigen::Quaterniond q_G_I_imu(state_.R_imu);
    // Eigen::AngleAxisd imu_rotation_vector(state_.R_imu);
    // Eigen::Vector3d eulerAngle_imu=state_.R_imu.eulerAngles(2,1,0);
    // eulerAngle_imu  = eulerAngle_imu*180.0/M_PI;
    Eigen::VectorXd bias_cov = state_.P.diagonal().tail(10).transpose();
            std::ofstream file("/data/yhy/imu_wheel.txt", std::ios::out | std::ios::app);
    if (!file.is_open()) {
        // std::cerr << "Error: Unable to open file " << filename << " for writing." << std::endl;
        return;
    }
    // 遍历轨迹，写入 TUM 格式
    // double timestamp=gps_msg->header.stamp.toSec();
    Eigen::Quaterniond state_q(state_.R_q);
        file << std::fixed << std::setprecision(6)
             << state_.time << " "
             << state_.p[0] << " "              // ENU x
             <<state_.p[1]<< " "              // ENU y
             << state_.p[2]<< " "              // ENU z
             <<state_q.x() << " " 
             << state_q.y() << " " 
             << state_q.z() << " " 
             << state_q.w()
             << std::endl;
   
    // file_our_state_ << std::fixed << std::setprecision(15) << state_.time << ", "
    //                 << state_.p[0] << ", " << state_.p[1] << ", " << state_.p[2] << ", "
    //                 << q_G_I.x() << ", " << q_G_I.y() << ", " << q_G_I.z() << ", " << q_G_I.w() << ", "
    //                 << state_.v[0] << ", " << state_.v[1] << ", " << state_.v[2] << ", "
    //                 << state_.ba[0] << ", " << state_.ba[1] << ", " << state_.ba[2] << ", "
    //                 << state_.bg[0] << ", " << state_.bg[1] << ", " << state_.bg[2] << ", "
    //                 << bias_cov[0] << ", " << bias_cov[1] << ", " << bias_cov[2] << ", " << bias_cov[3] << ", " << bias_cov[4] << ", " << bias_cov[5] << ", " << bias_cov[6] << ", " << bias_cov[7] << ", " << bias_cov[8] << ", " << bias_cov[9] << ", "
    //                 << q_G_I_imu.x() << ", " << q_G_I_imu.y() << ", " << q_G_I_imu.z() << ", " << q_G_I_imu.w() << ", "
    //                 << state_.ws
    //                 << std::endl;

    // std::cout << "bias of acc: " << state_.ba.transpose() << std::endl;
    // std::cout << "bias of gyr: " << state_.bg.transpose() << std::endl;
    // std::cout << "ws " << state_.ws << std::endl;
    // std::cout << "R_imu " << state_.R_imu << std::endl;

    // pub_odometry_.publish(odometry);

    // Eigen::Quaterniond state_q(state_.R_q);
    // file_odom << std::fixed << std::setprecision(15) << state_.time << " "
    //               << state_.p[0] << " " << state_.p[1] << " " << state_.p[2] << " "
    //               << state_q.x() << " " << state_q.y() << " " << state_q.z() << " " << state_q.w()
    //               << std::endl;
    ekfodom_new_msg.header.stamp = ros::Time::now();
    ekfodom_new_msg.header.frame_id = "world";
    geometry_msgs::PoseStamped ekfodom_new_stamped;
    ekfodom_new_stamped.header.frame_id = "world";
    ekfodom_new_stamped.header.stamp = ros::Time(time);
    ekfodom_new_stamped.pose = odom_msg.pose.pose;
    ekfodom_new_msg.poses.push_back(ekfodom_new_stamped);
    ekfpath_new_pub.publish(ekfodom_new_msg);
}

void RvizDisplay::displayParkingSpots(const std::vector<AssociatedPair>& worldassociatedPairs) {
    visualization_msgs::MarkerArray markerArray;
    for (const auto& pair : worldassociatedPairs) {
        visualization_msgs::Marker spotMarker;
        spotMarker.header.frame_id = "world"; // 假设车位在"map"坐标系中
        spotMarker.header.stamp = ros::Time::now();
        spotMarker.ns = "parking_spot"+ std::to_string(pair.ID);;
        spotMarker.id = spotMarker_id++;
        spotMarker.type = visualization_msgs::Marker::LINE_STRIP;
        spotMarker.action = visualization_msgs::Marker::ADD;
        // spotMarker.pose.orientation.w = 1.0;

        // 设置车位坐标点
        // spotMarker.points.push_back(tf2::pointToMsg(tf2::Vector3(pair.spot.x1, pair.spot.y1, 0)));
        // spotMarker.points.push_back(tf2::pointToMsg(tf2::Vector3(pair.spot.x2, pair.spot.y2, 0)));
        // spotMarker.points.push_back(tf2::pointToMsg(tf2::Vector3(pair.spot.x3, pair.spot.y3, 0)));
        // spotMarker.points.push_back(tf2::pointToMsg(tf2::Vector3(pair.spot.x4, pair.spot.y4, 0)));
        // spotMarker.points.push_back(spotMarker.points[0]); // 闭合多边形
        geometry_msgs::Point point1;
        point1.x = pair.spot.x1;
        point1.y = pair.spot.y1;
        point1.z = 0; // 假设在二维平面上，z 坐标为 0

        geometry_msgs::Point point2;
        point2.x = pair.spot.x2;
        point2.y = pair.spot.y2;
        point2.z = 0;

        geometry_msgs::Point point3;
        point3.x = pair.spot.x3;
        point3.y = pair.spot.y3;
        point3.z = 0;

        geometry_msgs::Point point4;
        point4.x = pair.spot.x4;
        point4.y = pair.spot.y4;
        point4.z = 0;

        // 将点添加到 Marker 的 points 数组中，形成闭合的多边形
        spotMarker.points.push_back(point1);
        spotMarker.points.push_back(point2);
        spotMarker.points.push_back(point4);
        spotMarker.points.push_back(point3);
        spotMarker.points.push_back(point1); // 闭合多边形，返回第一个点

        // 设置颜色和大小
        if(pair.spot.vacant==1){
        spotMarker.color.r = 1.0f; // 红色
        spotMarker.color.g = 0.0f; // 绿色
        spotMarker.color.b = 0.0f; // 蓝色
        spotMarker.color.a = 1.0f; // 完全不透明
        spotMarker.scale.x = 0.1; // 线宽
        spotMarker.lifetime = ros::Duration(0);
        markerArray.markers.push_back(spotMarker);
        }
        else{

        spotMarker.color.r = 0.0f; // 红色
        spotMarker.color.g = 1.0f; // 绿色
        spotMarker.color.b = 0.0f; // 蓝色
        spotMarker.color.a = 1.0f; // 完全不透明
        spotMarker.scale.x = 0.1; // 线宽
        spotMarker.lifetime = ros::Duration(0);
        markerArray.markers.push_back(spotMarker);
        }

        markerArray.markers.push_back(spotMarker);

    if(!pair.ocrPoint.text.empty()){
        // std::cout<<"enter"<<std::endl;
        // 为车位号创建Marker
        visualization_msgs::Marker ocrMarker;
        ocrMarker.header.frame_id= "world";
        ocrMarker.ns = "text_boxes";
        ocrMarker.id = spotMarker_id++;
        ocrMarker.type = visualization_msgs::Marker::LINE_STRIP;
        ocrMarker.action = visualization_msgs::Marker::ADD;
        // ocrMarker.points.push_back(tf::pointToMsg(tf::Vector3(pair.ocrPoint.x1, pair.ocrPoint.y1, 0)));
        // ocrMarker.points.push_back(tf::pointToMsg(tf::Vector3(pair.ocrPoint.x2, pair.ocrPoint.y1, 0)));
        // ocrMarker.points.push_back(tf::pointToMsg(tf::Vector3(pair.ocrPoint.x2, pair.ocrPoint.y2, 0)));
        // ocrMarker.points.push_back(tf::pointToMsg(tf::Vector3(pair.ocrPoint.x1, pair.ocrPoint.y2, 0)));
        // ocrMarker.points.push_back(spotMarker.points[0]); // 闭合多边形
        // 为车位号边框设置点坐标
        geometry_msgs::Point point11;
        point11.x = pair.ocrPoint.x1;
        point11.y = pair.ocrPoint.y1;
        point11.z = 0; // 如果是二维情况，z坐标通常设置为0

        geometry_msgs::Point point12;
        point12.x = pair.ocrPoint.x2;
        point12.y = pair.ocrPoint.y1;
        point12.z = 0;

        geometry_msgs::Point point13;
        point13.x = pair.ocrPoint.x2;
        point13.y = pair.ocrPoint.y2;
        point13.z = 0;

        geometry_msgs::Point point14;
        point14.x = pair.ocrPoint.x1;
        point14.y = pair.ocrPoint.y2;
        point14.z = 0;

        // 添加点以闭合多边形（返回第一个点）
        ocrMarker.points.push_back(point11);
        ocrMarker.points.push_back(point12);
        ocrMarker.points.push_back(point13);
        ocrMarker.points.push_back(point14);
        ocrMarker.points.push_back(point11); // 闭合边框
               // 设置颜色和大小
        ocrMarker.color.r = 1.0f;
        ocrMarker.color.g = 0.0f;
        ocrMarker.color.b = 0.0f;
        ocrMarker.color.a = 1.0f;
        ocrMarker.scale.x = 0.1;
        ocrMarker.lifetime = ros::Duration(0);
        markerArray.markers.push_back(ocrMarker);
      
       
        // 将车位号作为文本显示
        visualization_msgs::Marker textMarker;
        double centerX = (pair.ocrPoint.x1 + pair.ocrPoint.x2) / 2.0;
        double centerY = (pair.ocrPoint.y1 + pair.ocrPoint.y2) / 2.0;
        geometry_msgs::Point pose_center;
        pose_center.x = centerX;
        pose_center.y = centerY;
        textMarker.header.frame_id = "world"; ;
        textMarker.ns = "text";
        textMarker.id = spotMarker_id++;
        textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        textMarker.action = visualization_msgs::Marker::ADD;
        textMarker.pose.position = pose_center;
        textMarker.text = pair.ocrPoint.text;
        textMarker.scale.z = 1.0; // 文本大小
                // 设置文本颜色和大小
        textMarker.color.r = 1.0f;
        textMarker.color.g = 1.0f;
        textMarker.color.b = 1.0f;
        textMarker.color.a = 1.0f;
        textMarker.scale.z = 1.0; // 文本高度
        textMarker.lifetime = ros::Duration(0);
        markerArray.markers.push_back(textMarker);
    }

        
    }

    slots_pub.publish(markerArray);
        // // 设置文本位置为车位中心点
        
        // pose_center.z = 0.5; // 稍微抬高z值，确保文本显示在地面上方
        // textMarker.pose.position = tf::pointToMsg(pose_center);
        // textMarker.pose.orientation = tf::createQuaternionMsgFromYaw(0); // 确保文本正面朝向相机


}

void RvizDisplay::displaymap(const AssociatedPair& pair) {
        visualization_msgs::Marker spotMarker;
        spotMarker.header.frame_id = "world"; // 假设车位在"map"坐标系中
        spotMarker.header.stamp = ros::Time::now();
        spotMarker.ns = "parking_spot"+ std::to_string(pair.ID);;
        spotMarker.id = spotMarker_id++;
        spotMarker.type = visualization_msgs::Marker::LINE_STRIP;
        spotMarker.action = visualization_msgs::Marker::ADD;
        // spotMarker.pose.orientation.w = 1.0;

        // 设置车位坐标点
        // spotMarker.points.push_back(tf2::pointToMsg(tf2::Vector3(pair.spot.x1, pair.spot.y1, 0)));
        // spotMarker.points.push_back(tf2::pointToMsg(tf2::Vector3(pair.spot.x2, pair.spot.y2, 0)));
        // spotMarker.points.push_back(tf2::pointToMsg(tf2::Vector3(pair.spot.x3, pair.spot.y3, 0)));
        // spotMarker.points.push_back(tf2::pointToMsg(tf2::Vector3(pair.spot.x4, pair.spot.y4, 0)));
        // spotMarker.points.push_back(spotMarker.points[0]); // 闭合多边形
        geometry_msgs::Point point1;
        point1.x = pair.spot.x1;
        point1.y = pair.spot.y1;
        point1.z = 0; // 假设在二维平面上，z 坐标为 0

        geometry_msgs::Point point2;
        point2.x = pair.spot.x2;
        point2.y = pair.spot.y2;
        point2.z = 0;

        geometry_msgs::Point point3;
        point3.x = pair.spot.x3;
        point3.y = pair.spot.y3;
        point3.z = 0;

        geometry_msgs::Point point4;
        point4.x = pair.spot.x4;
        point4.y = pair.spot.y4;
        point4.z = 0;

        // 将点添加到 Marker 的 points 数组中，形成闭合的多边形
        spotMarker.points.push_back(point1);
        spotMarker.points.push_back(point2);
        spotMarker.points.push_back(point4);
        spotMarker.points.push_back(point3);
        spotMarker.points.push_back(point1); // 闭合多边形，返回第一个点

        // 设置颜色和大小
        if(pair.spot.vacant==1){
        spotMarker.color.r = 1.0f; // 红色
        spotMarker.color.g = 0.0f; // 绿色
        spotMarker.color.b = 0.0f; // 蓝色
        spotMarker.color.a = 1.0f; // 完全不透明
        spotMarker.scale.x = 0.1; // 线宽
        spotMarker.lifetime = ros::Duration(0);
        map_pub.publish(spotMarker);
        }
        else{

        spotMarker.color.r = 0.0f; // 红色
        spotMarker.color.g = 1.0f; // 绿色
        spotMarker.color.b = 0.0f; // 蓝色
        spotMarker.color.a = 1.0f; // 完全不透明
        spotMarker.scale.x = 0.1; // 线宽
        spotMarker.lifetime = ros::Duration(0);
        map_pub.publish(spotMarker);

        }
        // markers.push_back(spotMarker);

    if(!pair.ocrPoint.text.empty()){
        // std::cout<<"enter"<<std::endl;
        // 为车位号创建Marker
        visualization_msgs::Marker ocrMarker;
        ocrMarker.header.frame_id= "world";
        ocrMarker.ns = "text_boxes";
        ocrMarker.id = spotMarker_id++;
        ocrMarker.type = visualization_msgs::Marker::LINE_STRIP;
        ocrMarker.action = visualization_msgs::Marker::ADD;
        // ocrMarker.points.push_back(tf::pointToMsg(tf::Vector3(pair.ocrPoint.x1, pair.ocrPoint.y1, 0)));
        // ocrMarker.points.push_back(tf::pointToMsg(tf::Vector3(pair.ocrPoint.x2, pair.ocrPoint.y1, 0)));
        // ocrMarker.points.push_back(tf::pointToMsg(tf::Vector3(pair.ocrPoint.x2, pair.ocrPoint.y2, 0)));
        // ocrMarker.points.push_back(tf::pointToMsg(tf::Vector3(pair.ocrPoint.x1, pair.ocrPoint.y2, 0)));
        // ocrMarker.points.push_back(spotMarker.points[0]); // 闭合多边形
        // 为车位号边框设置点坐标
        geometry_msgs::Point point11;
        point11.x = pair.ocrPoint.x1;
        point11.y = pair.ocrPoint.y1;
        point11.z = 0; // 如果是二维情况，z坐标通常设置为0

        geometry_msgs::Point point12;
        point12.x = pair.ocrPoint.x2;
        point12.y = pair.ocrPoint.y1;
        point12.z = 0;

        geometry_msgs::Point point13;
        point13.x = pair.ocrPoint.x2;
        point13.y = pair.ocrPoint.y2;
        point13.z = 0;

        geometry_msgs::Point point14;
        point14.x = pair.ocrPoint.x1;
        point14.y = pair.ocrPoint.y2;
        point14.z = 0;

        // 添加点以闭合多边形（返回第一个点）
        ocrMarker.points.push_back(point11);
        ocrMarker.points.push_back(point12);
        ocrMarker.points.push_back(point13);
        ocrMarker.points.push_back(point14);
        ocrMarker.points.push_back(point11); // 闭合边框
               // 设置颜色和大小
        ocrMarker.color.r = 1.0f;
        ocrMarker.color.g = 0.0f;
        ocrMarker.color.b = 0.0f;
        ocrMarker.color.a = 1.0f;
        ocrMarker.scale.x = 0.1;
        ocrMarker.lifetime = ros::Duration(0);
        map_pub.publish(ocrMarker);
        // markerArray.markers.push_back(ocrMarker);
      
       
        // 将车位号作为文本显示
        visualization_msgs::Marker textMarker;
        double centerX = (pair.ocrPoint.x1 + pair.ocrPoint.x2) / 2.0;
        double centerY = (pair.ocrPoint.y1 + pair.ocrPoint.y2) / 2.0;
        geometry_msgs::Point pose_center;
        pose_center.x = centerX;
        pose_center.y = centerY;
        textMarker.header.frame_id = "world"; ;
        textMarker.ns = "text";
        textMarker.id = spotMarker_id++;
        textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        textMarker.action = visualization_msgs::Marker::ADD;
        textMarker.pose.position = pose_center;
        textMarker.text = pair.ocrPoint.text;
        textMarker.scale.z = 1.0; // 文本大小
                // 设置文本颜色和大小
        textMarker.color.r = 1.0f;
        textMarker.color.g = 1.0f;
        textMarker.color.b = 1.0f;
        textMarker.color.a = 1.0f;
        textMarker.scale.z = 1.0; // 文本高度
        textMarker.lifetime = ros::Duration(0);
        map_pub.publish(textMarker);
        // markerArray.markers.push_back(textMarker);
    }

    // slots_pub.publish(markerArray);
        // // 设置文本位置为车位中心点
        
        // pose_center.z = 0.5; // 稍微抬高z值，确保文本显示在地面上方
        // textMarker.pose.position = tf::pointToMsg(pose_center);
        // textMarker.pose.orientation = tf::createQuaternionMsgFromYaw(0); // 确保文本正面朝向相机


}
    
void RvizDisplay::visualizeMapPoints(const std::vector<AssociatedPair>& mapPoints) 
{
 visualization_msgs::MarkerArray markerArray;
    for (const auto& pair : mapPoints) {
        std::cout<<pair.ocrPoint.text<<":"<<pair.ocrPoint.x1<<","<<pair.spot.x1<<std::endl;
        visualization_msgs::Marker spotMarker;
        spotMarker.header.frame_id = "world"; // 假设车位在"map"坐标系中
        spotMarker.header.stamp = ros::Time::now();
        spotMarker.ns = "map"+ std::to_string(pair.ID);
        spotMarker.id =spotMarker_id++ ;
        spotMarker.type = visualization_msgs::Marker::LINE_STRIP;
        spotMarker.action = visualization_msgs::Marker::ADD;
        // spotMarker.pose.orientation.w = 1.0;

        // 设置车位坐标点
        // spotMarker.points.push_back(tf2::pointToMsg(tf2::Vector3(pair.spot.x1, pair.spot.y1, 0)));
        // spotMarker.points.push_back(tf2::pointToMsg(tf2::Vector3(pair.spot.x2, pair.spot.y2, 0)));
        // spotMarker.points.push_back(tf2::pointToMsg(tf2::Vector3(pair.spot.x3, pair.spot.y3, 0)));
        // spotMarker.points.push_back(tf2::pointToMsg(tf2::Vector3(pair.spot.x4, pair.spot.y4, 0)));
        // spotMarker.points.push_back(spotMarker.points[0]); // 闭合多边形
        geometry_msgs::Point point1;
        point1.x = pair.spot.x1;
        point1.y = pair.spot.y1;
        point1.z = 0; // 假设在二维平面上，z 坐标为 0

        geometry_msgs::Point point2;
        point2.x = pair.spot.x2;
        point2.y = pair.spot.y2;
        point2.z = 0;

        geometry_msgs::Point point3;
        point3.x = pair.spot.x3;
        point3.y = pair.spot.y3;
        point3.z = 0;

        geometry_msgs::Point point4;
        point4.x = pair.spot.x4;
        point4.y = pair.spot.y4;
        point4.z = 0;

        // 将点添加到 Marker 的 points 数组中，形成闭合的多边形
        spotMarker.points.push_back(point1);
        spotMarker.points.push_back(point2);
        spotMarker.points.push_back(point4);
        spotMarker.points.push_back(point3);
        spotMarker.points.push_back(point1); // 闭合多边形，返回第一个点

        // 设置颜色和大小
        if(pair.spot.vacant==1){
        spotMarker.color.r = 1.0f; // 红色
        spotMarker.color.g = 0.0f; // 绿色
        spotMarker.color.b = 0.0f; // 蓝色
        spotMarker.color.a = 1.0f; // 完全不透明
        spotMarker.scale.x = 0.1; // 线宽
        spotMarker.lifetime = ros::Duration(0);
        markerArray.markers.push_back(spotMarker);
        }
        else{

        spotMarker.color.r = 0.0f; // 红色
        spotMarker.color.g = 1.0f; // 绿色
        spotMarker.color.b = 0.0f; // 蓝色
        spotMarker.color.a = 1.0f; // 完全不透明
        spotMarker.scale.x = 0.1; // 线宽
        spotMarker.lifetime = ros::Duration(0);
        markerArray.markers.push_back(spotMarker);
        }

        markerArray.markers.push_back(spotMarker);

    if(!pair.ocrPoint.text.empty()){
        // std::cout<<"enter"<<std::endl;
        // 为车位号创建Marker
        visualization_msgs::Marker ocrMarker;
        ocrMarker.header.frame_id= "world";
        ocrMarker.ns = "text_boxes";
        ocrMarker.id = spotMarker_id++;
        ocrMarker.type = visualization_msgs::Marker::LINE_STRIP;
        ocrMarker.action = visualization_msgs::Marker::ADD;
        // ocrMarker.points.push_back(tf::pointToMsg(tf::Vector3(pair.ocrPoint.x1, pair.ocrPoint.y1, 0)));
        // ocrMarker.points.push_back(tf::pointToMsg(tf::Vector3(pair.ocrPoint.x2, pair.ocrPoint.y1, 0)));
        // ocrMarker.points.push_back(tf::pointToMsg(tf::Vector3(pair.ocrPoint.x2, pair.ocrPoint.y2, 0)));
        // ocrMarker.points.push_back(tf::pointToMsg(tf::Vector3(pair.ocrPoint.x1, pair.ocrPoint.y2, 0)));
        // ocrMarker.points.push_back(spotMarker.points[0]); // 闭合多边形
        // 为车位号边框设置点坐标
        geometry_msgs::Point point11;
        point11.x = pair.ocrPoint.x1;
        point11.y = pair.ocrPoint.y1;
        point11.z = 0; // 如果是二维情况，z坐标通常设置为0

        geometry_msgs::Point point12;
        point12.x = pair.ocrPoint.x2;
        point12.y = pair.ocrPoint.y1;
        point12.z = 0;

        geometry_msgs::Point point13;
        point13.x = pair.ocrPoint.x2;
        point13.y = pair.ocrPoint.y2;
        point13.z = 0;

        geometry_msgs::Point point14;
        point14.x = pair.ocrPoint.x1;
        point14.y = pair.ocrPoint.y2;
        point14.z = 0;

        // 添加点以闭合多边形（返回第一个点）
        ocrMarker.points.push_back(point11);
        ocrMarker.points.push_back(point12);
        ocrMarker.points.push_back(point13);
        ocrMarker.points.push_back(point14);
        ocrMarker.points.push_back(point11); // 闭合边框
               // 设置颜色和大小
        ocrMarker.color.r = 1.0f;
        ocrMarker.color.g = 0.0f;
        ocrMarker.color.b = 0.0f;
        ocrMarker.color.a = 1.0f;
        ocrMarker.scale.x = 0.1;
        ocrMarker.lifetime = ros::Duration(0);
        markerArray.markers.push_back(ocrMarker);
      
       
        // 将车位号作为文本显示
        visualization_msgs::Marker textMarker;
        double centerX = (pair.ocrPoint.x1 + pair.ocrPoint.x2) / 2.0;
        double centerY = (pair.ocrPoint.y1 + pair.ocrPoint.y2) / 2.0;
        geometry_msgs::Point pose_center;
        pose_center.x = centerX;
        pose_center.y = centerY;
        textMarker.header.frame_id = "world"; ;
        textMarker.ns = "text";
        textMarker.id = spotMarker_id++;
        textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        textMarker.action = visualization_msgs::Marker::ADD;
        textMarker.pose.position = pose_center;
        textMarker.text = pair.ocrPoint.text;
        textMarker.scale.z = 1.0; // 文本大小
                // 设置文本颜色和大小
        textMarker.color.r = 1.0f;
        textMarker.color.g = 1.0f;
        textMarker.color.b = 1.0f;
        textMarker.color.a = 1.0f;
        textMarker.scale.z = 1.0; // 文本高度
        textMarker.lifetime = ros::Duration(0);
        markerArray.markers.push_back(textMarker);

        // std::cout << textMarker << std::endl;
    }

        
    }

    // ros::Rate rate(1);  // 1 Hz

    // while (ros::ok()) {
    //     // markerArray.header.stamp = ros::Time::now();  // 更新时间戳
    //     localizationmap.publish(markerArray);  // 发布标记

    //     rate.sleep();  // 控制循环频率
    //     localizationmap.publish(markerArray);
    // }


        // // 设置文本位置为车位中心点
}


void RvizDisplay::matchdisplay(const std::vector<Eigen::Vector3d>  matched_points) {

              // 创建点Marker
            std::cout<<"enter"<<std::endl;
            visualization_msgs::Marker points_marker;
            points_marker.header.frame_id = "world";
            points_marker.header.stamp = ros::Time::now();
            points_marker.ns = "matched_points";
            points_marker.id = 0;
            points_marker.type = visualization_msgs::Marker::POINTS;
            points_marker.action = visualization_msgs::Marker::ADD;
            points_marker.scale.x = 1;  // 点的大小
            points_marker.scale.y = 1;
            points_marker.color.r = 0.0f;
            points_marker.color.g = 0.0f;
            points_marker.color.b = 1.0f;
            points_marker.color.a = 1.0;

            // 添加匹配点到Marker
            for (const auto& point : matched_points) {
                geometry_msgs::Point p;
                p.x = point[0];
                p.y = point[1];
                p.z = point[2];
                points_marker.points.push_back(p);
            }
            points_marker.lifetime = ros::Duration(0);
            // 发布点Marker
            match_pub.publish(points_marker);

            // 模拟实时更新
      
       
  
        // match_pub.publish( points_marker);
   
}


void RvizDisplay::displayPlate(const std::vector<LicensePlate>& plates) {
    visualization_msgs::MarkerArray markerArray;
    for (const auto& pair : plates) {
        visualization_msgs::Marker ocrMarker;
        ocrMarker.header.frame_id= "world";
        ocrMarker.ns = "plates";
        ocrMarker.id = spotMarker_id++;
        ocrMarker.type = visualization_msgs::Marker::LINE_STRIP;
        ocrMarker.action = visualization_msgs::Marker::ADD;
        // ocrMarker.points.push_back(tf::pointToMsg(tf::Vector3(pair.ocrPoint.x1, pair.ocrPoint.y1, 0)));
        // ocrMarker.points.push_back(tf::pointToMsg(tf::Vector3(pair.ocrPoint.x2, pair.ocrPoint.y1, 0)));
        // ocrMarker.points.push_back(tf::pointToMsg(tf::Vector3(pair.ocrPoint.x2, pair.ocrPoint.y2, 0)));
        // ocrMarker.points.push_back(tf::pointToMsg(tf::Vector3(pair.ocrPoint.x1, pair.ocrPoint.y2, 0)));
        // ocrMarker.points.push_back(spotMarker.points[0]); // 闭合多边形
        // 为车位号边框设置点坐标
        geometry_msgs::Point point11;
        point11.x = pair.points[0].x();  // Eigen::Vector3d 的 x() 方法获取 x 值
        point11.y = pair.points[0].y();  // 获取 y 值
        point11.z = pair.points[0].z();  // 获取 z 值
     

        geometry_msgs::Point point12;
        point12.x = pair.points[1].x();  // Eigen::Vector3d 的 x() 方法获取 x 值
        point12.y = pair.points[0].y();  // 获取 y 值
        point12.z = pair.points[1].z();  // 获取 z 值

        geometry_msgs::Point point13;
             point13.x = pair.points[1].x();  // Eigen::Vector3d 的 x() 方法获取 x 值
        point13.y = pair.points[1].y();  // 获取 y 值
        point13.z = pair.points[0].z();  // 获取 z 值

        geometry_msgs::Point point14;
             point14.x = pair.points[0].x();  // Eigen::Vector3d 的 x() 方法获取 x 值
        point14.y = pair.points[1].y();  // 获取 y 值
        point14.z = pair.points[0].z();  // 获取 z 值

        // 添加点以闭合多边形（返回第一个点）
        ocrMarker.points.push_back(point11);
        ocrMarker.points.push_back(point12);
        ocrMarker.points.push_back(point13);
        ocrMarker.points.push_back(point14);
        ocrMarker.points.push_back(point11); // 闭合边框
               // 设置颜色和大小
        ocrMarker.color.r = 0.0f;
        ocrMarker.color.g = 0.0f;
        ocrMarker.color.b = 1.0f;
        ocrMarker.color.a = 1.0f;
        ocrMarker.scale.x = 0.1;
        ocrMarker.lifetime = ros::Duration(0);
        markerArray.markers.push_back(ocrMarker);
      
       
        // 将车位号作为文本显示
        visualization_msgs::Marker textMarker;
        double centerX = (point11.x + point13.x) / 2.0;
        double centerY = (point11.y + point13.y) / 2.0;
        geometry_msgs::Point pose_center;
        pose_center.x = centerX;
        pose_center.y = centerY;
        textMarker.header.frame_id = "world"; ;
        textMarker.ns = "platenumber";
        textMarker.id = spotMarker_id++;
        textMarker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        textMarker.action = visualization_msgs::Marker::ADD;
        textMarker.pose.position = pose_center;
        textMarker.text = pair.plateNumber;
        textMarker.scale.z = 1.0; // 文本大小
                // 设置文本颜色和大小
        textMarker.color.r = 1.0f;
        textMarker.color.g = 1.0f;
        textMarker.color.b = 1.0f;
        textMarker.color.a = 1.0f;
        textMarker.scale.z = 1.0; // 文本高度
        textMarker.lifetime = ros::Duration(0);
        markerArray.markers.push_back(textMarker);
    }

        
    

    plates_pub.publish(markerArray);



}