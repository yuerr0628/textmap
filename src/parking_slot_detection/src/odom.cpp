#include "odom.h"
#include"drawmap.h"
#include "rvizshow.h"

RvizDisplay odomdisplay;
cv::Mat canvasodom(1000, 1000, CV_8UC3, cv::Scalar(255, 255, 255));
// cv::String windowName1 = "slots";
std::vector<Eigen::Vector3d> trajectory; // 存储相机轨迹
    // 假设手动指定初始位置
Eigen::Matrix4d initial_pose = Eigen::Matrix4d::Identity();

    // 当前帧的位姿
Eigen::Matrix4d current_pose = initial_pose;
Odometry::Odometry(ros::NodeHandle& nh) {
    imu_sub_odom = nh.subscribe("/Inertial/imu/data", 10, &Odometry::imuCallback_odom, this);
    wheel_speed_sub = nh.subscribe("/Inertial/gps/odom", 1000, &Odometry::wheelSpeedCallback, this);
    image_sub = nh.subscribe("/driver/fisheye/avm/compressed", 10, &Odometry::imageCallback_odo,this);
    
    client = nh.serviceClient<parking_slot_detection::gcn_parking>("gcn_service");
    odomdisplay.RvizDisplay_init(nh);
     state_ = Eigen::VectorXd::Zero(10);
    state_(3) = 1.0;  // 初始四元数w=1，表示无旋转
    P_ = Eigen::MatrixXd::Identity(10, 10);
    Q_ = Eigen::MatrixXd::Identity(10, 10) * 0.01;  // 过程噪声
    R_ = Eigen::MatrixXd::Identity(6, 6) * 0.1;     // 观测噪声
    F_ = Eigen::MatrixXd::Identity(10, 10);         // 状态转移矩阵
    H_ = Eigen::MatrixXd::Zero(6, 10);              // 观测矩阵
    bool isFirstFrame=true; 
}


void Odometry::drawline(const Eigen::Matrix4d&relative_pose)
{
     
                // 更新当前帧的全局位姿
    current_pose = current_pose * relative_pose;
    // 提取当前帧的平移向量
    Eigen::Vector3d position = current_pose.block<3, 1>(0, 3);
    // 将当前帧的位置添加到轨迹中
    trajectory.push_back(position);
    odomdisplay.publishodomPath(position);
}

void Odometry::ekfodometry(const VectorXd &state)
{
    std::cout<<"display"<<std::endl;
     odomdisplay.publishekfodom(state);
}

void Odometry::computeStateTransitionJacobian(const Eigen::VectorXd &u, double dt)
{
    // 初始化状态转移矩阵 F 为单位矩阵
    // F_.setIdentity();

    // 位置对速度的偏导
    F_(0, 7) = dt;  // x 对 v_x 的偏导
    F_(1, 8) = dt;  // y 对 v_y 的偏导
    F_(2, 9) = dt;  // z 对 v_z 的偏导

    // 姿态四元数对角速度的偏导：这里假设 dt 很小，可以近似处理为线性
    Eigen::Quaterniond q(state_(3), state_(4), state_(5), state_(6));  // 当前姿态四元数
    Eigen::Matrix3d R = q.toRotationMatrix();  // 将四元数转换为旋转矩阵

    // 角速度的反对称矩阵 (skew-symmetric matrix)
    Eigen::Matrix3d W;
    W <<  0,        -u(5),   u(4),
         u(5),   0,        -u(3),
        -u(4),   u(3),    0;

    // 更新四元数对角速度的偏导 (近似为 q 和 W 的关系)
    F_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() - 0.5 * dt * W;
}

    // 预测步：根据IMU数据预测位姿
void Odometry::predict(const Eigen::VectorXd &u, double dt) {
        // MatrixXd F = computeStateTransitionJacobian(accel, gyro, dt); // 计算雅可比矩阵
        // // 预测状态
        // state_ = imuOdometryUpdate(accel, gyro, dt, state_);
        // // 更新协方差矩阵
        // P_ = F * P_ * F.transpose() + Q_;
            // 位置和速度的更新
                // 更新状态转移矩阵 F
    computeStateTransitionJacobian(u, dt);
    state_.segment<3>(0) = state_.segment<3>(0) + state_.segment<3>(7) * dt; // 位置 = 位置 + 速度 * dt
    
    // 使用IMU角速度更新姿态（四元数）
    Eigen::Quaterniond q(state_(3), state_(4), state_(5), state_(6)); // 当前四元数
    Eigen::Quaterniond dq;
    dq.w() = 0;
    dq.vec() = u.segment<3>(3); // 角速度向量

    // Eigen::Quaterniond dq_dt = 0.5 * q * dq;
    Eigen::Quaterniond dq_dt = q * dq;  // 计算四元数乘积
    dq_dt.coeffs() *= 0.5;              // 四元数系数整体乘以 0.5
    q.w() += dq_dt.w() * dt;
    q.x() += dq_dt.x() * dt;
    q.y() += dq_dt.y() * dt;
    q.z() += dq_dt.z() * dt;
    q.normalize();

    state_(3) = q.w();
    state_(4) = q.x();
    state_(5) = q.y();
    state_(6) = q.z();

    // 更新速度
    // state_.segment<3>(7) = state_.segment<3>(7) + u.segment<3>(0) * dt; // 速度 = 速度 + 加速度 * dt

    // 更新协方差矩阵
    P_ = F_ * P_ * F_.transpose() + Q_;

    }

    // 更新步：根据ICP结果更新位姿
    void Odometry::update(const Matrix4d& icp_transformation) {
        if (icp_transformation.isIdentity()) {
            ekfodometry(state_);
            return; // 如果ICP未成功，跳过更新
        }
     // 更新观测矩阵 H（观测为 [x, y, z, q_x, q_y, q_z]）
    H_.setZero();
    H_(0, 0) = 1; // x
    H_(1, 1) = 1; // y
    H_(2, 2) = 1; // z
    H_(3, 3) = 1; // q_x
    H_(4, 4) = 1; // q_y
    H_(5, 5) = 1; // q_z
        // 提取ICP结果的旋转和平移
        Matrix3d R_icp = icp_transformation.block<3, 3>(0, 0);
        Vector3d t_icp = icp_transformation.block<3, 1>(0, 3);
        
        // 观测向量（旋转和位移）
        VectorXd z(6);
        z.head<3>() = R_icp.eulerAngles(0, 1, 2);
        z.tail<3>() = t_icp;
        // 计算残差
        VectorXd y = z - H_ * state_;

        // 计算残差协方差
        MatrixXd S = H_ * P_ * H_.transpose() + R_;

        // 计算卡尔曼增益
        MatrixXd K = P_ * H_.transpose() * S.inverse();

        // 更新状态向量
        state_ = state_ + K * y;
         // 更新协方差矩阵
        //    std::cout<<"update2"<<std::endl;
        ekfodometry(state_);
        P_ = (MatrixXd::Identity(10, 10) - K * H_) * P_;
       
    }

// 估计相机位姿
Eigen::Matrix4d Odometry::estimatePoseICP(const std::vector<Eigen::Vector3d>& prevPoints,
                                                const std::vector<Eigen::Vector3d>& currPoints) {
    // std::cout<<"enterpicp"<<std::endl;
    if (prevPoints.size() != currPoints.size() || prevPoints.size() < 3) {
        std::cerr << "Error: Point cloud size mismatch or insufficient points." << std::endl;
        return Eigen::Matrix4d::Identity();
    }

    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    computeTransformation(prevPoints, currPoints, R, t);

    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    pose.block<3, 3>(0, 0) = vpose.R;
    pose.block<3, 1>(0, 3) = vpose.t;
    // vpose.R=R;
    // vpose.t=t;
    drawline(pose);
    update(pose);
    // ekfodometry(state_);
    
    return pose;
}

// 计算质心
Eigen::Vector3d Odometry::computeCentroid(const std::vector<Eigen::Vector3d>& points) {
    Eigen::Vector3d centroid(0, 0, 0);
    for (const auto& point : points) {
        centroid += point;
    }
    centroid /= points.size();
    return centroid;
}

// 计算 ICP 的旋转和平移矩阵
void Odometry::computeTransformation(const std::vector<Eigen::Vector3d>& prevPoints,
                                           const std::vector<Eigen::Vector3d>& currPoints,
                                           Eigen::Matrix3d& R, Eigen::Vector3d& t) {
    Eigen::Vector3d prevCentroid = computeCentroid(prevPoints);
    Eigen::Vector3d currCentroid = computeCentroid(currPoints);

    std::vector<Eigen::Vector3d> prevNormalized, currNormalized;
    for (size_t i = 0; i < prevPoints.size(); ++i) {
        prevNormalized.push_back(prevPoints[i] - prevCentroid);
        currNormalized.push_back(currPoints[i] - currCentroid);
    }

    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < prevNormalized.size(); ++i) {
        H += prevNormalized[i] * currNormalized[i].transpose();
    }

    Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();
    R = V * U.transpose();

    if (R.determinant() < 0) {
        V.col(2) *= -1;
        R = V * U.transpose();
    }

    t = currCentroid - R * prevCentroid;
    vpose.R=R;
    vpose.t=t;

}

// void Odometry::matchpoint(const std::vector<cv::Point2f> & currentFrameSpots,const std::vector<cv::Point2f> & currentFrameSpots_ocr)
// {
    
// }

// Function to match the same parking spot between two frames
void Odometry::matchframes(const std::vector<cv::Point2f> & currentFrameSpots,const std::vector<cv::Point2f> & currentFrameSpots_ocr)
 {
    std::cout<<"entermatch"<<std::endl;
    std::vector<cv::Point2f> cFrameSpots;
    std::vector<cv::Point2f> preFrameSpots;
      std::vector<cv::Point2f> cFrameSpots_ocr;
    std::vector<cv::Point2f> preFrameSpots_ocr;
    std::vector<Eigen::Vector3d> prevehiclePoints;
    std::vector<Eigen::Vector3d> curvehiclePoints;
    int m=preFramespot.size()+preFramespot_ocr.size();
    int n=currentFrameSpots.size()+currentFrameSpots_ocr.size();
        if (isFirstFrame|| m==0||n==0) {
        preFramespot = currentFrameSpots;
        preFramespot_ocr = currentFrameSpots_ocr;
        isFirstFrame = false;
        // std::cout<<"intial"<<std::endl;
        return; // 第一帧没有足够的信息进行里程计计算
    }
   
     std::vector<std::vector<double>> cost_matrix(preFramespot.size(), std::vector<double>(currentFrameSpots.size(), 10000.0));
                    // Check if the positions are close enough
    float distanceThreshold = 0.6; // Define an appropriate threshold
    for (size_t i = 0; i < preFramespot.size(); ++i) {
        for (size_t j = 0; j < currentFrameSpots.size(); ++j) {
            // std::cout<<"5"<<std::endl;
            double distance = cv::norm(preFramespot[i] - currentFrameSpots[j]);
            if (distance <= distanceThreshold) {
                std::cout<<"pre:"<<preFramespot[i].x<<","<<preFramespot[i].y<<std::endl;
                std::cout<<"cur:"<<currentFrameSpots[j].x<<","<<currentFrameSpots[j].y<<std::endl;
            cost_matrix[i][j] = distance; } 
            else {
            //  std::cout<<"enterif2"<<std::endl;
            cost_matrix[i][j] = std::numeric_limits<double>::max(); // 使用一个大的值表示不可能的匹配
            // std::cout<<"7"<<std::endl;
            //  std::cout<<"entermax"<<std::endl;
            }
        }
        //  std::cout<<"out"<<std::endl;
    }
    //    std::cout<<"1"<<std::endl;
    HungarianAlgorithm hungarian;
    std::vector<int> Assignment;
    double cost = hungarian.Solve(cost_matrix, Assignment);
    for(int i = 0; i < Assignment.size(); ++i) {
        int match_index = Assignment[i];
        if(Assignment[i] != -1&&cost_matrix[i][Assignment[i]] < 0.6) 
        {
            cFrameSpots.push_back(currentFrameSpots[match_index]);
            preFrameSpots.push_back(preFramespot[i]);
            //   std::cout<<"2"<<std::endl;
        }

     }


    // ocr点匹配
     std::vector<std::vector<double>> cost_matrix_ocr(preFramespot_ocr.size(), std::vector<double>(currentFrameSpots_ocr.size(), 10000.0));
    //  std::cout<<preFramespot_ocr.size()<<std::endl;
                    // Check if the positions are close enough
    float distanceThreshold_ocr = 0.6; // Define an appropriate threshold
    for (size_t i = 0; i < preFramespot_ocr.size(); ++i) {
        for (size_t j = 0; j < currentFrameSpots_ocr.size(); ++j) {
            // std::cout<<"5"<<std::endl;
            double distance_ocr = cv::norm(preFramespot_ocr[i] - currentFrameSpots_ocr[j]);
            if (distance_ocr <= distanceThreshold_ocr) {
                // std::cout<<"pre:"<<preFramespot_ocr[i].x<<","<<preFramespot_ocr[i].y<<std::endl;
                // std::cout<<"cur:"<<currentFrameSpots_ocr[j].x<<","<<currentFrameSpots_ocr[j].y<<std::endl;
            cost_matrix_ocr[i][j] = distance_ocr; } 
            else {
            //  std::cout<<"enterif2"<<std::endl;
            cost_matrix_ocr[i][j] = std::numeric_limits<double>::max(); // 使用一个大的值表示不可能的匹配
            // std::cout<<"7"<<std::endl;
            //  std::cout<<"entermax"<<std::endl;
            }
        }
        //  std::cout<<"out"<<std::endl;
    }
    //    std::cout<<"1"<<std::endl;
    HungarianAlgorithm hungarian_ocr;
    std::vector<int> Assignment_ocr;
    double cost_ocr = hungarian_ocr.Solve(cost_matrix_ocr, Assignment_ocr);
    for(int i = 0; i < Assignment_ocr.size(); ++i) {
        int match_index_ocr = Assignment_ocr[i];
        if(Assignment_ocr[i] != -1&&cost_matrix_ocr[i][Assignment_ocr[i]] < 0.6) 
        {
            cFrameSpots_ocr.push_back(currentFrameSpots_ocr[match_index_ocr]);
            preFrameSpots_ocr.push_back(preFramespot_ocr[i]);
            //   std::cout<<"2"<<std::endl;
        }

     }


    for (const auto& point : cFrameSpots) {
        // 假设点在地面上，z 坐标为 0
        curvehiclePoints.push_back(Eigen::Vector3d(point.x, -point.y, 0.0));
    }

    for (const auto& point : preFrameSpots) {
        // 假设点在地面上，z 坐标为 0
        prevehiclePoints.push_back(Eigen::Vector3d(point.x, -point.y, 0.0));
    }

        for (const auto& point : cFrameSpots_ocr) {
        // 假设点在地面上，z 坐标为 0
        curvehiclePoints.push_back(Eigen::Vector3d(point.x, -point.y, 0.0));
    }

    for (const auto& point : preFrameSpots_ocr) {
        // 假设点在地面上，z 坐标为 0
        prevehiclePoints.push_back(Eigen::Vector3d(point.x, -point.y, 0.0));
    }
     std::cout<<prevehiclePoints.size()<<","<<curvehiclePoints.size()<<std::endl;

    estimatePoseICP(prevehiclePoints,curvehiclePoints);
    preFramespot = currentFrameSpots;
    preFramespot_ocr = currentFrameSpots_ocr;
   
}


void Odometry::piextocamera(const parking_slot_detection::gcn_parking & srv)
{
    double association_distance=0.5;
    std::vector<cv::Point2f> currvehiclePoints;
    std::vector<cv::Point2f> currvehiclePoints_ocr;
    // std::cout<<"enterpiexto"<<std::endl;
       
     for (size_t i = 0; i < srv.response.point0_x.size(); ++i){
          double dis=(abs(srv.response.point2_x[i]-srv.response.point1_x[i]))/50.1;
            double dis1=(abs(srv.response.point3_y[i]-srv.response.point1_y[i]))/49;
            std::cout<<"dis:"<<dis<<",dis1:"<<dis1<<std::endl;
        // std::cout<<srv.response.text<<":"<<-256+srv.response.point0_x[i]<<","<<-256+srv.response.point0_y[i]<<std::endl;
        currvehiclePoints.push_back(cv::Point2d((-256+srv.response.point0_x[i])/64.56, (-256+srv.response.point0_y[i])/63.15));
        currvehiclePoints.push_back(cv::Point2d((-256+srv.response.point1_x[i])/64.56, (-256+srv.response.point1_y[i])/63.15));
     }
    for (size_t j = 0; j < srv.response.ocrpointx1.size(); ++j){
        std::cout<<srv.response.texts[j]<<":"<<srv.response.ocrpointx1[j]<<","<<srv.response.ocrpointy1[j]<<std::endl;
        std::cout<<srv.response.texts[j]<<":"<<-256+srv.response.ocrpointx1[j]<<","<<-256+srv.response.ocrpointy1[j]<<std::endl;
        currvehiclePoints_ocr.push_back(cv::Point2d((-256+srv.response.ocrpointx1[j])/64.56, (-256+srv.response.ocrpointy1[j])/63.15));
        currvehiclePoints_ocr.push_back(cv::Point2d((-256+srv.response.ocrpointx2[j])/64.56, (-256+srv.response.ocrpointy2[j])/63.15));
    }
    // std::vector<Eigen::Vector3d> vehiclePoints;

    // for (const auto& point : currvehiclePoints) {
    //     // 假设点在地面上，z 坐标为 0
    //     vehiclePoints.push_back(Eigen::Vector3d(point.x, point.y, 0.0));
    // }
    int m=currvehiclePoints_ocr.size();int n=currvehiclePoints.size();
    if(m!=0&&n!=0)
        matchframes(currvehiclePoints,currvehiclePoints_ocr);
    
}


void Odometry::imageCallback_odo(const sensor_msgs::CompressedImageConstPtr& msg)
{
    canvasodom.setTo(cv::Scalar(255, 255, 255));
     try
    {
        // 解压缩图像
        cv::Mat image = cv::imdecode(cv::Mat(msg->data),cv::IMREAD_COLOR);
        // 确保图像尺寸符合预期
    // // 原图像大小
    // int original_width = image.cols;
    // int original_height = image.rows;

    // // 裁剪的目标大小
    // int crop_width = 660;
    // int crop_height = 660;

    // // 从图像中心开始裁剪
    // int x = (original_width - crop_width) / 2;
    // int y = (original_height - crop_height) / 2;

    // // 定义裁剪区域 (x, y 是矩形左上角的坐标，crop_width 和 crop_height 是裁剪的宽高)
    // cv::Rect roi(x, y, crop_width, crop_height);

    // // 使用 roi 裁剪图像
    // cv::Mat cropped_image = image(roi);

    // 显示裁剪后的图像
        cv::resize(image, image, cv::Size(512, 512));
        // cv::imshow(windowName1, image);
        // cv::waitKey(1);

        // 转换为ROS图像消息
        sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
       
        // 创建ROS服务客户端
        // ros::NodeHandle mh;
        // ros::ServiceClient client = mh.serviceClient<parking_slot_detection::gcn_parking>("gcn_service");

        // 创建请求和响应消息
        // parking_slot_detection::gcn_parking srv;
        srv.request.image_data = *image_msg;
        
    // while(ros::ok()){
        // 发送请求
        if (client.call(srv))
        {
            
            
            piextocamera(srv);
            // drawslotwithnumber(image);
            // 成功接收响应
            cv::Mat image_vis = image.clone();
            std::cout<<"spot:"<<srv.response.point0_x.size()<<std::endl;
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
            // cv::imshow("detected_results", image_vis);
            // cv::waitKey(1);

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

void Odometry::imuCallback_odom(const sensor_msgs::Imu::ConstPtr& msg) {
        // Eigen::Vector3d accel(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
        // Eigen::Vector3d gyro(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
        double dt = 0.01; // 假设固定的时间步长（可从ROS时间计算实际dt）
           // IMU数据作为预测步骤的输入
    Eigen::VectorXd u(6); // 加速度和角速度
    u << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
         msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
        // ekf.initialize(accel, gyro);
    //         ros::Time current_time = ros::Time::now();
    // double dt = (current_time - last_time_).toSec();
    // predictState(u, dt);
    // last_time_ = current_time;
    
        predict(u, dt);
}

void Odometry::wheelSpeedCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // 使用轮速计更新状态
    Eigen::VectorXd u(6);
    u << msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z,
         0, 0, 0; // 轮速计只提供线速度
        // 使用轮速计数据直接更新速度状态向量中的 v_x, v_y, v_z
    state_.segment<3>(7) = u.segment<3>(0);  // 用轮速计的速度替代状态中的速度
    // ros::Time current_time = ros::Time::now();
    // double dt = (current_time - last_time_).toSec();
    // double dt=0.01
    // ekf_.predict(u, dt);
    // last_time_ = current_time;
}



// IMU预测：根据加速度和角速度进行位姿预测
/*State EKF::imuOdometryUpdate(const Vector3d& accel, const Vector3d& gyro, double dt, const State& prev_state) {
    // 四元数更新
    Quaterniond delta_rotation(AngleAxisd(gyro.norm() * dt, gyro.normalized()));
    Quaterniond new_orientation = prev_state.orientation * delta_rotation;
    new_orientation.normalize();

    // 速度和位置更新
    Vector3d new_velocity = prev_state.velocity + (accel - prev_state.bias_accel) * dt;
    Vector3d new_position = prev_state.position + prev_state.velocity * dt + 0.5 * (accel - prev_state.bias_accel) * dt * dt;

    return {new_position, new_velocity, new_orientation, prev_state.bias_accel, prev_state.bias_gyro};
}*/


// IMU状态转移雅可比矩阵计算
/*MatrixXd EKF::computeStateTransitionJacobian(const Vector3d& accel, const Vector3d& gyro, double dt) {
    MatrixXd F = MatrixXd::Identity(15, 15);

    // 对位置和速度部分的雅可比矩阵
    F.block<3, 3>(0, 3) = Matrix3d::Identity() * dt;  // 位置对速度的偏导
    F.block<3, 3>(3, 9) = -Matrix3d::Identity() * dt; // 速度对加速度偏置的偏导

    // 对四元数姿态部分的雅可比矩阵（简化版本）
    Matrix3d skew_gyro = Matrix3d::Zero(); // 简化：陀螺仪的角速度向量的反对称矩阵
    F.block<3, 3>(6, 12) = -skew_gyro * dt; // 四元数对陀螺仪偏置的偏导
    // std::cout<<"CTJ"<<std::endl;

    return F;
}*/




