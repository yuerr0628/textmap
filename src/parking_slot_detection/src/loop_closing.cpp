#include "loop_closing.h"
#include <thread> // 用于模拟程序执行时间
#include <chrono>


RvizDisplay mapdisplay;
std::chrono::high_resolution_clock::time_point start;
// Odometry ekfodom;
int mp=0;
Eigen::MatrixXd Q;
Eigen::MatrixXd R1;
int noimage=1;
uint32_t avmseq2=-1;
uint32_t frontseq2=-1;
Eigen::Vector3d pre_loop;
LoopClosing::LoopClosing(ros::NodeHandle& nh,const std::string& filename):ekfodom(nh) {
   
    mapdisplay.RvizDisplay_init(nh);
     loadMap(filename);
    // image_sub = nh.subscribe("/driver/fisheye/avm/compressed", 10, &LoopClosing::imageCallback_map,this);
     avmimage_sub1 = nh.subscribe("/driver/fisheye/avm/compressed", 10, &LoopClosing::avm_callback1,this);
    frontimage_sub1= nh.subscribe("/driver/fisheye/front/compressed", 10, &LoopClosing::front_callback1,this);
    client = nh.serviceClient<parking_slot_detection::gcn_parking>("gcn_service");
        // 过程噪声和测量噪声
     Q = Eigen::MatrixXd::Identity(6, 6) * 0.01;
     R1 = Eigen::MatrixXd::Identity(6, 6) * 0.05;
    
}

void LoopClosing::loadMap(const std::string& filename) {
    YAML::Node data = YAML::LoadFile(filename);
    for (const auto& node : data) {
        AssociatedPair pair;
        pair.ID = node["ID"].as<int>();
        pair.unmatch_ID = node["unmatch_ID"].as<int>();
        pair.age = node["age"].as<int>();
        pair.distanceocr = node["distanceocr"].as<double>();
        pair.IF_TEXT_DET = node["IF_TEXT_DET"].as<bool>();
        pair.IF_SPOT_DET = node["IF_SPOT_DET"].as<bool>();

    // 解析ParkingSpot
        pair.spot.x1 = node["ParkingSpot"]["x1"].as<double>();
        pair.spot.y1 = node["ParkingSpot"]["y1"].as<double>();
        pair.spot.x2 = node["ParkingSpot"]["x2"].as<double>();
        pair.spot.y2 = node["ParkingSpot"]["y2"].as<double>();
        pair.spot.x3 = node["ParkingSpot"]["x3"].as<double>();
        pair.spot.y3 = node["ParkingSpot"]["y3"].as<double>();
        pair.spot.x4 = node["ParkingSpot"]["x4"].as<double>();
        pair.spot.y4 = node["ParkingSpot"]["y4"].as<double>();
        pair.spot.vacant = node["ParkingSpot"]["vacant"].as<int>();
        pair.spot.vacant = node["ParkingSpot"]["vacant"].as<int>();
        pair.spot.vacant_update = node["ParkingSpot"]["vacant_update"].as<int>();

    // 解析OCRPoint
        pair.ocrPoint.text = node["OCRPoint"]["text"].as<std::string>();
        pair.ocrPoint.confidence = node["OCRPoint"]["confidence"].as<double>();
        pair.ocrPoint.x1 = node["OCRPoint"]["x1"].as<double>();
        pair.ocrPoint.y1 = node["OCRPoint"]["y1"].as<double>();
        pair.ocrPoint.x2 = node["OCRPoint"]["x2"].as<double>();
        pair.ocrPoint.y2 = node["OCRPoint"]["y2"].as<double>();
        
        pair.ocrPoint.text = node["OCRPoint"]["text"].as<std::string>();
        pair.ocrPoint.confidence = node["OCRPoint"]["confidence"].as<double>();
        

        // mapdisplay.visualizeMapPoints(pair);
        mapPoints.push_back(pair);
    }
    mapdisplay.visualizeMapPoints(mapPoints);

}
bool intialekf=false;
void LoopClosing::initializeEKF(EKFState &ekf) {
   
      
    ekf.position = pose_loop.t;   // 初始位置
    // Eigen::Quaterniond q = Eigen::Quaterniond(tmp_R);
    ekf.orientation = Eigen::Quaterniond (pose_loop.R);  // 初始旋转单位四元数
    ekf.P = Eigen::MatrixXd::Identity(6, 6) * 0.1;     // 状态协方差
    double yaw_loop = std::atan2(pose_loop.R(1, 0), pose_loop.R(0, 0));
    cout<<"yaw_loop:"<<yaw_loop<<endl;
    intialloop=false;
    pre_loop=pose_loop.t;
    
    // cout<<"intialloop:"<<ekf.orientation<<endl;
    
  
}

void LoopClosing::predict( const Eigen::Vector3d &delta_p, const Eigen::Quaterniond &delta_q, 
             const Eigen::MatrixXd &Q) {
    // 1. 预测位置：加上平移增量
    ekf.position=delta_p;
    ekf.orientation =delta_q;
    // std::cout<<"delta_p"<<delta_p<<endl;
    
    //  Eigen::Vector3d euler = ekf.orientation.toRotationMatrix().eulerAngles(2, 1, 0); // roll, pitch, yaw
    //  double yaw_ekf = euler[0];
    // cout<<"yaw_ekf:"<<yaw_ekf<<endl;
    // // 取反 yaw（z 轴旋转）
    // euler[0] = -euler[0];

    // // 将修改后的欧拉角转换回四元数
    // Eigen::Quaterniond q_new = Eigen::Quaterniond(
    //     Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()) * // roll
    //     Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) * // pitch
    //     Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ())); // yaw
    //     ekf.orientation=q_new;
    //     Eigen::Vector3d euler1 = ekf.orientation.toRotationMatrix().eulerAngles(2, 1, 0); // roll, pitch, yaw

    // // 取反 yaw（z 轴旋转）
    //     cout<<"yaw_ekf1:"<<euler1[2] <<endl;
    // ekf.position += ekf.orientation *(delta_p);  // 转换到当前坐标系下

    // // ekf.position[0]=-ekf.position[0];

    // // 2. 预测方向：旋转增量
    // if(!vio_intial){
    //     ekf.orientation =Eigen::Quaterniond (pose_loop.R) ;
    //     vio_intial=true;
    //     Eigen::Vector3d euler = ekf.orientation.toRotationMatrix().eulerAngles(2, 1, 0); // ZYX 顺序
    //        // 获取 yaw（绕 Z 轴旋转角）
    //     double yaw_ekf = euler[0];
    //     cout<<"yaw_firt:"<<yaw_ekf<<endl;
        
    // }
    // else{
        

    // ekf.orientation = ekf.orientation * delta_q;
  
    
//     ekf.orientation.normalize();  // 保持四元数单位化
//   double yaw_loop = std::atan2(pose_loop.R(1, 0), pose_loop.R(0, 0));
    // cout<<"yaw_dect:"<<yaw_loop<<endl;
    
    // ekfodom.vpose
    // update(pose_loop.t,Eigen::Quaterniond (pose_loop.R),R1);
    
    // mapdisplay.publishodomPath(pose_loop.t);
    
    // saveTrajectoryToTUM("/data/yhy/localization_trajectory.txt", ekf);
    // // saveTrajectoryToTUM("/data/yhy/localization_trajectory.txt", pose_loop);
    // mapdisplay.publishodomPath(ekf.position);

    // 3. 更新协方差
    ekf.P += Q;  // 简化模型：直接加过程噪声
}


void LoopClosing::update(const Eigen::Vector3d &measured_p, const Eigen::Quaterniond &measured_q, 
            const Eigen::MatrixXd &R) {
    // 1. 计算位置误差
    cout<<"update"<<endl;
    Eigen::Vector3d position_error = measured_p - ekf.position;
    cout<<"position_error="<<position_error<<endl;

    // 2. 计算方向误差（四元数误差）
    Eigen::Quaterniond orientation_error = measured_q * ekf.orientation.inverse();
    Eigen::AngleAxisd angle_axis(orientation_error);

     // 3. 计算残差范数
    double position_error_norm = position_error.norm();
    cout<<"position_error_norm ="<<position_error_norm<<endl;
    double rotation_error_norm = std::abs(angle_axis.angle());
    // 4. 异常检测：如果残差超过阈值，舍弃观测数据
    if (position_error_norm > 3) { // 阈值可调
        std::cout << "Outlier detected! Skipping this update." << std::endl;
        ekf.orientation=pose_loop.R;
         ekf.position=pose_loop.t;
        return; // 跳过更新步骤
    }


    // 3. 构建观测残差向量 [位置误差; 旋转误差（角轴表示）]
    Eigen::VectorXd z(6);
    z.head<3>() = position_error;
    // Eigen::AngleAxisd angle_axis(orientation_error);
    z.tail<3>() = angle_axis.axis() * angle_axis.angle();

    // 4. 计算卡尔曼增益 K = P * H^T * (H * P * H^T + R)^-1
    Eigen::MatrixXd H = Eigen::MatrixXd::Identity(6, 6);  // 观测矩阵（简化为单位矩阵）
    Eigen::MatrixXd S = H * ekf.P * H.transpose() + R;
    Eigen::MatrixXd K = ekf.P * H.transpose() * S.inverse();

    // 5. 更新状态
    Eigen::VectorXd dx = K * z;
    ekf.position += dx.head<3>();  // 更新位置
    Eigen::Quaterniond dq = Eigen::Quaterniond(Eigen::AngleAxisd(dx.tail<3>().norm(), dx.tail<3>().normalized()));
    ekf.orientation = ekf.orientation * dq;
    ekf.orientation.normalize();
    // ekf.position=ekf.position*2;

    // 6. 更新协方差 P = (I - K * H) * P
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(6, 6);
    ekf.P = (I - K * H) * ekf.P;
}




std::vector<AssociatedPair> LoopClosing::getMapPoints() const {
    return mapPoints;
}

//icp计算pose
Eigen::Matrix4d LoopClosing::icp_pose(std::vector<Eigen::Vector3d>  prevPoints,const std::vector<Eigen::Vector3d>  currPoints)
{
    if(!intialloop)
    {
        predict(ekfodom.state_.p,Eigen::Quaterniond (ekfodom.state_.R_q),Q);
    }

        if (prevPoints.size() != currPoints.size() || prevPoints.size() < 3) {
        // std::cout << "Error: Point cloud size mismatch or insufficient points." << std::endl;
        // IF_USE_ODOM=false;   
        // odom_need_init=true;
        // noimage++
      
        return Eigen::Matrix4d::Identity();
    }
    // IF_USE_ODOM=true;
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    R = Matrix3d::Identity();
    t = Vector3d::Zero();
    Eigen::Vector3d prevt; // 临时保存之前的平移
    Eigen::Matrix3d preR;
    const double convergenceThreshold = 1e-4; // 收敛判断阈值
    const int maxIterations = 100; // 最大迭代次数

    for (int iteration = 0; iteration < maxIterations; ++iteration) {
        // 2. 计算变换
        prevt=t;
        preR=R;
       ekfodom.computeTransformation(prevPoints, currPoints, R, t);

        // 3. 更新当前点云位置
        // 应用当前的R和t到prevPoints上，生成新的currPoints（待更新的当前点集）
        for (size_t i = 0; i < prevPoints.size(); ++i) {
            prevPoints[i] = R * prevPoints[i] + t; // 更新点
        }
        pose_loop.R=preR*R;
        pose_loop.t=preR*t+prevt;
        // 4. 判断收敛
            // 计算质心变化的范数
        // double centroidChangeNorm = t.norm();
        // if (centroidChangeNorm < threshold) {
        //     std::cout << "ICP has converged." <<centroidChangeNorm<<std::endl;
        //  } 
        //  else {
        //     std::cout << "ICP is still converging. Centroid change: " << centroidChangeNorm << std::endl;
        // }
        double centroidChangeNorm = (t).norm();
        if (centroidChangeNorm < convergenceThreshold) {
            // std::cout << "ICP has converged in " << centroidChangeNorm<< std::endl;
            break;
        }
        else{
            //  std::cout << "ICP has converged." <<centroidChangeNorm<<std::endl;
        }
    }
    // computeTransformation(prevPoints, currPoints, R, t);

    // t=prevt;
    // R=preR;

    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    
    pose.block<3, 3>(0, 0) = pose_loop.R;
    pose.block<3, 1>(0, 3) = pose_loop.t;
    if(intialloop)
        {
            // intialloop=false;
            
            initializeEKF(ekf);
            noimage=1;
        }

    double yaw_loop = std::atan2(pose_loop.R(1, 0), pose_loop.R(0, 0));
    // cout<<"yaw_dect:"<<yaw_loop<<endl;
    
    // ekfodom.vpose
    update(pose_loop.t,Eigen::Quaterniond (pose_loop.R),R1);
    
    // mapdisplay.publishodomPath(pose_loop.t);
    ekf.position=pose_loop.t;
    ekf.orientation=Eigen::Quaterniond (pose_loop.R);
    Eigen::Vector3d p_error = pre_loop - pose_loop.t;
    double per=p_error.norm();
    if(per<=3)
    {
    auto end = std::chrono::high_resolution_clock::now();

    // 计算运行时间
    std::chrono::duration<double> duration = end - start;
    cout<< "time:"<<duration.count()<<endl;
   saveTrajectoryToTUM("/data/yhy/localization_trajectory.txt", ekf);
    // saveTrajectoryToTUM("/data/yhy/localization_trajectory.txt", pose_loop);
    mapdisplay.publishodomPath(ekf.position);
    }
    pre_loop=pose_loop.t;
    

    // update(pose);
    // ekfodometry(state_);
    
    return pose;

}


// 辅助函数：去掉字符串中的字母 'A'
std::string LoopClosing::removeA(const std::string& str) {
    std::string result;
    std::copy_if(str.begin(), str.end(), std::back_inserter(result),
                 [](char c) { return c != 'A' ; });  // 去掉 'A' 和 'a'
                //  std::cout<<result<<std::endl;
    return result;
}

  
double LoopClosing::computeSimilarity(const std::string& si, const std::string& sj) {
    std::string modifiedSi = removeA(si);
    std::string modifiedSj = removeA(sj);
    int distance = levenshtein(modifiedSi, modifiedSj);
    // std::cout<<"distance:"<<distance<<std::endl;
    return (std::max(modifiedSi.size(), modifiedSj.size()) - distance) / static_cast<double>(std::max(modifiedSi.size(), modifiedSj.size()));
}

void LoopClosing::detectLoop(const std::vector<AssociatedPair>& lastPairs) {
    std::unordered_map<std::string, int> matchedWords;
    std::vector<AssociatedPair> map_pair;
    std::vector<AssociatedPair> cur_pair;
    // auto mapPoints = loopClosing.getMapPoints();
    // std::cout<<"lastpair:"<<lastPairs.size()<<std::endl;
    // std::cout<<"pointpair:"<<mapPoints.size()<<std::endl;
    for (const auto& lastPair : lastPairs) 
    {
        for (const auto& mapPoint : mapPoints)
            {
                if(!lastPair.ocrPoint.text.empty()&&!mapPoint.ocrPoint.text.empty())
                {
                    
                     double similarity = computeSimilarity(lastPair.ocrPoint.text, mapPoint.ocrPoint.text);
                if (similarity ==1) {
                    map_pair.push_back(mapPoint);
                    cur_pair.push_back(lastPair);
                   break;
                    // matchedWords[mapPoint.ocrPoint.text]++;
                }
                    
                }
               
            }
    }
    std::cout<<"pointpair:"<<map_pair.size()<<std::endl;
    std::cout<<"lastpair:"<<cur_pair.size()<<std::endl;

     piexl_to_3d(map_pair,cur_pair);
    //  mapdisplay.matchdisplay(map_pair);

    // selectCandidates(matchedWords);
}

void LoopClosing::selectCandidates(const std::unordered_map<std::string, int>& matchedWords) {
    std::vector<int> candidateIDs;
    int minCovisible = 3;  // 根据实际情况调整
    for (const auto& pair : matchedWords) {
        if (pair.second > minCovisible) {
            // 假设 mapPoints 中的 ID 对应于候选关键帧
            for (const auto& mapPoint : mapPoints) {
                if (mapPoint.ocrPoint.text == pair.first) {
                    candidateIDs.push_back(mapPoint.ID);
                    break;
                }
            }
        }
    }

    // 选择前 10 个候选关键帧
    if (candidateIDs.size() > 10) {
        candidateIDs.resize(10);
    }

    // 调用定位函数
    for (int id : candidateIDs) {
        locateFrame(mapPoints[id]);
    }
}

void LoopClosing::locateFrame(const AssociatedPair& latestPair) {
    // 在这里实现具体的定位逻辑
    // 例如，计算当前帧与候选关键帧之间的相对变换
}

int LoopClosing::levenshtein(const std::string& s1, const std::string& s2) {
    size_t len1 = s1.size();
    size_t len2 = s2.size();
    
    std::vector<std::vector<int>> dp(len1 + 1, std::vector<int>(len2 + 1));

    for (size_t i = 0; i <= len1; ++i) {
        dp[i][0] = i;  // 删除操作
    }

    for (size_t j = 0; j <= len2; ++j) {
        dp[0][j] = j;  // 插入操作
    }

    for (size_t i = 1; i <= len1; ++i) {
        for (size_t j = 1; j <= len2; ++j) {
            int cost ; // 相同为0，不同为1
            if(s1[i - 1] == s2[j - 1])
                cost = 0;
            else cost = 1;
            dp[i][j] = std::min({dp[i - 1][j] + 1,       // 删除
                                 dp[i][j - 1] + 1,       // 插入
                                 dp[i - 1][j - 1] + cost}); // 替换
        }
    }
    
    return dp[len1][len2];
}

void LoopClosing::piexl_to_3d(const std::vector<AssociatedPair>&map_point,const std::vector<AssociatedPair>&cur_point)
{
    std::vector<Eigen::Vector3d> mappoints;
    std::vector<Eigen::Vector3d> curpoints;
    //  std::vector<Eigen::Vector3d> currvehiclePoints;
    //     std::vector<Eigen::Vector3d> currvehiclePoints_ocr;
  for (size_t i = 0; i < map_point.size(); ++i) {
        const auto& pairMap = map_point[i];
        const auto& pairCur = cur_point[i];

        // 检查条件
        if (pairCur.spot.x1!=0 && pairMap.spot.x1!= 0) {
            
            mappoints.push_back(Eigen::Vector3d(pairMap.ocrPoint.x1, pairMap.ocrPoint.y1, 0));
            mappoints.push_back(Eigen::Vector3d(pairMap.ocrPoint.x2, pairMap.ocrPoint.y2, 0));
            curpoints.push_back(Eigen::Vector3d((-330+pairCur.ocrPoint.y1)/ 63.15, -(-330 + pairCur.ocrPoint.x1) /64.56, 0));
            curpoints.push_back(Eigen::Vector3d((-330+pairCur.ocrPoint.y2)/ 63.15, -(-330 + pairCur.ocrPoint.x2) /64.56, 0));
            // curpoints.push_back(Eigen::Vector3d((-330+pairCur.ocrPoint.x1)/ 64.56, -(-330 + pairCur.ocrPoint.y1) /63.15, 0));
            // curpoints.push_back(Eigen::Vector3d((-330+pairCur.ocrPoint.x2)/ 64.56, -(-330 + pairCur.ocrPoint.y2) /63.15, 0));
            // 将 cur_point 的两个 OCR 点压入
            mappoints.push_back(Eigen::Vector3d(pairMap.spot.x1,  pairMap.spot.y1, 0));
            mappoints.push_back(Eigen::Vector3d(pairMap.spot.x2, pairMap.spot.y2, 0));
            mappoints.push_back(Eigen::Vector3d(pairMap.spot.x3,  pairMap.spot.y3, 0));
            mappoints.push_back(Eigen::Vector3d(pairMap.spot.x4, pairMap.spot.y4, 0));
            curpoints.push_back(Eigen::Vector3d((-256+pairCur.spot.y1)/ 48.99, -(-256 + pairCur.spot.x1) /50.08, 0));
            curpoints.push_back(Eigen::Vector3d((-256+pairCur.spot.y2)/ 48.99, -(-256 + pairCur.spot.x2) /50.08, 0));
            curpoints.push_back(Eigen::Vector3d((-256+pairCur.spot.y3)/ 48.99, -(-256 + pairCur.spot.x3) /50.08, 0));
            curpoints.push_back(Eigen::Vector3d((-256+pairCur.spot.y4)/ 48.99, -(-256 + pairCur.spot.x4) /50.08, 0));
            //   curpoints.push_back(Eigen::Vector3d((-256+pairCur.spot.x1)/ 50.08, -(-256+pairCur.spot.y1)/ 50.08, 0));
            // curpoints.push_back(Eigen::Vector3d((-256+pairCur.spot.x2)/ 50.08, -(-256+pairCur.spot.y2)/ 50.08, 0));
            // curpoints.push_back(Eigen::Vector3d((-256+pairCur.spot.x3)/ 50.08, -(-256+pairCur.spot.y3)/ 50.08, 0));
            // curpoints.push_back(Eigen::Vector3d((-256+pairCur.spot.x4)/ 50.08, -(-256+pairCur.spot.y4)/ 50.08, 0));
        }
        else{
            mappoints.push_back(Eigen::Vector3d(pairMap.ocrPoint.x1, pairMap.ocrPoint.y1, 0));
            mappoints.push_back(Eigen::Vector3d(pairMap.ocrPoint.x2, pairMap.ocrPoint.y2, 0));
            //  curpoints.push_back(Eigen::Vector3d((-330+pairCur.ocrPoint.x1)/ 64.56, -(-330 + pairCur.ocrPoint.y1) /63.15, 0));
            // curpoints.push_back(Eigen::Vector3d((-330+pairCur.ocrPoint.x2)/ 64.56, -(-330 + pairCur.ocrPoint.y2) /63.15, 0));
            curpoints.push_back(Eigen::Vector3d((-330+pairCur.ocrPoint.y1)/ 63.15, -(-330 + pairCur.ocrPoint.x1) /64.56, 0));
            curpoints.push_back(Eigen::Vector3d((-330+pairCur.ocrPoint.y2)/ 63.15, -(-330 + pairCur.ocrPoint.x2) /64.56, 0));
        }
    }
    // std::cout<<curpoints.size()<<","<<mappoints.size()<<std::endl;
    // odom.computepose(curpoints, mappoints);
    mapdisplay.matchdisplay(mappoints);
    icp_pose(curpoints,mappoints);


}


void LoopClosing::avm_callback1(const sensor_msgs::CompressedImageConstPtr& msg) {
       cout<<"enteravm"<<endl;
    avmseq2 = ++avmseq2; // 获取序列号
     cout<<avmseq2<<endl;
    avm_buffer2[avmseq2] = msg;          // 将消息存入缓冲区

    // 检查是否有匹配的 front 图像
    if (front_buffer2.count(avmseq2)) {
        // 找到匹配的 front 图像
        auto front_msg = front_buffer2[avmseq2];

        // 移除已匹配的消息
        front_buffer2.erase(avmseq2);
        avm_buffer2.erase(avmseq2);

        // 触发后续处理逻辑
        imageCallback_map(msg, front_msg);
    }
}

void LoopClosing::front_callback1(const sensor_msgs::CompressedImageConstPtr& msg) {
    // uint32_t seq = msg->header.seq; // 获取序列号
    frontseq2 = ++frontseq2;
     cout<<frontseq2<<endl;
    front_buffer2[frontseq2] = msg;        // 将消息存入缓冲区

    // 检查是否有匹配的 AVM 图像
    if (avm_buffer2.count(frontseq2)) {
        // 找到匹配的 AVM 图像
        auto avm_msg = avm_buffer2[frontseq2];

        // 移除已匹配的消息
        avm_buffer2.erase(frontseq2);
        front_buffer2.erase(frontseq2);

        // 触发后续处理逻辑
        imageCallback_map(avm_msg, msg);
    }
}

void LoopClosing::imageCallback_map(const sensor_msgs::CompressedImageConstPtr& msg,const sensor_msgs::CompressedImageConstPtr&front_msg)
{
    // canvasodom.setTo(cv::Scalar(255, 255, 255));
    
     try
    {
        // 解压缩图像
        cv::Mat image = cv::imdecode(cv::Mat(msg->data),cv::IMREAD_COLOR);
    
        // 转换为ROS图像消息
        sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        srv.request.image_data = *image_msg;
        
    // while(ros::ok()){
        // 发送请求
        if (client.call(srv))
        {    
        start = std::chrono::high_resolution_clock::now();
        double association_distance=0.6;
        mp=mp+1;
        associatedPairs_map.clear();
        AssociatedPair pair;
        pair.ID=mp;
        cout<<" srv.response.point0_x.size()"<< srv.response.point0_x.size()<<endl;
        std::vector<bool> matched_spot_flags(srv.response.point0_x.size(), false); // 标记车位是否已匹配
        std::vector<bool> matched_number_flags(srv.response.ocrpointx1.size(), false); // 标记车位号是否已匹配
        for (size_t i = 0; i < srv.response.point0_x.size(); ++i){
            for (size_t j = 0; j < srv.response.ocrpointx1.size(); ++j){
                    double point0_x = (srv.response.point0_x[i]+srv.response.point1_x[i])/2.0;
                    double point0_y = (srv.response.point0_y[i]+srv.response.point1_y[i])/2.0;
                    double x1 = (srv.response.ocrpointx1[j]+srv.response.ocrpointx2[j])/2.0;
                    double y1 = (srv.response.ocrpointy1[j]+srv.response.ocrpointy2[j])/2.0;
                    double disocr_x=(x1-330)/64.56;double disocr_y=(y1-330)/63.15;
                    double disspot_x=(point0_x-256)/50.08;double disspot_y=(point0_y-256)/48.99;
                    double distance = sqrt(pow((disocr_x - disspot_x), 2) + pow((disocr_y - disspot_y), 2));
                    // double distance = sqrt(pow((x1 - point0_x)/48.99, 2) + pow((y1 - point0_y)/48.99, 2));
                    if (distance < association_distance) {
                        //  std::cout<<"enter"<<std::endl;
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
                        pair.spot.vacant=srv.response.label[i];
                        pair.IF_SPOT_DET=true;
                        pair.IF_TEXT_DET=true;
                        pair.ID=mp;
                        pair.ocrPoint.confidence=srv.response.confidence[j];
                        associatedPairs_map.push_back(pair); // 存储关联对
                        matched_spot_flags[i] = true;
                        matched_number_flags[j] = true;
                        // std::cout<<pair.spot.x1<<","<<pair.spot.y1<<","<<pair.spot.x2<<","<<pair.spot.y2<<std::endl;
                        // std::cout<<pair.ocrPoint.text<<std::endl;
                        break;
                        

                    }

            }
        }
            // 添加未匹配的车位信息
        for (size_t i = 0; i < matched_spot_flags.size(); ++i) {
            if (!matched_spot_flags[i]) {
                // 填充车位信息，车位号信息留空或设置默认值
                // std::cout<<pair.ocrPoint.x1;
                pair.ocrPoint.text="";
                pair.ocrPoint.x1=pair.ocrPoint.y1=pair.ocrPoint.x2=pair.ocrPoint.y2=0;
                pair.ocrPoint.confidence=0;
                pair.IF_SPOT_DET=true;
                pair.IF_TEXT_DET=false;
                pair.spot.x1 = srv.response.point0_x[i];
                pair.spot.y1 = srv.response.point0_y[i];
                pair.spot.x2 = srv.response.point1_x[i];
                pair.spot.y2 = srv.response.point1_y[i];
                pair.spot.x3 = srv.response.point2_x[i];
                pair.spot.y3 = srv.response.point2_y[i];
                pair.spot.x4 = srv.response.point3_x[i];
                pair.spot.y4 = srv.response.point3_y[i];
                pair.spot.vacant=srv.response.label[i];
                pair.ID=mp;
                associatedPairs_map.push_back(pair);
            }
        }
        // 添加未匹配的车位号信息
        for (size_t j = 0; j < matched_number_flags.size(); ++j) {
            if (!matched_number_flags[j]) {
                // 车位信息留空或设置默认值，填充车位号信息
                // pair.spot 留空 ...
                pair.spot.x1=pair.spot.y1=pair.spot.x2=pair.spot.y2=pair.spot.x3=pair.spot.y3=pair.spot.x4=pair.spot.y4=0;
                pair.spot.vacant=0;
                pair.IF_SPOT_DET=false;
                pair.IF_TEXT_DET=true;
                pair.ocrPoint.x1 = srv.response.ocrpointx1[j];
                pair.ocrPoint.y1 = srv.response.ocrpointy1[j];
                pair.ocrPoint.x2 = srv.response.ocrpointx2[j];
                pair.ocrPoint.y2 = srv.response.ocrpointy2[j];
                pair.ocrPoint.text = srv.response.texts[j];
                pair.ocrPoint.confidence=srv.response.confidence[j];
                pair.ID=mp;
                associatedPairs_map.push_back(pair);
                

            }
        }
        // 2d_to_3d(associatedPairs_map);
        if(associatedPairs_map.size()>=2)
        {
            ekf.timestamp= front_msg->header.stamp.toSec();
            detectLoop(associatedPairs_map);
        }
        else{
            intialloop=true;
        }

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

void LoopClosing::saveTrajectoryToTUM(const std::string& filename, const EKFState& pose) {
    // file(filename, std::ios::out | std::ios::app);
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open file " << filename << " for writing." << std::endl;
        return;
    }

    // 遍历轨迹，写入 TUM 格式
    
        file << std::fixed << std::setprecision(6)
             << pose.timestamp << " "                   // 时间戳
             << pose.position.x() << " "               // 平移 x
             << pose.position.y() << " "               // 平移 y
             << pose.position.z() << " "               // 平移 z
             << pose.orientation.x() << " "           // 四元数 x
             << pose.orientation.y() << " "           // 四元数 y
             << pose.orientation.z() << " "           // 四元数 z
             << pose.orientation.w()                  // 四元数 w
             << std::endl;
    

    // file.close();
    // std::cout << "Trajectory saved to " << filename << std::endl;
}
