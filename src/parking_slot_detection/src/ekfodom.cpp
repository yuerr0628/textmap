#include "ekfodom.h"
#include"drawmap.h"
#include "rvizshow.h"
#include "loop_closing.h"

#define IF_WS_ERROR 0
#define IF_IMU_MOUNT_ERROR 0

RvizDisplay odomdisplay;
uint32_t avmseq1=-1;
uint32_t frontseq1=-1;
int stable=0;
bool odom_need_init = true;
// LoopClosing ekfloop;
cv::Mat canvasodom(1000, 1000, CV_8UC3, cv::Scalar(255, 255, 255));
// cv::String windowName1 = "slots";
std::vector<Eigen::Vector3d> trajectory; // 存储相机轨迹
Eigen::Matrix4d intial1 = Eigen::Matrix4d::Identity();
Eigen::Matrix4d curr1 =intial1;
std::vector<Eigen::Vector2d> deny_time = {Eigen::Vector2d(80, 100), Eigen::Vector2d(120, 190), Eigen::Vector2d(220, 240)};

// Odometry::Odometry() {
//     ROS_WARN("Odometry initialized without NodeHandle. Some functionality may be limited.");
// }

Odometry::Odometry(ros::NodeHandle& nh):pose1(nh) {

    
    imu_sub_odom = nh.subscribe("/Inertial/imu/data", 10, &Odometry::imuCallback_odom, this);
    wheel_speed_sub = nh.subscribe("/rock_can/speed_feedback", 10, &Odometry::WheelCallback, this);
    // gps_sub_odom= nh.subscribe("/Inertial/gps/fix", 10, &Odometry::GpsCallback, this);
    avmimage_sub = nh.subscribe("/driver/fisheye/avm/compressed", 10, &Odometry::avm_callback,this);
    frontimage_sub = nh.subscribe("/driver/fisheye/front/compressed", 10, &Odometry::front_callback,this);
    
    client = nh.serviceClient<parking_slot_detection::gcn_parking>("gcn_service");
    client_plate = nh.serviceClient<parking_slot_detection::PlateRecognition>("license_plate_recognition");

    odomdisplay.RvizDisplay_init(nh);
   
    bool isFirstFrame=true; 
    current_pose = Eigen::Matrix4d::Identity();

    nh.param<double>("acc_noise", acc_n_var, 1e-2);
    nh.param<double>("gyr_noise", gyr_n_var, 1e-3);
    nh.param<double>("acc_bias_noise", acc_rw_var, 1e-7);
    nh.param<double>("gyr_bias_noise", gyr_rw_var, 1e-8);
    nh.param<double>("imu_mount_rw_var", imu_mount_rw_var, 1e-6);
    nh.param<double>("whl_odom_var", whl_odom_var, 5);
    // nh.param<double>("odometry_var",odom_var,1);
    nh.param<double>("odometry_rot_var",odom_rot_var,100);
    nh.param<double>("odometry_tran_var",odom_tran_var,100);
    nh.param<double>("gnss_noise_var", gnss_noise_var, 1e-2);
        nh.param<bool>("IF_GPS_DENY", IF_GPS_DENY, false);
    Eigen::Matrix3d R_I = Eigen::Matrix3d::Identity();
    state_.R_q = R_I;
    state_.p.setZero();
    state_.p={-8.671878,-10.913547,0};
    
    state_.v.setZero();
    state_.ba.setZero();
    state_.bg.setZero();
    // tf::Quaternion q(0.000511, 0.001482,
    //                  0.468098, 0.883676);
    
    // 将四元数转换为 Eigen 四元数
    // Eigen::Quaterniond eigen_q(q.w(), q.x(), q.y(), q.z());
    double theta=23 * M_PI / 180.0;
    // double theta=55 * M_PI / 180.0;
    // // 转换为旋转矩阵
    Eigen::Matrix3d rotation_matrix;
   rotation_matrix << cos(theta), -sin(theta), 0,
                       sin(theta),  cos(theta), 0,
                       0,           0,           1;

    // 0.000269 0.000936 -0.468098 0.883676
    state_.R_imu = R_I;
    state_.R_q = rotation_matrix;
    state_.ws = 1.0;

    state_.P = Eigen::MatrixXd::Zero(StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL);
    state_.P.block<3, 3>(StateIndex::P, StateIndex::P) = 100. * Eigen::Matrix3d::Identity();
    state_.P(StateIndex::P + 2, StateIndex::P + 2) = 1;
    state_.P.block<3, 3>(StateIndex::V, StateIndex::V) = 100. * Eigen::Matrix3d::Identity();
    state_.P(StateIndex::V + 2, StateIndex::V + 2) = 1;
    state_.P.block<3, 3>(StateIndex::R, StateIndex::R) = 1. * Eigen::Matrix3d::Identity();
    state_.P(StateIndex::R+2, StateIndex::R+2) = 9.;
    state_.P.block<3, 3>(StateIndex::BA, StateIndex::BA) = 0.0004 * Eigen::Matrix3d::Identity();
    state_.P.block<3, 3>(StateIndex::BG, StateIndex::BG) = 0.000436332313 * Eigen::Matrix3d::Identity();
    state_.P.block<3, 3>(StateIndex::IMU_INSTALL_ANGLE, StateIndex::IMU_INSTALL_ANGLE) = imu_mount_rw_var * Eigen::Matrix3d::Identity();
    // state_.P(StateIndex::IMU_INSTALL_ANGLE+2, StateIndex::IMU_INSTALL_ANGLE+2) = 1e-6; //限制yaw
    state_.P(StateIndex::WS, StateIndex::WS) = 1e-5;
    gravity_ = Eigen::Vector3d(0, 0, -9.796);
    Q_ = Eigen::Matrix<double, StateNoiseIndex::NOISE_TOTAL, StateNoiseIndex::NOISE_TOTAL>::Identity();
    Q_.block<3, 3>(StateNoiseIndex::ACC_NOISE, StateNoiseIndex::ACC_NOISE) =
        acc_n_var * Eigen::Matrix3d::Identity();
    Q_.block<3, 3>(StateNoiseIndex::GYRO_NOISE, StateNoiseIndex::GYRO_NOISE) =
        gyr_n_var * Eigen::Matrix3d::Identity();
    Q_.block<3, 3>(StateNoiseIndex::ACC_RANDOM_WALK, StateNoiseIndex::ACC_RANDOM_WALK) =
        acc_rw_var * Eigen::Matrix3d::Identity();
    Q_.block<3, 3>(StateNoiseIndex::GYRO_RANDOM_WALK, StateNoiseIndex::GYRO_RANDOM_WALK) =
        gyr_rw_var * Eigen::Matrix3d::Identity();
        gyr_rw_var * Eigen::Matrix3d::Identity();
    whl_Rm_ = Eigen::Matrix3d::Identity() * whl_odom_var;
    odom_Rm_ = Eigen::MatrixXd::Identity(6, 6);
    odom_Rm_.topLeftCorner<3, 3>() = Eigen::MatrixXd::Identity(3, 3) * odom_rot_var;
    odom_Rm_.bottomRightCorner<3, 3>() = Eigen::MatrixXd::Identity(3, 3) * odom_tran_var;
   
    // odom_Rm_ = Eigen::MatrixXd::Identity(6, 6) * odom_var;
    gps_Rm_ = Eigen::Matrix3d::Identity() * gnss_noise_var;
}


void Odometry::avm_callback(const sensor_msgs::CompressedImageConstPtr& msg) {
       cout<<"enteravm"<<endl;
    avmseq1 = ++avmseq1; // 获取序列号
     cout<<avmseq1<<endl;
    avm_buffer1[avmseq1] = msg;          // 将消息存入缓冲区

    // 检查是否有匹配的 front 图像
    if (front_buffer1.count(avmseq1)) {
        // 找到匹配的 front 图像
        auto front_msg = front_buffer1[avmseq1];

        // 移除已匹配的消息
        front_buffer1.erase(avmseq1);
        avm_buffer1.erase(avmseq1);

        // 触发后续处理逻辑
        process_synced_images(msg, front_msg);
    }
}

void Odometry::front_callback(const sensor_msgs::CompressedImageConstPtr& msg) {
    // uint32_t seq = msg->header.seq; // 获取序列号
    frontseq1 = ++frontseq1;
     cout<<frontseq1<<endl;
    front_buffer1[frontseq1] = msg;        // 将消息存入缓冲区

    // 检查是否有匹配的 AVM 图像
    if (avm_buffer1.count(frontseq1)) {
        // 找到匹配的 AVM 图像
        auto avm_msg = avm_buffer1[frontseq1];

        // 移除已匹配的消息
        avm_buffer1.erase(frontseq1);
        front_buffer1.erase(frontseq1);

        // 触发后续处理逻辑
        process_synced_images(avm_msg, msg);
    }
}

void Odometry::cleanup_buffers(uint32_t current_seq) {
    for (auto it = avm_buffer1.begin(); it != avm_buffer1.end();) {
        if (it->first < current_seq - 100) { // 假设保留最多 100 个序列号
            it = avm_buffer1.erase(it);
        } else {
            ++it;
        }
    }

    for (auto it = front_buffer1.begin(); it != front_buffer1.end();) {
        if (it->first < current_seq - 100) { // 假设保留最多 100 个序列号
            it = front_buffer1.erase(it);
        } else {
            ++it;
        }
    }
}

bool Odometry::InitState()
{
    ROS_ERROR("init state");

    Eigen::Vector3d sum_acc(0., 0., 0.);
    for (auto imu_data : imu_buf_)
    {
        sum_acc += imu_data->acc;
    }
    const Eigen::Vector3d mean_acc = sum_acc / (double)imu_buf_.size();
    // std::cout << "[ ESKF ] Mean acc: " << mean_acc[0] << " " << mean_acc[1] << " " << mean_acc[2] << std::endl;

    Eigen::Vector3d sum_err2(0., 0., 0.);
    for (auto imu_data : imu_buf_)
        sum_err2 += (imu_data->acc - mean_acc).cwiseAbs2();
    const Eigen::Vector3d std_acc = (sum_err2 / (double)imu_buf_.size()).cwiseSqrt();
    // std::cout << "[ ESKF ] Std acc : " << std_acc[0] << " " << std_acc[1] << " " << std_acc[2] << std::endl;
    if (std_acc.maxCoeff() > IMU_Std)
    {
        // std::cout << "[ ESKF ] Big acc std: " << std_acc[0] << " " << std_acc[1] << " " << std_acc[2] << std::endl;
        return false;
    }

    state_.time = last_imu_ptr_->timestamp;
    

    // z-axis
    const Eigen::Vector3d &z_axis = mean_acc.normalized();

    // x-axis
    Eigen::Vector3d x_axis = Eigen::Vector3d::UnitX() - z_axis * z_axis.transpose() * Eigen::Vector3d::UnitX();
    x_axis.normalize();

    // y-axis
    Eigen::Vector3d y_axis = z_axis.cross(x_axis);
    y_axis.normalize();

    // Eigen::Matrix3d R_I_G;
    // R_I_G.block<3, 1>(0, 0) = x_axis;
    // R_I_G.block<3, 1>(0, 1) = y_axis;
    // R_I_G.block<3, 1>(0, 2) = z_axis;

    // state_.R_q = Eigen::Quaterniond(R_I_G);

    stateInit_ = true;
    // IF_USE_GPS=false;
    ROS_ERROR("init state finish");
    return true;
}
 
bool ekf_q_init=true;
bool Odometry::process_IMU_Data(IMUDataPtr imu_data_ptr)
{
    if (!stateInit_)
    {
        imu_buf_.push_back(imu_data_ptr);
        if (imu_buf_.size() > IMU_BUF_SIZE)
            imu_buf_.pop_front();
        return false;
    }
    // std::cout<<"[ ESKF ] imu buffer size: "<<imu_buf_.size()<<std::endl;
    // new imu_data_ptr = std::make_shared<sensor_msgs::Imu>(*imu_msg);
    // 创建一个新的指针，指向imu_msg的内容
    // if(ekf_q_init){
    //     state_.R_q=imu_data_ptr->q;
    //     // state_.R_imu=state_.R_q;
    //     ekf_q_init=false;
    // }
    PredictByImu(last_imu_ptr_, imu_data_ptr);
    last_imu_ptr_ = imu_data_ptr;
    return true;
}

void Odometry::drawline(const Eigen::Matrix4d&Rpose)
{
     
    //             // 更新当前帧的全局位姿
    // curr1 = curr1 * Rpose;
    // 提取当前帧的平移向量
    Eigen::Vector3d position = Rpose.block<3, 1>(0, 3);
    // 将当前帧的位置添加到轨迹中
    trajectory.push_back(position);
    // odomdisplay.publishodomPath(position);
}


  void Odometry::PredictByImu(IMUDataPtr last_imu_ptr, IMUDataPtr cur_imu_ptr)
{
    // ROS_INFO("predict by imu");
    // mean prediction
    double dt = cur_imu_ptr->timestamp - last_imu_ptr->timestamp;
    double dt_2 = dt * dt;

    // timestamp
    state_.time = cur_imu_ptr->timestamp;

    // p v R
    Eigen::Vector3d acc_unbias = 0.5 * (last_imu_ptr->acc + cur_imu_ptr->acc) - state_.ba;
    Eigen::Vector3d gyr_unbias = 0.5 * (last_imu_ptr->gyro + cur_imu_ptr->gyro) - state_.bg;

    // std::cout << "acc_unbias " << acc_unbias.transpose() << std::endl;
    // std::cout << "gyr_unbias " << gyr_unbias.transpose()  << std::endl;

    Eigen::Vector3d acc_nominal = state_.R_q * state_.R_imu * acc_unbias + gravity_;

    // std::cout << "acc_nominal " << acc_nominal.transpose()  << std::endl;
    // std::cout << "state_.v " << state_.v.transpose()  << std::endl;

    state_.p += state_.v * dt + 0.5 * acc_nominal * dt_2; // 0.5
    state_.v += acc_nominal * dt;
    const Eigen::Vector3d omg = state_.R_imu * (gyr_unbias)*dt;           // 2.0
    Eigen::Quaterniond dq(1.0, omg(0) / 2.0, omg(1) / 2.0, omg(2) / 2.0); // 2
    // Eigen::Matrix3d dR = Eigen::Matrix3d::Identity();
    const auto &dR = Utility::delta_rot_mat(omg);
    if (omg.norm() > DBL_EPSILON)
    {
        // dR = Eigen::AngleAxisd(omg.norm(), omg.normalized()).toRotationMatrix();
        state_.R_q = state_.R_q * dR;
    }

    
    

    state_.R_imu = state_.R_imu;

    // variance propogation
    Eigen::Matrix3d mI = Eigen::Matrix3d::Identity();
    Eigen::MatrixXd mF = Eigen::MatrixXd::Zero(StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL);

    // dq
    if (omg.norm() > DBL_EPSILON)
    {
        mF.block<3, 3>(StateIndex::R, StateIndex::R) = dR.transpose(); // 3 //dq.toRotationMatrix().transpose() //mI - Utility::SkewSymmetric(imuGyro - state_.bg) * dt
    }
    else
    {
        mF.block<3, 3>(StateIndex::R, StateIndex::R) = mI;
    }
    mF.block<3, 3>(StateIndex::R, StateIndex::BG) = -state_.R_imu * dt;

    if (IF_IMU_MOUNT_ERROR)                                                                                                     // 3
        mF.block<3, 3>(StateIndex::R, StateIndex::IMU_INSTALL_ANGLE) = -state_.R_imu * Utility::SkewSymmetric(gyr_unbias) * dt; // 3
                                                                                                                                // dp

    mF.block<3, 3>(StateIndex::P, StateIndex::P) = mI;      // 3
    mF.block<3, 3>(StateIndex::P, StateIndex::V) = dt * mI; // 3
                                                            // dv
    mF.block<3, 3>(StateIndex::V, StateIndex::R) =
        -dt * state_.R_q * Utility::SkewSymmetric(state_.R_imu * (acc_unbias)); // 3
    mF.block<3, 3>(StateIndex::V, StateIndex::V) = mI;                          // 3
    mF.block<3, 3>(StateIndex::V, StateIndex::BA) = -state_.R_q * state_.R_imu * dt;

    if (IF_IMU_MOUNT_ERROR)                                                                                                                    // 3
        mF.block<3, 3>(StateIndex::V, StateIndex::IMU_INSTALL_ANGLE) = -state_.R_q * state_.R_imu * Utility::SkewSymmetric((acc_unbias)) * dt; // 3
                                                                                                                                               // dba
    mF.block<3, 3>(StateIndex::BA, StateIndex::BA) = mI;                                                                                       // 3
                                                                                                                                               // dbg
    mF.block<3, 3>(StateIndex::BG, StateIndex::BG) = mI;                                                                                       // 3
                                                                                                                                               // d_imu install angle
    mF.block<3, 3>(StateIndex::IMU_INSTALL_ANGLE, StateIndex::IMU_INSTALL_ANGLE) = mI;
    // 3
    mF(StateIndex::WS, StateIndex::WS) = 1.0; // 1

    Eigen::MatrixXd mU = Eigen::MatrixXd::Zero(StateIndex::STATE_TOTAL, StateNoiseIndex::NOISE_TOTAL);
    // dq
    mU.block<3, 3>(StateIndex::R, StateNoiseIndex::GYRO_NOISE) = -mI * dt;        //-mI * dt;                         // 3
                                                                                  // dv
    mU.block<3, 3>(StateIndex::V, StateNoiseIndex::ACC_NOISE) = -state_.R_q * dt; //-state_.R_q * dt; // 3
                                                                                  // dba
    mU.block<3, 3>(StateIndex::BA, StateNoiseIndex::ACC_RANDOM_WALK) = mI * dt;   // 3
                                                                                  // dbg
    mU.block<3, 3>(StateIndex::BG, StateNoiseIndex::GYRO_RANDOM_WALK) = mI * dt;  // 3

    state_.P = mF * state_.P * mF.transpose() + mU * Q_ * mU.transpose();
}

   void Odometry::UpdateByWheel(WheelDataPtr wheel_data_ptr)
{
    // ROS_INFO("update by wheel");

    Eigen::Vector3d wheelSpeed = wheel_data_ptr->speed;
    //     // 计算Yaw（偏航角）
    // double yaw = atan2(state_.R_q(1, 0), state_.R_q(0, 0)); 
    
    // wheelSpeed[0]=wheelSpeed[0]* std::cos(yaw);
    // wheelSpeed[1]=wheelSpeed[0]* std::sin(yaw);
    // wheelSpeed[2]=0;
    // std::cout<<wheelSpeed[0]<<","<<wheelSpeed[1]<<","<<wheelSpeed[2]<<std::endl;

    // update
    Eigen::VectorXd r(3);                                           // 9
    r = wheelSpeed - state_.ws * (state_.R_q.inverse() * state_.v); // 3
    
    Eigen::MatrixXd mH = Eigen::MatrixXd::Zero(3, StateIndex::STATE_TOTAL);                                   // 9
    // mH.block<3, 3>(0, StateIndex::R) = Utility::SkewSymmetric(state_.ws * (state_.R_q.inverse() * state_.v)); // 3

    // mH.block<3, 3>(0, StateIndex::V) = state_.ws * state_.R_q.transpose(); // 3
     mH.block<3, 3>(0, StateIndex::R) = Eigen::Matrix3d::Identity(); // 3
    mH.block<3, 3>(0, StateIndex::V) = state_.ws * state_.R_q.transpose(); // 3

    if (IF_WS_ERROR)
        mH.block<3, 1>(0, StateIndex::WS) = state_.R_q.inverse() * state_.v; // 1

    Eigen::MatrixXd mS = mH * state_.P * mH.transpose() + whl_Rm_;
    Eigen::MatrixXd mK = state_.P * mH.transpose() * mS.inverse();
    Eigen::VectorXd dx = mK * r;

    // state_.R_q = state_.R_q * Eigen::Quaterniond(1.0, dx(StateIndex::R) / 2.0, // 2.0
    //                                          dx(StateIndex::R + 1) / 2.0, dx(StateIndex::R + 2) / 2.0); // 2.0

    const Eigen::Vector3d omg = dx.block<3, 1>(StateIndex::R, 0);         // 2.0
    Eigen::Quaterniond dq(1.0, omg(0) / 2.0, omg(1) / 2.0, omg(2) / 2.0); // 2

    if (dx.block<3, 1>(StateIndex::R, 0).norm() > DBL_EPSILON)
    {
        // dx(StateIndex::R + 2, 0) = 0;
        state_.R_q *= v_expmap(dx.block<3, 1>(StateIndex::R, 0));
        // state_.R_q = state_.R_q * dq;
    }

    // }
    state_.p += dx.segment<3>(StateIndex::P); // 3
    state_.v += dx.segment<3>(StateIndex::V); // 3
    state_.P = (Eigen::MatrixXd::Identity(StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL) - mK * mH) *
               state_.P;
    if (!parameter_lock)
    {
        state_.ba += dx.segment<3>(StateIndex::BA); // 3
        state_.bg += dx.segment<3>(StateIndex::BG); // 3
        // std::cout<<"ba bg: "<<state_.ba.transpose()<<" "<<state_.bg.transpose()<<std::endl;
        if (IF_IMU_MOUNT_ERROR && dx.block<3, 1>(StateIndex::IMU_INSTALL_ANGLE, 0).norm() > DBL_EPSILON)
        {
            state_.R_imu *= v_expmap(dx.block<3, 1>(StateIndex::IMU_INSTALL_ANGLE, 0));
            // state_.R_q = state_.R_q * dq;
        }
      
        if (IF_WS_ERROR)
            state_.ws += dx(StateIndex::WS); // 1
    }
}




void Odometry::UpdateByOdom(VIOODOMDataPtr odom_data_ptr)
{
    // ROS_INFO("update by odom");
    // std::cout<<"step1"<<std::endl;
    // std::cout<<odom_data_ptr->pose.position.x<<std::endl;
    Eigen::Vector3d odompose = Eigen::Vector3d(odom_data_ptr->pose.position.x, odom_data_ptr->pose.position.y, odom_data_ptr->pose.position.z);
    Eigen::Matrix3d odom_q = Eigen::Quaterniond(odom_data_ptr->pose.orientation.w, odom_data_ptr->pose.orientation.x, odom_data_ptr->pose.orientation.y, odom_data_ptr->pose.orientation.z).toRotationMatrix();
    
    Eigen::VectorXd r(6); // 6
    //turn rotationmatrix to quaternion
    Eigen::Matrix3d tmp_R = state_.R_q.inverse() * odom_q;
    // std::cout<<"step2"<<std::endl;
    Eigen::Quaterniond q = Eigen::Quaterniond(tmp_R);
    q.normalize();
    
    r.head(3) = LogMap(q); // 3
    r.tail(3) = odompose - state_.p;             // 3
    
    Eigen::MatrixXd mH = Eigen::MatrixXd::Zero(6, StateIndex::STATE_TOTAL); // 6
    // mH.block<3, 3>(0, StateIndex::R) = -Utility::SkewSymmetric(state_.R_q);
    mH.block<3, 3>(0, StateIndex::R) = Eigen::Matrix3d::Identity();
    mH.block<3, 3>(3, StateIndex::P) = Eigen::Matrix3d::Identity();

    Eigen::MatrixXd mS = mH * state_.P * mH.transpose() + odom_Rm_;
    Eigen::MatrixXd mK = state_.P * mH.transpose() * mS.inverse();
    Eigen::VectorXd dx = mK * r;
    // std::cout<<"step3"<<std::endl;
     if (dx.block<3, 1>(StateIndex::R, 0).norm() > DBL_EPSILON)
    {
        // dx(StateIndex::R + 2, 0) = 0;
        state_.R_q *= v_expmap(dx.block<3, 1>(StateIndex::R, 0));
        // state_.R_q = state_.R_q * dq;
    }


    // state_.R_q.normalize();
 
    // state_.R_q.normalize();
    state_.p += dx.segment<3>(StateIndex::P); // 3
    state_.v += dx.segment<3>(StateIndex::V); // 3
    state_.P = (Eigen::MatrixXd::Identity(StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL) - mK * mH) *
               state_.P;
    if (!parameter_lock)
    {
        state_.ba += dx.segment<3>(StateIndex::BA); // 3
        state_.bg += dx.segment<3>(StateIndex::BG); // 3
        // std::cout<<"ba bg: "<<state_.ba.transpose()<<" "<<state_.bg.transpose()<<std::endl;
        if (IF_IMU_MOUNT_ERROR && dx.block<3, 1>(StateIndex::IMU_INSTALL_ANGLE, 0).norm() > DBL_EPSILON)
        {
            state_.R_imu *= v_expmap(dx.block<3, 1>(StateIndex::IMU_INSTALL_ANGLE, 0));
            // state_.R_q = state_.R_q * dq;
        }
      
        if (IF_WS_ERROR)
            state_.ws += dx(StateIndex::WS); // 1
    }

}


void Odometry::UpdateByGps(GNSSDataPtr gnss_data_ptr)
{
    ROS_INFO("update by gps");
    bool return_flag = false;
    if (!init_time_)
    {
        init_time_ = gnss_data_ptr->timestamp;
    }
    // std::cout << "init_time " <<init_time_<<std::endl;

    // gnss_count++;
    // if (gnss_count < 10)
    // {
    //     return_flag = true;
    // }
    // else
    // {
    //     gnss_count = 0;
    // }
    Eigen::Vector3d gpsPosition, lla;

    // lla<<gps_msg->pose.position.x, gps_msg->pose.position.y, gps_msg->pose.position.z;
    // gpsPosition = lla;
    Utility::convert_lla_to_enu(init_lla_, gnss_data_ptr->lla, &gpsPosition);

    Eigen::VectorXd r(3);       // 9
    r = gpsPosition - state_.p; // 3

    Eigen::MatrixXd mH = Eigen::MatrixXd::Zero(3, StateIndex::STATE_TOTAL); // 9
    mH.block<3, 3>(0, StateIndex::P) = Eigen::Matrix3d::Identity();         // 3
    Eigen::Matrix3d V = gnss_data_ptr->cov;

    Eigen::MatrixXd mS = mH * state_.P * mH.transpose() + gps_Rm_;
    Eigen::MatrixXd mK = state_.P * mH.transpose() * (mH * state_.P * mH.transpose() + V).inverse();
    Eigen::VectorXd dx = mK * r;

    // state_.R_q = state_.R_q * Eigen::Quaterniond(1.0, dx(StateIndex::R) / 2.0, // 2.0
    //                                          dx(StateIndex::R + 1) / 2.0, dx(StateIndex::R + 2) / 2.0); // 2.0

    const Eigen::Vector3d omg = dx.block<3, 1>(StateIndex::R, 0);         // 2.0
    Eigen::Quaterniond dq(1.0, omg(0) / 2.0, omg(1) / 2.0, omg(2) / 2.0); // 2

    if (dx.block<3, 1>(StateIndex::R, 0).norm() > DBL_EPSILON)
    {
        state_.R_q *= v_expmap(dx.block<3, 1>(StateIndex::R, 0));
        // state_.R_q = state_.R_q * dq;
    }

    // state_.R_q.normalize();
    // state_.p += dx.segment<3>(StateIndex::P); // 3
    state_.v += dx.segment<3>(StateIndex::V); // 3
    state_.P = (Eigen::MatrixXd::Identity(StateIndex::STATE_TOTAL, StateIndex::STATE_TOTAL) - mK * mH) *
               state_.P;

    if (!parameter_lock)
    {
        state_.ba += dx.segment<3>(StateIndex::BA); // 3
        state_.bg += dx.segment<3>(StateIndex::BG);
        if (IF_IMU_MOUNT_ERROR && dx.block<3, 1>(StateIndex::IMU_INSTALL_ANGLE, 0).norm() > DBL_EPSILON)
        {
            state_.R_imu *= v_expmap(dx.block<3, 1>(StateIndex::IMU_INSTALL_ANGLE, 0));
            // state_.R_q = state_.R_q * dq;
        }
        // if(dx(StateIndex::WS) > 1e-6)
        // {
        //     state_.ws += dx(StateIndex::WS); // 1
        // }
        // 3
        if (IF_WS_ERROR)
            state_.ws += dx(StateIndex::WS); // 1
    }
}


// 估计相机位姿s
// bool odom_need_init = false;
Eigen::Matrix4d Odometry::estimatePoseICP(const TEXTDATA&ptdata,const TEXTDATA&cdata) {
    // std::cout<<"enterpicp"<<std::endl;
    std::vector<Eigen::Vector3d>  prevPoints;
    std::vector<Eigen::Vector3d>  currPoints;
    //  std::cout<<ptdata.spot.corners.size()<<std::endl;
    //   std::cout<<cdata.spot.corners.size()<<std::endl;
    //    std::cout<<ptdata.ocrPoints.textcorners.size()<<std::endl;
    // std::cout<<cdata.ocrPoints.textcorners.size()<<std::endl;
    for (size_t i = 0; i < ptdata.spots.size(); ++i) {
        prevPoints.push_back(ptdata.spots[i].corners[0]);
        prevPoints.push_back(ptdata.spots[i].corners[1]);
        prevPoints.push_back(ptdata.spots[i].corners[2]);
        prevPoints.push_back(ptdata.spots[i].corners[3]);
        // std::cout<<"pre"<<std::endl;
    }
    for (size_t i = 0; i < cdata.spots.size(); ++i) {
        currPoints.push_back(cdata.spots[i].corners[0]);
        currPoints.push_back(cdata.spots[i].corners[1]);
        currPoints.push_back(cdata.spots[i].corners[2]);
        currPoints.push_back(cdata.spots[i].corners[3]);
        // std::cout<<"cur"<<std::endl;
    }
        for (size_t i = 0; i < ptdata.ocrPoints.size(); ++i) {
        prevPoints.push_back(ptdata.ocrPoints[i].textcorners[0]);
        prevPoints.push_back(ptdata.ocrPoints[i].textcorners[1]);
        // std::cout<<"pre1"<<std::endl;
    }
        for (size_t i = 0; i < cdata.ocrPoints.size(); ++i) {
            // std::cout<<"cur1"<<std::endl;
        currPoints.push_back(cdata.ocrPoints[i].textcorners[0]);
        currPoints.push_back(cdata.ocrPoints[i].textcorners[1]);
    }
    // computepose(prevPoints,currPoints);
std::cout<<prevPoints.size()<<std::endl;
// std::cout<<currPoints.size()<<std::endl;
    if (prevPoints.size() != currPoints.size() || prevPoints.size() < 3) {
        std::cout << "Error: Point cloud size mismatch or insufficient points." << std::endl;
        IF_USE_ODOM=false;
        odom_need_init=true;
        return Eigen::Matrix4d::Identity();
    }
    IF_USE_ODOM=true;
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
        computeTransformation(prevPoints, currPoints, R, t);

        // 3. 更新当前点云位置
        // 应用当前的R和t到prevPoints上，生成新的currPoints（待更新的当前点集）
        for (size_t i = 0; i < prevPoints.size(); ++i) {
            prevPoints[i] = R * prevPoints[i] + t; // 更新点
        }

        // 4. 判断收敛
            // 计算质心变化的范数
        // double centroidChangeNorm = t.norm();
        // if (centroidChangeNorm < threshold) {
        //     std::cout << "ICP has converged." <<centroidChangeNorm<<std::endl;
        //  } 
        //  else {
        //     std::cout << "ICP is still converging. Centroid change: " << centroidChangeNorm << std::endl;
        // }
                // Update R and t
        // R = Rk * R;
        // t = Rk * t + tk;
        vpose.R=preR*R;
        vpose.t=preR*t+prevt;
        double centroidChangeNorm = (t).norm();
        if (centroidChangeNorm < convergenceThreshold) {
            // std::cout << "centroidChangeNorm " <<centroidChangeNorm<< std::endl;
            break;
        }
        else{
            //  std::cout << "ICP has converged." <<centroidChangeNorm<<std::endl;
            //    std::cout << "ICP has converged." <<centroidChangeNorm<<std::endl;
        }
    }
    // computeTransformation(prevPoints, currPoints, R, t);

    // t=prevt;
    // R=preR;
    // vpose.t=t;
    // vpose.R=R;

    Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
    
    pose.block<3, 3>(0, 0) = vpose.R;
    pose.block<3, 1>(0, 3) = vpose.t;
    // cout<<"tbefore:" << vpose.t<<endl;
    vpose.t[1]=vpose.t[1];
    double yaw_pose = std::atan2(vpose.R(1, 0), vpose.R(0, 0));
    // cout<<"yaw:"<<yaw_pose<<endl;
    // cout<<"t:" << vpose.t<<endl;
    //  drawline(pose);
    if(odom_need_init&&stable>=20){
        current_pose.block<3, 3>(0, 0)=state_.R_q;
        current_pose.block<3, 1>(0, 3)=state_.p;
        odom_need_init=false;
        cout<<"intialodom"<<endl;
        stable=0;
    }
    // std::cout<<state_.p<<std::endl;
    current_pose = current_pose * pose;
    Eigen::Vector3d position = current_pose.block<3, 1>(0, 3);
    // vpose.t=current_pose.block<3, 1>(0, 3);
    // vpose.R=current_pose.block<3, 3>(0, 0);
    viodata.pose.position.x = position(0);
    viodata.pose.position.y = position(1);
    viodata.pose.position.z = position(2);

    // 提取旋转矩阵并转换为四元数
    Eigen::Quaterniond quaternion(current_pose.block<3, 3>(0, 0));
    viodata.pose.orientation.w = quaternion.w();
    viodata.pose.orientation.x = quaternion.x();
    viodata.pose.orientation.y = quaternion.y();
    viodata.pose.orientation.z = quaternion.z();
    viodata.timestamp=cdata.timestamp;
    if(!odom_need_init)
    {vio_process( viodata);}
    pose.block<3, 3>(0, 0) = current_pose.block<3, 3>(0, 0);
    pose.block<3, 1>(0, 3) = current_pose.block<3, 1>(0, 3);
    drawline(pose);
    Eigen::MatrixXd Q1= Eigen::MatrixXd::Identity(6, 6) * 0.01;
    // ekfloop.predict(vpose.t,quaternion,Q1);
    // update(pose);
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
    // std::cout << "Error: Point cloud size mismatch or insufficient points." << std::endl;
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
    t=t;
    

    //     // 收敛判断
    // return;
    // double threshold = 1e-6; // 收敛阈值
    // double prevCentroidNorm = prevCentroid.norm();
    // double currCentroidNorm = currCentroid.norm();
    
    // // 计算质心变化的范数
    // double centroidChangeNorm = (currCentroid - R * prevCentroid).norm();

    // if (centroidChangeNorm < threshold) {
    //     std::cout << "ICP has converged." << std::endl;
    // } else {
    //     std::cout << "ICP is still converging. Centroid change: " << centroidChangeNorm << std::endl;
    // }

    

}

// void Odometry::matchpoint(const std::vector<cv::Point2f> & currentFrameSpots,const std::vector<cv::Point2f> & currentFrameSpots_ocr)
// {
    
// }

// Function to match the same parking spot between two frames
void Odometry::matchframes(const TEXTDATA &ctextdata)
 {
    // std::cout<<"entermatch"<<std::endl;
    TEXTDATA Curtextdata,pretextdata;//存储匹配点
    // int m=lasttextdata.spot.corners.size()+lasttextdata.ocrPoints.textcorners.size();
    // int n=ctextdata.spot.corners.size()+ctextdata.ocrPoints.textcorners.size();
    int m=lasttextdata.spots.size()+lasttextdata.ocrPoints.size();
    int n=ctextdata.spots.size()+ctextdata.ocrPoints.size();
    //  cout<<m<<","<<n<<endl;
    int mp=lasttextdata.spots.size();int mc=lasttextdata.ocrPoints.size();
    int np=ctextdata.spots.size();int nc=ctextdata.ocrPoints.size();
        if (isFirstFrame|| m==0||n==0) {
        lasttextdata=ctextdata;
       
        isFirstFrame = false;
       odom_need_init=true;
        // std::cout<<"intial"<<std::endl;
        return; // 第一帧没有足够的信息进行里程计计算
    }
   
   stable++;
   if(mp!=0&&np!=0){
     std::vector<std::vector<double>> cost_matrix(lasttextdata.spots.size(), std::vector<double>(ctextdata.spots.size(), 10000.0));
                    // Check if the positions are close enough
    float distanceThreshold = 0.75; // Define an appropriate threshold
                //   std::cout<<"pre:"<<lasttextdata.spot.corners.size()<<std::endl;
                // std::cout<<"cur:"<<ctextdata.spot.corners.size()<<std::endl;
    for (size_t i = 0; i < lasttextdata.spots.size(); ++i) {
        for (size_t j = 0; j < ctextdata.spots.size(); ++j) {
            // std::cout<<"5"<<std::endl;
            Eigen::Vector3d center_last=(lasttextdata.spots[i].corners[0]+lasttextdata.spots[i].corners[1])/2;
            Eigen::Vector3d center_cur=(ctextdata.spots[j].corners[0]+ctextdata.spots[j].corners[1])/2;
            double distance = (center_last - center_cur).norm();
            if (distance <= distanceThreshold) {
                // std::cout<<"pre:"<<preFramespot[i].x<<","<<preFramespot[i].y<<std::endl;
                // std::cout<<"cur:"<<currentFrameSpots[j].x<<","<<currentFrameSpots[j].y<<std::endl;
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
    // std::cout<<"before1:"<<Assignment.size()<<std::endl;
    double cost = hungarian.Solve(cost_matrix, Assignment);
    // std::cout<<"after1:"<<Assignment.size()<<std::endl;
    
    for(int i = 0; i < Assignment.size(); ++i) {
        int match_index = Assignment[i];
        if(Assignment[i] != -1&&cost_matrix[i][Assignment[i]] <=0.75) 
        {
            Curtextdata.spots.push_back(ctextdata.spots[match_index]);
            pretextdata.spots.push_back(lasttextdata.spots[i]);
            // j++;
            //   std::cout<<"2"<<std::endl;
        }

     }
   }

    
    // ocr点匹配
    if(mc!=0&&nc!=0){
     std::vector<std::vector<double>> cost_matrix_ocr(lasttextdata.ocrPoints.size(), std::vector<double>(ctextdata.ocrPoints.size(), 10000.0));
    //  std::cout<<preFramespot_ocr.size()<<std::endl;
                    // Check if the positions are close enough
    float distanceThreshold_ocr = 0.75; // Define an appropriate threshold
                // std::cout<<"pre:"<<lasttextdata.ocrPoints.textcorners.size()<<std::endl;
                // std::cout<<"cur:"<<ctextdata.ocrPoints.textcorners.size()<<std::endl;
    for (size_t i = 0; i < lasttextdata.ocrPoints.size(); ++i) {
        for (size_t j = 0; j < ctextdata.ocrPoints.size(); ++j) {
            // std::cout<<"5"<<std::endl;
            Eigen::Vector3d textc1=(lasttextdata.ocrPoints[i].textcorners[0]+lasttextdata.ocrPoints[i].textcorners[1])/2;
            Eigen::Vector3d textc2=(ctextdata.ocrPoints[j].textcorners[0]+ctextdata.ocrPoints[j].textcorners[1])/2;
            double distance_ocr = (textc2 - textc1).norm();
            if (distance_ocr <= distanceThreshold_ocr) {
    
            cost_matrix_ocr[i][j] = distance_ocr; } 
            else {
            //  std::cout<<"enterif2"<<std::endl;
            cost_matrix_ocr[i][j] = std::numeric_limits<double>::max(); // 使用一个大的值表示不可能的匹配
            //  std::cout<<"7"<<std::endl;
            //  std::cout<<"entermax"<<std::endl;
            }
        }
        //  std::cout<<"out"<<std::endl;
    }
    //    std::cout<<"ocr"<<std::endl;
    HungarianAlgorithm hungarian_ocr;
    std::vector<int> Assignment_ocr;
    
    // std::cout<<"before:"<<Assignment_ocr.size()<<std::endl;
    double cost_ocr = hungarian_ocr.Solve(cost_matrix_ocr, Assignment_ocr);
    // std::cout<<"after:"<<Assignment_ocr.size()<<std::endl;
    for(int i = 0; i < Assignment_ocr.size(); ++i) {
        int match_index_ocr = Assignment_ocr[i];
        if(Assignment_ocr[i] != -1&&cost_matrix_ocr[i][Assignment_ocr[i]] <= 0.75) 
        {
            Curtextdata.ocrPoints.push_back(ctextdata.ocrPoints[match_index_ocr]);
            pretextdata.ocrPoints.push_back(lasttextdata.ocrPoints[i]);
            // Curtextdata.ocrPoints.push_back(ctextdata.ocrPoints.ocrdata[match_index_ocr]);
            // pretextdata.ocrPoints.push_back(lasttextdata.ocrPoints.ocrdata[i]);
            //   std::cout<<"2"<<std::endl;
        }

     }
    }
     Curtextdata.timestamp=ctextdata.timestamp;
     pretextdata.timestamp=pretextdata.timestamp;
    // for (const auto& point : cFrameSpots) {
    //     // 假设点在地面上，z 坐标为 0
    //     curvehiclePoints.push_back(Eigen::Vector3d(point.x, -point.y, 0.0));
    // }

    // for (const auto& point : preFrameSpots) {
    //     // 假设点在地面上，z 坐标为 0
    //     prevehiclePoints.push_back(Eigen::Vector3d(point.x, -point.y, 0.0));
    // }

    //     for (const auto& point : cFrameSpots_ocr) {
    //     // 假设点在地面上，z 坐标为 0
    //     curvehiclePoints.push_back(Eigen::Vector3d(point.x, -point.y, 0.0));
    // }

    // for (const auto& point : preFrameSpots_ocr) {
    //     // 假设点在地面上，z 坐标为 0
    //     prevehiclePoints.push_back(Eigen::Vector3d(point.x, -point.y, 0.0));
    // }
    //  std::cout<<Curtextdata.ocrPoints.textcorners.size()<<","<<pretextdata.ocrPoints.textcorners.size()<<std::endl;
    
    estimatePoseICP(pretextdata,Curtextdata);
    lasttextdata=ctextdata;
   
}


/*void Odometry::piextocamera(const parking_slot_detection::gcn_parking & srv)
{
    double association_distance=0.5;
    TEXTDATA textdata;
    std::vector<cv::Point2f> currvehiclePoints;
    std::vector<cv::Point2f> currvehiclePoints_ocr;
    // std::cout<<"enterpiexto"<<std::endl;
     for (size_t i = 0; i < srv.response.point0_x.size(); ++i){
        // std::cout<<srv.response.text<<":"<<-256+srv.response.point0_x[i]<<","<<-256+srv.response.point0_y[i]<<std::endl;
        currvehiclePoints.push_back(cv::Point2d((-256+srv.response.point0_x[i])/51.2, (-256+srv.response.point0_y[i])/57.6));
        currvehiclePoints.push_back(cv::Point2d((-256+srv.response.point1_x[i])/51.2, (-256+srv.response.point1_y[i])/57.6));
     }
    for (size_t j = 0; j < srv.response.ocrpointx1.size(); ++j){
        std::cout<<srv.response.texts[j]<<":"<<srv.response.ocrpointx1[j]<<","<<srv.response.ocrpointy1[j]<<std::endl;
        std::cout<<srv.response.texts[j]<<":"<<-256+srv.response.ocrpointx1[j]<<","<<-256+srv.response.ocrpointy1[j]<<std::endl;
        currvehiclePoints_ocr.push_back(cv::Point2d((-256+srv.response.ocrpointx1[j])/51.2, (-256+srv.response.ocrpointy1[j])/57.6));
        currvehiclePoints_ocr.push_back(cv::Point2d((-256+srv.response.ocrpointx2[j])/51.2, (-256+srv.response.ocrpointy2[j])/57.6));
    }
    // std::vector<Eigen::Vector3d> vehiclePoints;

    // for (const auto& point : currvehiclePoints) {
    //     // 假设点在地面上，z 坐标为 0
    //     vehiclePoints.push_back(Eigen::Vector3d(point.x, point.y, 0.0));
    // }
    matchframes(currvehiclePoints,currvehiclePoints_ocr);
    
}*/


void Odometry::process_synced_images(const sensor_msgs::CompressedImageConstPtr&avm_msg,const sensor_msgs::CompressedImageConstPtr& front_msg)
{
    canvasodom.setTo(cv::Scalar(255, 255, 255));
     try
    {
        // 解压缩图像
        cv::Mat image = cv::imdecode(cv::Mat(avm_msg->data),cv::IMREAD_COLOR);
        cv::Mat image_front = cv::imdecode(cv::Mat(front_msg->data),cv::IMREAD_COLOR);
    //     // 确保图像尺寸符合预期
    //     int original_width = image.cols;
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

    // // 显示裁剪后的图像
    //     cv::resize(cropped_image, cropped_image, cv::Size(512, 512));
        // cv::imshow(windowName1, image);
        // cv::waitKey(1);

        // 转换为ROS图像消息
        sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        sensor_msgs::ImagePtr frontimage_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_front).toImageMsg();
       
        // 创建ROS服务客户端
        // ros::NodeHandle mh;
        // ros::ServiceClient client = mh.serviceClient<parking_slot_detection::gcn_parking>("gcn_service");

        // 创建请求和响应消息
        // parking_slot_detection::gcn_parking srv;
        srv.request.image_data = *image_msg;
        srv_plate.request.image= *frontimage_msg;
    // while(ros::ok()){
        // 发送请求
        if (client.call(srv))
        {    
        // double association_distance=0.6;
        // std::cout<<"spot:"<<srv.response.point0_x.size()<<std::endl;
        TEXTDATA textdata;
        std::vector<Eigen::Vector3d> currvehiclePoints;
        std::vector<Eigen::Vector3d> currvehiclePoints_ocr;
        std::vector<Point3D> worldpts3D;
        // for (size_t i = 0; i < srv_plate.response.plate_numbers.size(); ++i)
        // {
        //     LicensePlate plate;
        //         // const auto& corners = srv.response.corners[i];
        //         double x1=srv_plate.response.corners_x1[i],y1=srv_plate.response.corners_y1[i];
        //         double x2=srv_plate.response.corners_x2[i],y2=srv_plate.response.corners_y2[i];
        //         // 车牌角点 3D 世界坐标（假设车牌平面在世界坐标系的 Z = 0 平面上）
        //     if(srv_plate.response.plate_numbers[i].length() == 7)
        //     {
        //         worldpts3D = {
        //         {-0.22, 0.07, 0}, {0.22, 0.07, 0}, {0.22, -0.07, 0}, {-0.22, -0.07, 0} // 假设车牌为蓝牌
        //      };    
        //     }
        //     else{
        //           worldpts3D = {
        //         {-0.24, 0.07, 0}, {0.24, 0.07, 0}, {0.24, -0.07, 0}, {-0.24, -0.07, 0} // 假设车牌为绿牌
        //     };}
        //     // 车牌角点 2D 图像坐标
        //     std::vector<Point2D> pts2D={{x1,y1},{x2,y1},{x2,y2},{x1,y2}};
        //     std::vector<Point3D> camera_coords;
        //     p3p.P3PComputePoses(worldpts3D, pts2D,camera_coords);
        //     plate.plateNumber=srv_plate.response.plate_numbers[i];
        //     plate.points.push_back(camera_coords[0] );
        //     plate.points.push_back(camera_coords[1]);
        //     plate.points.push_back(camera_coords[2]);
        //     plate.points.push_back(camera_coords[3]);
        //     plate.confidence=srv_plate.response.confidence_lic[i];
        //     plate.timestamp=front_msg->header.stamp.toSec();
        //     textdata.licplate.push_back(plate);
        //     plate.points.clear();
        // }
        // std::cout<<"spot:"<<srv.response.point0_y.size()<<","<<"text:"<<srv.response.ocrpointx1.size()<<std::endl;
        // std::cout<<"enterpiexto"<<std::endl;
        for (size_t i = 0; i < srv.response.point0_x.size(); ++i){
        // std::cout<<"x0:"<<-256+srv.response.point0_x[i]<<","<<-256+srv.response.point0_y[i]<<std::endl;
        //  std::cout<<"x1:"<<-256+srv.response.point1_x[i]<<","<<-256+srv.response.point1_y[i]<<std::endl;
            // currvehiclePoints.push_back(Eigen::Vector3d(-(-256+srv.response.point0_x[i])/50.08, (-256+srv.response.point0_y[i])/48.99,0));
            // currvehiclePoints.push_back(Eigen::Vector3d(-(-256+srv.response.point1_x[i])/50.08, (-256+srv.response.point1_y[i])/48.99,0));
            //  currvehiclePoints.push_back(Eigen::Vector3d(-(-256+srv.response.point2_x[i])/50.08, (-256+srv.response.point2_y[i])/48.99,0));
            // currvehiclePoints.push_back(Eigen::Vector3d(-(-256+srv.response.point3_x[i])/50.08, (-256+srv.response.point3_y[i])/48.99,0));
            //  std::cout<<"spot"<<-256+srv.response.point0_y[i]<<","<<-256+srv.response.point3_y[i]<<std::endl;
            
            double dis=(abs(srv.response.point3_x[i]-srv.response.point1_x[i]))/50.08;
            // std::cout<<"width:"<<(srv.response.point0_y[i]-srv.response.point3_y[i])/48.99<<std::endl;
            //    std::cout<<"length:"<<(srv.response.point3_x[i]-srv.response.point1_x[i])/50.08<<std::endl;
            //      std::cout<<"width:"<<(srv.response.point0_y[i]-srv.response.point1_y[i])/48.99<<std::endl;
            double dis1=(abs(srv.response.point1_y[i]-srv.response.point0_y[i]))/48.99;
            // std::cout<<"dis:"<<dis<<",dis1:"<<dis1<<std::endl;
            if(dis1>=1.5&&dis>3.8){
            Spot nspots;
            nspots.corners.push_back(Eigen::Vector3d((-256+srv.response.point0_y[i])/48.99, -(-256+srv.response.point0_x[i])/50.08,0));
            nspots.corners.push_back(Eigen::Vector3d((-256+srv.response.point1_y[i])/48.99, -(-256+srv.response.point1_x[i])/50.08,0));
            nspots.corners.push_back(Eigen::Vector3d((-256+srv.response.point2_y[i])/48.99, -(-256+srv.response.point2_x[i])/50.08,0));
            nspots.corners.push_back(Eigen::Vector3d((-256+srv.response.point3_y[i])/48.99, -(-256+srv.response.point3_x[i])/50.08,0));
            //      nspots.corners.push_back(Eigen::Vector3d((-256+srv.response.point0_x[i])/50.08, -(-256+srv.response.point0_y[i])/48.99,0));
            // nspots.corners.push_back(Eigen::Vector3d((-256+srv.response.point1_x[i])/50.08, -(-256+srv.response.point1_y[i])/48.99,0));
            // nspots.corners.push_back(Eigen::Vector3d((-256+srv.response.point2_x[i])/50.08, -(-256+srv.response.point2_y[i])/48.99,0));
            // nspots.corners.push_back(Eigen::Vector3d((-256+srv.response.point3_x[i])/50.08, -(-256+srv.response.point3_y[i])/48.99,0));
            textdata.spots.push_back(nspots);
            nspots.corners.clear();
            // cout<<textdata.spots.size()<<endl;
            //       currvehiclePoints.push_back(Eigen::Vector3d((-256+srv.response.point0_x[i])/50.08, -(-256+srv.response.point0_y[i])/48.99,0));
            // currvehiclePoints.push_back(Eigen::Vector3d((-256+srv.response.point1_x[i])/50.08, -(-256+srv.response.point1_y[i])/48.99,0));
            //  currvehiclePoints.push_back(Eigen::Vector3d((-256+srv.response.point2_x[i])/50.08, -(-256+srv.response.point2_y[i])/48.99,0));
            // currvehiclePoints.push_back(Eigen::Vector3d((-256+srv.response.point3_x[i])/50.08, -(-256+srv.response.point3_y[i])/48.99,0));
            }
        //    std::cout<<"2"<<std::endl;
        }
        for (size_t j = 0; j < srv.response.ocrpointx1.size(); ++j){
            // std::cout<<srv.response.texts[j]<<":"<<(-330+srv.response.ocrpointy1[j])/63.15<<std::endl;
            // std::cout<<srv.response.texts[j]<<":"<<(-330+srv.response.ocrpointx1[j])/64.56<<std::endl;
            // std::cout<<srv.response.texts[j]<<std::endl;
            OCRtext ocrtext;
            // textdata.ocrPoints.ocrdata.push_back({srv.response.texts[j],Eigen::Vector3d((-256+srv.response.ocrpointx1[j])/50.08, -(-256+srv.response.ocrpointy1[j])/48.99,0)});
            // textdata.ocrPoints.ocrdata.push_back({srv.response.texts[j],Eigen::Vector3d((-256+srv.response.ocrpointx2[j])/50.08, -(-256+srv.response.ocrpointy2[j])/48.99,0)});
            ocrtext.ocrdata.push_back({srv.response.texts[j],Eigen::Vector3d((-330+srv.response.ocrpointy1[j])/63.15, -(-330+srv.response.ocrpointx1[j])/64.56,0)});
            ocrtext.ocrdata.push_back({srv.response.texts[j],Eigen::Vector3d((-330+srv.response.ocrpointy2[j])/63.15, -(-330+srv.response.ocrpointx2[j])/64.56,0)});
            // ocrtext.ocrdata.push_back({srv.response.texts[j],Eigen::Vector3d((-330+srv.response.ocrpointx1[j])/64.56, -(-330+srv.response.ocrpointy1[j])/63.15,0)});
            // ocrtext.ocrdata.push_back({srv.response.texts[j],Eigen::Vector3d((-330+srv.response.ocrpointx2[j])/64.56, -(-330+srv.response.ocrpointy2[j])/63.15,0)});
            // std::cout<<srv.response.texts[j]<<":"<<(-256+srv.response.ocrpointx2[j])/63.15<<","<<(-256+srv.response.ocrpointy2[j])/64.56<<std::endl;
            // std::cout<<srv.response.texts[j]<<":"<<(-256+srv.response.ocrpointx1[j])/63.15<<","<<(-256+srv.response.ocrpointy1[j])/64.56<<std::endl;
            // currvehiclePoints_ocr.push_back(Eigen::Vector3d((-256+srv.response.ocrpointx1[j])/50.08, -(-256+srv.response.ocrpointy1[j])/48.99,0));
            // currvehiclePoints_ocr.push_back(Eigen::Vector3d((-256+srv.response.ocrpointx2[j])/50.08, -(-256+srv.response.ocrpointy2[j])/48.99,0));
            ocrtext.textcorners.push_back(Eigen::Vector3d((-330+srv.response.ocrpointy1[j])/63.15,-(-330+srv.response.ocrpointx1[j])/64.56,0));
            ocrtext.textcorners.push_back(Eigen::Vector3d((-330+srv.response.ocrpointy2[j])/63.15, -(-330+srv.response.ocrpointx2[j])/64.56,0));
            // ocrtext.textcorners.push_back(Eigen::Vector3d((-330+srv.response.ocrpointx1[j])/64.56, -(-330+srv.response.ocrpointy1[j])/63.15,0));
            // ocrtext.textcorners.push_back(Eigen::Vector3d((-330+srv.response.ocrpointx2[j])/64.56, -(-330+srv.response.ocrpointy2[j])/63.15,0));
            textdata.ocrPoints.push_back(ocrtext);
            ocrtext.textcorners.clear();
        }
            textdata.timestamp=front_msg->header.stamp.toSec();
            // textdata.spot.corners=currvehiclePoints;
            // textdata.ocrPoints.textcorners=currvehiclePoints_ocr;
            int m= textdata.spots.size();int n= textdata.ocrPoints.size();
            if(m!=0&&n!=0)
                matchframes(textdata);

        }
        else {
            ROS_ERROR("Failed to call service");
        }
    }
    // }

    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", avm_msg->format.c_str());
    }


}

void Odometry::imuCallback_odom(const sensor_msgs::ImuConstPtr &imu_msg)
{

     IMUDataPtr imu_data_ptr = std::make_shared<IMUData>();
    imu_data_ptr->timestamp = imu_msg->header.stamp.toSec();
    // imu_data_ptr->acc[0] = imu_msg->linear_acceleration.x;
    // imu_data_ptr->acc[1] = imu_msg->linear_acceleration.y;
    // imu_data_ptr->acc[2] = imu_msg->linear_acceleration.z;
    // imu_data_ptr->gyro[0] = imu_msg->angular_velocity.x;
    // imu_data_ptr->gyro[1] = imu_msg->angular_velocity.y;
    // imu_data_ptr->gyro[2] = imu_msg->angular_velocity.z;

    imu_data_ptr->acc[0] = imu_msg->linear_acceleration.x;
    imu_data_ptr->acc[1] = -1.0 * imu_msg->linear_acceleration.y;
    imu_data_ptr->acc[2] = imu_msg->linear_acceleration.z;
    imu_data_ptr->gyro[0] = imu_msg->angular_velocity.x;
    imu_data_ptr->gyro[1] = -1.0 * imu_msg->angular_velocity.y;
    imu_data_ptr->gyro[2] = imu_msg->angular_velocity.z;

    // ROS_INFO("deal with imu. time stamp: %lf", imu_msg->header.stamp.toSec());

    process_IMU_Data(imu_data_ptr);
    // if (!stateInit_)
    //     return;

    if (!stateInit_)
    {


        if (imu_buf_.size() < IMU_BUF_SIZE)
        {
            std::cout << "[ ESKF ] Wait. Insufficient IMU data." << std::endl;
            return;
        }

        // ROS_INFO("imu_buf_ size %d", imu_buf_.size());
        last_imu_ptr_ = imu_buf_.back();
        
        // ROS_INFO("last_imu_ptr_ timestamp %lf", last_imu_ptr_->timestamp);

        if (!InitState())
            return;
        // lla << gps_msg->latitude, gps_msg->longitude, gps_msg->altitude;
        // convert_lla_to_enu(init_lla_,lla, &gpsPosition);

        stateInit_ = true;
    }
    
    publish();

}

void Odometry::WheelCallback(const parking_slot_detection::SpeedFeedbackConstPtr &wheel_msg)
// void EkfEstimator::WheelCallback(const diankong::VehicleFeedbackConstPtr &wheel_msg)
{
    // std::cout << "[ ESKF ] Wheel data." << std::endl;
    if (!IF_USE_WHL)
    {
        // std::cout<<"[ ESKF ] Wheel data."<<std::endl;
        return;
    }
    // ROS_INFO("step 1 ");
    
    WheelDataPtr wheel_data_ptr = std::make_shared<WHLData>();
    wheel_data_ptr->timestamp = wheel_msg->header.stamp.toSec();
    wheel_data_ptr->speed[0] = wheel_msg->speed_cms / 100.0;
          if (wheel_msg->gear == 9)
    {
        wheel_data_ptr->speed[0] = - wheel_msg->speed_cms / 100.0;
    }
    wheel_data_ptr->speed[1] = 0;
    wheel_data_ptr->speed[2] = 0;
    // std::cout << "Wheel data1." << std::endl;


    if (!stateInit_)
    {
        // std::cout << "[ ESKF ] Wait. Insufficient IMU data." << std::endl;
        return;
    }

    if (std::abs(last_imu_ptr_->timestamp - wheel_msg->header.stamp.toSec()) > 0.2)
    {
        // std::cout << "[ ESKF ] WHL and IMU are not sychonized." << std::endl;
        return;
    }
    // ROS_INFO("step 2 ");
    // ROS_INFO("deal with wheel. time stamp: %lf", wheel_msg->header.stamp.toSec());
    UpdateByWheel(wheel_data_ptr);

}


void Odometry::vio_process(const VIOODOMData& viodata)
{
    if (!IF_USE_ODOM)
    {
        // std::cout<<"[ ESKF ] Wheel data."<<std::endl;
        return;
    }
    if (!stateInit_)
    {
        // std::cout << "[ ESKF ] Wait. Insufficient IMU data." << std::endl;
        return;
    }

    // if (std::abs(last_imu_ptr_->timestamp - lidar_odom_msg->header.stamp.toSec()) > 0.2)
    // {
    //     std::cout << "[ ESKF ] ODOM and IMU are not sychonized." << std::endl;
    //     return;
    // }

    // std::cout<<"LidarOdomCallback"<<std::endl;
    VIOODOMDataPtr odom_data_ptr = std::make_shared<VIOODOMData>();
    odom_data_ptr->timestamp = viodata.timestamp;
    odom_data_ptr->pose=viodata.pose;
    
    // if(IF_USE_ODOM) {

        
    // // 提取当前帧的平移向量
        
    //     state_.R_q = Eigen::Quaterniond(odom_data_ptr->pose.orientation.w, odom_data_ptr->pose.orientation.x, odom_data_ptr->pose.orientation.y, odom_data_ptr->pose.orientation.z).toRotationMatrix();
    //     state_.p = Eigen::Vector3d(odom_data_ptr->pose.position.x, odom_data_ptr->pose.position.y, odom_data_ptr->pose.position.z);
    //     // ekf_need_init = false;
    // }

    // std::cout<<odom_data_ptr->timestamp<<std::endl;
    // std::cout<<odom_data_ptr->pose.position.x<<std::endl;
    UpdateByOdom(odom_data_ptr);
}

bool ekf_need_init = true;
    void Odometry::GpsCallback(const sensor_msgs::NavSatFixConstPtr &gps_msg)
{
       bool return_flag = false;
    if (!init_time_)
    {
        init_time_ = gps_msg->header.stamp.toSec();
    }

    GNSSDataPtr gnss_data_ptr = std::make_shared<GNSSData>();
    gnss_data_ptr->timestamp = gps_msg->header.stamp.toSec();
    gnss_data_ptr->lla[0] = gps_msg->latitude;
    gnss_data_ptr->lla[1] = gps_msg->longitude;
    gnss_data_ptr->lla[2] = gps_msg->altitude;
    gnss_data_ptr->cov = Eigen::Map<const Eigen::Matrix3d>(gps_msg->position_covariance.data());

    if (gps_msg->status.status != 2 )
    {
        parameter_lock = true;
        std::cout << "[ ESKF ] Bad GNSS data." << std::endl;
        return;
    }
    
    // parameter_lock = false;
    // ROS_INFO("step 1 ");
    
    // if (!stateInit_)
    // {

    //     if (imu_buf_.size() < IMU_BUF_SIZE)
    //     {
    //         std::cout << "[ ESKF ] Wait. Insufficient IMU data." << std::endl;
    //         return;
    //     }

    //     // ROS_INFO("imu_buf_ size %d", imu_buf_.size());
    //     last_imu_ptr_ = imu_buf_.back();
    //     // ROS_INFO("last_imu_ptr_ timestamp %lf", last_imu_ptr_->timestamp);
    //     if (std::abs(last_imu_ptr_->timestamp - gps_msg->header.stamp.toSec()) > 0.1)
    //     {
    //         std::cout << "[ ESKF ] GNSS and IMU are not sychonized." << std::endl;
    //         return;
    //     }
    //     if (!InitState())
    //         return;
    //     // lla << gps_msg->latitude, gps_msg->longitude, gps_msg->altitude;
    //     // convert_lla_to_enu(init_lla_,lla, &gpsPosition);

    //     init_lla_ = gnss_data_ptr->lla;
    //     stateInit_ = true;
    //     ROS_INFO("init lla %f %f %f", init_lla_[0], init_lla_[1], init_lla_[2]);
    // }
    // ROS_INFO("deal with gps. time stamp: %lf", gps_msg->header.stamp.toSec());

    if (!stateInit_gps)
    {
        init_lla_ = gnss_data_ptr->lla;
        stateInit_gps = true;
        ROS_INFO("init lla %f %f %f", init_lla_[0], init_lla_[1], init_lla_[2]);
    }
    Eigen::Vector3d p_G_GNSS, lla;
    Utility::convert_lla_to_enu(init_lla_, gnss_data_ptr->lla, &p_G_GNSS);

    // if (stateInit_)
    // {
    //     geometry_msgs::PoseStamped pose_stamped;
    //     pose_stamped.header = gps_msg->header;
    //     pose_stamped.pose.position.x = p_G_GNSS[0];
    //     pose_stamped.pose.position.y = p_G_GNSS[1];
    //     pose_stamped.pose.position.z = p_G_GNSS[2];
    //     // gnss_path_.header = pose_stamped.header;
    //     gnss_path_.poses.push_back(pose_stamped);
    //     // gnss_path_pub_.publish(gnss_path_);

    //     // file_gt_lla << std::fixed << std::setprecision(15)
    //     //             << gnss_data_ptr->lla[1] << ","
    //     //             << gnss_data_ptr->lla[0] << ","
    //     //             << gnss_data_ptr->lla[2] << " ";

    //     // file_gt_xyz << std::fixed << std::setprecision(15)
    //     //             << gnss_data_ptr->timestamp << ", "
    //     //             << p_G_GNSS[0] << ", "
    //     //             << p_G_GNSS[1] << ", "
    //     //             << p_G_GNSS[2] << std::endl;
    // }
    if(!IF_USE_GPS)
    {
        // ROS_INFO("no use gps");
        return;
    }
    //deny after 20s
    if (gps_msg->header.stamp.toSec() - init_time_ > 40)
    {
        // modified_msg.status.status = 0;
        std::cout << "[ ESKF ] Deny Time!" << std::endl;
        // last_deny_cnt = deny_cnt;
        return;
    }
    
    if (IF_GPS_DENY)
    {
        double time_now = gps_msg->header.stamp.toSec();
        for (auto time : deny_time)
        {
            if (time_now - init_time_ > time(0) && time_now - init_time_ < time(1))
            {
                // modified_msg.status.status = 0;
                // std::cout << "[ ESKF ] Deny Time!" << std::endl;
                // last_deny_cnt = deny_cnt;
                parameter_lock = true;
                return_flag = true;
                break;
            }
        }
    }

    // 如果covariance太大，认为是错误的数据，不使用
    if (gnss_data_ptr->cov(0, 0) > 1 || gnss_data_ptr->cov(1, 1) > 1 )
    {   
        ROS_INFO("cov too big");
        parameter_lock = true;
        return_flag = true;
    }

    if (return_flag || !stateInit_ )
    {
        return;
    }

    // file_gt_ << std::fixed << std::setprecision(15)
    // << gnss_data_ptr->timestamp << ", "
    // << gnss_data_ptr->lla[0] << ", "
    // << gnss_data_ptr->lla[1] << ", "
    // << gnss_data_ptr->lla[2] << std::endl;

    // file_gt_lla << std::fixed << std::setprecision(15)
    //         << p_G_GNSS[0] << ", "
    //             << p_G_GNSS[1] << ", "
    //             << p_G_GNSS[2] << std::endl;

    if (std::abs(last_imu_ptr_->timestamp - gps_msg->header.stamp.toSec()) > 0.1)
    {
        std::cout << "[ ESKF ] GNSS and IMU are not sychonized." << std::endl;
        return;
    }
    UpdateByGps(gnss_data_ptr);
   
}

void Odometry::publish()
{
    // std::cout<<"display"<<std::endl;
     odomdisplay.publishekfodom_new(state_);
}




