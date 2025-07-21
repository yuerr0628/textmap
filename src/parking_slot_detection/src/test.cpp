

void AssociatedParkingInfo::process_synced_images(const sensor_msgs::CompressedImageConstPtr&avm_msg,const sensor_msgs::CompressedImageConstPtr& front_msg)
{
    cout<<"enterprocess_synced_images"<<endl;
    canvas1.setTo(cv::Scalar(255, 255, 255));
     try
    {
        // 解压缩图像
        cv::Mat image = cv::imdecode(cv::Mat(avm_msg->data),cv::IMREAD_COLOR);
        cv::Mat image_front = cv::imdecode(cv::Mat(front_msg->data),cv::IMREAD_COLOR);
    //     // 确保图像尺寸符合预期
        int original_width = image.cols;
     int original_height = image.rows;
    //  std::cout<<original_width<<","<<original_height<<std::endl;

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
    cv::Mat image1;
    // 显示裁剪后的图像
        // cv::resize(image, image1, cv::Size(512, 512));
    //     // cv::imshow(windowName1, image);
    //     // cv::waitKey(1);

    //     // 转换为ROS图像消息
    //     sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cropped_image).toImageMsg();
        // cv::resize(image, image, cv::Size(512, 512));
        // cv::imshow(windowName1, image);
        // // cv::waitKey(1);

        // // 转换为ROS图像消息
        // sensor_msgs::ImagePtr image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
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
        if(client_plate.call(srv_plate))
        {
            cout<<"srv_plate.response.plate_numbers.size()"<<srv_plate.response.plate_numbers.size()<<endl;
                for (size_t i = 0; i < srv_plate.response.plate_numbers.size(); ++i)
            {
                LicensePlate plate;
                // std::vector<Point3D> camera_coords;
                std::vector<Point3D> worldpts3D;
                Eigen::Vector3d camera_point1;
                Eigen::Vector3d camera_point2;
                 
                // camera_point1[1]=(-(srv_plate.response.corners_y1[i])/63.15 * cos(vehiclepose.pose.yaw) +  (srv_plate.response.corners_x1[i])/64.56 * sin(vehiclepose.pose.yaw)+vehiclepose.pose.y);
                //  camera_point1[0]=(-(srv_plate.response.corners_x1[i])/63.15 * cos(vehiclepose.pose.yaw) +  (srv_plate.response.corners_y1[i])/64.56 * sin(vehiclepose.pose.yaw)+vehiclepose.pose.x);
                //  camera_point2[1]=(-(srv_plate.response.corners_y2[i])/63.15 * cos(vehiclepose.pose.yaw) +  (srv_plate.response.corners_x2[i])/64.56 * sin(vehiclepose.pose.yaw)+vehiclepose.pose.y);
                //  camera_point2[0]=(-(srv_plate.response.corners_x2[i])/63.15 * cos(vehiclepose.pose.yaw) +  (srv_plate.response.corners_y2[i])/64.56 * sin(vehiclepose.pose.yaw)+vehiclepose.pose.x);
                    // camera_coords[2][1]=(-(srv_plate.response.corners_y1[i])/63.15 * cos(vehiclepose.pose.yaw) +  (srv_plate.response.corners_x1[i])/64.56 * sin(vehiclepose.pose.yaw)+vehiclepose.pose.y);
                    //  camera_coords[2][0]=(-(srv_plate.response.corners_y1[i])/63.15 * cos(vehiclepose.pose.yaw) +  (srv_plate.response.corners_x1[i])/64.56 * sin(vehiclepose.pose.yaw)+vehiclepose.pose.x);
                    //   camera_coords[3][1]=(-(srv_plate.response.corners_y1[i])/63.15 * cos(vehiclepose.pose.yaw) +  (srv_plate.response.corners_x1[i])/64.56 * sin(vehiclepose.pose.yaw)+vehiclepose.pose.y);
                    //    camera_coords[3][0]=(-(srv_plate.response.corners_y1[i])/63.15 * cos(vehiclepose.pose.yaw) +  (srv_plate.response.corners_x1[i])/64.56 * sin(vehiclepose.pose.yaw)+vehiclepose.pose.x);
                // camera_point1[2]=0;
                // camera_point2[2]=0;
                
                //     // const auto& corners = srv.response.corners[i];
                    double x1=srv_plate.response.corners_x1[i],y1=srv_plate.response.corners_y1[i];
                    double x2=srv_plate.response.corners_x2[i],y2=srv_plate.response.corners_y2[i];
                    cout<<srv_plate.response.plate_numbers[i]<<"length:"<<srv_plate.response.plate_numbers[i].length()<<endl;
                    // 车牌角点 3D 世界坐标（假设车牌平面在世界坐标系的 Z = 0 平面上）
                if(srv_plate.response.plate_numbers[i].length() == 9){
                    //  worldpts3D = {{-0.22, 0.07, 0}, {0.22, 0.07, 0}, {0.22, -0.07, 0}, {-0.22, -0.07, 0} };
                     worldpts3D = {{0, -0.22, 0.07}, {0, 0.22, 0.07}, {0, 0.22, -0.07}, {0, -0.22, -0.07}};
                }
                    // worldpts3D = {{-220, 70, 0}, {220, 70, 0}, {220, -70, 0}, {-220, -70, 0} };// 假设车牌为蓝牌
                    
                else{
                    //  worldpts3D = {
                    // {-240, 70, 0}, {240, 70, 0}, {240, -70, 0}, {-240, -70, 0} };// 假设车牌为绿牌
                    //  worldpts3D = {{-0.24, 0.07, 0}, {0.24, 0.07, 0}, {0.24, -0.07, 0}, {-0.24, -0.07, 0}};
                   worldpts3D = {{0, -0.24, 0.07}, {0, 0.24, 0.07}, {0, 0.24, -0.07}, {0, -0.24, -0.07}};
                }
                // 车牌角点 2D 图像坐标
                std::vector<Point2D> pts2D={{x1,y1},{x2,y1},{x2,y2},{x1,y2}};
                std::vector<Point3D> camera_coords;
                p3p1.P3PComputePoses(worldpts3D, pts2D,camera_coords);
                plate.plateNumber=srv_plate.response.plate_numbers[i];
                camera_point1[1]=(-(camera_coords[0][0]) * cos(vehiclepose.pose.yaw) +  (camera_coords[0][2]) * sin(vehiclepose.pose.yaw)+vehiclepose.pose.y);
                 camera_point1[0]=(-(camera_coords[0][2]) * cos(vehiclepose.pose.yaw) +  (camera_coords[0][0]) * sin(vehiclepose.pose.yaw)+vehiclepose.pose.x);
                 camera_point2[1]=(-(camera_coords[2][0]) * cos(vehiclepose.pose.yaw) +  (camera_coords[2][2]) * sin(vehiclepose.pose.yaw)+vehiclepose.pose.y);
                 camera_point2[0]=(-(camera_coords[2][2]) * cos(vehiclepose.pose.yaw) +  (camera_coords[2][0]) * sin(vehiclepose.pose.yaw)+vehiclepose.pose.x);
                camera_point1[2]=0;
                camera_point2[2]=0;
                plate.points.push_back(camera_point1 );
                plate.points.push_back(camera_point2);
                // plate.points.push_back(camera_coords[0]);
                // plate.points.push_back(camera_coords[1]);
                // plate.points.push_back(camera_coords[2]);
                // plate.points.push_back(camera_coords[3]);
                plate.confidence=srv_plate.response.confidence_lic[i];
                plate.timestamp=front_msg->header.stamp.toSec();
                plates.push_back(plate);
                plate.points.clear();
                cout<<"point[0]:"<<camera_coords[0]<<",point1:"<<camera_coords[1]<<",point2:"<<camera_coords[2]<<",point3:"<<camera_coords[3]<<endl;
            }
            display.displayPlate(plates);
        }
        if (client.call(srv))
        {
            // std::cout<<"enter"<<std::endl;
            // std::cout<<srv.response.point0_x.size()<<std::endl;
      
            associateSpotsAndNumbers(srv);
            // drawslotwithnumber(image);
            // 成功接收响应
            cv::Mat image_vis =image1.clone();
            
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
                //  std::cout<<"length:"<<(srv.response.point3_x[i]-srv.response.point1_x[i])/50.08<<std::endl;
                //  std::cout<<"width:"<<(srv.response.point0_y[i]-srv.response.point1_y[i])/48.99<<std::endl;
                // std::cout<<"spotnumber:"<<srv.response.point0_y.size()<<std::endl;
                cv::circle(image_vis, cv::Point(point0_x, point0_y), 3, color, 2);
                cv::circle(image_vis, cv::Point(point1_x, point1_y), 3, color, 2);
                cv::line(image_vis, cv::Point(point0_x, point0_y), cv::Point(point1_x, point1_y), cv::Scalar(0, 0, 255), 2);
                cv::line(image_vis, cv::Point(point0_x, point0_y), cv::Point(point2_x, point2_y), cv::Scalar(0, 0, 255), 2);
                cv::line(image_vis, cv::Point(point1_x, point1_y), cv::Point(point3_x, point3_y), cv::Scalar(0, 0, 255), 2);
            }
             std::cout<<"newframe"<<endl;
            for (size_t i = 0; i < srv.response.ocrpointx1.size(); ++i)
            {
                double x1 = srv.response.ocrpointx1[i];
                double x2 = srv.response.ocrpointx2[i];
                double y1 = srv.response.ocrpointy1[i];
                double y2 = srv.response.ocrpointy2[i];
                string text=srv.response.texts[i];
                std::cout<<"text:"<<srv.response.texts[i]<<"con:"<<srv.response.confidence[i]<<std::endl;
                cout<<"text:"<<(srv.response.ocrpointx1[i]-330)/64.56<<","<<(srv.response.ocrpointy1[i]-330)/63.15<<endl;

                // std::cout<<"textnum:"<<srv.response.texts.size()<<std::endl;
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
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", avm_msg->format.c_str());
    }


}
