#include "associate.h"
#include"pose.h"
#include"drawmap.h"
#include "rvizshow.h"
#include <nlohmann/json.hpp>
#include <fstream>

using json = nlohmann::json;


int m=0;
int n=0;
 int uniqueID = 0;
 double distanceocrcur=0;
RvizDisplay display;
uint32_t avmseq=-1;
uint32_t frontseq=-1;
cv::Mat canvas1(1000, 1000, CV_8UC3, cv::Scalar(255, 255, 255));
cv::String windowName1 = "slots";
AssociatedParkingInfo::AssociatedParkingInfo(ros::NodeHandle& nh): vehiclepose(nh) {
    
    // image_sub = nh.subscribe("/driver/fisheye/avm/compressed", 10, &AssociatedParkingInfo::imageCallback,this);
     avmimage_sub = nh.subscribe("/driver/fisheye/avm/compressed", 10, &AssociatedParkingInfo::avm_callback,this);
    frontimage_sub = nh.subscribe("/driver/fisheye/left/compressed", 10, &AssociatedParkingInfo::front_callback,this);
    
    // client = nh.serviceClient<parking_slot_detection::gcn_parking>("gcn_service");
    client_plate = nh.serviceClient<parking_slot_detection::PlateRecognition>("license_plate_recognition");
    client = nh.serviceClient<parking_slot_detection::gcn_parking>("gcn_service");
    display.RvizDisplay_init(nh);
}

void AssociatedParkingInfo::avm_callback(const sensor_msgs::CompressedImageConstPtr& msg) {
    // cout<<"enteravm"<<endl;
    avmseq = ++avmseq; // 获取序列号
    //  cout<<avmseq<<endl;
    avm_buffer[avmseq] = msg;          // 将消息存入缓冲区

    // 检查是否有匹配的 front 图像
    if (front_buffer.count(avmseq)) {
        // 找到匹配的 front 图像
        auto front_msg = front_buffer[avmseq];

        // 移除已匹配的消息
        front_buffer.erase(avmseq);
        avm_buffer.erase(avmseq);

        // 触发后续处理逻辑
        process_synced_images(msg, front_msg);
    }
}

void AssociatedParkingInfo::front_callback(const sensor_msgs::CompressedImageConstPtr& msg) {
    // cout<<"enterfront"<<endl;
    // uint32_t seq = msg->header.seq; // 获取序列号
    frontseq = ++frontseq;
    //  cout<<frontseq<<endl;
    front_buffer[frontseq] = msg;        // 将消息存入缓冲区

    // 检查是否有匹配的 AVM 图像
    if (avm_buffer.count(frontseq)) {
        // 找到匹配的 AVM 图像
        auto avm_msg = avm_buffer[frontseq];

        // 移除已匹配的消息
        avm_buffer.erase(frontseq);
        front_buffer.erase(frontseq);

        // 触发后续处理逻辑
        process_synced_images(avm_msg, msg);
    }
}

void AssociatedParkingInfo::HandleDeath(const std::vector<AssociatedPair>& cFrame,const  std::vector<AssociatedPair>& deathtemp)
{

    // std::cout<<death.size()<<std::endl;
    // vector<AssociatedPair> deathtemp;
    death=deathtemp;
    std::vector<std::vector<double>> cost_matrix(death.size(), std::vector<double>(cFrame.size(), 10000.0));
    double p1_x,p1_y,p2_x,p2_y,center_x,center_y;
    double c1_x,c1_y,c2_x,c2_y,center1_x,center1_y;
     for (size_t i = 0; i < death.size(); ++i) {
            // std::cout<<"ocr:"<<death[i].ocrPoint.text<<endl;
        if(death[i].spot.x1!=0&&!death[i].ocrPoint.text.empty()){
            //  std::cout<<"enterif"<<std::endl;
        center1_x=(death[i].spot.x1+death[i].spot.x2+death[i].ocrPoint.x1+death[i].ocrPoint.x2)/4;
        center1_y=(death[i].spot.y1+death[i].spot.y2+death[i].ocrPoint.y1+death[i].ocrPoint.y2)/4;
        }
        else{
            // std::cout<<"enterelse"<<std::endl;
        center1_x=(death[i].spot.x1+death[i].spot.x2+death[i].ocrPoint.x1+death[i].ocrPoint.x2)/2;
        center1_y=(death[i].spot.y1+death[i].spot.y2+death[i].ocrPoint.y1+death[i].ocrPoint.y2)/2;
        }
        for (size_t j = 0; j < cFrame.size(); ++j) {
                // Check if the spot numbers are the same
                if(cFrame[j].spot.x1!=0&&!cFrame[j].ocrPoint.text.empty()){
                center_x=(cFrame[j].spot.x1+cFrame[j].spot.x2+cFrame[j].ocrPoint.x1+cFrame[j].ocrPoint.x2)/4;
                center_y=(cFrame[j].spot.y1+cFrame[j].spot.y2+cFrame[j].ocrPoint.y1+cFrame[j].ocrPoint.y2)/4;
                // std::cout<<"enterif"<<std::endl;
                }
                else{
                center_x=(cFrame[j].spot.x1+cFrame[j].spot.x2+cFrame[j].ocrPoint.x1+cFrame[j].ocrPoint.x2)/2;
                center_y=(cFrame[j].spot.y1+cFrame[j].spot.y2+cFrame[j].ocrPoint.y1+cFrame[j].ocrPoint.y2)/2;
                // std::cout<<"enterelse"<<std::endl;
                }
                double distance = sqrt(pow(center1_x-center_x, 2) + pow((center1_y - center_y), 2));
                    // Check if the positions are close enough
                float distanceThreshold = 0.8; // Define an appropriate threshold
                 if (distance <= distanceThreshold) {
                    cost_matrix[i][j] = distance;
                    } else {
            cost_matrix[i][j] = std::numeric_limits<double>::max(); // 使用一个大的值表示不可能的匹配
            // std::cout<<"umatched"<<std::endl;
                }
    

                // cost_matrix[i][j] = distance;
            }
        }
    HungarianAlgorithm hungarian;
    std::vector<int> Assignment;
    double cost = hungarian.Solve(cost_matrix, Assignment);
    // std::cout<<"xiong"<<std::endl;
    std::vector<int> out_of_death;

    // out_of_death = std::vector();

  for (int i = 0; i < Assignment.size(); ++i) { 
    // std::cout<<Assignment.size()<<std::endl;
    // std::cout << "Assignment[i]: " << Assignment[i] << std::endl;
    int match_index=Assignment[i];
    if(Assignment[i] != -1&&cost_matrix[i][Assignment[i]] < 0.8) 
        {
            // std::cout<<"pipei"<<std::endl;
            // 如果重新出现，加入 preFrame 进行匹配
            AssociatedPair deadSpot = death[i];
            preFrame.push_back(deadSpot);
            //  std::cout<<"entermap"<<std::endl;
            // disappearanceCount[deadSpot.ID] = 0; // 重置计数
            disappearanceCount.erase(deadSpot.unmatch_ID);
            // death.erase(death.begin() + i);
            out_of_death.push_back(i);
          
            // Assignment.erase(Assignment.begin() + i); // 更新 Assignment 数组
            //   i--; 
            //   std::cout<<"entermap1"<<std::endl;
             
        } else {
            // 如果未重新出现，计数加1
            // std::cout<<"map"<<std::endl;
            disappearanceCount[death[i].unmatch_ID]++;
            if (disappearanceCount[death[i].unmatch_ID] >=20) {
                AssociatedPair mapSpot = death[i];
                // 从 death 列表和计数映射中移除
                // std::cout<<mapSpot.ocrPoint.text<<","<<disappearanceCount[mapSpot.unmatch_ID]<<std::endl;
                if(death[i].age<=3)
                {
                        disappearanceCount.erase(mapSpot.unmatch_ID);
                        out_of_death.push_back(i);
                        std::cout<<mapSpot.ocrPoint.text<<","<<death[i].age<<std::endl;
                }
                else{
                    if(mapSpot.spot.vacant_update>=1)
                    // if(mapSpot.spot.noccupy>=mapSpot.spot.nfree)
                    {
                        mapSpot.spot.vacant=1;
                    }
                    else{
                        mapSpot.spot.vacant=0;}
                     disappearanceCount.erase(mapSpot.unmatch_ID);
                    out_of_death.push_back(i);
                    
                    display.displaymap(mapSpot);
                    mapassociatedPairs.push_back(mapSpot);
                    // saveToYAML(mapassociatedPairs,"/data/yhy/map.yaml");
                    saveToJSON(mapassociatedPairs,"/data/yhy/map20250323new.json");
                }
            }
        }
    }

    // 从后往前删除元素
    for (auto it = out_of_death.rbegin(); it != out_of_death.rend(); ++it) {
        // std::cout<<"out_of_death"<<death[*it].ocrPoint.text<<std::endl;
        death.erase(death.begin() + *it);
    }

// std::cout<<"death"<<death.size()<<std::endl;
// std::cout<<"test1\n";

    return;

}


// Function to match the same parking spot between two frames
void AssociatedParkingInfo::matchParkingSpots(const std::vector<AssociatedPair>& currentFrame)
 {
    double p1_x,p1_y,p2_x,p2_y,center_x,center_y;
    double c1_x,c1_y,c2_x,c2_y,center1_x,center1_y;
    double distanceocrpre;
    
    double centerx,centery;
    
    AssociatedPair updatemap;
        // 初始化匹配向量
    std::vector<bool> matchedCurrentIndices, matchedPreFrameIndices;
    matchedCurrentIndices.resize(currentFrame.size(), false);
    matchedPreFrameIndices.resize(preFrame.size(), false);
    updates.clear();
    
  
    // std::vector<std::vector<double>> cost_matrix(preFrame.size(), std::vector<double>(currentFrame.size(), 10000.0));
    //  std::cout<<"enter1"<<std::endl;
    
    if (!currentFrame.empty()){
        n=n+1;

    if(n==1) 
    {
           preFrame=currentFrame;
        //    mapassociatedPairs=currentFrame;
        //    std::cout<<"enterif"<<std::endl;
        //    std::cout<<n<<endl;
           return;
           
        }
    else{
        // std::cout<<"enterelse"<<std::endl;
    if(!death.empty()){
        
        HandleDeath(currentFrame,death);
        // std::cout<<"enterif1"<<std::endl;
        // continue;
    }
     std::vector<std::vector<double>> cost_matrix(preFrame.size(), std::vector<double>(currentFrame.size(), 10000.0));

    for (size_t i = 0; i < currentFrame.size(); ++i) {
        // if(currentFrame[i].spot.vacant==1){
        //     currentFrame[i].spot.vacant++;
        // }
        if(currentFrame[i].spot.x1!=0&&!currentFrame[i].ocrPoint.text.empty()){
        center1_x=(currentFrame[i].spot.x1+currentFrame[i].spot.x2+currentFrame[i].ocrPoint.x1+currentFrame[i].ocrPoint.x2)/4;
        center1_y=(currentFrame[i].spot.y1+currentFrame[i].spot.y2+currentFrame[i].ocrPoint.y1+currentFrame[i].ocrPoint.y2)/4;

        }
        else{
        center1_x=(currentFrame[i].spot.x1+currentFrame[i].spot.x2+currentFrame[i].ocrPoint.x1+currentFrame[i].ocrPoint.x2)/2;
        center1_y=(currentFrame[i].spot.y1+currentFrame[i].spot.y2+currentFrame[i].ocrPoint.y1+currentFrame[i].ocrPoint.y2)/2;
        }
        for (size_t j = 0; j < preFrame.size(); ++j) {
        
                // Check if the spot numbers are the same
                if(preFrame[j].spot.x1!=0&&!preFrame[j].ocrPoint.text.empty()){
                center_x=(preFrame[j].spot.x1+preFrame[j].spot.x2+preFrame[j].ocrPoint.x1+preFrame[j].ocrPoint.x2)/4;
                center_y=(preFrame[j].spot.y1+preFrame[j].spot.y2+preFrame[j].ocrPoint.y1+preFrame[j].ocrPoint.y2)/4;
                }
                else{
                center_x=(preFrame[j].spot.x1+preFrame[j].spot.x2+preFrame[j].ocrPoint.x1+preFrame[j].ocrPoint.x2)/2;
                center_y=(preFrame[j].spot.y1+preFrame[j].spot.y2+preFrame[j].ocrPoint.y1+preFrame[j].ocrPoint.y2)/2;
                }
                double distance = sqrt(pow(center1_x-center_x, 2) + pow((center1_y - center_y), 2));
                    // Check if the positions are close enough
                float distanceThreshold = 0.8; // Define an appropriate threshold

                // cost_matrix[j][i] = distance;
                        // 检查位置是否足够近
        if (distance <= distanceThreshold) {
            cost_matrix[j][i] = distance;
        } else {
            //  std::cout<<"enterif2"<<std::endl;
            cost_matrix[j][i] = std::numeric_limits<double>::max(); // 使用一个大的值表示不可能的匹配
            //  std::cout<<"entermax"<<std::endl;
        }
            }
        }

        // std::cout<<currentFrame.size()<<std::endl;
    //  std::cout<<preFrame.size()<<std::endl;
    HungarianAlgorithm hungarian;
    std::vector<int> Assignment;
    double cost = hungarian.Solve(cost_matrix, Assignment);
    for(int i = 0; i < Assignment.size(); ++i) {
        if(Assignment[i] != -1&&cost_matrix[i][Assignment[i]] < 0.8) {
        
        int match_index = Assignment[i];
        //  std::cout<<preFrame[i].spot.vacant<<currentFrame[match_index].spot.vacant<<std::endl;
        if(!preFrame[i].ocrPoint.text.empty()){
        centery=(preFrame[i].ocrPoint.y1+preFrame[i].ocrPoint.y2)/2;
        centerx=(preFrame[i].ocrPoint.x1+preFrame[i].ocrPoint.x2)/2;
        distanceocrpre=sqrt(pow(vehiclepose.pose.x - centerx, 2) + pow(vehiclepose.pose.y-2.2575 - centery, 2));
        }

            if(currentFrame[match_index].spot.x1!=0&&preFrame[i].spot.x1!=0){
            updatemap.spot.x1=(preFrame[i].spot.x1+currentFrame[match_index].spot.x1)/2;
            updatemap.spot.y1=(preFrame[i].spot.y1+currentFrame[match_index].spot.y1)/2;
            updatemap.spot.x2=(preFrame[i].spot.x2+currentFrame[match_index].spot.x2)/2;
            updatemap.spot.y2=(preFrame[i].spot.y2+currentFrame[match_index].spot.y2)/2;
            updatemap.spot.x3=(preFrame[i].spot.x3+currentFrame[match_index].spot.x3)/2;
            updatemap.spot.y3=(preFrame[i].spot.y3+currentFrame[match_index].spot.y3)/2;
            updatemap.spot.x4=(preFrame[i].spot.x4+currentFrame[match_index].spot.x4)/2;
            updatemap.spot.y4=(preFrame[i].spot.y4+currentFrame[match_index].spot.y4)/2;
           
            }
            else{
            updatemap.spot.x1=(preFrame[i].spot.x1+currentFrame[match_index].spot.x1);
            updatemap.spot.y1=(preFrame[i].spot.y1+currentFrame[match_index].spot.y1);
            updatemap.spot.x2=(preFrame[i].spot.x2+currentFrame[match_index].spot.x2);
            updatemap.spot.y2=(preFrame[i].spot.y2+currentFrame[match_index].spot.y2);
            updatemap.spot.x3=(preFrame[i].spot.x3+currentFrame[match_index].spot.x3);
            updatemap.spot.y3=(preFrame[i].spot.y3+currentFrame[match_index].spot.y3);
            updatemap.spot.x4=(preFrame[i].spot.x4+currentFrame[match_index].spot.x4);
            updatemap.spot.y4=(preFrame[i].spot.y4+currentFrame[match_index].spot.y4);
            }
             if(!currentFrame[match_index].ocrPoint.text.empty()&&!preFrame[i].ocrPoint.text.empty())
             {
            updatemap.ocrPoint.x1=(preFrame[i].ocrPoint.x1+currentFrame[match_index].ocrPoint.x1)/2;
            updatemap.ocrPoint.y1=(preFrame[i].ocrPoint.y1+currentFrame[match_index].ocrPoint.y1)/2;
            updatemap.ocrPoint.x2=(preFrame[i].ocrPoint.x2+currentFrame[match_index].ocrPoint.x2)/2;
            updatemap.ocrPoint.y2=(preFrame[i].ocrPoint.y2+currentFrame[match_index].ocrPoint.y2)/2;
             }
             else{
            updatemap.ocrPoint.x1=(preFrame[i].ocrPoint.x1+currentFrame[match_index].ocrPoint.x1);
            updatemap.ocrPoint.y1=(preFrame[i].ocrPoint.y1+currentFrame[match_index].ocrPoint.y1);
            updatemap.ocrPoint.x2=(preFrame[i].ocrPoint.x2+currentFrame[match_index].ocrPoint.x2);
            updatemap.ocrPoint.y2=(preFrame[i].ocrPoint.y2+currentFrame[match_index].ocrPoint.y2);
             }
            updatemap.ID=currentFrame[match_index].ID;
            updatemap.ocrPoint.confidence=preFrame[i].ocrPoint.confidence;
            if(currentFrame[match_index].ocrPoint.confidence>preFrame[i].ocrPoint.confidence)
            {
                updatemap.ocrPoint.text=currentFrame[match_index].ocrPoint.text;
                updatemap.ocrPoint.confidence=currentFrame[match_index].ocrPoint.confidence;
            }
            else{
                 updatemap.ocrPoint.confidence=preFrame[i].ocrPoint.confidence;
                 updatemap.ocrPoint.text=preFrame[i].ocrPoint.text;
            }
            // updatemap.ocrPoint.text=preFrame[i].ocrPoint.text;
            // if(!preFrame[i].ocrPoint.text.empty()&&!currentFrame[match_index].ocrPoint.text.empty()){
            // if(distanceocrcur!=0){
            //     if(distanceocrpre<=distanceocrcur){
            //         updatemap.ocrPoint.text=currentFrame[match_index].ocrPoint.text;
                    
            //     }
            //      if(distanceocrpre>distanceocrcur){
            //         updatemap.ocrPoint.text=preFrame[i].ocrPoint.text;
            //     }
            // }
            // }
            if(preFrame[i].spot.vacant==1||currentFrame[match_index].spot.vacant==1){
                    updatemap.spot.vacant_update=preFrame[i].spot.vacant_update+1;
                    updatemap.spot.noccupy=updatemap.spot.noccupy+1;
            }
            updatemap.spot.vacant=0;
            updatemap.spot.nfree=updatemap.spot.nfree+1;
            // updatemap.ocrPoint.text=preFrame[i].ocrPoint.text;
            preFrame[i].age=preFrame[i].age+1;
            updatemap.age=preFrame[i].age;
            // updatemap.distanceocr=currentFrame[match_index].distanceocr;
            
            // mapassociatedPairs.push_back(updatemap);
           
            updates.push_back(updatemap);
            matchedCurrentIndices[match_index] = true;
            matchedPreFrameIndices[i] = true;
            // matched! update
            }
        
        else{
            AssociatedPair temp = preFrame[i];
            temp.unmatch_ID= uniqueID++; // 在添加到 death 向量时设置 ID
            death.push_back(temp);
            // death.push_back(preFrame[i]);
            // death[i].ID = uniqueID++;
            disappearanceCount[temp.unmatch_ID]=1;

        }
        
         
        }
        // std::cout<<Assignment.size()<<std::endl;
    std::vector<int> birthIndices;
    for (int i = 0; i < currentFrame.size(); ++i) {
        if (!matchedCurrentIndices[i]) {
            birthIndices.push_back(i);
            //  std::cout<<"enterif3"<<std::endl;
        }
    }

    for (int idx : birthIndices) {
        updates.push_back(currentFrame[idx]);
        //  std::cout<<"enterif4"<<std::endl;
    }


}
      distanceocrcur=distanceocrpre;   
    preFrame.clear();
    preFrame=updates;
    // distanceocrcur=distanceocrpre;
    
    
    // std::cout<<"enterif5"<<std::endl;
    }
    
    // drawslot(preFrame);

   
}

void AssociatedParkingInfo::worldlocationspots(const std::vector<AssociatedPair>& pairspots )
{
    worldassociatedPairs.clear();
    AssociatedPair spotpair;
    AssociatedPair pairspot1;
        double distanceocrpre;
    
    double centerx,centery;
 
    for (const auto& pairspot : pairspots){
            // std::cout<<"ocr:"<<pairspot.ocrPoint.text<<"id:"<<pairspot.ID<<std::endl;
            pairspot1.spot.x1=-256+pairspot.spot.x1;
            pairspot1.spot.x2=-256+pairspot.spot.x2;
            pairspot1.spot.x3=-256+pairspot.spot.x3;
            pairspot1.spot.x4=-256+pairspot.spot.x4;
            pairspot1.spot.y1=-256+pairspot.spot.y1;
            pairspot1.spot.y2=-256+pairspot.spot.y2;
            pairspot1.spot.y3=-256+pairspot.spot.y3;
            pairspot1.spot.y4=-256+pairspot.spot.y4;
            pairspot1.ocrPoint.x1=-330+pairspot.ocrPoint.x1;
            pairspot1.ocrPoint.y1=-330+pairspot.ocrPoint.y1;
            pairspot1.ocrPoint.x2=-330+pairspot.ocrPoint.x2;
            pairspot1.ocrPoint.y2=-330+pairspot.ocrPoint.y2;
        if(pairspot.spot.x1!=0&&!pairspot.ocrPoint.text.empty()){
    spotpair.spot.y1=(-(pairspot1.spot.y1)/48.99 * cos(vehiclepose.pose.yaw) +  (pairspot1.spot.x1)/50.08 * sin(vehiclepose.pose.yaw)+vehiclepose.pose.y);
    spotpair.spot.x1=(pairspot1.spot.x1)/50.08 * cos(vehiclepose.pose.yaw) + (pairspot1.spot.y1)/48.99 * sin(vehiclepose.pose.yaw) +vehiclepose.pose.x;
    spotpair.spot.y2=(-(pairspot1.spot.y2)/48.99 * cos(vehiclepose.pose.yaw) + (pairspot1.spot.x2)/50.08 * sin(vehiclepose.pose.yaw)+vehiclepose.pose.y);
    spotpair.spot.x2=(pairspot1.spot.x2)/50.08 * cos(vehiclepose.pose.yaw) + (pairspot1.spot.y2)/48.99 * sin(vehiclepose.pose.yaw) +vehiclepose.pose.x;
    spotpair.spot.y3=(-(pairspot1.spot.y3)/48.99 * cos(vehiclepose.pose.yaw) +  (pairspot1.spot.x3)/50.08 * sin(vehiclepose.pose.yaw)+vehiclepose.pose.y);
    spotpair.spot.x3=(pairspot1.spot.x3)/50.08 * cos(vehiclepose.pose.yaw) +  (pairspot1.spot.y3)/48.99 * sin(vehiclepose.pose.yaw) +vehiclepose.pose.x;
    spotpair.spot.y4=(-(pairspot1.spot.y4)/48.99* cos(vehiclepose.pose.yaw) +  (pairspot1.spot.x4)/50.08 * sin(vehiclepose.pose.yaw)+vehiclepose.pose.y);
    spotpair.spot.x4=(pairspot1.spot.x4)/50.08 * cos(vehiclepose.pose.yaw) + (pairspot1.spot.y4)/48.99 * sin(vehiclepose.pose.yaw) +vehiclepose.pose.x;
    spotpair.ocrPoint.y1=(-(pairspot1.ocrPoint.y1)/63.15 * cos(vehiclepose.pose.yaw) +  (pairspot1.ocrPoint.x1)/64.56 * sin(vehiclepose.pose.yaw)+vehiclepose.pose.y);
    spotpair.ocrPoint.x1=(pairspot1.ocrPoint.x1)/64.56 * cos(vehiclepose.pose.yaw) +  (pairspot1.ocrPoint.y1)/63.15 * sin(vehiclepose.pose.yaw) +vehiclepose.pose.x;
    spotpair.ocrPoint.y2=(-(pairspot1.ocrPoint.y2)/63.15 * cos(vehiclepose.pose.yaw) +  (pairspot1.ocrPoint.x2)/64.56 * sin(vehiclepose.pose.yaw)+vehiclepose.pose.y);
    spotpair.ocrPoint.x2=(pairspot1.ocrPoint.x2)/64.56 * cos(vehiclepose.pose.yaw) +  (pairspot1.ocrPoint.y2)/63.15 * sin(vehiclepose.pose.yaw) +vehiclepose.pose.x;
    spotpair.ocrPoint.text=pairspot.ocrPoint.text;
    spotpair.ID=pairspot.ID;
    spotpair.spot.vacant=pairspot.spot.vacant;
    spotpair.IF_SPOT_DET=pairspot.IF_SPOT_DET;
    spotpair.IF_TEXT_DET=pairspot.IF_TEXT_DET;
    spotpair.ocrPoint.confidence=pairspot.ocrPoint.confidence;
   double length=abs(spotpair.spot.y1-spotpair.spot.y2);
    double width=abs(spotpair.spot.x1-spotpair.spot.x2);
    double lenth=sqrt(length*length+width*width);
    // std::cout<<"wolrdpoint:"<<spotpair.spot.y1<<","<<spotpair.spot.x1<<","<<spotpair.spot.y2<<","<<spotpair.spot.x2<<","<<lenth<<endl;
    
                    // 计算车位号和车辆中心距离
        centery=(spotpair.ocrPoint.y1+spotpair.ocrPoint.y2)/2;
        centerx=(spotpair.ocrPoint.x1+spotpair.ocrPoint.x2)/2;
        distanceocrpre=sqrt(pow(vehiclepose.pose.x - centerx, 2) + pow(vehiclepose.pose.y+2.2725- centery, 2));
        spotpair.distanceocr=distanceocrpre;
        //  std::cout<< spotpair.distanceocr<<endl;
        // if(width>3.5)
        worldassociatedPairs.push_back(spotpair);
        }
     else if(pairspot.spot.x1==0&&!pairspot.ocrPoint.text.empty()){
        spotpair.spot.x1=spotpair.spot.y1=0;
        spotpair.spot.x2=spotpair.spot.y2=0;
        spotpair.spot.x3=spotpair.spot.y3=0;
        spotpair.spot.x4=spotpair.spot.y4=0;
    spotpair.ocrPoint.y1=(-(pairspot1.ocrPoint.y1)/63.15 * cos(vehiclepose.pose.yaw) +  (pairspot1.ocrPoint.x1)/64.56 * sin(vehiclepose.pose.yaw)+vehiclepose.pose.y);
    spotpair.ocrPoint.x1=(pairspot1.ocrPoint.x1)/64.56 * cos(vehiclepose.pose.yaw) +  (pairspot1.ocrPoint.y1)/63.15 * sin(vehiclepose.pose.yaw) +vehiclepose.pose.x;
    spotpair.ocrPoint.y2=(-(pairspot1.ocrPoint.y2)/63.15 * cos(vehiclepose.pose.yaw) +  (pairspot1.ocrPoint.x2)/64.56 * sin(vehiclepose.pose.yaw)+vehiclepose.pose.y);
    spotpair.ocrPoint.x2=(pairspot1.ocrPoint.x2)/64.56 * cos(vehiclepose.pose.yaw) +  (pairspot1.ocrPoint.y2)/63.15 * sin(vehiclepose.pose.yaw) +vehiclepose.pose.x;
    spotpair.ocrPoint.text=pairspot.ocrPoint.text;
    spotpair.ID=pairspot.ID;
    spotpair.spot.vacant=pairspot.spot.vacant;
     spotpair.IF_SPOT_DET=pairspot.IF_SPOT_DET;
    spotpair.IF_TEXT_DET=pairspot.IF_TEXT_DET;
    spotpair.ocrPoint.confidence=pairspot.ocrPoint.confidence;
    double length=abs(spotpair.spot.y1-spotpair.spot.y4);
    double width=abs(spotpair.spot.x1-spotpair.spot.x4);
                      // 计算车位号和车辆中心距离
        centery=(spotpair.ocrPoint.y1+spotpair.ocrPoint.y2)/2;
        centerx=(spotpair.ocrPoint.x1+spotpair.ocrPoint.x2)/2;
        distanceocrpre=sqrt(pow(vehiclepose.pose.x - centerx, 2) + pow(vehiclepose.pose.y+2.2725 - centery, 2));
        spotpair.distanceocr=distanceocrpre;
        // std::cout<<spotpair.distanceocr<<endl;
    // if(width>3.5)
        worldassociatedPairs.push_back(spotpair);
    

        }
       else if(pairspot.spot.x1!=0&&pairspot.ocrPoint.text.empty())
        {
            // std::cout<<"enter"<<std::endl;
    //    std::cout<<pairspot.ocrPoint.text<<endl;
    spotpair.spot.y1=(-(pairspot1.spot.y1)/48.99 * cos(vehiclepose.pose.yaw) +  (pairspot1.spot.x1)/ 50.08* sin(vehiclepose.pose.yaw) +vehiclepose.pose.y);
    spotpair.spot.x1=(pairspot1.spot.x1)/50.08 * cos(vehiclepose.pose.yaw) +  (pairspot1.spot.y1)/48.99 * sin(vehiclepose.pose.yaw) +vehiclepose.pose.x;
    spotpair.spot.y2=(-(pairspot1.spot.y2)/48.99 * cos(vehiclepose.pose.yaw) +  (pairspot1.spot.x2)/50.08 * sin(vehiclepose.pose.yaw) +vehiclepose.pose.y);
    spotpair.spot.x2=(pairspot1.spot.x2)/50.08 * cos(vehiclepose.pose.yaw) +  (pairspot1.spot.y2)/48.99 * sin(vehiclepose.pose.yaw) +vehiclepose.pose.x;
    spotpair.spot.y3=(-(pairspot1.spot.y3)/48.99 * cos(vehiclepose.pose.yaw) +  (pairspot1.spot.x3)/50.08 * sin(vehiclepose.pose.yaw)+vehiclepose.pose.y);
    spotpair.spot.x3=(pairspot1.spot.x3)/50.08 * cos(vehiclepose.pose.yaw) + (pairspot1.spot.y3)/48.99 * sin(vehiclepose.pose.yaw) +vehiclepose.pose.x;
    spotpair.spot.y4=(-(pairspot1.spot.y4)/48.99 * cos(vehiclepose.pose.yaw) +  (pairspot1.spot.x4)/50.08 * sin(vehiclepose.pose.yaw)+vehiclepose.pose.y);
    spotpair.spot.x4=(pairspot1.spot.x4)/50.08 * cos(vehiclepose.pose.yaw) +  (pairspot1.spot.y4)/48.99 * sin(vehiclepose.pose.yaw) +vehiclepose.pose.x;
    spotpair.ocrPoint.x1=spotpair.ocrPoint.y1=0;
    spotpair.ocrPoint.x2=spotpair.ocrPoint.y2=0;
    spotpair.ocrPoint.text=pairspot.ocrPoint.text;
    spotpair.ID=pairspot.ID;
    spotpair.spot.vacant=pairspot.spot.vacant;
    spotpair.IF_SPOT_DET=pairspot.IF_SPOT_DET;
    spotpair.IF_TEXT_DET=pairspot.IF_TEXT_DET;
    spotpair.ocrPoint.confidence=0;
    double length=abs(spotpair.spot.y1-spotpair.spot.y4);
    double width=abs(spotpair.spot.x1-spotpair.spot.x4);
    spotpair.distanceocr=500;
    // if(width>3.5)
        worldassociatedPairs.push_back(spotpair);
    // worldassociatedPairs.push_back(spotpair);
        }

 
    }
   
    display.displayParkingSpots(worldassociatedPairs);
     matchParkingSpots(worldassociatedPairs);

    
 
    
   
    

}


void AssociatedParkingInfo::addFrameAssociatedPairs(const std::vector<AssociatedPair>& framePairs)
{
    AllFramesAssociatedPairs.push_back(framePairs);
    
}


void AssociatedParkingInfo::associateSpotsAndNumbers(const parking_slot_detection::gcn_parking & srv)
{
    double association_distance=0.6;
    m=m+1;
    associatedPairs.clear();
    AssociatedPair pair;
    pair.ID=m;
    std::vector<bool> matched_spot_flags(srv.response.point0_x.size(), false); // 标记车位是否已匹配
    std::vector<bool> matched_number_flags(srv.response.ocrpointx1.size(), false); // 标记车位号是否已匹配
     for (size_t i = 0; i < srv.response.point0_x.size(); ++i){
         for (size_t j = 0; j < srv.response.ocrpointx1.size(); ++j){
                double point0_x = (srv.response.point0_x[i]+srv.response.point1_x[i])/2;
                double point0_y = (srv.response.point0_y[i]+srv.response.point1_y[i])/2;
                double x1 = (srv.response.ocrpointx1[j]+srv.response.ocrpointx2[j])/2;
                double y1 = (srv.response.ocrpointy1[j]+srv.response.ocrpointy2[j])/2;
                double disocr_x=(x1-330)/64.56;double disocr_y=(y1-330)/63.15;
                double disspot_x=(point0_x-256)/50.08;double disspot_y=(point0_y-256)/48.99;
                double distance = sqrt(pow((disocr_x - disspot_x), 2) + pow((disocr_y - disspot_y), 2));
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
                    pair.ID=m;
                    pair.ocrPoint.confidence=srv.response.confidence[j];
                    associatedPairs.push_back(pair); // 存储关联对
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
            pair.ID=m;
            associatedPairs.push_back(pair);
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
            pair.ID=m;
            associatedPairs.push_back(pair);
            

        }
    }
    addFrameAssociatedPairs(associatedPairs);
    worldlocationspots(associatedPairs);
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
        
    }
    // cv::imshow("Associated Parking Spots and Numbers", output_image);
    // cv::waitKey(1); // 等待按键继续
    
}


void AssociatedParkingInfo::process_synced_images(const sensor_msgs::CompressedImageConstPtr&avm_msg,const sensor_msgs::CompressedImageConstPtr& front_msg)
{
    // cout<<"enterprocess_synced_images"<<endl;
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
     /*   if(client_plate.call(srv_plate))
        {
            // cout<<"srv_plate.response.plate_numbers.size()"<<srv_plate.response.plate_numbers.size()<<endl;
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
                    // cout<<srv_plate.response.plate_numbers[i]<<"length:"<<srv_plate.response.plate_numbers[i].length()<<endl;
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
                // cout<<"point[0]:"<<camera_coords[0]<<",point1:"<<camera_coords[1]<<",point2:"<<camera_coords[2]<<",point3:"<<camera_coords[3]<<endl;
            }
            // display.displayPlate(plates);
        }*/
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
                 std::cout<<"length:"<<(srv.response.point3_x[i]-srv.response.point1_x[i])/50.08<<std::endl;
                 std::cout<<"width:"<<(srv.response.point0_y[i]-srv.response.point1_y[i])/48.99<<std::endl;
                  std::cout<<"length1:"<<(srv.response.point3_x[i]-srv.response.point1_x[i])/48.99<<std::endl;
                 std::cout<<"width1:"<<(srv.response.point0_y[i]-srv.response.point1_y[i])/50.08<<std::endl;
                std::cout<<"spotnumber:"<<srv.response.point0_y.size()<<std::endl;
                cv::circle(image_vis, cv::Point(point0_x, point0_y), 3, color, 2);
                cv::circle(image_vis, cv::Point(point1_x, point1_y), 3, color, 2);
                cv::line(image_vis, cv::Point(point0_x, point0_y), cv::Point(point1_x, point1_y), cv::Scalar(0, 0, 255), 2);
                cv::line(image_vis, cv::Point(point0_x, point0_y), cv::Point(point2_x, point2_y), cv::Scalar(0, 0, 255), 2);
                cv::line(image_vis, cv::Point(point1_x, point1_y), cv::Point(point3_x, point3_y), cv::Scalar(0, 0, 255), 2);
            }
            //  std::cout<<"newframe"<<endl;
            for (size_t i = 0; i < srv.response.ocrpointx1.size(); ++i)
            {
                double x1 = srv.response.ocrpointx1[i];
                double x2 = srv.response.ocrpointx2[i];
                double y1 = srv.response.ocrpointy1[i];
                double y2 = srv.response.ocrpointy2[i];
                string text=srv.response.texts[i];
                // std::cout<<"text:"<<srv.response.texts[i]<<"con:"<<srv.response.confidence[i]<<std::endl;
                // cout<<"text:"<<(srv.response.ocrpointx1[i]-330)/64.56<<","<<(srv.response.ocrpointy1[i]-330)/63.15<<endl;

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



// void AssociatedParkingInfo::saveToYAML(const std::vector<AssociatedPair>& pairs, const std::string& filename) {
//   YAML::Emitter out;
//     out << YAML::BeginSeq;
//     for (const auto& pair : pairs) {
//         out << YAML::BeginMap;
//         out << YAML::Key << "ID" << YAML::Value << pair.ID;
//         out << YAML::Key << "unmatch_ID" << YAML::Value << pair.unmatch_ID;
//         out << YAML::Key << "age" << YAML::Value << pair.age;
//         out << YAML::Key << "distanceocr" << YAML::Value << pair.distanceocr;
//         out << YAML::Key << "IF_TEXT_DET" << YAML::Value << pair.IF_TEXT_DET;
//         out << YAML::Key << "IF_SPOT_DET" << YAML::Value << pair.IF_SPOT_DET;

//         // 序列化 ParkingSpot
//         out << YAML::Key << "ParkingSpot" << YAML::Value << YAML::BeginMap;
//         out << YAML::Key << "x1" << YAML::Value << pair.spot.x1;
//         out << YAML::Key << "y1" << YAML::Value << pair.spot.y1;
//         out << YAML::Key << "x2" << YAML::Value << pair.spot.x2;
//         out << YAML::Key << "y2" << YAML::Value << pair.spot.y2;
//         out << YAML::Key << "x3" << YAML::Value << pair.spot.x3;
//         out << YAML::Key << "y3" << YAML::Value << pair.spot.y3;
//         out << YAML::Key << "x4" << YAML::Value << pair.spot.x4;
//         out << YAML::Key << "y4" << YAML::Value << pair.spot.y4;
//         out << YAML::Key << "vacant" << YAML::Value << pair.spot.vacant;
//         out << YAML::Key << "vacant_update" << YAML::Value << pair.spot.vacant_update;
//         out << YAML::EndMap;

//         // 序列化 OCRPoint
//         out << YAML::Key << "OCRPoint" << YAML::Value << YAML::BeginMap;
//         out << YAML::Key << "text" << YAML::Value << pair.ocrPoint.text;
//         out << YAML::Key << "confidence" << YAML::Value << pair.ocrPoint.confidence;
//         out << YAML::Key << "x1" << YAML::Value << pair.ocrPoint.x1;
//         out << YAML::Key << "y1" << YAML::Value << pair.ocrPoint.y1;
//         out << YAML::Key << "x2" << YAML::Value << pair.ocrPoint.x2;
//         out << YAML::Key << "y2" << YAML::Value << pair.ocrPoint.y2;
//         out << YAML::EndMap;

//         out << YAML::EndMap;
//     }
//     out << YAML::EndSeq;

//     std::ofstream fout(filename);
//     fout << out.c_str();
// }

void AssociatedParkingInfo::saveToJSON(const std::vector<AssociatedPair>& pairs, const std::string& filename) {
    json j_array = json::array();

    for (const auto& pair : pairs) {
        json j_pair;
        j_pair["ID"] = pair.ID;
        // j_pair["unmatch_ID"] = pair.unmatch_ID;
        // j_pair["age"] = pair.age;
        // j_pair["distanceocr"] = pair.distanceocr;
        // j_pair["IF_TEXT_DET"] = pair.IF_TEXT_DET;
        // j_pair["IF_SPOT_DET"] = pair.IF_SPOT_DET;

        // 序列化 ParkingSpot
        json j_spot;
        j_spot["x1"] = pair.spot.x1;
        j_spot["y1"] = pair.spot.y1;
        j_spot["x2"] = pair.spot.x2;
        j_spot["y2"] = pair.spot.y2;
        j_spot["x3"] = pair.spot.x3;
        j_spot["y3"] = pair.spot.y3;
        j_spot["x4"] = pair.spot.x4;
        j_spot["y4"] = pair.spot.y4;
        j_spot["vacant"] = pair.spot.vacant;
        // j_spot["vacant_update"] = pair.spot.vacant_update;
        j_pair["ParkingSpot"] = j_spot;

        // 序列化 OCRPoint
        json j_ocr;
        j_ocr["text"] = pair.ocrPoint.text;
        j_ocr["confidence"] = pair.ocrPoint.confidence;
        j_ocr["x1"] = pair.ocrPoint.x1;
        j_ocr["y1"] = pair.ocrPoint.y1;
        j_ocr["x2"] = pair.ocrPoint.x2;
        j_ocr["y2"] = pair.ocrPoint.y2;
        j_pair["OCRPoint"] = j_ocr;

        j_array.push_back(j_pair);
    }

    std::ofstream fout(filename);
    fout << j_array.dump(4); // 带缩进的格式化输出
}