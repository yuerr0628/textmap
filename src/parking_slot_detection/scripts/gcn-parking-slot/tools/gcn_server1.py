#!/usr/bin/env python

import rospy
import sys
sys.path.append("/mnt/pool/yhy/ocr_video/src")
sys.path.append("/mnt/pool/yhy/ocr_video/src/parking_slot_detection/scripts/gcn-parking-slot")
from parking_slot_detection.srv import gcn_parking

import cv2
import time
import torch
import pprint
import numpy as np
import math
import random

# model
from psdet.utils.config import get_config
from psdet.utils.common import get_logger
from psdet.models.builder import build_model
from geometry_msgs.msg import Polygon, Point32
from showtrajectory.msg import Slots

slot_pub = rospy.Publisher('slots', Slots, queue_size=1)
i=0
class gcn_detector_server():
    def __init__(self):
        print("start init gcn server")
        self.input_image_size = (512, 512, 3)
        self.cfg = get_config()
        print("start building model")
        self.model = build_model(self.cfg.model)
        
        print("start loading model params")
        self.model.load_params_from_file(filename=self.cfg.ckpt, logger=None, to_cpu=False)
        self.model.cuda()
        self.model.eval()

        print("start warming up")
        with torch.no_grad():
            warm_up_data_dict = {} 
            warm_up_image = np.zeros(self.input_image_size, dtype=np.uint8)/255.0
            warm_up_data_dict['image'] = torch.from_numpy(warm_up_image).float().permute(2, 0, 1).unsqueeze(0).cuda()
            self.model(warm_up_data_dict)
            print("model warm up finish")
        
        rospy.init_node('server_node1')
        rospy.Service('gcn_service', gcn_parking, self.handle_request)
        rospy.loginfo("Server node is ready to receive requests.")
        

    def handle_request(self, req):
        global i  
        # print("get request")
        i=i+1
        start_time = time.time()
        point0_x = []
        point0_y = []
        point1_x = []
        point1_y = []
        point2_x = []
        point2_y = []
        point3_x = []
        point3_y = []
        types = []

        VSLOT_MIN_DIST = 0.044771278151623496
        VSLOT_MAX_DIST = 0.1099427457599304
        HSLOT_MIN_DIST = 0.15057789144568634
        HSLOT_MAX_DIST = 0.44449496544202816

        SHORT_SEPARATOR_LENGTH = 0.199519231
        LONG_SEPARATOR_LENGTH = 0.46875

        image = self.imgmsg_to_cv2(req.image_data)
        if image.shape[0] != self.input_image_size[0] or image.shape[1] != self.input_image_size[1] or image.shape[2] != self.input_image_size[2]:
            print("image size error!!! reciving image size is: ", image.shape)
            return point0_x, point0_y, point1_x, point1_y, types
        image = image/255.0
        data_dict = {}
        data_dict['image'] = torch.from_numpy(image).float().permute(2, 0, 1).unsqueeze(0).cuda()
        with torch.no_grad():
            pred_dicts, ret_dict = self.model(data_dict)
            
            slots_pred = pred_dicts['slots_pred']
            for parking_slot_idx in range(len(slots_pred[0])):
                position = slots_pred[0][parking_slot_idx][1]  # 车位对角线的位置
                p0_x = int(self.input_image_size[0] * position[0] - 0.5)
                p0_y = int(self.input_image_size[1] * position[1] - 0.5)
                p1_x = int(self.input_image_size[0] * position[2] - 0.5)
                p1_y = int(self.input_image_size[1] * position[3] - 0.5)
                point0_x.append(p0_x)
                point0_y.append(p0_y)
                point1_x.append(p1_x)
                point1_y.append(p1_y)
                types.append(1)
                vec = np.array([p1_x - p0_x, p1_y - p0_y])
                vec = vec / np.linalg.norm(vec)  # 对角线方向向量
                distance =( position[0] - position[2] )**2 + ( position[1] - position[3] )**2 # 这个距离的平方是归一化之后的
        # p0,p1进入车位的两个点，然后判断这两个点是车位的长还是宽
                if VSLOT_MIN_DIST <= distance <= VSLOT_MAX_DIST:
                    separating_length = LONG_SEPARATOR_LENGTH
                else:
                    separating_length = SHORT_SEPARATOR_LENGTH
        # 计算其余两个点
                p2_x = p0_x + self.input_image_size[0] * separating_length * vec[1]
                p2_y = p0_y - self.input_image_size[1] * separating_length * vec[0]
                p3_x = p1_x + self.input_image_size[0] * separating_length * vec[1]
                p3_y = p1_y - self.input_image_size[1] * separating_length * vec[0]
                slot = Slots()
              # 创建 Header 对象，设置时间戳和其他字段
          # text_with_box.stamp=gps_1
                slot.polygon.points = [
                  Point32(x=p0_x/108, y=p0_y/108),
                  Point32(x=p1_x/108, y=p1_y/108),
                  Point32(x=p2_x/108, y=p2_y/108),
                  Point32(x=p3_x/108, y=p3_x/108)
              ]
                slot.ID=i
                slot_pub.publish(slot)
                p0_x = int(round(p0_x))
                p0_y = int(round(p0_y))
                p1_x = int(round(p1_x))
                p1_y = int(round(p1_y))
                p2_x = int(round(p2_x))
                p2_y = int(round(p2_y))
                p3_x = int(round(p3_x))
                p3_y = int(round(p3_y))
                point2_x.append(p2_x)
                point2_y.append(p2_y)
                point3_x.append(p3_x)
                point3_y.append(p3_y)
                # cv2.line(image, (p0_x, p0_y), (p1_x, p1_y), (255, 0, 0), 2)
                # cv2.line(image, (p0_x, p0_y), (p2_x, p2_y), (255, 0, 0), 2)
                # cv2.line(image, (p1_x, p1_y), (p3_x, p3_y), (255, 0, 0), 2)

            sec_per_example = (time.time() - start_time)
            # print('Parking Slot Detection time is : %.4f ms.' % (sec_per_example * 1000))
            
            return point0_x, point0_y, point1_x, point1_y,point2_x, point2_y,point3_x, point3_y, types
    
    @staticmethod
    def imgmsg_to_cv2(img_msg):
        if img_msg.encoding != "bgr8":
            rospy.logerr("This Coral detect node has been hardcoded to the 'bgr8' encoding.  Come change the code if you're actually trying to implement a new camera")
        dtype = np.dtype("uint8") # Hardcode to 8 bits...
        dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
        image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
                        dtype=dtype, buffer=img_msg.data)
        # If the byt order is different between the message and the system.
        if img_msg.is_bigendian == (sys.byteorder == 'little'):
            image_opencv = image_opencv.byteswap().newbyteorder()
        # print("image_opencv: ", type(image_opencv))
        return image_opencv




if __name__ == '__main__':
    server = gcn_detector_server()
    rospy.spin()
