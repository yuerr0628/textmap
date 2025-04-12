#!/usr/bin/env python

import ctypes
libgcc_s = ctypes.CDLL('libgcc_s.so.1')

import sys
sys.path.append("/data/yhy/ocr/ocrmap/src")

import cv2
import rospy
from parking_slot_detection.srv import gcn_parking
from sensor_msgs.msg import Image
import time
import numpy as np
from sensor_msgs.msg import CompressedImage
import rosbag


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
    return image_opencv

def cv2_to_imgmsg(cv_image):
    img_msg = Image()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian = 0
    # img_msg.data = cv_image.tostring()
    img_msg.data = cv_image.tobytes()
    img_msg.step = len(img_msg.data) // img_msg.height # That double line is actually integer division, not a comment
    return img_msg



class client_node():
    def __init__(self):
        rospy.init_node('client_node')
        self.rate = rospy.Rate(25)  # 设置频率为25Hz

    def call_service(self, image_data):
        # print("image_data: ", type(image_data))
        rospy.wait_for_service('my_service')
        try:
            service_proxy = rospy.ServiceProxy('my_service', gcn_parking)
            # send_msg.image_data = image_data
            resp = service_proxy(image_data)
            rospy.loginfo("Received random points: %s and %s, with type: %s" % (resp.point1_x, resp.point1_y, resp.types))
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
    

if __name__ == '__main__':
    client = client_node()
    
    # 替换为你的ROSbag文件路径
    bag_path = rospy.get_param('~bag_path', '/mnt/pool/yhy/Close.bag')
    bag = rosbag.Bag(bag_path)
    
    # 替换为你的压缩图像话题名称
    image_topic = rospy.get_param('~image_topic', '/driver/fisheye/avm/compressed')

    for topic, msg, t in bag.read_messages(topics=[image_topic]):
        if topic == image_topic:
            start_time = time.time()
            # 将压缩图像消息解码为OpenCV图像格式
            # 注意：这里假设压缩图像使用的是JPEG格式，如果不是，需要调整解码器
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            img = cv2.resize(img, (512, 512))
            img_msg = cv2_to_imgmsg(img)
    #     client.call_service(img_msg)
            client.call_service(img_msg)
            # 如果需要，这里可以添加图像处理代码，例如调整大小
            # img = cv2.resize(img, (512, 512))
            
            client.call_service(msg)
            end_time = time.time()
            print("duration: ", end_time - start_time)
            time.sleep(0.1)

    
    # client = client_node()
    
    # image_path = rospy.get_param('~image_path', '/default/image/path.jpg')
    # img = cv2.imread(image_path)
    # img = cv2.resize(img, (512, 512))
    # while not rospy.is_shutdown():
    #     start_time = time.time()
    #     img_msg = cv2_to_imgmsg(img)
    #     client.call_service(img_msg)
    #     end_time = time.time()
    #     print("duration: ", end_time - start_time)
    #     time.sleep(0.1)
        # rospy.spinOnce()
        # client.rate.sleep()