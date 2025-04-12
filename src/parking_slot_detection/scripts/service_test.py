#!/usr/bin/env python

import rospy
import sys
sys.path.append("/data/yhy/ocr/ocrmap/src")
from parking_slot_detection.srv import gcn_parking
from sensor_msgs.msg import Image
from random import randint
import cv2
import numpy as np
import time

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

def handle_request(req):
    # print("get image: ", type(req.image_data))
    img = imgmsg_to_cv2(req.image_data)
    # print("get opencv img")
    # print("img: ", img.shape)
    # cv2.imshow("test", img)
    # cv2.waitKey()
    # cv2.destroyAllWindows()
    # 生成两个随机点
    time.sleep(0.2)
    point1 = (randint(0, 100), randint(0, 100))
    point2 = (randint(0, 100), randint(0, 100))
    
    # 生成随机的type
    type_value = randint(0, 10)
    
    rospy.loginfo("Received image from client. Generating random points and type...")
    
    return point1, point2, type_value

def server():
    rospy.init_node('server_node1')
    rospy.Service('my_service', gcn_parking, handle_request)
    rospy.loginfo("Server node is ready to receive requests.")
    rospy.spin()

if __name__ == '__main__':
    server()
    # 关闭窗口
    # cv2.destroyAllWindows()