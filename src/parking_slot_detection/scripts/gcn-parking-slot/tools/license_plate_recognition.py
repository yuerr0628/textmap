#!/usr/bin/env python3
#coding=utf-8

import rospy
# from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import hyperlpr3 as lpr3
import numpy as np
from PIL import Image, ImageDraw, ImageFont
# from FisheyeParam import *
# from showtrajectory.msg import OCRResult
from geometry_msgs.msg import Polygon, Point32
from sensor_msgs.msg import NavSatFix
from parking_slot_detection.srv import PlateRecognition, PlateRecognitionResponse


# 初始化 cv_bridge


def process_plate(req):
   
    try:
          # 提取 `sensor_msgs/Image` 数据
        width = req.image.width
        height = req.image.height
        encoding = req.image.encoding
        step = req.image.step

        # 将 ROS Image 数据转换为 NumPy 数组
        image_data = np.frombuffer(req.image.data, dtype=np.uint8)

        if encoding == "rgb8":
            image = np.reshape(image_data, (height, width, 3))
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)  # 转换为 OpenCV 的默认 BGR 格式
        elif encoding == "bgr8":
            image = np.reshape(image_data, (height, width, 3))
        elif encoding == "mono8":
            image = np.reshape(image_data, (height, width))
        else:
            rospy.logerr(f"Unsupported encoding: {encoding}")
        # 
        # 将 ROS 图像消息转换为 OpenCV 图像
        # cv_image =bridge.imgmsg_to_cv2(req.image)
        #           #  cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        # np_arr = np.fromstring(req.image.data, np.uint8)
        # cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
          # 执行车牌识别算法j 
        catcher = lpr3.LicensePlateCatcher(detect_level=lpr3.DETECT_LEVEL_HIGH)
        image = cv2.resize(image, (1024, 1024))
        results = catcher(image)
        # pil_image = Image.fromarray(cv_image)  # 将 OpenCV 图像转换为 PIL 图像
            # draw = ImageDraw.Draw(pil_image)  # 创建绘制对象
        
        corners_x1 = []
        corners_y1 = []
        corners_x2 = []
        corners_y2 = []
        plate_numbers = []
        confidence_lic=[]
        font_ch = ImageFont.truetype("/data/yhy/ocr/ocrmap/src/license_plate_recognition/src/platech.ttf", 20, 0)
        if results is not None:
            for code, confidence, type_idx, box in results:
                # 在图像上绘制车牌框和文本
                plate_number = code  # 车牌号
                x1, y1, x2, y2= box  # 四个角点
        #         M = np.array([
        #         [6.19087214455194, -13.6783918575607, -5114.04257928648],
        #         [-0.07103670281201, -7.1647058925724, -2210.0437877731],
        #         [-0.000452864931393468, -0.0140691132595466, -5.29547420841964]
        #         ])
        #         # 将点转换为齐次坐标
        #         point_1 = np.array([[x1],
        #                                 [y1],
        #                             [1]])

        # # 使用变换矩阵 T 对点进行逆透视变换
        #         transformed_point_homogeneous = np.dot(M, point_1)

        # # 将齐次坐标转换回普通坐标
        #         x1 = transformed_point_homogeneous[0] / transformed_point_homogeneous[2]
        #         y1 = transformed_point_homogeneous[1] / transformed_point_homogeneous[2]
        #         point_2 = np.array([[x2],
        #                                 [y2],
        #                             [1]])

        # # 使用变换矩阵 T 对点进行逆透视变换
        #         transformed_point_homogeneous = np.dot(M, point_2)

        # # 将齐次坐标转换回普通坐标
        #         x2 = transformed_point_homogeneous[0] / transformed_point_homogeneous[2]
        #         y2 = transformed_point_homogeneous[1] / transformed_point_homogeneous[2]
                plate_confidence=confidence
                # corners = [Point(x=x1,y=y1),Point(x=x2,y=y1),Point(x=x2,y=y2),Point(x=x1,y=y2)]
                corners_x1.append(x1)
                corners_x2.append(x2)
                corners_y1.append(y1)
                corners_y2.append(y2)
                plate_numbers.append(plate_number)
                confidence_lic.append(plate_confidence)

            return PlateRecognitionResponse(corners_x1,corners_x2,corners_y1,corners_y2, plate_numbers,confidence_lic)

    except Exception as e:
        rospy.logerr(f"Error processing plate: {e}")
def plate_recognition_server():
    rospy.init_node('plate_recognition_server')
    rospy.Service('license_plate_recognition', PlateRecognition, process_plate)
    rospy.loginfo("Plate Recognition Service Ready")
    rospy.spin()

if __name__ == '__main__':
    plate_recognition_server()


# def imgmsg_to_cv2(img_msg):
#     if img_msg.encoding != "bgr8":
#         rospy.logerr("This Coral detect node has been hardcoded to the 'bgr8' encoding.  Come change the code if you're actually trying to implement a new camera")
#     dtype = np.dtype("uint8") # Hardcode to 8 bits...
#     dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else '<')
#     image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr natively, we don't need to reorder the channels.
#                     dtype=dtype, buffer=img_msg.data)
#     # If the byt order is different between the message and the system.
#     if img_msg.is_bigendian == (sys.byteorder == 'little'):
#         image_opencv = image_opencv.byteswap().newbyteorder()
#     # print("image_opencv: ", type(image_opencv))
#     return image_opencv
