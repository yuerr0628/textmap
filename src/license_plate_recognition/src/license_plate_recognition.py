#!/usr/bin/env python3
#coding=utf-8

import rospy
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
import cv2
import hyperlpr3 as lpr3
import numpy as np
from PIL import Image, ImageDraw, ImageFont
# from FisheyeParam import *
# from showtrajectory.msg import OCRResult
from geometry_msgs.msg import Polygon, Point32
from sensor_msgs.msg import NavSatFix
# from license_plate_recognition.srv import PlateRecognition, PlateRecognitionResponse

# license_pub = rospy.Publisher('license_with_box', OCRResult, queue_size=1)
# gps_1 = 0

# def gps_time(gps):
#    gps_t = 1


def license_plate_recognition(compressed_image):
    # bridge = CvBridge()
    # global gps_1
    # if gps_1 % 10 == 0 :
      #  cv_image = bridge.imgmsg_to_cv2(image, desired_encoding="bgr8")
        np_arr = np.fromstring(compressed_image.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
      # 执行车牌识别算法
        catcher = lpr3.LicensePlateCatcher(detect_level=lpr3.DETECT_LEVEL_HIGH)
        results = catcher(cv_image)
        # pil_image = Image.fromarray(cv_image)  # 将 OpenCV 图像转换为 PIL 图像
            # draw = ImageDraw.Draw(pil_image)  # 创建绘制对象
        
        # corners_x1 = []
        # corners_y1 = []
        # corners_x2 = []
        # corners_y2 = []
        # plate_numbers = []
        # confidence_lic=[]
        font_ch = ImageFont.truetype("/data/yhy/ocr/ocrmap/src/license_plate_recognition/src/platech.ttf", 20, 0)
        if results is not None:
          for code, confidence, type_idx, box in results:
            # 在图像上绘制车牌框和文本
            # plate_number = code  # 车牌号
            x1, y1, x2, y2= box  # 四个角点
            print(code)
            print(box)
            # plate_confidence=confidence
            # # corners = [Point(x=x1,y=y1),Point(x=x2,y=y1),Point(x=x2,y=y2),Point(x=x1,y=y2)]
            # corners_x1.append(x1)
            # corners_x2.append(x2)
            # corners_y1.append(y1)
def main():
    rospy.init_node("license_plate_recognition_node", anonymous=True)
    rospy.Subscriber("/driver/fisheye/front/compressed", CompressedImage, license_plate_recognition)
    # rospy.Subscriber("/Inertial/gps/fix", NavSatFix, gps_time)
    rospy.spin()

if __name__ == '__main__':
    main()
