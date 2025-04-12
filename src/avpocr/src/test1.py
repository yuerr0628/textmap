#!/usr/bin/env python3
# coding=utf-8

import rospy
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import NavSatFix
# from cv_bridge import CvBridge
import cv2
import numpy as np
from paddleocr import PaddleOCR
from PIL import Image, ImageDraw, ImageFont
import logging
from std_msgs.msg import Header
from std_msgs.msg import String
from geometry_msgs.msg import Polygon, Point32
import time


logging.disable(logging.DEBUG)
logging.disable(logging.WARNING)

ocr = PaddleOCR(use_angle_cls=True)
print("enter")

def avpocr(compressed_image):

  bridge = CvBridge()
  
  np_arr = np.fromstring(compressed_image.data, np.uint8)

  cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
      # 正确的用法
  cv_image = cv2.resize(cv_image, (512, 512))
  img_pil = Image.fromarray(cv_image)
      # 使用PaddleOCR识别车位号码
      # st=time.time()
  result = ocr.ocr(cv_image)
      # print(time.time()-st)
      # print(cv_image)
      # print(result)
  draw = ImageDraw.Draw(img_pil)
      # 在图像上绘制识别结果
      # for line in result:
      #      print(line)
      # font_ch = ImageFont.truetype("/home/user/下载/platech.ttf", 26, 0)
  result=result[0]
      # print(result)
  if result is not None:
      boxes = [line[0] for line in result]
      txts = [line[1][0] for line in result]
      scores = [line[1][1] for line in result]
      
      # 遍历边界框
      for box, text,score in zip(boxes, txts,scores):
        x1 = box[0][0]
        y1 =box[0][1]
        x2 = box[1][0]
        y2 = box[2][1]
          #  在图像上绘制边界框
        draw.rectangle([(x1, y1), (x2, y2)], outline=(0, 255, 0))
          # 创建带有置信度的文本
        text_with_confidence = f"{text} ({score:.2f})"
          # print(time.time()-st)  
          # 在图像上绘制文本
          # draw.text((x1, y1 - 10), text_with_confidence, fill=(0, 255, 0),font)
          # 创建 Header 对象，设置时间戳和其他字段
        x1_1 = 540-box[0][0]
        y1_1 = 480-box[0][1]
        x2_1 = 540-box[1][0]
        y2_1= 480-box[2][1]

  cv_image = np.array(img_pil)
    # gps_1 = gps_1 + 1
  # cv2.imshow("Result", cv_image)
  # cv2.waitKey(1)


def main():
    print("enter")
    rospy.init_node("avpocr_node", anonymous=True)
    rospy.Subscriber("/driver/fisheye/avm/compressed", CompressedImage, avpocr)
    # rospy.Subscriber("/Inertial/gps/fix", NavSatFix, gps_time)
    rospy.spin()

if __name__ == '__main__':
    print("enter")
    main()
