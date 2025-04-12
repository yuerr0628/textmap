import cv2
import numpy as np
from cam_model import *

# 读取图像
image = cv2.imread('/home/user/图片/test1.jpg')

corners = np.array([[71,129], [198,132], [65, 142], [201,144]])
corners = corners.T
print(corners)
points_depth = np.array([0.5, 0.5,0.2, 0.2])




cm = CamModel("left")

PointCam = cm. image2cam(corners, points_depth)
print(PointCam)



# # 检查是否成功读取图像
# if image is not None:
#     # 显示图像
#     cv2.imshow('Image', image)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()
# else:
#     print("Failed to read image.")