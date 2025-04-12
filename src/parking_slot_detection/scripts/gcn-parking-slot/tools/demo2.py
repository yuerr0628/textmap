import cv2
import time
import torch
import pprint
import numpy as np
from pathlib import Path
import glob
import math
import matplotlib.pyplot as plt
import random

import sys
sys.path.append('/home/sychen/Project/APA/ParkingDetection/parking_slot_detection/gcn-parking-slot')

from psdet.utils.config import get_config
from psdet.utils.common import get_logger
from psdet.models.builder import build_model

K_f = 0.74  # 前轴到前方的距离
K_r = 0.96  # 后轴到后方的距离
L_f = 1.56   # 前轴到车中心位置
L_r = 1.34  # 后轴到车中心位置
vehicle_length = K_f + K_r + L_f + L_r
L = L_f + L_r
vehicle_width = 2.0  # 车辆宽度
vehicle_r_min = 3.5  # 车辆最小转弯半径
parking_slot_length = 5.25
traj_thickness = 1

def get_r_fc(s, b1, b2, b3):
    square_value = (((s - b2 + K_r) ** 2 + b1 ** 2 + b1 * vehicle_width) / (2 * b1)) ** 2 + L_r ** 2
    return math.sqrt(square_value)


def get_r_sc(s, b1, b2, b3):
    square_value = (math.sqrt((s - b3) * (s - b3 -2 * K_f - 2 * L)) - vehicle_width / 2.0) ** 2 + L_r ** 2
    return math.sqrt(square_value)


def calculate_rectangle_vertices(traj_point, theta, vehicle_length_image, vehicle_width_image):
    # 计算矩形的旋转角度对应的正弦值和余弦值
    sin_val = math.sin(theta)
    cos_val = math.cos(theta)

    # 计算矩形的四个顶点坐标
    x1 = traj_point[0] - vehicle_length_image / 2 * cos_val - vehicle_width_image / 2 * sin_val
    y1 = traj_point[1] - vehicle_length_image / 2 * sin_val + vehicle_width_image / 2 * cos_val

    x2 = traj_point[0] + vehicle_length_image / 2 * cos_val - vehicle_width_image / 2 * sin_val
    y2 = traj_point[1] + vehicle_length_image / 2 * sin_val + vehicle_width_image / 2 * cos_val

    x3 = traj_point[0] + vehicle_length_image / 2 * cos_val + vehicle_width_image / 2 * sin_val
    y3 = traj_point[1] + vehicle_length_image / 2 * sin_val - vehicle_width_image / 2 * cos_val

    x4 = traj_point[0] - vehicle_length_image / 2 * cos_val + vehicle_width_image / 2 * sin_val
    y4 = traj_point[1] - vehicle_length_image / 2 * sin_val - vehicle_width_image / 2 * cos_val

    return [(int(x1), int(y1)), (int(x2), int(y2)), (int(x3), int(y3)), (int(x4), int(y4))]


def get_path(image, pred_dicts):
    slots_pred = pred_dicts['slots_pred']
    width = 512
    height = 512
    per_pixel_length = 0.02
    slots_path_param = []
    junctions = []
    junctions2 = []
    for j in range(len(slots_pred[0])):
        path_color = [random.randint(0, 255), random.randint(0, 255), random.randint(0, 255)]
        position = slots_pred[0][j][1]  # 车位入口线的位置
        p0_x = width * position[0] - 0.5
        p0_y = height * position[1] - 0.5
        p1_x = width * position[2] - 0.5
        p1_y = height * position[3] - 0.5

        parking_in_length = math.sqrt((p0_x - p1_x) ** 2 + (p0_y - p1_y) ** 2)
        if parking_in_length * per_pixel_length < 2.0:  # 车位小于2.0m，不可能泊入，排除一些误检
            continue
        b1 = (parking_in_length * per_pixel_length - vehicle_width)/2
        b2 = parking_slot_length  # 旁边障碍物的长度，现在直接把旁边车位视为障碍物
        b3 = 5.5  # 向前能够移动的距离，目前用道路宽度代替
        path_s = 0
        path_r = 0
        r_fc_list = []
        r_sc_list = []
        s_list = []
        for s in list(np.arange(0.1, 5.0, 0.1)):
            # print("current s: ", s)
            r_fc = get_r_fc(s, b1, b2, b3)
            r_sc = get_r_sc(s, b1, b2, b3)
            r_fc_list.append(r_fc)
            r_sc_list.append(r_sc)
            s_list.append(s)
            if r_fc < r_sc:
                # possible_r = (r_sc + r_fc) / 2.0
                possible_r = r_fc
                if possible_r > vehicle_r_min:
                    path_s = s
                    path_r = possible_r
                    print("find path s: ", path_s)
                    print("find possible r: ", path_r)
                    break

        # plt.plot(s_list, r_fc_list, label='r_fc')
        # plt.plot(s_list, r_sc_list, label='r_sc')
        # plt.legend()
        # plt.show()

        slots_path_param.append([path_s, path_r])
        # 在图片中绘制停车位
        vec = np.array([p1_x - p0_x, p1_y - p0_y])  # 对角线向量
        vec = vec / np.linalg.norm(vec)  # 对角线方向向量
        parking_theta = math.atan2(vec[1], vec[0])
        separating_length = parking_slot_length / per_pixel_length
        p2_x = p0_x + separating_length * vec[1]
        p2_y = p0_y - separating_length * vec[0]
        p3_x = p1_x + separating_length * vec[1]
        p3_y = p1_y - separating_length * vec[0]
        p0_x = int(round(p0_x))
        p0_y = int(round(p0_y))
        p1_x = int(round(p1_x))
        p1_y = int(round(p1_y))
        p2_x = int(round(p2_x))
        p2_y = int(round(p2_y))
        p3_x = int(round(p3_x))
        p3_y = int(round(p3_y))
        cv2.line(image, (p0_x, p0_y), (p1_x, p1_y), (255, 0, 0), 2)
        cv2.line(image, (p0_x, p0_y), (p2_x, p2_y), (255, 0, 0), 2)
        cv2.line(image, (p1_x, p1_y), (p3_x, p3_y), (255, 0, 0), 2)

        # 绘制泊车的轨迹
        in_center_x = (p0_x + p1_x) / 2.0
        in_center_y = (p0_y + p1_y) / 2.0
        # path_s = 1.0
        # path_r = 5.5
        parking_slot_center_x = in_center_x + separating_length / 2.0 * vec[1]
        parking_slot_center_y = in_center_y - separating_length / 2.0 * vec[0]
        first_stage_x = parking_slot_center_x - path_s / per_pixel_length * vec[1]
        first_stage_y = parking_slot_center_y + path_s / per_pixel_length * vec[0]

        cv2.line(image, (int(parking_slot_center_x), int(parking_slot_center_y)),
                 (int(first_stage_x), int(first_stage_y)), path_color, traj_thickness)
        # 第二段轨迹是四分之一圆，圆心由第一段轨迹末端导出
        second_stage_center_x = first_stage_x + path_r / per_pixel_length * vec[0]
        second_stage_center_y = first_stage_y + path_r / per_pixel_length * vec[1]
        # cv2.line(image, (int(first_stage_x), int(first_stage_y)),
        #          (int(second_stage_center_x), int(second_stage_center_y)), (0, 255, 0), 2)

        # 生成轨迹
        start_angle = parking_theta + math.pi / 2.0
        traj_point = []
        for angle in list(np.arange(0.0, math.pi / 2.0, 0.05)):
            current_angle = angle + start_angle
            radius = path_r / per_pixel_length
            x = second_stage_center_x + radius * np.cos(current_angle)
            y = second_stage_center_y + radius * np.sin(current_angle)
            traj_point.append([int(x), int(y)])
        for i in range(len(traj_point) - 1):
            cv2.line(image, traj_point[i], traj_point[i+1], path_color, traj_thickness)
            current_angle = math.atan2(traj_point[i+1][1] - traj_point[i][1], traj_point[i+1][0] - traj_point[i][0])
            vehicle_width_image = vehicle_width / per_pixel_length
            vehicle_length_image = vehicle_length / per_pixel_length
            rectangle_vertices = calculate_rectangle_vertices(traj_point[i], current_angle, vehicle_length_image, vehicle_width_image)
            for i in range(4):
                cv2.line(image, rectangle_vertices[i%4], rectangle_vertices[(i+1)%4], path_color, traj_thickness)
        # print("len(traj_point): ", len(traj_point))
        # cv2.ellipse(image, (int(second_stage_center_x), int(second_stage_center_y)), (int(path_r / per_pixel_length), int(path_r / per_pixel_length)), 0, -90 - parking_theta, 0 - parking_theta, (0, 255, 0), 2)
        traj_point.append([int(first_stage_x), int(first_stage_y)])
        # 记录车位的关键点
        junctions.append((p0_x, p0_y))
        junctions.append((p1_x, p1_y))
        junctions2.append((p2_x, p2_y))
        junctions2.append((p3_x, p3_y))
    for junction in junctions:
        cv2.circle(image, junction, 3,  (0, 0, 255), 4)
    for junction in junctions2:
        cv2.circle(image, junction, 3,  (0, 255, 0), 4)

    return image

def draw_parking_slot(image, pred_dicts):
    slots_pred = pred_dicts['slots_pred']

    width = 512
    height = 512
    VSLOT_MIN_DIST = 0.044771278151623496
    VSLOT_MAX_DIST = 0.1099427457599304
    HSLOT_MIN_DIST = 0.15057789144568634
    HSLOT_MAX_DIST = 0.44449496544202816

    SHORT_SEPARATOR_LENGTH = 0.199519231
    LONG_SEPARATOR_LENGTH = 0.46875
    junctions = []
    junctions2 = []
    for j in range(len(slots_pred[0])):
        position = slots_pred[0][j][1]  # 车位对角线的位置
        p0_x = width * position[0] - 0.5
        p0_y = height * position[1] - 0.5
        p1_x = width * position[2] - 0.5
        p1_y = height * position[3] - 0.5
        vec = np.array([p1_x - p0_x, p1_y - p0_y])  # 对角线向量
        vec = vec / np.linalg.norm(vec)  # 对角线方向向量
        # 从实际画的值来看，这两个点应该是车辆入口的两个点
        distance =( position[0] - position[2] )**2 + ( position[1] - position[3] )**2 # 这个距离的平方是归一化之后的
        # p0,p1进入车位的两个点，然后判断这两个点是车位的长还是宽
        if VSLOT_MIN_DIST <= distance <= VSLOT_MAX_DIST:
            separating_length = LONG_SEPARATOR_LENGTH
        else:
            separating_length = SHORT_SEPARATOR_LENGTH
        # 计算其余两个点
        p2_x = p0_x + height * separating_length * vec[1]
        p2_y = p0_y - width * separating_length * vec[0]
        p3_x = p1_x + height * separating_length * vec[1]
        p3_y = p1_y - width * separating_length * vec[0]
        p0_x = int(round(p0_x))
        p0_y = int(round(p0_y))
        p1_x = int(round(p1_x))
        p1_y = int(round(p1_y))
        p2_x = int(round(p2_x))
        p2_y = int(round(p2_y))
        p3_x = int(round(p3_x))
        p3_y = int(round(p3_y))
        cv2.line(image, (p0_x, p0_y), (p1_x, p1_y), (255, 0, 0), 2)
        cv2.line(image, (p0_x, p0_y), (p2_x, p2_y), (255, 0, 0), 2)
        cv2.line(image, (p1_x, p1_y), (p3_x, p3_y), (255, 0, 0), 2)

        #cv2.circle(image, (p0_x, p0_y), 3,  (0, 0, 255), 4)
        junctions.append((p0_x, p0_y))
        junctions.append((p1_x, p1_y))
        junctions2.append((p2_x, p2_y))
        junctions2.append((p3_x, p3_y))
    for junction in junctions:
        cv2.circle(image, junction, 3,  (0, 0, 255), 4)
    for junction in junctions2:
        cv2.circle(image, junction, 3,  (0, 255, 0), 4)

    return image


def main():

    cfg = get_config()
    logger = get_logger(cfg.log_dir, cfg.tag)
    logger.info(pprint.pformat(cfg))

    model = build_model(cfg.model)
    logger.info(model)
    
    image_dir = Path(cfg.data_root) / 'testing' / 'outdoor-normal daylight'
    display = True

    # load checkpoint
    model.load_params_from_file(filename=cfg.ckpt, logger=logger, to_cpu=False)
    model.cuda()
    model.eval()
    
    if display:
        car = cv2.imread('../images/car.png')
        car = cv2.resize(car, (512, 512))

    video = cv2.VideoWriter("../output_video.mp4", cv2.VideoWriter_fourcc(*'mp4v'), 5, (512, 512))

    with torch.no_grad():

        for img_idx, img_path in enumerate(glob.glob("../../avm_images/*.png")):
            img_name = img_path.split('/')[-1].split('.')[0]
            data_dict = {} 
            image = cv2.imread(str(img_path))
            image0 = cv2.resize(image, (512, 512))
            image = image0/255.
            # img_name: 001
            # image: (512, 512, 3)

            data_dict['image'] = torch.from_numpy(image).float().permute(2, 0, 1).unsqueeze(0).cuda()

            start_time = time.time()
            pred_dicts, ret_dict = model(data_dict)
            # print("pred_dicts: ", pred_dicts)
            # print("ret_dicts: ", ret_dict)

            sec_per_example = (time.time() - start_time)
            print('Info speed: %.4f ms per example.' % (sec_per_example * 1000))

            if display:
                # image = draw_parking_slot(image0, pred_dicts)
                image = get_path(image0, pred_dicts)
                # image[145:365, 210:300] = 0
                # image += car
                # cv2.imshow('image',image.astype(np.uint8))
                # cv2.waitKey(0)
                video.write(image.astype(np.uint8))
                cv2.imwrite("../demo_img/{}.jpg".format(img_idx), image.astype(np.uint8))

                
                # save_dir = Path(cfg.output_dir) / 'predictions'
                # save_dir.mkdir(parents=True, exist_ok=True)
                # save_path = save_dir / ('%s.jpg' % img_name)
                # cv2.imwrite(str(save_path), image)
    if display:
        cv2.destroyAllWindows()
    video.release()


if __name__ == '__main__':
    main()
