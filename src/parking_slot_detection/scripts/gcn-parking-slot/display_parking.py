import cv2
import json

if __name__ == '__main__':
    for i in range(1, 100):
        annotation_file_path = "./datasets/parking_slot/ps_json_label/testing/all/{:0>4d}.json".format(i)
        image_file_path = "./datasets/parking_slot/testing/all/{:0>4d}.jpg".format(i)

        # 打开JSON文件
        with open(annotation_file_path, 'r') as f:
            # 读取并解析JSON文件
            annotation_data = json.load(f)
        # print("annotation_data: ", annotation_data)
        annotation_marks = annotation_data["marks"]
        annotation_slots = annotation_data["slots"]

        if len(annotation_slots) == 0:
            continue

        if not isinstance(annotation_slots[0], list):
            annotation_slots = [annotation_slots]

        print("annotation_marks: ", annotation_marks)
        print("annotation_slots: ", annotation_slots)

        image = cv2.imread(image_file_path)
        for idx, slots_info in enumerate(annotation_slots):
            point_idx_list = slots_info[0:2]
            print("type: ", slots_info[2])
            for point_idx in point_idx_list:
                annotation_mark = annotation_marks[point_idx]
                cv2.circle(image, (int(annotation_mark[0]), int(annotation_mark[1])), 5, (255, 0, 0), -1)
                cv2.circle(image, (int(annotation_mark[2]), int(annotation_mark[3])), 5, (0, 255, 0), -1)
            # break
        # for points in annotation_marks:
        #     cv2.circle(image, (int(points[0]), int(points[1])), 5, (255, 0, 0), -1)
        #     cv2.circle(image, (int(points[2]), int(points[3])), 5, (0, 255, 0), -1)
        #     break

        cv2.imshow("images", image)
        cv2.waitKey(0)
