#!usr/bin/env/ python
# _*_ coding:utf-8 _*_

import cv2 as cv
import numpy as np
import os
from src.homography import get_homography
from src.intrinsics import get_intrinsics_param
from src.extrinsics import get_extrinsics_param
from src.distortion import get_distortion
from src.refine_all import refinall_all_param


def calibrate():
    # 求单应矩阵
    H = get_homography(pic_points, real_points_x_y)

    # 求内参
    intrinsics_param = get_intrinsics_param(H)

    # 求对应每幅图外参
    extrinsics_param = get_extrinsics_param(H, intrinsics_param)

    # 畸变矫正
    k = get_distortion(intrinsics_param, extrinsics_param, pic_points, real_points_x_y)

    # 微调所有参数
    [new_intrinsics_param, new_k, new_extrinsics_param] = refinall_all_param(intrinsics_param,
                                                                             k, extrinsics_param, real_points,
                                                                             pic_points)

    print("intrinsics_parm:\t", new_intrinsics_param)
    print("distortionk:\t", new_k)
    print("extrinsics_parm:\t", new_extrinsics_param)


def on_EVENT_LBUTTONDOWN(event, x, y, flags, param):
    if event == cv.EVENT_LBUTTONDOWN:
        xy = "%d,%d" % (x, y)
        coordinate.append((x, y))
        cv.circle(img, (x, y), 1, (0, 0, 255), thickness = -1)
        # cv.putText(img, xy, (x, y), cv.FONT_HERSHEY_PLAIN,
        #             1.0, (0,0,0), thickness = 1)
        cv.imshow("image", img)


# 摄像头畸变矫正函数，输入待矫正的图形变量
def undistort(frame):
    fx = 3067.45238223608
    cx = 2015.66479200130
    fy = 3073.45330990495
    cy = 1502.75437872166
    k1, k2, p1, p2, k3 = 0.344094299741459, -1.16352087196631, 2.89410448050091e-06, -0.00231497410712017, 0.0

    # 相机坐标系到像素坐标系的转换矩阵
    k = np.array([
        [fx, -0.193080475659317, cx],
        [0, fy, cy],
        [0, 0, 1]
    ])
    # 畸变系数
    d = np.array([
        k1, k2, p1, p2, k3
    ])
    height, weight = frame.shape[:2]
    mapx, mapy = cv.initUndistortRectifyMap(k, d, None, k, (weight, height), 5)

    # 返回矫正好的图形变量
    return cv.remap(frame, mapx, mapy, cv.INTER_LINEAR)


if __name__ == "__main__":
    file_dir = r'D:\\XXX\\temp\\board'
    # 标定所用图像
    pic_name = os.listdir(file_dir)

    # 由于棋盘为二维平面，设定世界坐标系在棋盘上，一个单位代表一个棋盘宽度，产生世界坐标系三维坐标
    cross_corners = [11, 7]  # 棋盘方块交界点排列 11 7
    real_coor = np.zeros((cross_corners[0] * cross_corners[1], 3), np.float32)
    real_coor[:, :2] = np.mgrid[0:11, 0:7].T.reshape(-1, 2)

    real_points = []
    real_points_x_y = []
    pic_points = []
    coordinate = []

    for pic in pic_name:
        pic_path = os.path.join(file_dir, pic)
        pic_data = cv.imread(pic_path)

        # 寻找到棋盘角点
        succ, pic_coor = cv.findChessboardCorners(pic_data, (cross_corners[0], cross_corners[1]), None)

        if succ:
            # 添加每幅图的对应3D-2D坐标
            pic_coor = pic_coor.reshape(-1, 2)
            pic_points.append(pic_coor)

            real_points.append(real_coor)
            real_points_x_y.append(real_coor[:, :2])
    calibrate()

    img = cv.imread("C:\\Users\\install\\Desktop\\pic\\pic\\test\\R444_2.jpg")
    h, w, c = img.shape
    undistort(img)
    cv.namedWindow("image")
    cv.setMouseCallback("image", on_EVENT_LBUTTONDOWN)
    while (1):
        cv.imshow("image", img)
        if cv.waitKey(0) & 0xFF == 27:
            break
    cv.destroyAllWindows()

    src_list = [coordinate[0], coordinate[1], coordinate[2], coordinate[3]]
    # src_list = [(395, 366), (438, 876), (1110, 872), (1037, 192)]
    for i, pt in enumerate(src_list):
        cv.circle(img, pt, 5, (0, 0, 255), -1)
        cv.putText(img, str(i + 1), (pt[0] + 5, pt[1] + 10), cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    pts1 = np.float32(src_list)
    pts2 = np.float32([[0, 0], [0, w - 2], [h - 2, w - 2], [h - 2, 0]])
    matrix = cv.getPerspectiveTransform(pts1, pts2)
    result = cv.warpPerspective(img, matrix, (h, w))
    cv.imshow("Image", img)
    cv.imshow("Perspective transformation", result)
    cv.waitKey(0)

    print('OK')