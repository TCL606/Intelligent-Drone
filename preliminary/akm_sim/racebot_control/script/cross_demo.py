#!/usr/bin/env python
# coding=utf-8

import rospy
import signal
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
import math
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import time


class CrossDemo:

    def __init__(self, robot_name):
        signal.signal(signal.SIGINT, self.sigint_handler)
        self.robot_name = robot_name
        self.cmd_twist = Twist()
        self.pose = Pose()
        self.laser = LaserScan()
        self.BoolSub_ = rospy.Subscriber('/tello/cmd_start', Bool,
                                         self.startcommandCallback)
        self.odom_sub = rospy.Subscriber(self.robot_name + '/odom', Odometry,
                                         self.odom_callback)
        self.laser_sub = rospy.Subscriber(self.robot_name + '/scan', LaserScan,
                                          self.laser_callback)
        self.image_sub_ = rospy.Subscriber("/AKM_1/camera/rgb/image_raw",
                                           Image, self.imagesubCallback)
        self.park_pub = rospy.Publisher("/AKM_1/parkstate",
                                        String,
                                        queue_size=1)
        self.cmd_pub = rospy.Publisher(self.robot_name + '/cmd_vel',
                                       Twist,
                                       queue_size=1)
        self.drone_sub = rospy.Subscriber('/message_node', String,
                                          self.RcvDmsg)
        self.res_pub = rospy.Publisher("/target_result", String, queue_size=1)

        self.delta_t = 0.2
        self.drone_msg = None

        self.is_begin_ = False

        self.ball_color = ['e', 'e', 'e', 'e', 'e']
        self.found_ball = False
        self.is_end = False
        # self.cruise_path = [[2.5, 6], [1, 7], [
        #     2, 12.5], [3.5, 13], [3, 14], [4, 14]]
        self.cruise_path = [[2.5, 6], [1, 7], [2, 12.5], [4, 13], [5.5, 13],
                            [3, 14], [4, 14]]
        self.spin_point = [[1, 7, 0, 1], [4, 13, -math.pi / 2, 3],
                           [5.5, 13, -math.pi / 2, 2], [3, 14, math.pi, 4]]

        self.Main()
        rospy.spin()

    def Main(self):
        while not rospy.is_shutdown():
            if self.is_begin_ and not self.is_end:
                while self.pose.position.y < 3:
                    # print(self.pose.position.y)
                    self.CarMove(1, 0)
                    rospy.sleep(self.delta_t)
                rospy.loginfo("Across the gate")
                for point in self.cruise_path:
                    # _ = self.image_recognition(self.orgFrame)
                    while math.sqrt(
                        (self.pose.position.x - point[0])**2 +
                        (self.pose.position.y - point[1])**2) > 0.2:
                        v, w = self.NextMove(point[0], point[1])
                        self.CarMove(v, w)
                        rospy.sleep(self.delta_t)
                    for sp_point in self.spin_point:
                        if sp_point[0] == point[0] and sp_point[1] == point[1]:
                            target_angle = sp_point[2]
                            while True:
                                yaw = self.Q2E(self.pose.orientation)
                                ang_temp = yaw - target_angle
                                while ang_temp > math.pi:
                                    ang_temp -= 2 * math.pi
                                while ang_temp < -math.pi:
                                    ang_temp += 2 * math.pi
                                if abs(ang_temp) < math.pi / 18:
                                    self.CarMove(0, 0)
                                    if sp_point[3] == 2:
                                        cv2.imwrite('/home/thudrone/test.jpg',
                                                    self.orgFrame)
                                        self.ball_color[
                                            2] = self.image_recognition_car(
                                                self.orgFrame)[0]
                                        self.ball_color[
                                            0] = self.image_recognition_car(
                                                self.orgFrame)[1]
                                    else:
                                        self.ball_color[sp_point[
                                            3]] = self.image_recognition(
                                                self.orgFrame)
                                    rospy.sleep(self.delta_t)
                                    break
                                else:
                                    omega = ang_temp / math.pi
                                    if omega > 0:
                                        self.CarMove(0.2, -1)
                                    else:
                                        self.CarMove(0.2, 1)
                                    rospy.sleep(self.delta_t)
                self.CarMove(0, 0)
                self.car_msg = ''.join(self.ball_color)
                print(self.ball_color)
                park = String()
                park.data = 'PARKING'
                self.park_pub.publish(park)
                self.is_end = True
                rospy.loginfo("Racecar reached, stop!")
                while not self.drone_msg:
                    pass
                print(self.drone_msg)
                self.ans_comb()
                res = String()
                res.data = self.summary
                self.res_pub.publish(res)
            else:
                pass

    def ans_comb(self):  # 合并信息
        for i in range(5):
            if self.car_msg[i] != 'e':
                self.drone_msg[i] = self.car_msg[i]
        if not 'r' in self.drone_msg:
            self.drone_msg[1] = 'r'
        if not 'y' in self.drone_msg:
            self.drone_msg[1] = 'y'
        if not 'b' in self.drone_msg:
            self.drone_msg[1] = 'b'
        self.summary = ''.join(self.drone_msg)

    def odom_callback(self, msg):
        self.pose = msg.pose.pose

    def laser_callback(self, msg):
        self.laser = msg

    def imagesubCallback(self, data):
        try:
            bridge_ = CvBridge()
            self.orgFrame = bridge_.imgmsg_to_cv2(data, 'bgr8')

        except CvBridgeError as err:
            print(err)

    def Q2E(self, q):
        yaw = math.atan2(2 * (q.x * q.y + q.w * q.z),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        return yaw

    # def image_recognition(self, img):
    #     hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    #     # Red color range 0, 235, 120, 10, 250, 130
    #     red_lower_range = np.array([0, 235, 120])
    #     red_upper_range = np.array([10, 250, 130])
    #     # Blue color range  110, 50, 50 , 130, 255, 255
    #     blue_lower_range = np.array([110, 50, 50])
    #     blue_upper_range = np.array([130, 255, 255])
    #     # Yellow color range  20, 43, 46 , 34, 255, 255
    #     yellow_lower_range = np.array([20, 43, 46])
    #     yellow_upper_range = np.array([34, 255, 255])

    #     red_mask = cv2.inRange(hsv, red_lower_range, red_upper_range)
    #     blue_mask = cv2.inRange(hsv, blue_lower_range, blue_upper_range)
    #     yellow_mask = cv2.inRange(hsv, yellow_lower_range, yellow_upper_range)
    # #cv2.imwrite("./red_mask.png", red_mask)

    #     if (np.sum(blue_mask == 255) > 200):
    #         return 0  # blue
    #     elif (np.sum(yellow_mask == 255) > 200):
    #         return 1  # yellow
    #     elif (np.sum(img[:, :, 0] + img[:, :, 1] == 0) + np.sum(red_mask == 255) > 100):
    #         return 2  # red
    #     else:
    #         return 3  # error

    def image_recognition(self, img):
        height = img.shape[0]
        width = img.shape[1]

        # Red color range 0, 235, 120, 10, 250, 130
        red_range = [(0, 235, 120), (10, 250, 130)]
        # Blue color range  94, 80, 2, 120, 255, 255
        blue_range = [(94, 80, 2), (120, 255, 255)]
        # Yellow color range  20, 43, 46 , 34, 255, 255
        yellow_range = [(20, 43, 46), (34, 255, 255)]

        frame = cv2.resize(img, (width, height),
                           interpolation=cv2.INTER_CUBIC)  # 将图片缩放
        frame = cv2.GaussianBlur(frame, (3, 3), 0)  # 高斯模糊
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
        h, s, v = cv2.split(frame)  # 分离出各个HSV通道
        v = cv2.equalizeHist(v)  # 直方图化
        frame = cv2.merge((h, s, v))  # 合并三个通道

        red_frame = cv2.inRange(frame, red_range[0],
                                red_range[1])  # 对原图像和掩模进行位运算
        blue_frame = cv2.inRange(frame, blue_range[0],
                                 blue_range[1])  # 对原图像和掩模进行位运算
        yellow_frame = cv2.inRange(frame, yellow_range[0],
                                   yellow_range[1])  # 对原图像和掩模进行位运算

        red_opened = cv2.morphologyEx(red_frame, cv2.MORPH_OPEN,
                                      np.ones((3, 3), np.uint8))  # 开运算
        red_closed = cv2.morphologyEx(red_opened, cv2.MORPH_CLOSE,
                                      np.ones((3, 3), np.uint8))  # 闭运算
        red_closed[(
            (img[:, :, 0] < 5) & (img[:, :, 1] < 5) &
            (img[:, :, 2] > 100))] = 255  # modified red_closed with RGB
        _, red_contours, _ = cv2.findContours(red_closed, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_NONE)  # 找出轮廓

        blue_opened = cv2.morphologyEx(blue_frame, cv2.MORPH_OPEN,
                                       np.ones((3, 3), np.uint8))  # 开运算
        blue_closed = cv2.morphologyEx(blue_opened, cv2.MORPH_CLOSE,
                                       np.ones((3, 3), np.uint8))  # 闭运算
        _, blue_contours, _ = cv2.findContours(blue_closed, cv2.RETR_EXTERNAL,
                                            cv2.CHAIN_APPROX_NONE)  # 找出轮廓

        yellow_opened = cv2.morphologyEx(yellow_frame, cv2.MORPH_OPEN,
                                         np.ones((3, 3), np.uint8))  # 开运算
        yellow_closed = cv2.morphologyEx(yellow_opened, cv2.MORPH_CLOSE,
                                         np.ones((3, 3), np.uint8))  # 闭运算
        _, yellow_contours, _ = cv2.findContours(yellow_closed, cv2.RETR_EXTERNAL,
                                              cv2.CHAIN_APPROX_NONE)  # 找出轮廓
        # cv2.imwrite("./red_new.jpg", red_closed)
        # cv2.imwrite("./blue_new.jpg", blue_closed)
        # cv2.imwrite("./yellow_new.jpg", yellow_closed)
        # print('blue:', np.sum(blue_closed == 255))
        # print('yellow:', np.sum(yellow_closed == 255))
        # print('red:', np.sum(red_closed == 255))
        # print(
        #     'red_modified:',
        #     np.sum(
        #         ((img[:, :, 0] < 5) & (img[:, :, 1] < 5) & (img[:, :, 2] > 100))))

        red_contour_area_max = 0
        for c in red_contours:  # 遍历所有轮廓
            contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
            if contour_area_temp > red_contour_area_max:
                red_contour_area_max = contour_area_temp

        blue_contour_area_max = 0
        for c in blue_contours:  # 遍历所有轮廓
            contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
            if contour_area_temp > blue_contour_area_max:
                blue_contour_area_max = contour_area_temp

        yellow_contour_area_max = 0
        for c in yellow_contours:  # 遍历所有轮廓
            contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
            if contour_area_temp > yellow_contour_area_max:
                yellow_contour_area_max = contour_area_temp

        contour_area_max = max(blue_contour_area_max, yellow_contour_area_max,
                               red_contour_area_max)
        # print('max_contour:', contour_area_max)
        # print('max_blue_contour:', blue_contour_area_max)
        # print('max_yellow_contour:', yellow_contour_area_max)
        # print('max_red_contour:', red_contour_area_max)

        if (contour_area_max < 500):
            # cv2.imwrite("/home/thudrone/err/" +
            # time.strftime("%H%M%S", time.localtime()) + '.jpg', img)
            return 'e'  # error
        elif (contour_area_max == blue_contour_area_max):
            # cv2.imwrite("/home/thudrone/blue/" +
            # time.strftime("%H%M%S", time.localtime()) + '.jpg', img)
            return 'b'  # blue
        elif (contour_area_max == yellow_contour_area_max):
            # cv2.imwrite("/home/thudrone/yellow/" +
            # time.strftime("%H%M%S", time.localtime()) + '.jpg', img)
            return 'y'  # yellow
        else:
            # cv2.imwrite("/home/thudrone/red/" +
            # time.strftime("%H%M%S", time.localtime()) + '.jpg', img)
            return 'r'  # red

    def image_recognition_car(self, img):
        height = img.shape[0]
        width = img.shape[1]

        # Red color range 0, 235, 120, 10, 250, 130
        red_range = [(0, 235, 120), (10, 250, 130)]
        # Blue color range  94, 80, 2, 120, 255, 255
        blue_range = [(94, 80, 2), (120, 255, 255)]
        # Yellow color range  20, 43, 46 , 34, 255, 255
        yellow_range = [(20, 43, 46), (34, 255, 255)]

        frame = cv2.resize(img, (width, height),
                           interpolation=cv2.INTER_CUBIC)  # 将图片缩放
        frame = cv2.GaussianBlur(frame, (3, 3), 0)  # 高斯模糊
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
        h, s, v = cv2.split(frame)  # 分离出各个HSV通道
        v = cv2.equalizeHist(v)  # 直方图化
        frame = cv2.merge((h, s, v))  # 合并三个通道

        red_frame = cv2.inRange(frame, red_range[0],
                                red_range[1])  # 对原图像和掩模进行位运算
        blue_frame = cv2.inRange(frame, blue_range[0],
                                 blue_range[1])  # 对原图像和掩模进行位运算
        yellow_frame = cv2.inRange(frame, yellow_range[0],
                                   yellow_range[1])  # 对原图像和掩模进行位运算

        red_opened = cv2.morphologyEx(red_frame, cv2.MORPH_OPEN,
                                      np.ones((3, 3), np.uint8))  # 开运算
        red_closed = cv2.morphologyEx(red_opened, cv2.MORPH_CLOSE,
                                      np.ones((3, 3), np.uint8))  # 闭运算
        red_closed[(
            (img[:, :, 0] < 8) & (img[:, :, 1] < 8) &
            (img[:, :, 2] > 100))] = 255  # modified red_closed with RGB
        _, red_contours, _ = cv2.findContours(red_closed, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_NONE)  # 找出轮廓

        blue_opened = cv2.morphologyEx(blue_frame, cv2.MORPH_OPEN,
                                       np.ones((3, 3), np.uint8))  # 开运算
        blue_closed = cv2.morphologyEx(blue_opened, cv2.MORPH_CLOSE,
                                       np.ones((3, 3), np.uint8))  # 闭运算
        _, blue_contours, _ = cv2.findContours(blue_closed, cv2.RETR_EXTERNAL,
                                            cv2.CHAIN_APPROX_NONE)  # 找出轮廓

        yellow_opened = cv2.morphologyEx(yellow_frame, cv2.MORPH_OPEN,
                                         np.ones((3, 3), np.uint8))  # 开运算
        yellow_closed = cv2.morphologyEx(yellow_opened, cv2.MORPH_CLOSE,
                                         np.ones((3, 3), np.uint8))  # 闭运算
        _, yellow_contours, _ = cv2.findContours(yellow_closed, cv2.RETR_EXTERNAL,
                                              cv2.CHAIN_APPROX_NONE)  # 找出轮廓
        cv2.imwrite("./red_new.jpg", red_closed)
        cv2.imwrite("./blue_new.jpg", blue_closed)
        cv2.imwrite("./yellow_new.jpg", yellow_closed)
        print('blue:', np.sum(blue_closed == 255))
        print('yellow:', np.sum(yellow_closed == 255))
        print('red:', np.sum(red_closed == 255))
        print(
            'red_modified:',
            np.sum(((img[:, :, 0] < 8) & (img[:, :, 1] < 8) &
                    (img[:, :, 2] > 100))))

        red_contour_area_max = 0
        red_area_max_contour = None
        for c in red_contours:  # 遍历所有轮廓
            contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
            if contour_area_temp > red_contour_area_max:
                red_contour_area_max = contour_area_temp
                red_area_max_contour = c

        blue_contour_area_max = 0
        blue_area_max_contour = None
        for c in blue_contours:  # 遍历所有轮廓
            contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
            if contour_area_temp > blue_contour_area_max:
                blue_contour_area_max = contour_area_temp
                blue_area_max_contour = c

        yellow_contour_area_max = 0
        yellow_area_max_contour = None
        for c in yellow_contours:  # 遍历所有轮廓
            contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
            if contour_area_temp > yellow_contour_area_max:
                yellow_contour_area_max = contour_area_temp
                yellow_area_max_contour = c

        if red_contour_area_max != 0:
            red_center = np.mean(red_area_max_contour, axis=0)
            print(red_center[0, 0])
        if blue_contour_area_max != 0:
            blue_center = np.mean(blue_area_max_contour, axis=0)
            print(blue_center[0, 0])
        if yellow_contour_area_max != 0:
            yellow_center = np.mean(yellow_area_max_contour, axis=0)
            print(yellow_center[0, 0])

        contour_area_max = max(blue_contour_area_max, yellow_contour_area_max,
                               red_contour_area_max)
        # print('max_contour:', contour_area_max)
        # print('max_blue_contour:', blue_contour_area_max)
        # print('max_yellow_contour:', yellow_contour_area_max)
        # print('max_red_contour:', red_contour_area_max)

        if ((red_contour_area_max > 300) + (blue_contour_area_max > 300) +
            (yellow_contour_area_max > 300) > 1):
            if (red_contour_area_max == 0
                    and blue_center[0, 0] < yellow_center[0, 0]):
                return ['y', 'b']
            elif (red_contour_area_max == 0
                  and blue_center[0, 0] > yellow_center[0, 0]):
                return ['b', 'y']
            elif (blue_contour_area_max == 0
                  and red_center[0, 0] < yellow_center[0, 0]):
                return ['y', 'r']
            elif (blue_contour_area_max == 0
                  and red_center[0, 0] > yellow_center[0, 0]):
                return ['r', 'y']
            elif (yellow_contour_area_max == 0
                  and blue_center[0, 0] < red_center[0, 0]):
                return ['r', 'b']
            else:
                return ['b', 'r']
        elif (contour_area_max < 300):
            return ['e', 'e']
        elif (contour_area_max == blue_contour_area_max
              and blue_center[0, 0] < 450):
            return ['e', 'b']  # blue
        elif (contour_area_max == blue_contour_area_max
              and blue_center[0, 0] > 450):
            return ['b', 'e']  # blue
        elif (contour_area_max == yellow_contour_area_max
              and yellow_center[0, 0] < 450):
            return ['e', 'y']  # yellow
        elif (contour_area_max == yellow_contour_area_max
              and yellow_center[0, 0] > 450):
            return ['y', 'e']  # yellow
        elif (contour_area_max == red_contour_area_max
              and red_center[0, 0] < 450):
            return ['e', 'r']  # red
        else:
            return ['r', 'e']  # red

    def NextMove(self, target_x, target_y):
        boudary = list(self.laser.ranges)
        self_x = self.pose.position.x
        self_y = self.pose.position.y
        angle = math.atan2(target_y - self_y, target_x - self_x)
        yaw = self.Q2E(self.pose.orientation)
        ang_temp = math.pi - yaw + angle
        while ang_temp > 2 * math.pi:
            ang_temp -= 2 * math.pi
        while ang_temp < 0:
            ang_temp += 2 * math.pi
        angle_quan = int(math.floor(ang_temp / math.pi * 180 / 0.5))
        dist = math.sqrt((target_y - self_y)**2 + (target_x - self_x)**2)

        # res =  self.Match(boudary, yaw, self_x, self_y)
        # if res is not None:
        #     ball_x, ball_y, theta = res
        #     v = 0.1
        #     theta_thre = 10 / 180 * math.pi
        #     if abs(theta) < theta_thre:
        #         self.ball_place.append([ball_x, ball_y])
        #         print(ball_x, ball_y)
        #         cv2.imshow('ball', self.orgFrame)
        #         cv2.waitKey(0)
        #     elif theta > 0:
        #         print('spanning')
        #         return v,  -abs(theta) * 4
        #     else:
        #         print('spanning')
        #         return v, abs(theta) * 4
        # el
        if dist <= boudary[angle_quan]:
            delta = yaw - angle
            while delta > math.pi:
                delta -= 2 * math.pi
            while delta < -math.pi:
                delta += 2 * math.pi
            omega = delta / math.pi
            amp = 1.2
            if omega > 0:
                return (amp - abs(omega)) / amp, -abs(omega)
            else:
                return (amp - abs(omega)) / amp, abs(omega)
        else:
            return 0.5, 0

    def FuncFit(self, x, y, d_e_f):
        return x**2 + y**2 + np.dot(d_e_f.flatten(), np.array([x, y, 1]))

    def RcvDmsg(self, msg):
        self.drone_msg = list(msg.data)

    # def Match(self, boundary, yaw, self_x, self_y):
    #     radius = 0.5
    #     win_len = 14
    #     mid_idx = win_len // 2
    #     threshold = 1e-6
    #     for i in range(len(boundary) - win_len):
    #         x_piece = []
    #         y_piece = []
    #         for j in range(win_len):
    #             len_temp = boundary[i + j]
    #             theta_temp = (i + j) / 2 / 180 * math.pi - (math.pi - yaw) # theta to x axis
    #             x_piece.append(self_x + len_temp * math.cos(theta_temp))
    #             y_piece.append(self_y + len_temp * math.sin(theta_temp))
    #         p_arr = np.array([
    #             [x_piece[0], y_piece[0], 1],
    #             [x_piece[-1], y_piece[-1], 1],
    #             [x_piece[mid_idx], y_piece[mid_idx], 1],
    #         ])
    #         v_arr = np.array([
    #             [-(x_piece[0]**2 + y_piece[0]**2)],
    #             [-(x_piece[-1]**2 + y_piece[-1]**2)],
    #             [-(x_piece[mid_idx]**2 + y_piece[mid_idx]**2)]
    #         ])
    #         if np.linalg.det(p_arr) != 0:
    #             d_e_f = np.linalg.solve(p_arr, v_arr)
    #             sum_temp = 0
    #             for j in range(win_len):
    #                 if j != 0 and j != win_len - 1 and j != mid_idx:
    #                     sum_temp += abs(self.FuncFit(x_piece[j], y_piece[j], d_e_f))
    #             sum_temp /= (win_len - 3)
    #             if sum_temp < threshold:
    #                 idx = i + mid_idx
    #                 angle = yaw - (180 - idx / 2) / 180 * math.pi  # center of ball to x axis
    #                 ball_x = self_x + math.cos(angle) * (radius + boundary[idx])
    #                 ball_y = self_y + math.sin(angle) * (radius + boundary[idx])
    #                 print(angle / math.pi * 180, ball_x, ball_y)
    #                 for center in self.ball_place:
    #                     if math.sqrt((ball_x - center[0])**2 + (ball_y - center[1])**2) < 1.5:
    #                         return None
    #                 return ball_x, ball_y, yaw - angle
    #     return None

    def CarMove(self, x, z):
        self.cmd_twist.linear.x = x
        self.cmd_twist.angular.z = z
        self.cmd_pub.publish(self.cmd_twist)

    def sigint_handler(self, signum, frame):
        self.CarMove(0, 0)
        rospy.logwarn("Catched interrupt signal! Stop and exit...")
        exit()

    def startcommandCallback(self, msg):
        self.is_begin_ = msg.data


if __name__ == '__main__':
    rospy.init_node("cross_demo_node")
    robot_name = rospy.get_param('~robot_name', 'AKM_1')
    try:
        CrossDemo(robot_name)
    except Exception as e:
        rospy.logerr(e)
