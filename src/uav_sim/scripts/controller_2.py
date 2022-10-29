#!/usr/bin/python
#-*- encoding: utf8 -*-

# 对windows.world的一个简单控制策略
# 结合tello的控制接口，控制无人机从指定位置起飞，识别模拟火情标记（红色），穿过其下方对应的窗户，并在指定位置降落
# 本策略尽量使无人机的偏航角保持在初始值（90度）左右
# 运行roslaunch uav_sim windows.launch后，再在另一个终端中运行rostopic pub /tello/cmd_start std_msgs/Bool "data: 1"即可开始飞行
# 代码中的decision()函数和switchNavigatingState()函数共有3个空缺之处，需要同学们自行补全（每个空缺之处需要填上不超过3行代码）

from scipy.spatial.transform import Rotation as R
from collections import deque
from enum import Enum
import rospy
import cv2
import numpy as np
import math
import datetime
from std_msgs.msg import String, Bool, Int8
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ControllerNode:

    class FlightState(Enum):  # 飞行状态
        WAITING = 1
        NAVIGATING = 2
        DETECTING_TARGET = 3
        LANDING = 4
        BATTLING = 5
        TURNING = 6
        JUDGING = 7

    def __init__(self):
        print("Begin")
        rospy.init_node('controller_node', anonymous=True)
        rospy.logwarn('Controller node set up.')

        # 无人机在世界坐标系下的位姿
        self.R_wu_ = R.from_quat([0, 0, 0, 1])
        self.t_wu_ = np.zeros([3], dtype=np.float64)

        self.image_ = None
        self.color_range_ = [(0, 43, 46), (6, 255, 255)]  # 红色的HSV范围
        self.bridge_ = CvBridge()

        self.flight_state_ = self.FlightState.WAITING
        self.navigating_queue_ = deque(
        )  # 存放多段导航信息的队列，队列元素为二元list，list的第一个元素代表导航维度（'x' or 'y' or 'z'），第二个元素代表导航目的地在该维度的坐标
        self.navigating_dimension_ = None  # 'x' or 'y' or 'z'
        self.navigating_destination_ = None
        self.next_navigating_destination = None
        self.next_navigating_dimension = None
        self.next_state_ = None  # 完成多段导航后将切换的飞行状态

        self.diff = [0, 0, 0, 0]
        self.target = [1, 1, 1.75, 90]
        self.bufferlen = 8
        self.posebuffer = np.zeros((self.bufferlen, 3))
        self.poseidx = 0
        self.max_dis_err = 0.25

        self.battling_lists = [
            deque([['y', 4.0], ['x', 6.75], ['y', 5.6], ['x', 6.3], ['x',
                                                                     5.0]]),
            deque([['z', 3.0], ['y', 7.0], ['y', 9.5]]),
            deque([['x', 6.3], ['z', 2.4], ['z', 1.75]]),
            deque([
                ['x', 6.0],
                ['y', 10.0],
                ['y', 12.5],
                ['x', 4.0],
            ]),
            deque([['x', 5.0], ['x', 6.8], ['y', 14.3]])
        ]

        self.angle_lists = [
            90.0,
            -90.0,
            -90.0,
            -90.0,
        ]
        self.is_turning = 0
        assert len(self.battling_lists) == len(self.angle_lists) + 1
        self.battle_idx = 0
        self.ball_idx = [2, 0, 0, 3]

        self.window_x_list_ = [1.8, 4.25,
                               6.75]  #[1.75, 4.25, 6.75] # 窗户中心点对应的x值
        self.flag = 0

        self.color_idx = [2, 0, 0, 3, 1, 4]
        self.color_list = ['e', 'e', 'e', 'e', 'e']
        self.is_begin_ = False

        self.commandPub_ = rospy.Publisher('/tello/cmd_string',
                                           String,
                                           queue_size=100)  # 发布tello格式控制信号

        self.poseSub_ = rospy.Subscriber('/tello/states', PoseStamped,
                                         self.poseCallback)  # 接收处理含噪无人机位姿信息
        self.imageSub_ = rospy.Subscriber('/iris/usb_cam/image_raw', Image,
                                          self.imageCallback)  # 接收摄像头图像
        self.BoolSub_ = rospy.Subscriber(
            '/tello/cmd_start', Bool, self.startcommandCallback)  # 接收开始飞行的命令
        self.carsub = rospy.Subscriber('tello/carmsg', Int8,
                                       self.carmsgCallback)

        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            if self.is_begin_:
                self.decision()
            rate.sleep()
        rospy.logwarn('Controller node shut down.')

    # 按照一定频率进行决策，并发布tello格式控制信号
    def decision(self):
        # print(self.diff)
        if self.flight_state_ == self.FlightState.WAITING:  # 起飞并飞至离墙体（y = 3.0m）适当距离的位置
            # rospy.logwarn('State: WAITING')
            self.publishCommand('takeoff')
            self.navigating_queue_ = deque([['z', 1.75], ['y', 1.8]])
            self.switchNavigatingState()
            self.next_state_ = self.FlightState.DETECTING_TARGET

        elif self.flight_state_ == self.FlightState.NAVIGATING:
            # rospy.logwarn('State: NAVIGATING')
            # 如果yaw与90度相差超过正负10度，需要进行旋转调整yaw
            (yaw, pitch, roll) = self.R_wu_.as_euler('zyx', degrees=True)
            self.diff[3] = yaw - self.target[3]
            if self.diff[3] > 180:
                self.diff[3] -= 360
            elif self.diff[3] < -180:
                self.diff[3] += 360
            # yaw_diff = yaw - 90 if yaw > -90 else yaw + 270
            # if yaw_diff > 10:  # clockwise
            #     self.publishCommand('cw %d' %
            #                         (int(yaw_diff) if yaw_diff > 15 else 15))
            #     return
            # elif yaw_diff < -10:  # counterclockwise
            #     # TODO 1: 发布相应的tello控制命令
            #     self.publishCommand(
            #         'ccw %d' %
            #         (int(abs(yaw_diff)) if abs(yaw_diff) > 15 else 15))
            #     return
            #     # end of TODO 1

            self.diff[3] = self.diff[3] / 30
            if self.diff[3] > 0.4 or self.is_turning:
                run_idx = 3
                err = 0.2
            else:
                for i in range(3):
                    self.diff[i] = self.target[i] - self.t_wu_[i]

                if self.next_navigating_dimension != None:
                    if self.diff[self.str2num(
                            self.next_navigating_dimension)] * (
                                self.next_navigating_destination -
                                self.t_wu_[self.str2num(
                                    self.next_navigating_dimension)]) < 0:
                        # print("PASS!!!!")
                        self.diff[self.str2num(
                            self.next_navigating_dimension)] = 0

                if self.battle_idx == len(self.battling_lists) - 1:
                    self.diff[self.str2num('z')] == 0

                abs_diff = map(abs, self.diff)
                run_idx = abs_diff.index(max(abs_diff))
                err = self.max_dis_err

            if self.t_wu_[2] > 14 or self.t_wu_[0] < 1.5:
                err = 0.5

            dist = self.diff[run_idx]
            abs_dist = abs(dist)

            if abs_dist < err:  # 当前段导航结束
                if self.is_turning == 1:
                    self.is_turning = -1
                    # rospy.sleep(1)
                    self.flight_state_ = self.FlightState.JUDGING
                    return
                else:
                    self.switchNavigatingState()
                    self.is_turning = 0
            else:
                if self.target[self.str2num('r')] == -90 and run_idx < 2:
                    dir_idx = 1 if dist > 0 else 0
                else:
                    dir_idx = 0 if dist > 0 else 1  # direction index

                command_matrix = [['right ', 'left '], ['forward ', 'back '],
                                  ['up ', 'down '], ['cw ', 'ccw ']]
                # TODO 2: 根据维度（dim_index）和导航方向（dir_idx）决定使用哪个命令
                command = command_matrix[run_idx][dir_idx]
                # print(self.is_turning, self.t_wu_)
                if self.is_turning == 1 and abs_dist > 1:
                    command = 'cw '
                # end of TODO 2
                if run_idx == 3:
                    self.publishCommand(
                        command + str(min(150, max(15, int(40 * abs_dist)))))
                else:
                    if abs_dist > 1.5:
                        self.publishCommand(command + '150')
                    else:
                        self.publishCommand(command + str(int(150 * abs_dist)))

        elif self.flight_state_ == self.FlightState.DETECTING_TARGET:
            # rospy.logwarn('State: DETECTING_TARGET')
            # 如果无人机飞行高度与标识高度（1.75m）相差太多，则需要进行调整
            if self.t_wu_[2] > 2.0:
                self.publishCommand('down %d' % int(100 *
                                                    (self.t_wu_[2] - 1.75)))
                return
            elif self.t_wu_[2] < 1.5:
                self.publishCommand('up %d' % int(-100 *
                                                  (self.t_wu_[2] - 1.75)))
                return
            # 如果yaw与90度相差超过正负10度，需要进行旋转调整yaw
            (yaw, pitch, roll) = self.R_wu_.as_euler('zyx', degrees=True)
            yaw_diff = yaw - 90 if yaw > -90 else yaw + 270
            if yaw_diff > 15:  # clockwise
                self.publishCommand('cw %d' %
                                    (int(yaw_diff) if yaw_diff > 15 else 15))
                return
            elif yaw_diff < -15:  # counterclockwise
                self.publishCommand('ccw %d' %
                                    (int(-yaw_diff) if yaw_diff < -15 else 15))
                return

            if self.t_wu_[0] > 1.5 and self.detectTarget():
                cv2.imwrite(
                    "cylinder_" + str(datetime.datetime.now()) + ".jpg",
                    self.image_)
                #rospy.loginfo('Target detected.')
                # 根据无人机当前x坐标判断正确的窗口是哪一个
                # 实际上可以结合目标在图像中的位置和相机内外参数得到标记点较准确的坐标，这需要相机成像的相关知识
                # 此处仅仅是做了一个粗糙的估计
                win_dist = [
                    abs(self.t_wu_[0] - win_x + 0.5)
                    for win_x in self.window_x_list_
                ]
                win_index = win_dist.index(min(win_dist))  # 正确的窗户编号
                if win_index == 0:
                    self.flag = 1
                print(win_index, self.t_wu_[0])
                self.navigating_queue_ = deque(
                    [['x', self.window_x_list_[win_index]], ['y', 2.4],
                     ['z', 0.95]])  # 通过窗户并导航至终点上方
                self.switchNavigatingState()
                self.next_state_ = self.FlightState.BATTLING
            else:
                if self.t_wu_[0] > 7.5:
                    # rospy.loginfo('Detection failed, ready to land.')
                    win_index = 2
                    print(win_index, self.t_wu_[0])
                    self.navigating_queue_ = deque(
                        [['x', self.window_x_list_[win_index]], ['y', 2.4],
                         ['z', 0.95]])  # 通过窗户并导航至终点上方
                    self.switchNavigatingState()
                    self.next_state_ = self.FlightState.BATTLING
                    # self.flight_state_ = self.FlightState.LANDING
                else:  # 向右侧平移一段距离，继续检测
                    self.publishCommand('right 150')

        elif self.flight_state_ == self.FlightState.BATTLING:
            # rospy.logwarn('State: BATTLING')
            if self.battle_idx < len(self.battling_lists):
                self.navigating_queue_ = self.battling_lists[self.battle_idx]
            self.switchNavigatingState()
            if self.battle_idx < len(self.angle_lists):
                self.next_state_ = self.FlightState.TURNING
            else:
                self.next_state_ = self.FlightState.LANDING

        elif self.flight_state_ == self.FlightState.TURNING:
            # rospy.logwarn('State: TURNING')
            (yaw, pitch, roll) = self.R_wu_.as_euler('zyx', degrees=True)
            self.target[3] = self.angle_lists[self.battle_idx]
            self.is_turning = 1
            # print(self.battle_idx)
            # yaw_diff = yaw - self.target[3]
            # while yaw_diff > 180:
            #     yaw_diff -= 360
            # while yaw_diff < -180:
            #     yaw_diff += 360
            # if yaw_diff > 10:  # clockwise
            #     self.publishCommand('cw %d' %
            #                         (int(yaw_diff) if yaw_diff > 15 else 15))
            # elif yaw_diff < -10:  # counterclockwise
            #     self.publishCommand(
            #         'ccw %d' %
            #         (int(abs(yaw_diff)) if abs(yaw_diff) > 15 else 15))
            # else:
            self.flight_state_ = self.FlightState.NAVIGATING
            return

        elif self.flight_state_ == self.FlightState.JUDGING:
            color = self.image_recognition(self.image_)
            if self.battle_idx != 1:
                cv2.imwrite("ball_" + str(datetime.datetime.now()) + ".jpg",
                            self.image_)
                self.color_list[self.ball_idx[self.battle_idx]] = color
            self.battle_idx += 1
            self.flight_state_ = self.FlightState.BATTLING

        elif self.flight_state_ == self.FlightState.LANDING:
            rospy.logwarn('State: LANDING')
            self.publishCommand('land')
            print(self.color_list)
        else:
            pass

    # 在向目标点导航过程中，更新导航状态和信息
    def switchNavigatingState(self):
        if len(self.navigating_queue_) == 0:
            self.flight_state_ = self.next_state_
        elif self.flag and self.navigating_dimension_ == 'y' and self.navigating_destination_ == 4.0:
            self.navigating_dimension_ = 'x'
            self.navigating_destination_ = 4.2
            self.target[0] = 4.2

        else:  # 从队列头部取出无人机下一次导航的状态信息
            next_nav = self.navigating_queue_.popleft()
            # TODO 3: 更新导航信息和飞行状态
            self.navigating_dimension_ = next_nav[0]
            self.navigating_destination_ = next_nav[1]

            if len(self.navigating_queue_) != 0:
                next_next_nav = self.navigating_queue_[0]
                self.next_navigating_dimension = next_next_nav[0]
                self.next_navigating_destination = next_next_nav[1]
            elif self.battle_idx == 0:
                self.next_navigating_dimension = 'y'
                self.next_navigating_destination = 7.1
            else:
                self.next_navigating_dimension = None
                self.next_navigating_destination = None

            idx = self.str2num(next_nav[0])
            self.target[idx] = next_nav[1]
            # print(self.target)

            self.flight_state_ = self.FlightState.NAVIGATING
            # end of TODO 3

    # 判断是否检测到目标
    def detectTarget(self):
        if self.image_ is None:
            return False
        image_copy = self.image_.copy()
        height = image_copy.shape[0]
        width = image_copy.shape[1]

        frame = cv2.resize(image_copy, (width, height),
                           interpolation=cv2.INTER_CUBIC)  # 将图片缩放
        frame = cv2.GaussianBlur(frame, (3, 3), 0)  # 高斯模糊
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
        h, s, v = cv2.split(frame)  # 分离出各个HSV通道
        v = cv2.equalizeHist(v)  # 直方图化
        frame = cv2.merge((h, s, v))  # 合并三个通道

        frame = cv2.inRange(frame, self.color_range_[0],
                            self.color_range_[1])  # 对原图像和掩模进行位运算
        opened = cv2.morphologyEx(frame, cv2.MORPH_OPEN,
                                  np.ones((3, 3), np.uint8))  # 开运算
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE,
                                  np.ones((3, 3), np.uint8))  # 闭运算
        (contours, hierarchy) = cv2.findContours(closed, cv2.RETR_EXTERNAL,
                                                 cv2.CHAIN_APPROX_NONE)  # 找出轮廓

        # 在contours中找出最大轮廓
        contour_area_max = 0
        area_max_contour = None
        for c in contours:  # 遍历所有轮廓
            contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
            if contour_area_temp > contour_area_max:
                contour_area_max = contour_area_temp
                area_max_contour = c

        if area_max_contour is not None:
            if contour_area_max > 50:
                return True
        return False

    def str2num(self, str):
        if str == 'x':
            return 0
        elif str == 'y':
            return 1
        elif str == 'z':
            return 2
        elif str == 'r':
            return 3
        else:
            return 4

    def num2str(self, num):
        if num == 0:
            return 'x'
        elif num == 1:
            return 'y'
        elif num == 2:
            return 'z'
        elif num == 3:
            return 'r'

    # 向相关topic发布tello命令
    def publishCommand(self, command_str):
        msg = String()
        msg.data = command_str
        self.commandPub_.publish(msg)
        # print(command_str)

    # 接收无人机位姿
    def poseCallback(self, msg):
        self.posebuffer[self.poseidx] = np.array(
            [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
        self.poseidx += 1
        if (self.poseidx >= self.bufferlen):
            self.poseidx = 0
        self.t_wu_ = np.average(self.posebuffer, axis=0)
        self.R_wu_ = R.from_quat([
            msg.pose.orientation.x, msg.pose.orientation.y,
            msg.pose.orientation.z, msg.pose.orientation.w
        ])
        pass

    # 接收相机图像
    def imageCallback(self, msg):
        try:
            self.image_ = self.bridge_.imgmsg_to_cv2(msg, 'bgr8')
        except CvBridgeError as err:
            print(err)

    # 接收开始信号
    def startcommandCallback(self, msg):
        self.is_begin_ = msg.data

    def carmsgCallback(self, msg):
        self.color_list[self.msg[0]] = msg[1]

    def image_recognition(self, image_):
        # Red color range 0, 235, 120, 10, 250, 130
        r_color_range_ = [(0, 235, 120), (10, 250, 130)]
        # Blue color range  94, 80, 2, 120, 255, 255
        b_color_range_ = [(94, 80, 2), (120, 255, 255)]
        # Yellow color range  20, 43, 46 , 34, 255, 255
        y_color_range_ = [(20, 43, 46), (34, 255, 255)]
        image_copy = image_.copy()
        height = image_copy.shape[0]
        width = image_copy.shape[1]

        frame = cv2.resize(image_copy, (width, height),
                        interpolation=cv2.INTER_CUBIC)  # 将图片缩放
        frame = cv2.GaussianBlur(frame, (3, 3), 0)  # 高斯模糊
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # 将图片转换到HSV空间
        h, s, v = cv2.split(frame)  # 分离出各个HSV通道
        v = cv2.equalizeHist(v)  # 直方图化
        frame = cv2.merge((h, s, v))  # 合并三个通道

        r_frame = cv2.inRange(frame, r_color_range_[0],
                            r_color_range_[1])  # 对原图像和掩模进行位运算
        r_opened = cv2.morphologyEx(r_frame, cv2.MORPH_OPEN,
                                    np.ones((3, 3), np.uint8))  # 开运算
        r_closed = cv2.morphologyEx(r_opened, cv2.MORPH_CLOSE,
                                    np.ones((3, 3), np.uint8))  # 闭运算
        r_closed[(
            (image_copy[:, :, 0] < 5) & (image_copy[:, :, 1] < 5) &
            (image_copy[:, :, 2] > 100))] = 255  # modified red_closed with RGB
        (r_contours, hierarchy) = cv2.findContours(r_closed, cv2.RETR_EXTERNAL,
                                                cv2.CHAIN_APPROX_NONE)  # 找出轮廓

        b_frame = cv2.inRange(frame, b_color_range_[0],
                            b_color_range_[1])  # 对原图像和掩模进行位运算
        b_opened = cv2.morphologyEx(b_frame, cv2.MORPH_OPEN,
                                    np.ones((3, 3), np.uint8))  # 开运算
        b_closed = cv2.morphologyEx(b_opened, cv2.MORPH_CLOSE,
                                    np.ones((3, 3), np.uint8))  # 闭运算
        (b_contours, hierarchy) = cv2.findContours(b_closed, cv2.RETR_EXTERNAL,
                                                cv2.CHAIN_APPROX_NONE)  # 找出轮廓

        y_frame = cv2.inRange(frame, y_color_range_[0],
                            y_color_range_[1])  # 对原图像和掩模进行位运算
        y_opened = cv2.morphologyEx(y_frame, cv2.MORPH_OPEN,
                                    np.ones((3, 3), np.uint8))  # 开运算
        y_closed = cv2.morphologyEx(y_opened, cv2.MORPH_CLOSE,
                                    np.ones((3, 3), np.uint8))  # 闭运算
        (y_contours, hierarchy) = cv2.findContours(y_closed, cv2.RETR_EXTERNAL,
                                                cv2.CHAIN_APPROX_NONE)  # 找出轮廓

        # 在contours中找出最大轮廓
        r_contour_area_max = 0
        r_area_max_contour = None
        for c in r_contours:  # 遍历所有轮廓
            r_contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
            if r_contour_area_temp > r_contour_area_max:
                r_contour_area_max = r_contour_area_temp
                r_area_max_contour = c

        # 在contours中找出最大轮廓
        b_contour_area_max = 0
        b_area_max_contour = None
        for c in b_contours:  # 遍历所有轮廓
            b_contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
            if b_contour_area_temp > b_contour_area_max:
                b_contour_area_max = b_contour_area_temp
                b_area_max_contour = c

        # 在contours中找出最大轮廓
        y_contour_area_max = 0
        y_area_max_contour = None
        for c in y_contours:  # 遍历所有轮廓
            y_contour_area_temp = math.fabs(cv2.contourArea(c))  # 计算轮廓面积
            if y_contour_area_temp > y_contour_area_max:
                y_contour_area_max = y_contour_area_temp
                y_area_max_contour = c

        if r_area_max_contour is not None:
            if r_contour_area_max > max(500, b_contour_area_max,
                                        y_contour_area_max):
                return 'r'
        if b_area_max_contour is not None:
            if b_contour_area_max > max(500, r_contour_area_max,
                                        y_contour_area_max):
                return 'b'
        if y_area_max_contour is not None:
            if y_contour_area_max > max(500, r_contour_area_max,
                                        b_contour_area_max):
                return 'y'
        return 'e'


if __name__ == '__main__':
    cn = ControllerNode()
