#!/usr/bin/python
#-*- encoding: utf8 -*-

from tools import pressAnyKeyExit, getPackagePath, rostime2str
from scipy.spatial.transform import Rotation as R
from enum import Enum
import rospy
import yaml
import numpy as np
import os
from std_msgs.msg import String, Bool
from gazebo_msgs.msg import ModelStates
from rosgraph_msgs.msg import Clock

class JudgeNode:
    class DroneNodeState(Enum):
        WAITING = 1
        FLYING = 2
        LANDING = 3
        LANDED = 4
    
    class CarNodeState(Enum):
        WAITING = 1
        MOVING = 2
        PARKING = 3
        PARKED = 4

    def __init__(self):
        rospy.init_node('judge_node', anonymous=True)

        self.ros_path_ = getPackagePath('uav_sim')
        self.readYaml()

        # 无人机在世界坐标系下的位姿
        self.Drone_R_Wu_ = R.from_quat([0, 0, 0, 1])
        self.Drone_t_Wu_ = np.zeros([3], dtype=np.float64)

        # 无人车在世界坐标系下的位姿
        self.Car_R_Wu_ = R.from_quat([0, 0, 0, 1])
        self.Car_t_Wu_ = np.zeros([3], dtype=np.float64)

        self.time_begin_ = None
        self.time_end_ = None
        self.time_now_ = None

        self.drone_node_state_ = self.DroneNodeState.WAITING
        self.car_node_state_ = self.CarNodeState.WAITING

        #self.target_groundtruth_ = 'rgbee'
        self.target_result_ = None
        self.is_result_received_ = False

        self.cmdstartPub_ = rospy.Publisher('/tello/cmd_start', Bool, queue_size=100)

        self.commandSub_ = rospy.Subscriber('/tello/cmd_string', String, self.commandCallback)
        self.carstateSub_ = rospy.Subscriber('/AKM_1/parkstate', String, self.carstateCallback)
        self.poseSub_ = rospy.Subscriber('/gazebo/model_states', ModelStates, self.poseCallback)
        self.strSub_ = rospy.Subscriber('/target_result', String, self.strCallback)
        self.simtimeSub_ = rospy.Subscriber('/clock', Clock, self.simtimeCallback)

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.echoMessage()
            rate.sleep()

    def readYaml(self):
        yaml_path = self.ros_path_ + '/config/target.yaml'
        file_handle = open(yaml_path, 'r')
        cfg = file_handle.read()
        cfg_dict = yaml.load(cfg)
        self.target_groundtruth_ = cfg_dict['target_groundtruth_']

        pass

    def echoMessage(self):
        if self.drone_node_state_ == self.DroneNodeState.WAITING or self.car_node_state_ == self.CarNodeState.WAITING:
            os.system('clear')
            print('裁判机状态: 等待指令')
            print('')
            print('')
            pressAnyKeyExit('按任意键发送比赛开始信号，并开始计时。')
            cmd_start = Bool()
            cmd_start.data = 1
            self.cmdstartPub_.publish(cmd_start)
            self.drone_node_state_ = self.DroneNodeState.FLYING
            self.car_node_state_ = self.CarNodeState.MOVING
            pass

        if self.drone_node_state_ == self.DroneNodeState.FLYING:
            os.system('clear')
            print('无人机状态: 正在比赛中')
            pass
        elif self.drone_node_state_ == self.DroneNodeState.LANDING:
            os.system('clear')
            print('无人机状态: 接收到降落信号')
            pass
        elif self.drone_node_state_ == self.DroneNodeState.LANDED:
            os.system('clear')
            print('无人机状态: 已降落')
            pass

        if self.car_node_state_ == self.CarNodeState.MOVING:
            print('无人车状态: 正在比赛中')
            pass
        elif self.car_node_state_ == self.CarNodeState.PARKING:
            print('无人车状态: 接收到停靠信号')
            pass
        elif self.car_node_state_ == self.CarNodeState.PARKED:
            print('无人车状态: 已停靠')
            pass


        if self.drone_node_state_ != self.DroneNodeState.WAITING and self.car_node_state_ != self.CarNodeState.WAITING:
            print('')
            self.printCompareResult()
            print('')
            try:
                if self.time_begin_ is not None:
                    print('比赛开始时间: ' + rostime2str(self.time_begin_))
                if self.time_now_ is not None:
                    print('当前时间:     ' + rostime2str(self.time_now_))
                if self.time_begin_ is not None and (self.drone_node_state_ != self.DroneNodeState.LANDED or self.car_node_state_ != self.CarNodeState.PARKED):
                    print('总用时:       ' + rostime2str(self.time_now_ - self.time_begin_))
                elif self.time_end_ is not None and self.time_begin_ is not None:
                    print('总用时:       ' + rostime2str(self.time_end_ - self.time_begin_))
            except:
                print("总用时: error")
        else:
            pass
        pass

    def printCompareResult(self):
        print('目标识别结果正确答案: ' + self.target_groundtruth_)
        if self.target_result_ is None:
            print('尚未收到目标识别结果……')
        else:
            print('你的目标识别结果:     ' + self.target_result_)
            correct_results = 0
            if len(self.target_result_) == 5:
                for i in range(5):
                    if self.target_groundtruth_[i] == self.target_result_[i]:
                        correct_results += 1
            print('正确识别目标数:       ' + str(correct_results))

    def commandCallback(self, msg):
        cmd = msg.data
        cmdBuffer = cmd.strip().split()
        if cmdBuffer[0] == 'land' and self.drone_node_state_ == self.DroneNodeState.FLYING:
            self.drone_node_state_ = self.DroneNodeState.LANDING
        pass

    def carstateCallback(self, msg):
        state = msg.data
        if state == 'PARKING' and self.car_node_state_ == self.CarNodeState.MOVING:
            self.car_node_state_ = self.CarNodeState.PARKING

    # 接收无人车和无人机位姿ground truth
    def poseCallback(self, msg):
        if 'iris' in msg.name:
            i = msg.name.index('iris')
            pose = msg.pose[i]
            self.Drone_t_Wu_ = np.array([pose.position.x, pose.position.y, pose.position.z])
            self.Drone_R_Wu_ = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
            if pose.position.z < 0.1 and self.drone_node_state_ == self.DroneNodeState.LANDING:
                self.drone_node_state_ = self.DroneNodeState.LANDED
        if 'AKM_1' in msg.name:
            i = msg.name.index('AKM_1')
            pose = msg.pose[i]
            self.Car_t_Wu_ = np.array([pose.position.x, pose.position.y, pose.position.z])
            self.Car_R_Wu_ = R.from_quat([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
            destination = np.array([4, 14.5])
            dis = np.sqrt(np.sum(self.Car_t_Wu_[:2]-destination)**2)
            if dis < 0.5 and self.car_node_state_ == self.CarNodeState.PARKING:
                self.car_node_state_ = self.CarNodeState.PARKED

    def strCallback(self, msg):
        # 只接受一次结果
        if not self.is_result_received_:
            self.target_result_ = msg.data
            self.is_result_received_ = True
        pass

    def simtimeCallback(self, msg):
        self.time_now_ = msg.clock
        if self.drone_node_state_ == self.DroneNodeState.FLYING and self.time_begin_ is None:
            self.time_begin_ = msg.clock
        if self.drone_node_state_ == self.DroneNodeState.LANDED and self.car_node_state_ == self.CarNodeState.PARKED and self.time_end_ is None:
            self.time_end_ = msg.clock

if __name__ == '__main__':
    jn = JudgeNode()


