#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
import threading
import random
import numpy as np
from collections import deque
import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
# if you can not find cv2 in your python, you can try this. usually happen when you use conda.
# sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')
import cv2
import tello_base as tello
from enum import Enum

y_max_th = 200
y_min_th = 170
num = 0
img = None
tello_state='mid:-1;x:100;y:100;z:-170;mpry:1,180,1;pitch:0;roll:0;yaw:-19;'
tello_state_lock = threading.Lock()    
img_lock = threading.Lock()    

# send command to tello
class control_handler: 
    def __init__(self, control_pub):
        self.control_pub = control_pub
    
    def forward(self, cm):
        command = "forward "+(str(cm))
        self.control_pub.publish(command)
    
    def back(self, cm):
        command = "back "+(str(cm))
        self.control_pub.publish(command)
    
    def up(self, cm):
        command = "up "+(str(cm))
        self.control_pub.publish(command)
    
    def down(self, cm):
        command = "down "+(str(cm))
        self.control_pub.publish(command)
    
    def right(self, cm):
        command = "right "+(str(cm))
        self.control_pub.publish(command)
    
    def left(self, cm):
        command = "left "+(str(cm))
        self.control_pub.publish(command)

    def cw(self, cm):
        command = "cw "+(str(cm))
        self.control_pub.publish(command)


    def ccw(self, cm):
        command = "ccw "+(str(cm))
        self.control_pub.publish(command)

    def takeoff(self):
        command = "takeoff"
        self.control_pub.publish(command)
        print ("ready")

    def land(self):
        command = "land"
        self.control_pub.publish(command)

    def stop(self):
        command = "stop"
        self.control_pub.publish(command)

#subscribe tello_state and tello_image
class info_updater():   
    def __init__(self):
        rospy.Subscriber("tello_state", String, self.update_state)
        rospy.Subscriber("tello_image", Image, self.update_img)
        self.con_thread = threading.Thread(target = rospy.spin)
        self.con_thread.start()

    def update_state(self,data):
        global tello_state, tello_state_lock
        tello_state_lock.acquire() #thread locker
        tello_state = data.data
        tello_state_lock.release()
        # print(tello_state)

    def update_img(self,data):
        global img, img_lock
        img_lock.acquire()#thread locker
        img = CvBridge().imgmsg_to_cv2(data, desired_encoding = "passthrough")
        img_lock.release()
        global num
        # print("/home/thudrone/final_catkin_ws" + str(num) + ".jpg")
        num += 1
        # print(img)


# put string into dict, easy to find
def parse_state():
    global tello_state, tello_state_lock
    tello_state_lock.acquire()
    statestr = tello_state.split(';')
    print(statestr)
    dict={}
    for item in statestr:
        if 'mid:' in item:
            mid = int(item.split(':')[-1])
            dict['mid'] = mid
        elif 'x:' in item:
            x = int(item.split(':')[-1])
            dict['x'] = x
        elif 'z:' in item:
            z = int(item.split(':')[-1])
            dict['z'] = z
        elif 'mpry:' in item:
            mpry = item.split(':')[-1]
            mpry = mpry.split(',')
            dict['mpry'] = [int(mpry[0]),int(mpry[1]),int(mpry[2])]
        # y can be recognized as mpry, so put y first
        elif 'y:' in item:
            y = int(item.split(':')[-1])
            dict['y'] = y
        elif 'pitch:' in item:
            pitch = int(item.split(':')[-1])
            dict['pitch'] = pitch
        elif 'roll:' in item:
            roll = int(item.split(':')[-1])
            dict['roll'] = roll
        elif 'yaw:' in item:
            yaw = int(item.split(':')[-1])
            dict['yaw'] = yaw
    tello_state_lock.release()
    return dict

def showimg():
    pass
    # global img, img_lock
    # img_lock.acquire()
    # cv2.imshow("tello_image", img)
    # cv2.waitKey(30)
    # img_lock.release()

# mini task: take off and fly to the center of the blanket.
class task_handle():
    class taskstages():
        finding_location  = 0 # find locating blanket 
        order_location  = 1 # find the center of locating blanket and adjust tello 
        finished = 6 # task done signal

    class FlightState(Enum):  # 飞行状态
        WAITING = 1
        NAVIGATING = 2
        DETECTING_TARGET = 3
        LANDING = 4
        BATTLING = 5
        TURNING = 6
        JUDGING = 7

    def __init__(self , ctrl):
        self.States_Dict = None
        self.ctrl = ctrl
        self.flight_state_ = self.FlightState.WAITING
        self.navigating_dimension_ = None  # 'x' or 'y' or 'z' or 'yaw'
        self.navigating_destination_ = None
        self.next_state_ = None  
        self.navigating_queue_ = deque(
        )

        self.judging_queue = deque([
            deque([['yaw', -70]]),
            deque([['yaw', 0], ['x', 65], ['y', 30], ['z', 127], ['yaw', -90]]),
            deque([['x', 98], ['y', 100], ['z', 164], ['yaw', 90]]),
            deque([['yaw', 0], ['y', 214], ['x', 0]]),
        ])
        self.now_stage = self.taskstages.finding_location
        self.img_num = 0
        self.yaw_times = 2

    def switchNavigatingState(self):
        if len(self.navigating_queue_) == 0:
            print('No more navigating task in queue')
            self.flight_state_ = self.next_state_
        else:  
            print('switchNavigatingState')
            next_nav = self.navigating_queue_.popleft()
            self.navigating_dimension_ = next_nav[0]
            self.navigating_destination_ = next_nav[1]
            self.flight_state_ = self.FlightState.NAVIGATING

    def decision(self):
        self.States_Dict = parse_state()
        if self.flight_state_ == self.FlightState.WAITING:  # 起飞并飞至离墙体（y = 3.0m）适当距离的位置
            print('State: WAITING')
            # self.ctrl.takeoff()
            self.navigating_queue_ = deque([['z', 187], ['x', 10], ['y', -85], 
                                            ['y', -60], ['x', 55], ['z', 140], ['yaw', -175]])
            self.adj_yaw = self.States_Dict['yaw']
            self.switchNavigatingState()
            self.save_img = True
            self.next_state_ = self.FlightState.JUDGING  # DETECTING TARGET

        elif self.flight_state_ == self.FlightState.NAVIGATING:
            print('State: NAVIGATING')

            dim_index = self.navigating_dimension_
            if dim_index == 'yaw':
                dist = self.yaw_times * (self.navigating_destination_ + self.adj_yaw - self.States_Dict['yaw'])
            else:
                dist = self.navigating_destination_ - self.States_Dict[dim_index]
            if abs(dist) < 15:  # 当前段导航结束
                self.switchNavigatingState()
            else:
                command_matrix = [['right', 'left'], ['forward', 'back'],
                                  ['up', 'down'], ['cw', 'ccw']]
                command_idx = 0 if dim_index == 'x' else \
                                1 if dim_index == 'y' else \
                                2 if dim_index == 'z' else \
                                3 if dim_index == 'yaw' else 0 
                if command_idx == 3:
                    dist /= self.yaw_times
                    while dist > 180:
                        dist -= 360
                    while dist < -180:
                        dist += 360
                    dir_index = 0 if dist > 0 else 1 
                    dist = 90 if abs(dist) > 90 else int(abs(dist))    
                else:
                    dir_index = 0 if dist > 0 else 1  # x,y,z的方向idx
                    dist = 100 if abs(dist) > 150 else int(abs(dist))    

                command = command_matrix[command_idx][dir_index]
                
                print(command)           
                if command == 'right':
                    self.ctrl.right(dist)
                elif command == 'left':
                    self.ctrl.left(dist)
                elif command == 'forward':
                    self.ctrl.forward(dist)
                elif command == 'back':
                    self.ctrl.back(dist)
                elif command == 'up':
                    self.ctrl.up(dist)
                elif command == 'down':
                    self.ctrl.down(dist)
                elif command == 'cw':
                    self.ctrl.cw(dist)
                elif command == 'ccw':
                    self.ctrl.ccw(dist)
        
        elif self.flight_state_ == self.FlightState.JUDGING:
            print('State: JUDGING')
            if len(self.judging_queue) == 0:
                self.next_state_ = self.FlightState.LANDING
                self.switchNavigatingState()
            else:
                # TODO
                # Judging
                path = '/home/thudrone/final_catkin_ws/src/tello_control/' + str(self.img_num) + '.jpg'
                self.img_num += 1
                img_lock.acquire() # thread locker
                cv2.imwrite(path, img)
                img_lock.release()

                self.navigating_queue_ = self.judging_queue.popleft()
                self.next_state_ = self.FlightState.JUDGING
                self.switchNavigatingState()

        elif self.flight_state_ == self.FlightState.LANDING:
            print('State: LANDING')
            if self.save_img:
                path = '/home/thudrone/final_catkin_ws/src/tello_control/test.jpg'
                img_lock.acquire()#thread locker
                cv2.imwrite(path, img)
                img_lock.release()
                self.save_img = False
            self.ctrl.land()
        else:
            pass

    def main(self): # main function: examine whether tello finish the task
        # while not (self.now_stage == self.taskstages.finished):
        #     if(self.now_stage == self.taskstages.finding_location):
        #         self.finding_location()
        #     elif(self.now_stage == self.taskstages.order_location):
        #         self.order_location()
        # self.ctrl.land()

        while not rospy.is_shutdown():
            self.decision()
            time.sleep(4)
        print("Task Done!")
    
    def finding_location(self): # find locating blanket (the higher, the easier)
        assert (self.now_stage == self.taskstages.finding_location)
        while not ( parse_state()['mid'] > 0 ): # if no locating blanket is found:
            distance = random.randint(20,30) # randomly select distance
            print (distance)
            self.ctrl.up(distance) # tello up
            time.sleep(4) # wait for command finished
            showimg()
        self.now_stage = self.taskstages.order_location

    def order_location(self):# adjust tello to the center of locating blanket
        assert (self.now_stage == self.taskstages.order_location)
        state_conf = 0
        self.States_Dict = parse_state()
        while not ( self.States_Dict['mpry'][1] + 90 <= 8 and self.States_Dict['mpry'][1] + 90 >= -8 and self.States_Dict['x'] <= 120 and self.States_Dict['x'] >= 80 and  self.States_Dict['y'] <= 120 and self.States_Dict['y'] >= 80 and abs(self.States_Dict['z']) >= 150 and abs(self.States_Dict['z']) <= 190):
            if ( abs(self.States_Dict['z']) > 190 or abs(self.States_Dict['z']) < 150 ):
                if (abs(self.States_Dict['z']) < 150):
                    self.ctrl.up(20)     
                    time.sleep(4)
                elif (abs(self.States_Dict['z']) > 190):
                    self.ctrl.down(20) 
                    time.sleep(4)
            elif ( self.States_Dict['mpry'][1] + 90 < -8 or self.States_Dict['mpry'][1] + 90 > 8 ):
                if (self.States_Dict['mpry'][1] + 90 < -8):
                    self.ctrl.cw(10)
                    time.sleep(4)
                elif(self.States_Dict['mpry'][1] + 90 > 8):
                    self.ctrl.ccw(10)
                    time.sleep(4)
            elif ( self.States_Dict['x'] < 80 or self.States_Dict['x'] > 120 ):
                if (self.States_Dict['x'] < 80):
                    self.ctrl.right(20)
                    time.sleep(4)
                elif(self.States_Dict['x'] > 120):
                    self.ctrl.left(20)
                    time.sleep(4)
            elif ( self.States_Dict['y'] < 80 or self.States_Dict['y'] > 120 ):
                if (self.States_Dict['y'] < 80):
                    self.ctrl.back(20)
                    time.sleep(4)
                elif(self.States_Dict['y'] > 120):
                    self.ctrl.forward(20)
                    time.sleep(4)
            else:
                time.sleep(2)
                self.ctrl.stop()
                state_conf += 1
                print("stop")
            self.States_Dict = parse_state()
            showimg()
            if self.States_Dict['mid'] < 0 :
                self.now_stage = self.taskstages.finding_location
                return
        self.now_stage = self.taskstages.finished     


if __name__ == '__main__':
    rospy.init_node('tello_control', anonymous=True)

    control_pub = rospy.Publisher('command', String, queue_size=1)
    ctrl = control_handler(control_pub)
    infouper = info_updater()
    tasker = task_handle(ctrl)

    time.sleep(5.2)
    ctrl.takeoff( )
    time.sleep(4)
    ctrl.up(60)
    time.sleep(4)

    tasker.main()

    # ctrl.land()

    

