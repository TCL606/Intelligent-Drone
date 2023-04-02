#!/usr/bin/env python
# coding=utf-8

from traceback import print_tb
import rospy
import signal
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
# import numpy as np


class FindBalls:
    def __init__(self, robot_name):
        signal.signal(signal.SIGINT, self.sigint_handler)
        self.robot_name = robot_name
        self.cmd_twist = Twist()
        self.pose = Pose()
        self.cmd_pub = rospy.Publisher(self.robot_name + '/cmd_vel',
                                       Twist,
                                       queue_size=1)
        self.odom_sub = rospy.Subscriber(self.robot_name + '/odom', Odometry,
                                         self.odom_callback)
        self.sleep_time = 0.15
        self.Main()
        rospy.spin()

    def Main(self):
        # rospy.sleep(self.sleep_time)
        self.CarMove(1, 0)
        while True:
            rospy.sleep(self.sleep_time)
            if (self.pose.position.y >= 3):
                break
        self.CarMove(0, 0)
        # while True:
        #     rospy.sleep(self.sleep_time)
        #     print(self.pose.orientation.z)
        #     if (self.pose.position.x <= 3.7):
        #         break
        # self.CarMove(0.2, -0.5)
        # while True:
        #     rospy.sleep(self.sleep_time)
        #     print(self.pose.orientation.z)
        #     if (self.pose.orientation.z <= 1/np.sqrt(2)):
        #         break
        # self.CarMove(0.5, 0)

    def odom_callback(self, msg):
        self.pose = msg.pose.pose

    def CarMove(self, x, z):
	print("start carmove")
        self.cmd_twist.linear.x = x
        self.cmd_twist.angular.z = z
	print("before pub")
	print(self.cmd_twist)
        self.cmd_pub.publish(self.cmd_twist)
	print("after pub")

    def sigint_handler(self, signum, frame):
        self.CarMove(0, 0)
        rospy.logwarn("Catched interrupt signal! Stop and exit...")
        exit()


if __name__ == '__main__':
    rospy.init_node("cross_demo_node")
    robot_name = rospy.get_param('~robot_name', 'AKM_1')
    try:
        FindBalls(robot_name)
    except Exception as e:
        rospy.logerr(e)
