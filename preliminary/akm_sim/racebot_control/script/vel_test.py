#!/usr/bin/env python
# coding=utf-8

import rospy
import signal
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry

class CrossDemo:
    def __init__(self, robot_name):
        signal.signal(signal.SIGINT, self.sigint_handler)
        self.robot_name = robot_name
        self.cmd_pub = rospy.Publisher(self.robot_name+'/cmd_vel', Twist, queue_size=1)
        self.cmd_twist = Twist()
        # self.pose = Pose()
        self.CarMove(0, 0)
        rospy.sleep(1)
        self.Main()
        rospy.spin()
    
    def Main(self):
        self.CarMove(1, 0)
        rospy.sleep(1)
        self.CarMove(0, 0)
        rospy.loginfo("Racecar reached, stop!")

    def CarMove(self, x, z):
        self.cmd_twist.linear.x = x
        self.cmd_twist.angular.z = z
        self.cmd_pub.publish(self.cmd_twist)
        rospy.loginfo("publish...")

    def sigint_handler(self, signum, frame):
        self.CarMove(0, 0)
        rospy.logwarn("Catched interrupt signal! Stop and exit...")
        exit()

if __name__ == '__main__':
    rospy.init_node("cross_demo_node")
    robot_name = rospy.get_param('~robot_name', 'AKM_1')
    try:
        CrossDemo(robot_name)
    except Exception as e:
        rospy.logerr(e)
    