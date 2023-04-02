#!/usr/bin/env python
import rospy
import std_msgs.msg
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist

import time 
import threading

class Twist2AKM:
    def __init__(self, robot_frame):
        self.robot_frame = robot_frame
        self.pub = rospy.Publisher("ackermann_cmd_mux/output", AckermannDriveStamped, queue_size=1)
        rospy.Subscriber('cmd_vel', Twist, self.callback, queue_size=1, buff_size=52428800)
        rospy.spin()


    def callback(self, data):
        speed = data.linear.x 
        turn = data.angular.z

        msg = AckermannDriveStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.robot_frame + "base_link"

        msg.drive.speed = speed
        msg.drive.acceleration = 1
        msg.drive.jerk = 1
        msg.drive.steering_angle = turn
        msg.drive.steering_angle_velocity = 1

        self.pub.publish(msg)


if __name__ == '__main__':
    try:
        rospy.init_node('cmd_trans_node', anonymous=True)
        robot_frame = rospy.get_param('~robot_name', 'AKM_1')
        Twist2AKM(robot_frame)
    except rospy.ROSInterruptException:
        pass


########################
