#!/usr/bin/env python
# coding=utf-8

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def imagesubCallback(data):
    try:
        bridge_ = CvBridge()
        #将sensor_msgs/Image类型的消息转化为BGR格式图像
        orgFrame_ = bridge_.imgmsg_to_cv2(data, 'bgr8')
        #打印图像尺寸
        orgFrame_copy = orgFrame_.copy()
        print("siza of raw image: ",orgFrame_copy.shape)
	#在原始图像上画出矩形框
        cv2.rectangle(orgFrame_copy, (100,100), (500,300), (255,0,0), 2)
	#将BGR图像再转换为sensor_msgs/Image消息格式发布
        cv2.imshow('redpoint', orgFrame_copy)
        cv2.waitKey(0)

    except CvBridgeError as err:
        print(err)

rospy.init_node('get_image_node', anonymous=True)
image_sub_ = rospy.Subscriber("/AKM_1/camera/rgb/image_raw", Image, imagesubCallback)
rospy.spin()
   


