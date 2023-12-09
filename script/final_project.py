#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Dec  7 15:45:13 2023

@author: ubuntu
"""

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np

class NesneTanima(): 
    
    def __init__(self):
        rospy.init_node("nesne_tanima")
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.hiz_mesaji = Twist()
        self.bridge = CvBridge()
        self.min_on = 0
        rospy.Subscriber("camera/rgb/image_raw",Image,self.kameraCallback)
        rospy.Subscriber("scan", LaserScan, self.lazerCallback)
        rospy.spin()
    def lazerCallback(self, mesaj):
        sol_on = list(mesaj.ranges[0:9])
        sag_on = list(mesaj.ranges[350:359])
        on = sol_on + sag_on
        self.min_on = min(on)
        print(self.min_on)
        """if min_on < 1.0:
            self.hiz_mesaji.linear.x = 0.0
            self.hiz_mesaji.angular.z = 0.2  
        else:
            self.hiz_mesaji.linear.x = 0.25  
            self.hiz_mesaji.angular.z = 0.0"""

        #self.pub.publish(self.hiz_mesaji)
        return self.min_on
    def kameraCallback(self,mesaj):
        #img = self.bridge.imgmsg_to_cv2(mesaj,"mono8")
        img2 = self.bridge.imgmsg_to_cv2(mesaj,"bgr8")
        hsv_img = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV)
        
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])
        
        mask = cv2.inRange(hsv_img, lower_red, upper_red)
        
        cv2.imshow('maske', mask)
        
        h, w, d = hsv_img.shape
        
        M = cv2.moments(mask)
        if self.min_on > 1.0:
            if M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                cv2.circle(img2,(cx,cy),5,(0,0,255),-1)
        
                sapma = cx-w/2
        
                self.hiz_mesaji.linear.x = 0.1
                self.hiz_mesaji.angular.z = -float(sapma) / 100
                self.pub.publish(self.hiz_mesaji)
            else:
                self.hiz_mesaji.linear.x = 0.1
                self.hiz_mesaji.angular.z = 0.5
                self.pub.publish(self.hiz_mesaji)
        else: 
            self.hiz_mesaji.linear.x = 0.0
            self.hiz_mesaji.angular.z = 0.0
            self.pub.publish(self.hiz_mesaji)
        cv2.imshow("Robot Kamerasi",img2)
        cv2.waitKey(1)
NesneTanima()
