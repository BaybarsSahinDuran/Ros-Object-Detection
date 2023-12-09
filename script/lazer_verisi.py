#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Uygulama: Lazer Sensörden Gelen Veriyi Kullanma ve Tepki Verme
"""

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class LazerVerisi():
    def __init__(self):
        rospy.init_node("lazer_dugumu")
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)
        self.hiz_mesaji = Twist()
        rospy.Subscriber("scan", LaserScan, self.lazerCallback)
        rospy.spin()

    def lazerCallback(self, mesaj):
        sol_on = list(mesaj.ranges[0:9])
        sag_on = list(mesaj.ranges[350:359])
        on = sol_on + sag_on
        min_on = min(on)
        print(min_on)
        if min_on < 1.0:
            self.hiz_mesaji.linear.x = 0.0
           #self.hiz_mesaji.angular.z = 0.2  
        else:
            self.hiz_mesaji.linear.x = 0.25  
            self.hiz_mesaji.angular.z = 0.0

        self.pub.publish(self.hiz_mesaji)

if __name__ == "__main__":
    try:
        LazerVerisi()
    except rospy.ROSInterruptException:
        pass
