#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from  geometry_msgs.msg import Twist
import math
import time

def hareket():
    rospy.init_node("ucgen_hareket")
    pub = rospy.Publisher("cmd_vel",Twist,queue_size=10)
    hiz_mesaji = Twist()

    for i in range(3):
        hiz_mesaji.linear.x = 0.3  
        mesafe = 1.0
        yer_degistirme = 0.0
        t0 = rospy.Time.now().to_sec()

        while (yer_degistirme < mesafe):
            pub.publish(hiz_mesaji)
            t1 = rospy.Time.now().to_sec()
            yer_degistirme = hiz_mesaji.linear.x * (t1 - t0)
            print(yer_degistirme)
        
        hiz_mesaji.linear.x = 0.0
        pub.publish(hiz_mesaji)
        rospy.loginfo("Hedefe varildi!!")

        derece_per_saniye = 120  #Dönme hızı
        radyan_per_saniye = math.radians(derece_per_saniye)  # radyan/saniye çevirme
        hiz_mesaji.angular.z = radyan_per_saniye

        donme_suresi = math.radians(120) / radyan_per_saniye  # Dönme süresi 

        pub.publish(hiz_mesaji)
        time.sleep(donme_suresi)

        hiz_mesaji.angular.z = 0.0
        pub.publish(hiz_mesaji)
hareket()


