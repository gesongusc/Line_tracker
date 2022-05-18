#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import cv_bridge
import rospy
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Tracker():
    def __init__(self):
        # Initialize subscriber & publisher
        self.sub = rospy.Subscriber('', Image, self.callback)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
	self.bridge = cv_bridge.CVBridge()
    
    def callback(self,msg):
        # Image processing
        image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        low_bound = np.array([126,126,126])
        up_bound = np.array([128,128,128])
        mask = cv2.inRange(hsv,low_bound,up_bound)
        M = cv2.moments(mask)
        
        # Compute velocity command
        if M['m00']>0:
            cX = int(M['m10']/M['m00'])
            cY = int(M['m01']/M['m00'])
            
            self.twist.linear.x = 0.1
            self.twist.angular.z = (image.shape[1]/2-cX)/100
            self.pub.publish(self.twist)
            
        else:
            self.twist.linear.x = 0
            self.twist.angular.z = 0
            self.pub.publish(self.twist)
            
        # Show image
        cv2.imshow(image)

def main():
    rospy.init_node('tracker',anonymous=True)
    tracker = Tracker()
    rate = rospy.Rate(10)

        
if __name__ == "__main__":
    main()            