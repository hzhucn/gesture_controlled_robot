#!/usr/bin/env python3

import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

rospy.init_node('video_camera')
image_publisher = rospy.Publisher("image_topic",Image)
bridge = CvBridge()

r = rospy.Rate(10) # 10hz 

while not rospy.is_shutdown():
    
    cap = cv2.VideoCapture(0)
    # Capture frame-by-frame
    
    ret, cv_image = cap.read()
    image_message = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
    
    # publish images 
    image_publisher.publish(image_message)
    
    r.sleep()
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break