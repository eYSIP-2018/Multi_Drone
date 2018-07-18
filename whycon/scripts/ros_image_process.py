#!/usr/bin/env python
import rospy
import sys
import select
from sensor_msgs.msg import Image
from geometry_msgs.msg import  Pose

import cv2
from cv_bridge import CvBridge, CvBridgeError

rospy.init_node('VideoPublisher', anonymous=True)

bridge = CvBridge()
x=[]
y=[]

def show(data):
  global x,y
  try:
       cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
       for i in range(len(x)):
          while sys.stdin in select.select([sys.stdin], [], [], 0)[0]:   #Non-blocking input
            line = sys.stdin.readline()
            if line=="c\n":         #to stop the node with disarming the drone if e is pressed followed by enter
             x=[]
             y=[]
             return
          cv2.circle(cv_image, (int(x[i]),int(y[i])), 1, (0,190,255),1)

       if len(x)>5000:
        x=[]
        y=[]   
       try:
        image_pub.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))
       except CvBridgeError as e:
         print(e)
  except CvBridgeError as e:
         print(e)


def poses(data):
  x.append(data.position.x)
  y.append(data.position.y)


image_pub= rospy.Publisher('VideoRaw', Image, queue_size=10)
img=rospy.Subscriber('/whycon/image_out',Image,show)
pos=rospy.Subscriber('/whycon/pixel_pose',Pose,poses)

  
rospy.spin()
