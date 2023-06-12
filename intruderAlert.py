#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

import numpy as np

rospy.init_node('opencv_example', anonymous=True)
bridge = CvBridge()

def image_callback(img_msg):
    pub=rospy.Publisher('/intruderAlert',String,queue_size=10)
    try:
    	# read in the image from ros to convert to a opencv image
        cv_image = bridge.imgmsg_to_cv2(img_msg,"bgr8")
    except CvBridgeError, e:
        rospy.logerr("CvBridge Error: {0}".format(e))
    
 
    # convert to hsv colour spec
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV) 
    # get the colour of the utility cart
    colour_g=np.uint8([[[5,5,255]]])
    hsvGreen=cv2.cvtColor(colour_g, cv2.COLOR_BGR2HSV) 
   
    min_g=hsvGreen[0][0][0]-1,100,100
    max_g=hsvGreen[0][0][0]+1,255,255
    # isolate the green in from above range
    mask_g = cv2.inRange(hsv_image, min_g,max_g) 
   
    # if the mask does not isolate any green the cart isn't in view
    if (np.all((mask_g==0))):
        # publish no to intruderAlert 
    	pub.publish("No")
    else:
        # if green is seen publish yes to intruderAlert
    	pub.publish("Yes")
    
   
    
   
global subscribe
subscribe = rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)


while not rospy.is_shutdown():
     
      rospy.spin()
      break
