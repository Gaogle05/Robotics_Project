import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

def image_callback(img_msg):
    bridge = CvBridge()
    pub = rospy.Publisher('/intruderAlert', String, queue_size=10)

    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))
        return

    # convert to hsv colour spec
    hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV) 
    # get the colour of the utility cart
    colour_g=np.uint8([[[5,5,255]]])
    hsvGreen=cv2.cvtColor(colour_g, cv2.COLOR_BGR2HSV) 
   
    lower_green=hsvGreen[0][0][0]-1,100,100
    upper_green=hsvGreen[0][0][0]+1,255,255

    # Create a mask for the green color range
    mask = cv2.inRange(hsv_image, lower_green, upper_green)

    if cv2.countNonZero(mask) > 0:
        pub.publish("Yes")
    else:
        pub.publish("No")

rospy.init_node('opencv_example', anonymous=True)
bridge = CvBridge()
rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)
rospy.spin()
