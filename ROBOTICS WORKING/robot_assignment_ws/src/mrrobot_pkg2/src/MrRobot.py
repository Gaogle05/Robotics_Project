#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion

# MrFile Imports
from pid_controller import PID
from RRT import path
from MrTurtlebot import TurtleBot


def MrRobot():
    try:
        x = TurtleBot()
        #x.follow_path(path[::-1])
        x.follow_path(path)
        print("Okay, Mission Accomplished.\nSuccessfully reached the destination.")

    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    MrRobot(),

"""
Salmaan Suliman | Aashna Gaogle | Khiyara Soma

888b     d888         8888888b.          888               888    
8888b   d8888         888   Y88b         888               888    
88888b.d88888         888    888         888               888    
888Y88888P888 888d888 888   d88P .d88b.  88888b.   .d88b.  888888 
888 Y888P 888 888P"   8888888P" d88""88b 888 "88b d88""88b 888    
888  Y8P  888 888     888 T88b  888  888 888  888 888  888 888    
888   "   888 888     888  T88b Y88..88P 888 d88P Y88..88P Y88b.  
888       888 888     888   T88b "Y88P"  88888P"   "Y88P"   "Y888
"""
