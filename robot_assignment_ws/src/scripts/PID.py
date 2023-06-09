#!/usr/bin/env python

import numpy as np
import rospy
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion

class PID(object):

     def __init__(self):

        # set all errors to 0
        self.error = 0 
        self.iError = 0 
        self.lError = 0 
        self.dError = 0 
        self.finalError = 0

        # set/change const
        self.kp = 0.3
        self.ki = 0.001
        self.kd = 0.03 


    # get the current state of robot
    def get_state(self): 
        rospy.wait_for_service("/gazebo/get_model_state")

        try:
            get_mstate = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
            curr_state = get_mstate(model_name = "mobile_base")

            return curr_state

        except rospy.ServiceException, error:
            print "Could not call service: %s"%error

    
    def get_rotation(self, state):
        orientation_q = state.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)


    # PID error calculations from lecture notes
    def PID_angular(self, yaw):

        # error is equal to the goal minus the current state
        self.error = 0 - yaw

        #integral error
        self.iError = self.iError + self.error

        #derivative error 
        self.dError = self.error - self.lError

        self.lError = self.error 

        self.finalError = self.kp*self.error + self.ki*self.iError + self.kd*self.dError
        
        
        return self.finalError


    def PID_linear(self, error):

        # error is equal to the goal minus the current state
        self.error = error

        #integral error
        self.iError = self.iError + self.error

        #derivative error 
        self.dError = self.error - self.errorL

        self.errorL = self.error 

        self.finalError = self.kp*self.error + self.ki*self.iError + self.kd*self.dError

        return self.finalError



