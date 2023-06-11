#!/usr/bin/env python
import rospy
import numpy as np
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion

Tu = 2*3.14159/(7)
Ku = 1

class PID(object):
    def __init__(self):
        self.kp = 0.5 # Kp - proportional (error)
        self.ki = 0.001  # Ki - integral 
        self.kd = 0.05#0.05  # kd - derivative (previous error)
        
        #self.kp = 0.6 * Ku
        #self.ki = 2 * self.kp / Tu
        #self.kd = self.kp * Tu / 8
        
        self.error = 0  # initialize error
        self.integral_error = 0  # initialize integral
        self.error_last = 0  # initialize previous error
        self.derivative_error = 0  # initialize derivative
        self.output = 0
        self.ang = np.zeros(3)

    # get current state of drone
    def get_state(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            gms = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            state = gms(model_name="mobile_base")
            return state
        except rospy.ServiceException, e:
            print "Service call failed: %s" % e

    def get_rotation(self, state):
        orientation_q = state.pose.orientation  # quaternion coordinates
        orientation_list = [orientation_q.x, orientation_q.y,
                            orientation_q.z, orientation_q.w]  # stores quartenion in a list
        (roll, pitch, yaw) = euler_from_quaternion(
            orientation_list)  # euler coordinates
        return yaw

        # will return PID for the linear coordinates
    def compute_pid(self, error):
        self.error = error  # error = goal-current
        self.integral_error += self.error  # integral is the accumulated sum
        self.derivative_error = self.error - self.error_last  # derivative is error-prev error
        self.error_last = self.error  # update previous error
        self.output = self.kp*self.error + self.ki*self.integral_error + self.kd*self.derivative_error  # calculate the PID value
        return self.output

    # will return the PID for the angular coordinates
    def compute_pid_angular(self, yaw):
        self.error = 0-yaw  # error is goal-current
        self.integral_error += self.error  # integral is the cummulative sum
        # derivative is error-previous error
        self.derivative_error = self.error - self.error_last
        self.error_last = self.error  # update the previous error
        self.output = self.kp*self.error + self.ki*self.integral_error + self.kd*self.derivative_error  # calc PID
        return self.output
