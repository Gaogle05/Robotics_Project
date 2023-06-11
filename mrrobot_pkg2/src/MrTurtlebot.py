#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion
from pid_controller import PID
from RRT import path


class TurtleBot:
    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a unique node (using anonymous=True).
        rospy.init_node('turtlebot_controller', anonymous=True)

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.velocity_publisher = rospy.Publisher(
            '/cmd_vel_mux/input/navi', Twist, queue_size=10)

        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called when a message of type Pose is received.
        self.pose_subscriber = rospy.Subscriber(
            "/gazebo/model_states", ModelStates, self.update_pose)

        self.pose = Pose()
        self.rate = rospy.Rate(10)

    def update_pose(self, msg):
        # Callback function which is called when a new message of type Pose is received by the subscriber.
        index = 0
        for i in range(len(msg.name)):
            if msg.name[i] == 'mobile_base':
                index = i
                break

        x = msg.pose[index].position.x
        y = msg.pose[index].position.y

        rot_q = msg.pose[index].orientation
        (roll, pitch, theta) = euler_from_quaternion(
            [rot_q.x, rot_q.y, rot_q.z, rot_q.w])

        self.pose.x = round(x, 4)
        self.pose.y = round(y, 4)
        self.pose.theta = theta

    def euclidean_distance(self, state, goal_pose):
        # Euclidean distance between current pose and the goal.
        return sqrt(pow((goal_pose.x - state.pose.position.x), 2) +
                    pow((goal_pose.y - state.pose.position.y), 2))

    def linear_vel(self, state, goal_pose, constant):
        return constant

    def steering_angle(self, state, goal_pose):
        return atan2(goal_pose.y - state.pose.position.y, goal_pose.x - state.pose.position.x)

    def angular_vel(self, state, theta, goal_pose, constant=3):
        ang_vel = constant * (self.steering_angle(state, goal_pose) - theta)
        max_vel = 7
        if abs(ang_vel) < max_vel:
            return ang_vel
        else:
            if ang_vel < 0:
                return -max_vel
            else:
                return max_vel

    def move2goal(self, goal_pose):
        vel_msg = Twist()
        while not rospy.is_shutdown():
            # Proportional controller.
            pos = PID()
            state = pos.get_state()
            theta = pos.get_rotation(state)

            pid_dist = pos.compute_pid(
                self.euclidean_distance(state, goal_pose))
            pid_rot = pos.compute_pid_angular(theta)

            # Linear velocity in the x-axis.
            vel_msg.linear.x = pid_dist

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(state, theta, goal_pose)

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            if (self.euclidean_distance(state, goal_pose) < 0.05):
                break

    def follow_path(self, path):
        for step in path:
            # Moves the turtle to goal
            goal_pose = Pose()
            goal_pose.x = step[0]
            goal_pose.y = step[1]
            self.move2goal(goal_pose)
