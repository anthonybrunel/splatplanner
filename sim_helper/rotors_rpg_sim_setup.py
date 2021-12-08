#!/usr/bin/env python

# ros
import rospy
from std_msgs.msg import String, Bool
from std_msgs.msg import Empty as EmptyMsg
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Pose, Twist, Transform, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from gazebo_msgs.msg import ModelState

from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetModelState, GetModelState
import tf

# Python
import sys
import math
import numpy as np
import time
from collections import deque


class SimulationManager:

    def __init__(self):
        '''  Initialize ros node and read params '''
        # Parse parameters
        self.ns_gazebo = rospy.get_param('~ns_gazebo', "/gazebo")
        self.ns_mav = rospy.get_param('~mav_name', "/hummingbird")
        self.initial_position = rospy.get_param('~initial_position', [0, 0, 0])  # x, y, z [m]
        self.x_init = rospy.get_param('~x_init', self.initial_position[0])  # x, y, z [m]
        self.y_init = rospy.get_param('~y_init', self.initial_position[1])  # x, y, z [m]
        self.z_init = rospy.get_param('~z_init', self.initial_position[2])  # x, y, z [m]
        self.initial_position[0] = self.x_init
        self.initial_position[1] = self.y_init
        self.initial_position[2] = self.z_init
        self.ready_pub = rospy.Publisher("~simulation_ready", String, queue_size=10)
        self.arm_pub = rospy.Publisher("~"+self.ns_mav+"/bridge/arm", Bool, queue_size=10)
        self.start_pub = rospy.Publisher("~"+self.ns_mav+"/autopilot/start", EmptyMsg, queue_size=10)
        self.hover_pub = rospy.Publisher("~"+self.ns_mav+"/autopilot/force_hover", EmptyMsg, queue_size=10)
        self.autopilot_odom_pub = rospy.Publisher("~/autopilot/odometry", Odometry, queue_size=10)
        self.start_pose_pub =  rospy.Publisher(self.ns_mav + "/autopilot/pose_command", PoseStamped, queue_size=1)

        self.setup_simulation()

    def setup_simulation(self):
        rospy.loginfo("Starting MAV simulation setup coordination...")
        rospy.wait_for_service(self.ns_gazebo + "/unpause_physics")
        rospy.wait_for_service(self.ns_gazebo + "/set_model_state")

        set_model_srv = rospy.ServiceProxy(self.ns_gazebo + "/set_model_state", SetModelState)
        get_model_srv = rospy.ServiceProxy(self.ns_gazebo + "/get_model_state", GetModelState)
        mav_name = self.ns_mav[np.max([i for i in range(len(self.ns_mav)) if self.ns_mav[i] == "/"]) + 1:]
        start_pos = PoseStamped()
        start_pos.pose.position.x = self.initial_position[0]
        start_pos.pose.position.y = self.initial_position[1]
        start_pos.pose.position.z = self.initial_position[2]
        model_state_set = ModelState(mav_name, start_pos.pose, Twist(), "world")
        start_pos.header.stamp = rospy.Time.now()
        start_pos.header.frame_id = "world"
        start_pos.pose.orientation.w = 1
        rospy.loginfo("Waiting for gazebo to wake up ...")
        unpause_srv = rospy.ServiceProxy(self.ns_gazebo + "/unpause_physics", Empty)
        while not unpause_srv():
            rospy.sleep(0.05)
        rospy.loginfo("Waiting for gazebo to wake up ... done.")

        rospy.loginfo("Waiting for MAV to spawn ...")
        rospy.wait_for_message(self.ns_mav + "/ground_truth/odometry", Odometry)
        rospy.loginfo("Waiting for MAV to spawn ... done.")
        rospy.sleep(1.)
        self.arm_pub.publish(True);
        self.start_pub.publish(EmptyMsg())
        rospy.sleep(1.)
        self.hover_pub.publish(EmptyMsg())


        # Initialize drone stable at [0, 0, 0]
        rospy.loginfo("Waiting for MAV to stabilize ...")
        dist = 10  # Position and velocity

        while dist >= 0.1 and not rospy.is_shutdown():
            state = get_model_srv(mav_name, "world")
            set_model_srv(model_state_set)
            pos = state.pose.position
            twist = state.twist.linear

            # msg = Odometry()
            # msg.header.stamp = rospy.Time.now()
            # msg.header.frame_id = "world" # i.e. '/odom'
            #
            # msg.pose.pose = state.pose
            # msg.twist.twist = state.twist
            #
            # self.autopilot_odom_pub.publish(msg)


            # if np.sqrt(twist.x**2+twist.y**2+twist.z**2) < 0.001:
            self.start_pose_pub.publish(start_pos)
            dist = np.sqrt((pos.x - self.initial_position[0]) ** 2 + (pos.y - self.initial_position[1]) ** 2 +
                           (pos.z - self.initial_position[2]) ** 2) + np.sqrt(twist.x**2+twist.y**2+twist.z**2)
            print(pos.x,pos.y,pos.z)
            print(self.initial_position[0],self.initial_position[1],self.initial_position[2])
            rospy.sleep(0.01)

        self.ready_pub.publish(String("Simulation Ready"))
        rospy.loginfo("MAV simulation setup successfully.")



if __name__ == '__main__':
    rospy.init_node('rotors_sim_setup', anonymous=True)
    sm = SimulationManager()
