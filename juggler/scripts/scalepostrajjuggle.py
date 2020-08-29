#!/usr/bin/env python
# license removed for brevity
import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from std_msgs.msg import Float64MultiArray
import os
import subprocess


from std_msgs.msg import String

pub = rospy.Publisher('/scaled_pos_traj_controller/command', JointTrajectory, queue_size=2000)
rospy.init_node('trajectory')
rate = rospy.Rate(500)  # 500Hz
trajectory = JointTrajectory()
point = JointTrajectoryPoint()
point.time_from_start.secs = 1
point.time_from_start.nsecs = 1

trajectory.header.seq = 0
trajectory.header.stamp.secs = 0
trajectory.header.stamp.nsecs = 0

trajectory.header.frame_id = ''

trajectory.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint',
                          'wrist_3_joint']

point.positions = [0, -1.5707, 0, -1.5707, 0, 2]

point.velocities = [6,6,6,6,6,6]
point.accelerations = [80,80,80,80,80,80]
point.effort = [1000,1000,1000,1000,1000,1000]

trajectory.points = point

rospy.loginfo(trajectory)
pub.publish(trajectory)

# pub.publish(hello_str)
rate.sleep()
# while not rospy.is_shutdown():