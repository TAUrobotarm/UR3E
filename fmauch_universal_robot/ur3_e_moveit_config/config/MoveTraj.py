#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Harels file


import time
import sys
import math
from math import radians
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState


shoulder_pan_joint = 0
shoulder_lift_joint = 1
elbow_joint = 2
wrist_1_joint = 3
wrist_2_joint = 4
wrist_3_joint = 5

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('Harels',anonymous=True)

robot=moveit_commander.RobotCommander()
scene=moveit_commander.PlanningSceneInterface()
move_group=moveit_commander.MoveGroupCommander("manipulator")

display_trajectory_publisher=rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory, queue_size=20)



##option 1 - take state from ur3e.srdf
move_group.set_planner_id("RRTConnectkConfigDefault")
move_group.set_planning_time(5)
move_group.set_num_planning_attempts(10)
move_group.set_max_acceleration_scaling_factor(1)
move_group.set_max_velocity_scaling_factor(1)
group_var_values = move_group.get_current_joint_values()



#step1
move_group.set_named_target("up")
plan = move_group.plan()
move_group.execute(plan_msg =plan)
time.sleep(3)
#move_group.go(wait=True)
# Calling ``stop()`` ensures that there is no residual movement




group_var_values[shoulder_pan_joint] = radians(90)
group_var_values[shoulder_lift_joint] = radians(-90)
group_var_values[elbow_joint] = radians(0)
group_var_values[wrist_1_joint] = radians(0)
group_var_values[wrist_2_joint] = radians(90)
group_var_values[wrist_3_joint] = radians(0)

move_group.set_joint_value_target(group_var_values)

plan = move_group.plan()
time.sleep(3)

move_group.execute(plan_msg =plan)


#print(group_var_values)