#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
import threading

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
Q1 = [2.2,0,-1.57,0,0,0]
Q2 = [1.5,0,-1.57,0,0,0]
Q3 = [1.5,-0.2,-1.57,0,0,0]

client = None #ROS driver client
jointsMaxSpeed = 0.2 #rad, default value (will be settable)
jointsAccel = 3 #rad/s², default value (will be settable)

#For now, the speed is sufficiently low and the accel is sufficiently high to assume max speed is reached immediately
def calcDur(orig, dest, maxSpeed, accel):
    return abs(dest-orig)/maxSpeed

class Robot:
    def __init__(self):
        #For joint states
        self.lock = threading.Lock()
        self.name = []
        self.jointsAng = []
        self.jointsVel = []
        self.thread = threading.Thread(target=self.joint_states_listener)
        self.thread.start()
        #end for joint states

    def joint_states_listener(self):
        rospy.Subscriber('joint_states', JointState, self.joint_states_callback)
        rospy.spin()

    #callback function: when a joint_states message arrives, save the values
    def joint_states_callback(self, msg):
        while not rospy.is_shutdown():
            self.lock.acquire()
            self.name = msg.name
            self.jointsAng = msg.position
            #print self.jointsAng #This works
            self.jointsVel = msg.velocity
            self.lock.release()

    #Joint values is a list of the angles of the joints, in the order set by JOINT_NAMES (in rad)
    def moveJ(self, jointReqs):
        self.lock.acquire()
        jointsAngles = self.jointsAng
        print self.jointsAng #Does not work (yields 0)
        self.lock.release()

        #Calculate minimum duration for the last joint to reach the specified location
        if(len(jointReqs) == len(JOINT_NAMES) and len(jointsAngles) == len(JOINT_NAMES)):
            #Then calculate the move duration
            minDur = 9999
            for i in range(len(jointReqs)):
                minDur = min(minDur, calcDur(jointsAngles[i], jointReqs[i], jointsMaxSpeed, jointsAccel))


            g = FollowJointTrajectoryGoal()
            g.trajectory = JointTrajectory()
            g.trajectory.joint_names = JOINT_NAMES
            g.trajectory.points = [JointTrajectoryPoint(positions=jointReqs, velocities=[0]*6, time_from_start=minDur)]
            client.send_goal(g)
            client.wait_for_result()
        else:
            print "Length mismatch between joint names, joint setpoints, and joint states" #Is triggered because length jointAngles is zero

#--- main loop

def main():
    global client
    rospy.init_node("test_ur")
    client = actionlib.SimpleActionClient('/scaled_pos_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    print "Waiting for server..."
    client.wait_for_server()
    print "Connected to server"

    ur10 = Robot()
    while not rospy.is_shutdown():
        ur10.moveJ(Q1)
        ur10.moveJ(Q2)
        ur10.moveJ(Q3)

if __name__ == '__main__': main()