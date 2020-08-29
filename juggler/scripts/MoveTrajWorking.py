#!/usr/bin/env python

import rospy
import sys
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
#from control_msgs.msg import JointTrajectoryControllerState
from sensor_msgs.msg import JointState


import math
from std_msgs.msg import String

moveFreq = 0.5 #freq
dt = 1 / moveFreq
dx = 360 / dt

def doLoop():
    point = JointTrajectoryPoint()
    #point.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    #point.accelerations = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    #point.effort = [600, 600, 600, 600, 600, 600]

    i = 0
    j = 0
    while not rospy.is_shutdown():
        j = j + 1
        i = i + dx

        if i > 360:
            i = 0

        now = rospy.get_rostime()

        trajectory = JointTrajectory()
        trajectory.header.seq = j
        trajectory.header.stamp = now
        trajectory.header.frame_id = ''

        trajectory.joint_names = ['elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint',
                                  'wrist_2_joint', 'wrist_3_joint']

        point.time_from_start = rospy.Duration(dt)
        point.positions = [0, -1.570770577793457, math.radians(i), -1.5708142719664515, 0, 0]

        trajectory.points.append(point)

        pub.publish(trajectory)
        rospy.loginfo(trajectory)
        rospy.sleep(dt)

def move(positionsArr, seq, timeToMove):

    point = JointTrajectoryPoint()

    #x = input("Press Enter to continue...")

    #point.velocities = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    #point.accelerations = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
    #point.effort = [600, 600, 600, 600, 600, 600]

    now = rospy.get_rostime()

    trajectory = JointTrajectory()
    trajectory.header.seq = seq
    trajectory.header.stamp = now
    trajectory.header.frame_id = ''

    trajectory.joint_names = ['elbow_joint', 'shoulder_lift_joint', 'shoulder_pan_joint', 'wrist_1_joint',
                              'wrist_2_joint', 'wrist_3_joint']

    point.time_from_start = rospy.Duration(timeToMove)
    point.positions = positionsArr
    trajectory.points.append(point)

    pub.publish(trajectory)
    rospy.loginfo(trajectory)

    rospy.sleep(timeToMove+0.1)

def doJuggling():
    j = 0

    #move to base location slow
    posArr = [-1.8321499824523926, -2.1463152370848597, -4.604554001485006, -0.7465875905803223, -1.5866034666644495, -0.9229624907123011]
    move(posArr, j, 5)
    j = j + 1

    while not rospy.is_shutdown():

        if (j > 1):
            #1
            posArr = [-1.8321499824523926, -2.1463152370848597, -4.604554001485006, -0.7465875905803223, -1.5866034666644495, -0.9229624907123011]
            move(posArr, j, 3)
            rospy.sleep(4)
            j = j + 1

        #2
        posArr = [-1.930574893951416, -2.3011685810484828, -4.573670212422506, -0.45394452035937505, -0.9714015165912073, -0.7671034971820276]
        move(posArr, j, 0.1)
        j = j + 1

        #3
        posArr = [-1.8321499824523926, -2.1462794742979945, -4.6046136061297815, -0.7465761464885254, -1.5865915457354944, -0.9229863325702112]
        move(posArr, j, 0.1)
        j = j + 1

        #4
        posArr = [-1.5853300094604492, -2.1355444393553675, -3.9349144140826624, -0.9817789357951661, -1.5936568419085901, -1.7743023077594202]
        move(posArr, j, 0.1)
        j = j + 1

        #5
        posArr = [-1.6596083641052246, -2.413976331750387, -3.934218231831686, -0.6290796560100098, -1.59222919145693, -1.7723591963397425]
        move(posArr, j, 2)
        j = j + 1

        #6
        posArr = [-1.8320422172546387, -2.1486784420409144, -4.100292030964987, -0.7485817235759278, -1.6183274427997034, -0.9229624907123011]
        move(posArr, j, 3)
        rospy.sleep(4)
        j = j + 1

        #7
        posArr = [-1.8399949073791504, -2.2195621929564417, -4.050782028828756, -0.9884026807597657, -1.9569199720965784, -0.9158151785479944]
        move(posArr, j, 0.2)
        j = j + 1

        #8
        posArr = [-1.831993579864502, -2.148630758325094, -4.100279633198873, -0.7486060422709961, -1.6183393637286585, -0.9229024092303675]
        move(posArr, j, 0.1)
        j = j + 1

        #9 -
        posArr = [-1.4642720222473145, -1.783295293847555, -5.131565395985739, -1.4786394399455567, -1.9937084356891077, 0.39697885513305664]
        move(posArr, j, 0.05)
        j = j + 1

        #10
        posArr = [-2.0394129753112793, -2.1694189510741175, -4.841173473988668, -0.5783837598613282, -1.5529807249652308, 0.3989739418029785]
        move(posArr, j, 0.5)
        j = j + 1

        rospy.sleep(3)

def callback(data):
    #rospy.loginfo(data.actual)
    #type(data.actual.positions)
    #print (data.actual.positions)

    datatemp2 =[]
    dataTemp = JointState()
    dataTemp = data.position
    datatemp2.append(dataTemp)
    print(datatemp2[0])

if __name__ == "__main__":
    pub = rospy.Publisher('/scaled_pos_traj_controller/command', JointTrajectory, queue_size=10000)
    #currStatePub = rospy.Publisher('/scaled_pos_traj_controller/state', JointTrajectoryControllerState, queue_size=10000)
    currStateSub = rospy.Subscriber('/joint_states', JointState, callback, queue_size=1,buff_size = 2)

    rospy.init_node('trajectory')




    rate = rospy.Rate(500)  # 500Hz
    #doLoop()
    #doJuggling()



