#!/usr/bin/env python

from timeit import default_timer as timer
import rospy
import sys
import time
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
import signal
import json
import threading
from os.path import expanduser

# global parameters
printDebugInfo = False
timeForTimeout = 10  # timeout in seconds
rospy.init_node('MoveTraj') # initiallize noooode
rate = rospy.Rate(500)  # publishing rate
pub = rospy.Publisher('/scaled_pos_traj_controller/command', JointTrajectory, queue_size=0, latch=True) # connect to publisher
rospy.sleep(0.1)  # need time to connect


def openPositionsFile():
    """
    this function reads joints positions from json file
    """
    jsonPath = raw_input("Enter JSON file path in the following format: path_to_file/file_name\n")
    with open(jsonPath) as json_file:
        data = json.load(json_file)
        if printDebugInfo:
            for p in data['position']:
                print(p)
    return data


def savePositions():
    """
    this function save joints positions into json file
    """
    home = expanduser("~")
    jointPositionsList = []
    jointPositions = {}

    jsonPath = raw_input("Enter JSON file path in the following format: path_to_file/file_name\n")
    while not rospy.is_shutdown():
        prof = raw_input("Press the S-key and Enter to save position, or any key to exit...\n")
        if prof == "s" or prof == "S":
            currentJointState = rospy.wait_for_message('/joint_states', JointState)
            roundedCurrentJointState = [x for x in currentJointState.position]
            jointPositionsList.append(roundedCurrentJointState)
            rate.sleep()
        else:
            if printDebugInfo:
                for positions in jointPositionsList:
                    print(positions)
            jointPositions['position'] = jointPositionsList
            with open(jsonPath, "w") as write_file:
                json.dump(jointPositions, write_file)

            # comment/uncomment next line to return the saved locations from the function
            return jointPositionsList


def saveTime(positionsArrs):
    """
    this function gets list of arrays <float64[] position>
    and return list the time between each of the positions in the array
    """
    isJointStateClose = [True]
    timeLst = []
    prof = raw_input("Press Enter to continue...")
    startArr = positionsArrs[1]

    # make sure the robot reached its first destination in the array
    timeout = time.time() + timeForTimeout
    while all(isJointStateClose) and not rospy.is_shutdown():
        currentJointState = rospy.wait_for_message('/joint_states', JointState)
        currentJointState = [x for x in currentJointState.position]
        isJointStateClose = [isclose(currentJointState_i, positionsArr_i, rel_tol=0, abs_tol=0.0005) for
                             currentJointState_i, positionsArr_i in
                             zip(currentJointState, startArr)]
        if time.time() > timeout:
            print("Time-Out: Robot did not reach his destination, exit program!")
            sys.exit(1)

    # start the timer from the moment the robot moved from its first position
    start = timer()
    timeout = time.time() + timeForTimeout
    for i in range(2, len(positionsArrs)):
        positionsArr = positionsArrs[i]
        isJointStateClose = [False]
        # wait until the robot reached its destination
        while not all(isJointStateClose) and not rospy.is_shutdown():
            currentJointState = rospy.wait_for_message('/joint_states', JointState)
            currentJointState = [x for x in currentJointState.position]
            isJointStateClose = [isclose(currentJointState_i, positionsArr_i, rel_tol=0, abs_tol=0.005) for
                                 currentJointState_i, positionsArr_i in
                                 zip(currentJointState, positionsArr)]
            if time.time() > timeout:
                print("Time-Out: Robot did not reach his destination, exit program!")
                sys.exit(1)
        # calculate time between positions
        end = timer()
        actualTime = end - start
        if printDebugInfo:
            print("time elapsed= ", actualTime)  # Time in seconds, e.g. 5.38091952400282
        timeLst.append(actualTime)

    return timeLst


class MainJugglerJob(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        # The shutdown_flag is a threading.Event object that
        # indicates whether the thread should be terminated.
        self.shutdown_flag = threading.Event()
        # set common msg params

    def isClose(self, a, b, rel_tolarance= 1e-09, abs_tol= 0):
        """
        this function check if two values are close
        Args:
            a (float): value a
            b (float): value b
            rel_tol (float): relative tolerance
            abs_tol (float): abs tolerance
        Returns:
            true if the values are close, otherwise returns false.
        """
        return abs(a - b) <= max(rel_tolarance * max(abs(a), abs(b)), abs_tol)

    def move(self, positionsArr, seq, timeToMove, timeToWait, useTimeAlgorithm= False, usePositionAlgorithm= False):
        """
        this function send message to the publisher
        Args:
            positionsArr (float64[]): array of positions
            seq (int): index of message sequence
            timeToMove (int): desired time that for the joints to reach their destinations
            timeToWait (int): time to wait for <Time algorithm>
            useTimeAlgorithm (bool): use time algorithm
            usePositionAlgorithm (bool): use location algorithm
        Returns:
            None.
        """
        # check that excatly one algorithm selected
        if useTimeAlgorithm and usePositionAlgorithm:
            print("Choose only one wait algorithm, exit program!")
            sys.exit(1)
        if not useTimeAlgorithm and not usePositionAlgorithm:
            print("missing wait algorithm, exit program!")
            sys.exit(1)

        # check for key-board interrupts
        if self.shutdown_flag.is_set():
            print("exit")
            sys.exit(1)

        trajectory = JointTrajectory()
        point = JointTrajectoryPoint()
        trajectory.joint_names = ['elbow_joint', 'shoulder_lift_joint',
                                       'shoulder_pan_joint', 'wrist_1_joint',
                                       'wrist_2_joint', 'wrist_3_joint']
        trajectory.header.frame_id = ''

        # set message parameters
        trajectory.header.seq = seq
        trajectory.header.stamp = rospy.Time.now()
        point.positions = positionsArr
        point.time_from_start = rospy.Duration(timeToMove)
        trajectory.points.append(point)

        # check that publisher is connected - exit program on timeout
        timeout = time.time() + timeForTimeout
        while pub.get_num_connections() < 1:
            print("pub.get_num_connections()= ", pub.get_num_connections())
            # check for timeout
            if time.time() > timeout:
                print("Time-Out: could not connect to Publisher, exit program!")
                sys.exit(1)
            pass

        pub.publish(trajectory)

        # wait for robot to reach its destinations based on the chosen algorithm
        if useTimeAlgorithm:
            rospy.sleep(timeToWait)

        # position based algorithm
        if usePositionAlgorithm:
            # for debug purposes save movement time
            start = timer()
            timeout = time.time() + timeForTimeout
            isJointStateClose = [False]
            # make sure the robot reached its destination - exit program on timeout
            while not all(isJointStateClose) and not rospy.is_shutdown():
                currentJointState = rospy.wait_for_message('/joint_states', JointState)
                currentJointState = [x for x in currentJointState.position]
                isJointStateClose = [self.isClose(currentJointState_i, positionsArr_i, abs_tol=0.0005)
                                     for currentJointState_i, positionsArr_i in
                                     zip(currentJointState, positionsArr)]
                # check for timeout
                if time.time() > timeout:
                    print("Time-Out: Robot did not reach his destination, exit program!")
                    sys.exit(1)
            end = timer()
            if printDebugInfo:
                # calculate the error between the desired time to move to
                # the actual time that took the robot to reach its destination
                actualTime = end - start
                error = abs((timeToMove - actualTime) / timeToMove) * 100
                print("error =", error)
                print(seq, "time elapsed= ", actualTime)  # Time in seconds, e.g. 5.38091952400282

        if printDebugInfo:
            currentJointState = rospy.wait_for_message('/joint_states', JointState)
            print("current joint state ", currentJointState.position)

    def run(self):
        if printDebugInfo:
            print('Thread #%s started' % self.ident)

        while not self.shutdown_flag.is_set():
            j = 0  # step index
            # move to base location slow
            posArr = [-1.8321623802185059, -2.1462675533690394, -4.604589764271871, -0.7465761464885254,
                      -1.5866273085223597, -0.9229624907123011]
            self.move(posArr, j, 2, 2, useTimeAlgorithm=True)

            while not self.shutdown_flag.is_set() and not rospy.is_shutdown():
                if j != 0:
                    # 1
                    posArr = [-1.8321623802185059, -2.1462675533690394, -4.604589764271871, -0.7465761464885254,
                              -1.5866273085223597, -0.9229624907123011]

                    self.move(posArr, j, 2, 2, useTimeAlgorithm=True)
                    rospy.sleep(5)
                    j = j + 1

                # 2
                posArr = [-1.930586338043213, -2.3011685810484828, -4.573670212422506, -0.4539087575725098,
                          -0.9714015165912073, -0.767127815877096]

                self.move(posArr, j, 0.1, 0.15559697151184082, useTimeAlgorithm=True)
                j = j + 1

                # 3
                posArr = [-1.8321623802185059, -2.1462675533690394, -4.604577843342916, -0.7466238302043458,
                          -1.5866273085223597, -0.922973934804098]
                self.move(posArr, j, 0.1, 0.17988109588623047, useTimeAlgorithm=True)
                j = j + 1

                # 4
                posArr = [-1.436574935913086, -2.2727281055846156, -3.836945358906881, -0.8686927121928711,
                          -1.3250954786883753, -1.8232329527484339]
                self.move(posArr, j, 0.1, 0.29159092903137207, useTimeAlgorithm=True)
                j = j + 1

                # 5
                posArr = [-1.6596322059631348, -2.41396488765859, -3.934218231831686, -0.6290915769389649,
                          -1.59222919145693, -1.772322956715719]
                self.move(posArr, j, 0.2, 0.2, useTimeAlgorithm=True)
                j = j + 1

                # second throw
                # 6
                posArr = [-1.8320422172546387, -2.148630758325094, -4.1002681891070765, -0.7486179631999512,
                          -1.59222919145693, -0.772322956715719]
                self.move(posArr, j, 3, 3, useTimeAlgorithm=True)
                rospy.sleep(4)
                j = j + 1

                # 7
                posArr = [-1.8399949073791504, -2.2195264301695765, -4.050806824360983, -0.9883907598308106,
                          -1.9569557348834437, -0.9158390204059046]
                self.move(posArr, j, 0.1, 0.16588687896728516, useTimeAlgorithm=True)
                j = j + 1

                # 8
                posArr = [-1.832054615020752, -2.1486546001830042, -4.10024339357485, -0.7486060422709961,
                          -1.6183274427997034, -0.9229262510882776]
                self.move(posArr, j, 0.1, 0.15493321418762207, useTimeAlgorithm=True)
                j = j + 1

                # 9
                posArr = [-1.4231805801391602, -1.8593775234618128, -5.016648594533102, -1.408862904911377,
                          -1.9674032370196741, 0.39703893661499023]
                self.move(posArr, j, 0.1, 0.31566882133483887, useTimeAlgorithm=True)
                j = j + 1

                # 10
                posArr = [-1.7522273063659668, -1.946951528588766, -5.008586708699362, -1.0303724569133301,
                          -1.8645198980914515, 0.39707517623901367]
                self.move(posArr, j, 0.25, 0.25, useTimeAlgorithm=True)
                j = j + 1

                rospy.sleep(3)
        # keeps node alive
        rospy.spin()
        # ... Clean shutdown code here ...
        if printDebugInfo:
            print('Thread #%s stopped' % self.ident)


class ServiceExit(Exception):
    """
    Custom exception which is used to trigger the clean exit
    of all running threads and the main program.
    """
    pass


def service_shutdown(signum, frame):
    print('Caught signal %d' % signum)
    raise ServiceExit


def main():
    global printDebugInfo
    # Register the signal handlers
    signal.signal(signal.SIGTERM, service_shutdown)
    signal.signal(signal.SIGINT, service_shutdown)
    threadStarted = False
    prof = raw_input("Press y for debug options, Press any other key to run program without debug options\n")
    if prof == 'y':
        printDebugInfo = True
    try:
        # waiting for the user command
        while True:
            print("Select command to run")
            prof = raw_input("1:Juggler function, 2:save pose, 3: open saved moves, 4:save time, 5:quit\n")
            if prof == "1":
                if threadStarted:
                    print("Thread already running")
                else:
                    jugglerThread = MainJugglerJob()
                    jugglerThread.start()
                    threadStarted = True
                # Keep the main thread running, otherwise signals are ignored.
                # while True:
                #    time.sleep(0.5)
            elif prof == "2":
                savePositions()
            elif prof == "3":
                openPositionsFile()
            elif prof == "4":
                saveTime()
            elif prof == "5":
                print('Exiting main program')
                sys.exit(1)
            else:
                print('wrong input: Try Again')
    except ServiceExit:
        # Terminate the running threads.
        # Set the shutdown flag on each thread to trigger a clean shutdown of each thread.
        jugglerThread.shutdown_flag.set()
        # Wait for the threads to close...
        jugglerThread.join()
        sys.exit(1)


if __name__ == '__main__':
    main()
