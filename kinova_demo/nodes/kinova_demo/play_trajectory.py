#!/usr/bin/env python2

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import actionlib
import csv
import kinova_msgs.msg
import numpy as np
import rospy

from os.path import expanduser
from robot_control_modules import argumentParser, joint_position_client
from time import sleep
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

DEBUG = False  # { True, False }
PATH = expanduser("~/data")

GRIPPER_OPEN = [0.0, 0.0]
GRIPPER_CLOSE = [5440.0, 5440.0]


def gripper_client(finger_positions):
    """Send a gripper goal to the action server."""
    action_address = '/' + prefix + 'driver/fingers_action/finger_positions'

    client = actionlib.SimpleActionClient(action_address,
                                          kinova_msgs.msg.SetFingersPositionAction)
    client.wait_for_server()

    goal = kinova_msgs.msg.SetFingersPositionGoal()
    goal.fingers.finger1 = float(finger_positions[0])
    goal.fingers.finger2 = float(finger_positions[1])
    # The MICO arm has only two fingers, but the same action definition is used
    if len(finger_positions) < 3:
        goal.fingers.finger3 = 0.0
    else:
        goal.fingers.finger3 = float(finger_positions[2])
    client.send_goal(goal)
    if client.wait_for_result(rospy.Duration(5.0)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        rospy.logwarn('        the gripper action timed-out')
        return None


def new_trajectory_msg(ts, pos, vel):
    msg_trajectory = JointTrajectory()
    msg_trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1)

    for i in range(1, nbJoints + 1):
        msg_trajectory.joint_names.append('{}joint_{}'.format(prefix, i))

    DEBUG and print(msg_trajectory.joint_names)

    samples = range(len(ts))[::20]
    print('Downsampling to {} points: {}'.format(len(samples), samples))

    for i in samples:
        point = JointTrajectoryPoint()
        point.positions = pos[i,:]
        point.velocities = vel[i,:]
        point.time_from_start = rospy.Duration(ts[i])
        msg_trajectory.points.append(point)

    return msg_trajectory


if __name__ == '__main__':
    try:
        # Parse command line arguments
        prefix, nbJoints = argumentParser(None)

        ts = None
        pos = None
        vel = None
        tau = None

        # Read file with trajectory data
        with np.load('{path}/traj-ramp-kinova.npz'.format(path=PATH)) as data:
            ts = data["ts"]
            pos = np.transpose(data["pos"])
            vel = np.transpose(data["vel"])
            tau = np.transpose(data["tau"])

        assert len(ts) == len(pos) == len(vel) == len(tau)+1

        # Initialize new ROS node
        rospy.init_node('my_playback')
        pub = rospy.Publisher('/j2s6s200_driver/trajectory_controller/command', JointTrajectory, queue_size=10)

        # Start configuration
        pos_start = np.r_[ pos[0,:], [0] ]
        print(pos_start)

        # Create trajectory message
        msg_trajectory = new_trajectory_msg(ts, pos, vel)
        # print(msg_trajectory)

        # Go to start
        nb = raw_input('Going to start point, press return to start, n to skip')
        if (nb != 'n' and nb != 'N'):
            joint_position_client(np.rad2deg(pos_start), prefix)

        # Open gripper
        print('Set finger positions to {}'.format(GRIPPER_OPEN))
        result_gripper = gripper_client(GRIPPER_OPEN)

        nb = raw_input('Starting trajectory playback, press return to start, n to skip')
        if (nb != 'n' and nb != 'N'):
            # Grasp object
            print('Set finger positions to {}'.format(GRIPPER_CLOSE))
            result_gripper = gripper_client(GRIPPER_CLOSE)

            # Play trajectory
            pub.publish(msg_trajectory)
            sleep(ts[-1])

            # Release object
            print('Set finger positions to {}'.format(GRIPPER_OPEN))
            result_gripper = gripper_client(GRIPPER_OPEN)

        print('Done!')

    except rospy.ROSInterruptException:
        print('program interrupted before completion')
