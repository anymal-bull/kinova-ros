#!/usr/bin/env python2

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import csv
import numpy as np
import rospy
import time

from os.path import expanduser
from robot_control_modules import argumentParser, joint_position_client
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

DEBUG = False  # { True, False }
PATH = expanduser("~/data")


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
        with np.load('{path}/traj-flat-kinova.npz'.format(path=PATH)) as data:
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

        # Play trajectory
        nb = raw_input('Starting trajectory playback, press return to start, n to skip')
        if (nb != 'n' and nb != 'N'):
            joint_position_client(np.rad2deg(pos_start), prefix)
            time.sleep(0.5)
            pub.publish(msg_trajectory)

        print('Done!')

    except rospy.ROSInterruptException:
        print('program interrupted before completion')
