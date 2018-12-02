#!/usr/bin/env python
import argparse
import struct
import sys
import copy

import rospy

from geometry_msgs.msg import (
	PoseStamped,
	Pose,
	Quaternion,
	Point,
)

import IPython
import tf
import cv2
from tcb.perception.kinect_rgbd import KinectRGBD
from tcb.perception.trash_frame import TrashFrame
from tcb.perception.trash import Trash

from tcb.manipulation.arm_manipulation import PickAndPlace
import time

def main():
	rospy.init_node("suction_demo")
	limb = 'right'
	hover_distance = 0.15

	pnp = PickAndPlace(limb, hover_distance, suction=True)
	IPython.embed()

if __name__ == "__main__":
	main()