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
import numpy as np

def main():
	rospy.init_node('ar_demo')
	listener = tf.TransformListener()
	found_tf = False
	world_frame = '/base'
	broom_frame = '/ar_marker_5'

	while not found_tf:
		try:
			broom_trans, broom_rot = listener.lookupTransform(world_frame, 
														broom_frame, 
														rospy.Time(0))
			found_tf = True
		except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
			continue

	# dustpan_frame = '/ar_marker_1'

	# while not found_tf:
	# 	try:
	# 		dustpan_trans, dustpan_rot = listener.lookupTransform(world_frame, 
	# 													dustpan_frame, 
	# 													rospy.Time(0))
	# 		found_tf = True
	# 	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
	# 		continue

	# overhead_orientation = tf.transformations.quaternion_from_euler(ai=ar_euler[0], aj=ar_euler[1], ak=ar_euler[2])
	# broom_quat = Quaternion(x=-0.0249590815779,
	# 									y=0.999649402929,
	# 									z=0.00737916180073,
	# 									w=0.00486450832011)

	broom_quat = Quaternion(x=0, y=1, z=0, w=0)

	broom_quat = tf.transformations.quaternion_from_euler(-np.pi, 0, -np.pi + np.pi/2)
	broom_quat = Quaternion(x=broom_quat[0], y=broom_quat[1], z=broom_quat[2], w=broom_quat[3])

	broom_pose = Pose(position=Point(x=broom_trans[0], y=broom_trans[1], z=broom_trans[2]),
					orientation=broom_quat)

	pnp_left = PickAndPlace('left', suction=False)
	# pnp_right = PickAndPlace('right', hover_distance, suction=True)

	IPython.embed()

if __name__ == "__main__":
	main()