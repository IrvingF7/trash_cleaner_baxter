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

def main():
	# note IK operates w.r.t. /base frame
	rospy.init_node("ik_pick_and_place_demo")
	limb = 'left'
	hover_distance = 0.15 # meters
	# starting joint angles for left arm
	starting_joint_angles = {'left_w0': 0.6699952259595108,
							'left_w1': 1.030009435085784,
							'left_w2': -0.4999997247485215,
							'left_e0': -1.189968899785275,
							'left_e1': 1.9400238130755056,
							'left_s0': -0.08000397926829805,
							'left_s1': -0.9999781166910306}
	pnp = PickAndPlace(limb, hover_distance)
	# an orientation for gripper fingers to be overhead and parallel to the obj
	overhead_orientation = Quaternion(x=-0.0249590815779,
										y=0.999649402929,
										z=0.00737916180073,
										w=0.00486450832011)
	pnp.move_to_start(starting_joint_angles)

	rgbd_cam = KinectRGBD()
	cam_info = rgbd_cam.get_cam_info()
	c_img = rgbd_cam.get_c_img()
	d_img = rgbd_cam.get_d_img()
	cv2.imwrite('c_img.png', c_img)
	cv2.imwrite('d_img.png', d_img)

	bbox = [300, 300, 340, 320]
	trash = Trash(bbox, 0, c_img, d_img)
	cg = trash.get_cg()
	depth = trash.get_depth()
	trash_point = (cg[0], cg[1], depth)

	trash_frame = TrashFrame(rgbd_cam)
	wt = trash_frame.generate_world2trash(trash_point)

	br = tf.TransformBroadcaster()
	br.sendTransform(wt[0], wt[1], rospy.Time.now(), '/trash_frame0', '/base')
	
	wt_trans = wt[0]

	obj_poses = list()
	# pose of objects in its initial location (world coord)
	# replace these poses with estimates from a perception node



	obj_poses.append(Pose(
		position = Point(x=0.7, y=0.15, z=-0.129),
		orientation = overhead_orientation))
	# add additional desired poses for the object
	# each additional pose will get its own pick and place
	obj_poses.append(Pose(
		position = Point(x=wt_trans[0], y=wt_trans[1], z=wt_trans[2] + 0.05),
		orientation = overhead_orientation))
	# move to the desired starting angles
	pnp.move_to_start(starting_joint_angles)
	idx = 0

	while not rospy.is_shutdown():
		print("\nPicking...")
		pnp.pick(obj_poses[idx])
		print("\nPlacing...")
		idx = (idx+1)%len(obj_poses)
		pnp.place(obj_poses[idx])
		pnp.move_to_start(starting_joint_angles)

	return 0

if __name__ == '__main__':
	sys.exit(main())
