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

def draw_rectangle_and_save(c_img, d_img, bbox):
	cv2.rectangle(c_img,(bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 3)
	cv2.imwrite('c_img.png', c_img)
	cv2.imwrite('d_img.png', d_img)

def main():
	# note IK operates w.r.t. /base frame
	rospy.init_node("ik_pick_and_place_demo")
	limb = 'left'
	hover_distance = 0.15 # meters

	pnp = PickAndPlace(limb, hover_distance, suction=False)
	# an orientation for gripper fingers to be overhead and parallel to the obj
	overhead_orientation = Quaternion(x=-0.0249590815779,
										y=0.999649402929,
										z=0.00737916180073,
										w=0.00486450832011)
	pnp.move_to_start()

	rgbd_cam = KinectRGBD()
	cam_info = rgbd_cam.get_cam_info()
	c_img = rgbd_cam.get_c_img()
	d_img = rgbd_cam.get_d_img()

	bbox = [220, 320, 260, 350]

	draw_rectangle_and_save(c_img, d_img, bbox)
	IPython.embed()

	trash = Trash(bbox, 0, c_img, d_img)
	cg = trash.get_cg()
	depth = trash.get_depth()
	trash_point = (cg[0], cg[1], depth)

	trash_frame = TrashFrame(rgbd_cam)
	wt = trash_frame.generate_world2trash(trash_point)

	# br = tf.TransformBroadcaster()
	# br.sendTransform(wt[0], wt[1], rospy.Time.now(), '/trash_frame0', '/base')
	
	wt_trans = wt[0]
	# print(wt_trans)

	obj_poses = list()
	# pose of objects in its initial location (world coord)
	# replace these poses with estimates from a perception node
	
	# add additional desired poses for the object
	# each additional pose will get its own pick and place
	# obj_poses.append(Pose(
	# 	position = Point(x=0.75, y=0.0, z=-0.129),
	# 	orientation = overhead_orientation))
	obj_poses.append(Pose(
		position = Point(x=wt_trans[0], y=wt_trans[1], z=wt_trans[2]),
		orientation = overhead_orientation))

	# obj_poses.append(Pose(
	# 	position = Point(x=0.7, y=0.15, z=-0.129),
	# 	orientation = overhead_orientation))
	# move to the desired starting angles
	pnp.move_to_start()
	idx = 0

	pose1 = Pose(
		position = Point(x=wt_trans[0], y=wt_trans[1], z=wt_trans[2] + 0.01),
		orientation = overhead_orientation)

	pose2 = Pose(
		position = Point(x=wt_trans[0], y=wt_trans[1] - 0.1, z=wt_trans[2] + 0.01),
		orientation = overhead_orientation)

	while not rospy.is_shutdown():
		# print("\nPicking...")
		# pnp.pick(obj_poses[idx])
		# print("\nPlacing...")
		# idx = (idx+1)%len(obj_poses)
		# pnp.place(obj_poses[idx])
		print("sweeping")
		pnp.pick(pose1)
		pnp.move_to_start()

	return 0

if __name__ == '__main__':
	sys.exit(main())
