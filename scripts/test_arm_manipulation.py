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

from tcb.manipulation.arm_manipulation import BaxterArm
from tcb.manipulation.tool import Tool, Broom, DustPan, Sticky
import time

def draw_rectangle_and_save(c_img, d_img, bbox):
	cv2.rectangle(c_img,(bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 3)
	cv2.imwrite('c_img.png', c_img)
	cv2.imwrite('d_img.png', d_img)

def main():
	# note IK operates w.r.t. /base frame
	rospy.init_node("ik_pick_and_place_demo")

	left_arm = BaxterArm('left')
	left_arm.move_to_start()

	right_arm = BaxterArm('right')
	right_arm.move_to_start()

	rgbd_cam = KinectRGBD()
	cam_info = rgbd_cam.get_cam_info()
	c_img = rgbd_cam.get_c_img()
	d_img = rgbd_cam.get_d_img()

	bbox = [230, 330, 320, 390]

	draw_rectangle_and_save(c_img, d_img, bbox)

	trash = Trash(bbox, 0, c_img, d_img)
	cg = trash.get_cg()
	depth = trash.get_depth()
	trash_point = (cg[0], cg[1], depth)

	trash_frame = TrashFrame(rgbd_cam)
	wt = trash_frame.generate_world2trash(trash_point)

	broom = Broom(left_arm)
	dustpan = DustPan(right_arm)

	broom.pick()

	dustpan.pick()
	broom.sweep(wt)

	broom.return_to_start()
	dustpan.return_to_start()

if __name__ == '__main__':
	sys.exit(main())
