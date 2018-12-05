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
import numpy as np
import tf
import cv2
from tcb.perception.kinect_rgbd import KinectRGBD
from tcb.perception.trash_frame import TrashFrame
from tcb.perception.trash import Trash, find_trash

from tcb.manipulation.arm_manipulation import BaxterArm
from tcb.manipulation.deposit_bin import DepositBin
from tcb.manipulation.tool import Tool, Broom, DustPan, Sticky
import time

sys.path.append('~/models/research')
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util
from tcb.obj_detection.object_detector import ObjectDetector

def draw_bbox(c_img, bbox):
	cv2.rectangle(c_img,(bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 3)

def main():
	# note IK operates w.r.t. /base frame
	rospy.init_node("ik_pick_and_place_demo")

	
	left_arm = BaxterArm('left')
	left_arm.move_to_start()

	right_arm = BaxterArm('right')
	right_arm.move_to_start()

	rgbd_cam = KinectRGBD()

	stop_demo = False
	while not stop_demo:
		cam_info = rgbd_cam.get_cam_info()
		c_img = rgbd_cam.get_c_img()
		d_img = rgbd_cam.get_d_img()

		c_img_path = 'debug_imgs/c_img.png'
		cv2.imwrite(c_img_path, c_img)
		cv2.imwrite('debug_imgs/d_img.png', d_img)
		# c_img = cv2.imread(c_img_path)
		# d_img = cv2.imread('debug_imgs/d_img.png')  

		obj_det = ObjectDetector()
		boxes, scores, classes, num = obj_det.detect(c_img_path)
		output_dict = {'detection_boxes': np.squeeze(boxes), 'detection_scores' : np.squeeze(scores), 
						'detection_classes' : np.squeeze(classes).astype(np.int32), 'detection_num' : np.squeeze(num)}
		valid_trash = find_trash(output_dict, 0.9, c_img, d_img)

		bboxes = []
		for trash in valid_trash:
			bbox = [trash.points['x_min'], trash.points['y_min'], trash.points['x_max'], trash.points['y_max']]
			if trash.label_num == 1:
				print("chips")
			bboxes.append(bbox)
			draw_bbox(c_img, bbox)
			cv2.imwrite('debug_imgs/c_img_bbox.png', c_img)

		trash = valid_trash[0]
		cg = trash.get_cg()
		depth = trash.get_depth()
		trash_point = (cg[0], cg[1], depth)

		trash_frame = TrashFrame(rgbd_cam)
		wt = trash_frame.generate_world2trash(trash_point)
		trash_pose = trash_frame.frame2pose(wt)

		broom = Broom(left_arm)
		dustpan = DustPan(right_arm)
		# deposit_bin = DepositBin(6, True)

		if trash.label_num == 1:
			broom.pick()
			dustpan.pick()
			broom.sweep(wt, num_times=3)
			broom.return_to_start()
			dustpan.return_to_start()
		elif trash.label_num == 2 or trash.label_num == 3:
			deposit_pose = deposit_bin.get_pose()
			right_arm.approach(trash_pose)
			right_arm.gripper_to_pose(trash_pose)
			right_arm.approach(deposit_pose)
			right_arm.suction_off()
			right_arm.move_to_start()

		IPython.embed()

if __name__ == '__main__':
	sys.exit(main())
