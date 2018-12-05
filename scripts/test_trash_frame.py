#!/usr/bin/env python
import rospy
import cv2
from tcb.perception.kinect_rgbd import KinectRGBD
from tcb.perception.trash_frame import TrashFrame
from tcb.perception.trash import Trash, find_trash
import IPython
import tf

import numpy as np
import os
import sys
import tensorflow as tf
import IPython
from matplotlib import pyplot as plt
from PIL import Image
import time as timer
import sys
import os
import cv2
# os.environ["CUDA_DEVICE_ORDER"]="PCI_BUS_ID"   # see issue #152
# os.environ["CUDA_VISIBLE_DEVICES"]="0"
sys.path.append('~/models/research')
from object_detection.utils import label_map_util
from object_detection.utils import visualization_utils as vis_util
from tcb.obj_detection.object_detector import ObjectDetector

def draw_bbox(c_img, bbox):
	cv2.rectangle(c_img,(bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 3)
	

def test():
	rospy.init_node("trash_frame_demo")
	rgbd_cam = KinectRGBD()
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
	valid_trash = find_trash(output_dict, 0.8, c_img, d_img)

	bboxes = []
	for trash in valid_trash:
		bbox = [trash.points['x_min'], trash.points['y_min'], trash.points['x_max'], trash.points['y_max']]
		print(trash.label_num)
		bboxes.append(bbox)
		draw_bbox(c_img, bbox)
		cv2.imwrite('debug_imgs/c_img_bbox.png', c_img)

	# cg = trash.get_cg()
	# depth = trash.get_depth()
	# print('cg', cg)
	# print('depth', depth)
	# trash_point = (cg[0], cg[1], depth)

	# trash_frame = TrashFrame(rgbd_cam)
	# wt = trash_frame.generate_world2trash(trash_point)
	# print('trash_frame0', wt)

	# br = tf.TransformBroadcaster()
	# while not rospy.is_shutdown():
	# 	br.sendTransform(wt[0], wt[1], rospy.Time.now(), '/trash_frame0', '/base')


if __name__ == "__main__":
	test()