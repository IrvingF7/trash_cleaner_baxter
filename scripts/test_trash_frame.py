#!/usr/bin/env python
import rospy
import cv2
from tcb.perception.kinect_rgbd import KinectRGBD
from tcb.perception.trash_frame import TrashFrame
from tcb.perception.trash import Trash
import IPython
import tf

def test():
	rgbd_cam = KinectRGBD()
	cam_info = rgbd_cam.get_cam_info()
	c_img = rgbd_cam.get_c_img()
	d_img = rgbd_cam.get_d_img()
	cv2.imwrite('c_img.png', c_img)
	cv2.imwrite('d_img.png', d_img)

	bbox = [300, 330, 310, 340]
	trash = Trash(bbox, 0, c_img, d_img)
	cg = trash.get_cg()
	depth = trash.get_depth()
	trash_point = (cg[0], cg[1], depth)

	trash_frame = TrashFrame(rgbd_cam)
	wt = trash_frame.generate_world2trash(trash_point)
	print('trash_frame0', wt)

	bbox = [300, 230, 310, 240]
	trash = Trash(bbox, 0, c_img, d_img)
	cg = trash.get_cg()
	depth = trash.get_depth()
	trash_point = (cg[0], cg[1], depth)
	wt = trash_frame.generate_world2trash(trash_point)
	print('trash_frame1', wt)
	# listener = tf.TransformListener()
	# found_tf = False
	# while not found_tf:
	# 	try:
	# 		wt_trans, wt_rot = listener.lookupTransform('/base', wt, rospy.Time(0))
	# 		print("TRANSFORMS", wt_trans, wt_rot)
	# 	except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
	# 		print('not found')
	# 		continue


if __name__ == "__main__":
	test()