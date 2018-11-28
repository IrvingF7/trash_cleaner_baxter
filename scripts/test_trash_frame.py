#!/usr/bin/env python
import rospy
import cv2
from tcb.perception.kinect_rgbd import KinectRGBD
from tcb.perception.trash_frame import TrashFrame
from tcb.perception.trash import Trash
import IPython
import tf

def test():
	rospy.init_node("trash_frame_demo")
	rgbd_cam = KinectRGBD()
	cam_info = rgbd_cam.get_cam_info()
	c_img = rgbd_cam.get_c_img()
	d_img = rgbd_cam.get_d_img()

	bbox = [200, 300, 240, 320]

	cv2.rectangle(c_img,(bbox[0], bbox[1]), (bbox[2], bbox[3]), (0, 255, 0), 3)
	cv2.imwrite('c_img.png', c_img)
	cv2.imwrite('d_img.png', d_img)

	trash = Trash(bbox, 0, c_img, d_img)
	cg = trash.get_cg()
	depth = trash.get_depth()
	print('cg', cg)
	print('depth', depth)
	trash_point = (cg[0], cg[1], depth)

	trash_frame = TrashFrame(rgbd_cam)
	wt = trash_frame.generate_world2trash(trash_point)
	print('trash_frame0', wt)

	br = tf.TransformBroadcaster()
	while not rospy.is_shutdown():
		br.sendTransform(wt[0], wt[1], rospy.Time.now(), '/trash_frame0', '/base')

	# bbox = [300, 230, 310, 240]
	# trash = Trash(bbox, 0, c_img, d_img)
	# cg = trash.get_cg()
	# depth = trash.get_depth()
	# trash_point = (cg[0], cg[1], depth)
	# wt = trash_frame.generate_world2trash(trash_point)
	# print('trash_frame1', wt)
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