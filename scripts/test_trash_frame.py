#!/usr/bin/env python
import rospy
import cv2
from tcb.perception.kinect_rgbd import KinectRGBD
from tcb.perception.trash_frame import TrashFrame
from tcb.perception.trash import Trash
import IPython

def test():
	rgbd_cam = KinectRGBD()
	cam_info = rgbd_cam.get_cam_info()
	IPython.embed()
	c_img = rgbd_cam.get_c_img()
	d_img = rgbd_cam.get_d_img()
	cv2.imwrite('c_img.png', c_img)
	cv2.imwrite('d_img.png', d_img)

	bbox = [150, 300, 200, 350]
	trash = Trash(bbox, 0, c_img, d_img)

	cg = trash.get_cg()
	depth = trash.get_depth()
	trash_point = (cg[0], cg[1], depth)

	trash_frame = TrashFrame(rgbd_cam)
	kt = trash_frame.generate_world2trash(trash_point)
	while not rospy.is_shutdown():
		pass

if __name__ == "__main__":
	test()