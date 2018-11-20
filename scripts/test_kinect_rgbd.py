#!/usr/bin/env python
import rospy
import cv2
from tcb.perception.kinect_rgbd import KinectRGBD

def test():
	rgbd_cam = KinectRGBD()
	cam_info = rgbd_cam.get_cam_info()
	c_img = rgbd_cam.get_c_img()
	d_img = rgbd_cam.get_d_img()

	rospy.loginfo(cam_info)
	cv2.imwrite('c_img.png', c_img)
	cv2.imwrite('d_img.png', d_img)

	cv2.waitKey(0)
	cv2.destroyAllWindows()

if __name__ == "__main__":
	test()