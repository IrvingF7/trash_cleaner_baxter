#!/usr/bin/env python
import rospy
import cv2
from tcb.perception.kinect_rgbd import KinectRGBD
import IPython

rospy.init_node('kinect_rgbd_listener')

rgbd_cam = KinectRGBD()

class ImgCollect:
	def __init__(self):
		self.img_num = 0

	def take_image(self):
		c_img = rgbd_cam.get_c_img()
		d_img = rgbd_cam.get_d_img()

		cv2.imwrite('c_img_' + str(self.img_num) + '.png', c_img)
		cv2.imwrite('d_img_' + str(self.img_num) + '.png', d_img)
		self.img_num += 1

if __name__ == "__main__":
	img_collect = ImgCollect()
	while not rospy.is_shutdown():
		IPython.embed()