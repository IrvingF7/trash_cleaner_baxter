import sys
import rospy
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

class KinectRGBD(object):

	def __init__(self):
		CAM_INFO_TOPIC = '/kinect2/hd/camera_info'
		C_IMG_TOPIC = '/kinect2/hd/image_color_rect'
		D_IMG_TOPIC = '/kinect2/hd/image_depth_rect'

		self._bridge = CvBridge()

		self.cam_info = None
		self.c_img = None
		self.d_img = None

		self._cam_info_sub = rospy.Subscriber(CAM_INFO_TOPIC, CameraInfo, self._cam_info_callback)
		self._c_img_sub = rospy.Subscriber(C_IMG_TOPIC, Image, self._c_img_callback)
		self._d_img_sub = rospy.Subscriber(D_IMG_TOPIC, Image, self._d_img_callback)

	def get_cam_info(self):
		return self.cam_info

	def get_c_img(self):
		return self.c_img

	def get_d_img(self):
		return self.d_img

	def _cam_info_callback(self, data):
		try:
			self.cam_info = data
		except CvBridgeError as e:
			rospy.logerr(e)

	def _c_img_callback(self, data):
		try:
			self.c_img = self._bridge.imgmsg_to_cv2(data, "bgr8")
		except CvBridgeError as e:
			rospy.logerr(e)

	def _d_img_callback(self, data):
		try:
			self.d_img = self._bridge.imgmsg_to_cv2(data)
		except CvBridgeError as e:
			rospy.logerr(e)

if __name__ == "__main__":
	rgbd_cam = KinectRGBD()

	cam_info = rgbd_cam.get_cam_info()
	c_img = rgbd_cam.get_c_img()
	d_img = rgbd_cam.get_d_img()

	rospy.loginfo(cam_info)
	cv2.imshow('color image', c_img)
	cv2.imshow('depth image', d_img)

	cv2.waitKey(0)
	cv2.destroyAllWindows()