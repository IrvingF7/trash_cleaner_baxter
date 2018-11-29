import sys
import rospy
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError
import IPython

class KinectRGBD(object):

	def __init__(self):
		# rospy.init_node('kinect_rgbd_listener')
		CAM_INFO_TOPIC = '/camera/rgb/camera_info'
		C_IMG_TOPIC = '/camera/rgb/image_color'
		D_IMG_TOPIC = '/camera/depth_registered/image_raw'

		self._bridge = CvBridge()

		self.cam_info = None
		self.c_img = None
		self.d_img = None

		self._cam_info_updated = False
		self._c_img_updated = False
		self._d_img_updated = False

		self._cam_info_sub = rospy.Subscriber(CAM_INFO_TOPIC, CameraInfo, self._cam_info_callback)
		self._c_img_sub = rospy.Subscriber(C_IMG_TOPIC, Image, self._c_img_callback)
		self._d_img_sub = rospy.Subscriber(D_IMG_TOPIC, Image, self._d_img_callback)

		while True:
			if self._cam_info_updated and self._c_img_updated and self._d_img_updated:
				break

	def get_cam_info(self):
		return self.cam_info

	def get_c_img(self):
		return self.c_img

	def get_d_img(self):
		return self.d_img

	def _cam_info_callback(self, data):
		try:
			self.cam_info = data
			self._cam_info_updated = True
		except CvBridgeError as e:
			rospy.logerr(e)

	def _c_img_callback(self, data):
		try:
			self.c_img = self._bridge.imgmsg_to_cv2(data, "bgr8")
			self._c_img_updated = True
		except CvBridgeError as e:
			rospy.logerr(e)

	def _d_img_callback(self, data):
		try:
			self.d_img = self._bridge.imgmsg_to_cv2(data)
			self._d_img_updated = True
		except CvBridgeError as e:
			rospy.logerr(e)