import sys
import rospy
import cv2
import tf
from image_geometry import PinholeCameraModel
import thread
import numpy as np

class TrashFrame(object):

	def __init__(self, cam):
		self._cam = cam
		self._cam_info = cam.get_cam_info()
		
		self._cam_model = PinholeCameraModel()
		self._cam_model.fromCameraInfo(self._cam_info)

		self._br = tf.TransformBroadcaster()
		self._listener = tf.TransformListener()
		self._world_frame = '/base'
		self._cam_frame = '/kinect'

		self._frame_num = 0

	def generate_world2trash(self, point, rot=(0.0, 0.0, 0.0)):
		trans_kt = self._compute_kinect2trash(point)
		frame_name = 'frame_' + str(self._frame_num)

		thread.start_new_thread(self._broadcast_trashframe, (frame_name, trans_kt, rot))

		self._frame_num += 1

		return frame_name

	def _compute_kinect2trash(self, point):
		x, y, z = point[0], point[1], point[2]

		ray = self._cam_model.projectPixelTo3dRay((x, y))

		ray = np.array(ray)
		trans_kt = ray / ray[2]
		# convert depth pixels in mm to m, then scale by the depth
		trans_kt = trans_kt * (z * 0.001)

		return trans_kt

	def _broadcast_trashframe(self, frame_name, trans_kt, rot):
		wt_trans, wt_rot = self._compute_world2trash(trans_kt, rot)
		wt_trans = (wt_trans[0], wt_trans[1], wt_trans[2])
		wt_rot = tf.transformations.quaternion_from_euler(wt_rot[0], wt_rot[1], wt_rot[2])

		while True:
			self._br.sendTransform(wt_trans, 
									wt_rot,
									rospy.Time.now(),
									frame_name,
									self._world_frame)

	def _compute_world2trash(self, trans_kt, rot):
		found_tf = False

		while not found_tf:
			try:
				wk_trans, wk_rot = self._listener.lookupTransform(self._cam_frame, 
																	self._world_frame, 
																	rospy.Time(0))
				found_tf = True
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue

		
		g_wk = tf.transformations.euler_matrix(ai=wk_rot[0], aj=wk_rot[1], ak=wk_rot[2])
		p_wk = tf.transformations.translation_matrix(wk_trans)
		g_wk[:,3] = p_wk[:,3]
		
		g_kt = tf.transformations.euler_matrix(ai=rot[0], aj=rot[1], ak=rot[2])
		p_kt = tf.transformations.translation_matrix(trans_kt)
		g_kt[:,3] = p_kt[:,3]

		g_wt = np.matmul(g_wk, g_kt)

		wt_trans = tf.transformations.translation_from_matrix(g_wt)
		wt_rot = tf.transformations.euler_from_matrix(g_wt)

		return wt_trans, wt_rot