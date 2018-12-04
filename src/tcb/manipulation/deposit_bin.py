import numpy as np
import rospy
import tf

from geometry_msgs.msg import (
	PoseStamped,
	Pose,
	Quaternion,
	Point,
)

from tcb.manipulation.arm_manipulation import BaxterArm
import abc

class DepositBin(object):

	def __init__(self, ar_num, suction_drop):
		self._ar_frame = '/ar_marker_' + str(ar_num)

		if not suction_drop:
			self._deposit_offset = np.array([0.09, 0, -0.03])
			self._grippe_rot = (-np.pi, 0, np.pi / 2)
		else:
			self._deposit_offset = np.array([0.03, 0, -0.03])
			self._gripper_rot = (-np.pi, 0, 0)

		self._listener = tf.TransformListener()
		self.pose = self._compute_pose()

	def get_pose(self):
		return self.pose

	def _compute_pose(self):
		found_tf = False
		while not found_tf:
			try:
				ar_trans, ar_rot = self._listener.lookupTransform(self._world_frame, 
																	self._ar_frame, 
																	rospy.Time(0))
				found_tf = True
			except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
				continue

		deposit_trans = ar_trans + self._deposit_offset

		deposit_rot = tf.transformations.quaternion_from_euler(self._gripper_rot[0], self._gripper_rot[1], 
															self._gripper_rot[2])
		deposit_rot = Quaternion(x=deposit_rot[0], y=deposit_rot[1], z=deposit_rot[2], w=deposit_rot[3])

		deposit_pose = Pose(position=Point(x=deposit_trans[0], y=deposit_trans[1], z=deposit_trans[2]),
						orientation=deposit_rot)

		return deposit_pose