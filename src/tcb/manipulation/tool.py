import numpy as np
import rospy
import tf

from geometry_msgs.msg import (
	PoseStamped,
	Pose,
	Quaternion,
)

from tcb.manipulation.arm_manipulation import PickAndPlace

class Tool(object):

	def __init__(self):
		self._ar_frame = '/ar_marker_0'
		self._world_frame = '/base'

		self._tool_offset = np.zeros(3)
		self._gripper_rot = (0, 1, 0)

		self._listener = tf.TransformListener()
		self.pose = self._compute_pose()

		self.arm = baxter_arm

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

		tool_trans = ar_trans + self._tool_offset

		tool_rot = tf.transformations.quaternion_from_euler(self._gripper_rot[0], self._gripper_rot[1], 
															self._gripper_rot[1])
		tool_rot = Quaternion(x=tool_rot[0], y=tool_rot[1], z=tool_rot[2], w=tool_rot[3])

		tool_pose = Pose(position=Point(x=tool_trans[0], y=tool_trans[1], z=tool_trans[2]),
						orientation=tool_rot)

		return tool_pose

	def pick(self, suction_time=0.0):
		if not self.arm.is_suction:
			self.arm.gripper_open()

		self.arm.approach(self.pose)

		self.arm.gripper_to_pose(self.pose)

		if not self.arm.is_suction:
			self.arm.gripper_close()
		else:
			self.arm.suction_on(suction_time)

	@abstractmethod
	def return_to_start(self):
		"""return tool to its starting location"""


class Broom(Tool):

	def __init__(self, baxter_arm):
		self._ar_frame = '/ar_marker_5'
		
		self._tool_offset = np.array([0.05, 0, 0])
		self._gripper_rot = (-np.pi, 0, -np.pi / 2)
		
	def sweep(self, trash_frame):
		x, y, z = trash_frame[0], trash_frame[1], trash_frame[2]
		start_y = y + 0.15
		end_y = y - 0.15

		start_pose = Pose(position=Point(x=x, y=start_y, z=z), 
							orientation=self._gripper_rot)

		end_pose = Pose(position=Point(x=x, y=end_y, z=z), 
						orientation=self._gripper_rot)

		self.arm.gripper_to_pose(start_pose)
		rospy.sleep(1.0)
		self.arm.gripper_to_pose(end_pose)
		self.arm.retract()

	def return_to_start(self):
		self.arm.gripper_to_pose(self.pose)
		self.arm.gripper_open()
		self.arm.move_to_start()

class DustPan(Tool):

	def __init__(self):
		self._ar_frame = '/ar_marker_4'
		
		self._tool_offset = np.array([0.05, 0, 0])
		self._gripper_rot = (-np.pi, 0, -np.pi / 2)

	def place_pan(self, trash_frame):
		x, y, z = trash_frame[0], trash_frame[1], trash_frame[2]
		set_y = y - 0.15

		set_pose = Pose(position=Point(x=x, y=set_y, z=z), 
						orientation=self._gripper_rot)

		self.arm.gripper_to_pose(set_pose)

	def return_to_start(self):
		self.arm.suction_on()
		self.arm.gripper_to_pose(self.pose)
		self.arm.move_to_start()

class Sticky(Tool):

	def __init__(self):
		self._ar_frame = '/ar_marker_6'
		
		self._tool_offset = np.array(0.05, 0, 0)
		self._gripper_rot = (-np.pi, 0, -np.pi / 2)

	