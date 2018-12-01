import argparse
import struct
import sys
import copy

import rospy

from geometry_msgs.msg import (
	PoseStamped,
	Pose,
	Quaternion,
)
from std_msgs.msg import (
	Header,
	Empty,
)

from baxter_core_msgs.srv import (
	SolvePositionIK,
	SolvePositionIKRequest,
)

import baxter_interface

class PickAndPlace(object):
	def __init__(self, limb, hover_distance=0.15, verbose=True, suction=False):
		self._limb_name = limb # string
		self._suction = suction
		self._hover_distance = hover_distance # in meters
		self._verbose = verbose # bool (print debug statements)
		self._limb = baxter_interface.Limb(limb)
		self._gripper = baxter_interface.Gripper(limb)
		# service name nx
		nx = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
		# An object for the service call initialized by passing service name nx
		# and the Request-Response message type SolvePositionIK.SolvePositionIKRequst
		self._iksvc = rospy.ServiceProxy(nx, SolvePositionIK)
		# check existence of action server for limb, timeout 5 seconds
		rospy.wait_for_service(nx, 5.0)
		# verify robot is enabled
		print("Getting robot state...")
		self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
		self._init_state = self._rs.state().enabled
		print("Enabling robot...")
		self._rs.enable()
		if not suction:
			self._gripper.calibrate()

	# move arm to a desired initial pose, if none supplied, 0 angle for every joint
	def move_to_start(self, start_angles=None):
		print("moving the {0} arm to start pose...".format(self._limb_name))

		if not start_angles:
			start_angles = dict(zip(self._joint_names, [0]*7))

		self._guarded_move_to_joint_position(start_angles)
		self.gripper_open()
		rospy.sleep(1.0)
		print("Running. Ctrl-c to quit.")

	# Requests an Inverse Kinematics solution with the desired end effector pose.
	# Returns the solved joint angles if there is a solution, false otherwise.
	def ik_request(self, pose):
		hdr = Header(stamp=rospy.Time.now(), frame_id='base')
		ikreq = SolvePositionIKRequest()
		ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))

		try:
			resp = self._iksvc(ikreq)
		except (rospy.ServiceException, rospy.ROSException), e:
			rospy.logerr("Service call failed: %s" % (e,))

			return False
		# Check if result valid, and type of seed ultimately used to get solution
		# convert rosopy's string representation of unit8[]'s to int's
		resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
		limb_joints = {}

		if (resp_seeds[0] != resp.RESULT_INVALID):
			seed_str = {ikreq.SEED_USER: 'User Provided Seed',
						ikreq.SEED_CURRENT: 'Current Joint Angles',
						ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
						}.get(resp_seeds[0], 'None')
			if self._verbose:
				print("IK Solution SUCCESS - Seed Type: {0}".format(seed_str))
			# Format solution into Limb API-compatible dictionary
			limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
			if self._verbose:
				print("IK Joint Solution:\n{0}".format(limb_joints))
				print("--------------------")
		else:
			rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")

			return False

		return limb_joints

	def _guarded_move_to_joint_position(self, joint_angles):
		if joint_angles:
			self._limb.move_to_joint_positions(joint_angles)
		else:
			rospy.logerr("No Joint Angles provided for move_to_joint_positions.")

	def gripper_open(self):
		try:
			self._gripper.command_position(100)
			rospy.sleep(1.0)
		except:
			rospy.logerr("not a parallel jaw gripper")

	def gripper_close(self):
		try:
			self._gripper.command_position(0)
			rospy.sleep(1.0)
		except:
			rospy.logerr("not a parallel jaw gripper")

	def suction_on(self):
		try:
			self._gripper.command_suction()
			rospy.sleep(1.0)
		except:
			rospy.logerr("not a suction gripper")

	def suction_off(self):
		try:
			self._gripper.set_blow_off(2.0)
			rospy.sleep(1.0)
		except:
			rospy.logerr("not a suction gripper")

	def _approach(self, pose):
		approach = copy.deepcopy(pose)
		# approach with a pose hover-distance above the requested pose
		approach.position.z = approach.position.z + self._hover_distance
		joint_angles = self.ik_request(approach)

		self._guarded_move_to_joint_position(joint_angles)

	def _retract(self):
		# retrieve current pose from endpoint
		current_pose = self._limb.endpoint_pose()
		ik_pose = Pose()
		ik_pose.position.x = current_pose['position'].x
		ik_pose.position.y = current_pose['position'].y
		ik_pose.position.z = current_pose['position'].z + self._hover_distance
		ik_pose.orientation.x = current_pose['orientation'].x
		ik_pose.orientation.y = current_pose['orientation'].y
		ik_pose.orientation.z = current_pose['orientation'].z
		ik_pose.orientation.w = current_pose['orientation'].w
		joint_angles = self.ik_request(ik_pose)
		# servo up from current pose
		self._guarded_move_to_joint_position(joint_angles)

	def _servo_to_pose(self, pose):
		# servo down to release
		joint_angles = self.ik_request(pose)
		self._guarded_move_to_joint_position(joint_angles)

	def sweep(self, pose1, pose2):
		self._approach(pose1)
		# servo to pose
		self._servo_to_pose(pose1)
		self.gripper_close()
		self._servo_to_pose(pose2)

	def pick(self, pose):
		# open the gripper
		if not self._suction:
			self.gripper_open()
		else:
			self.suction_on()
		# servo above pose
		self._approach(pose)
		# servo to pose
		self._servo_to_pose(pose)
		# close gripper
		if not self._suction:
			self.gripper_close()
		else:
			self.suction_off()
		# retract to clear object
		self._retract()

	def place(self, pose):
		# servo above pose
		self._approach(pose)
		# servo to pose
		self._servo_to_pose(pose)
		#open the gripper
		if not self._suction:
			self.gripper_open()
		else:
			self.suction_off()
		# retract to clear object
		self._retract()
