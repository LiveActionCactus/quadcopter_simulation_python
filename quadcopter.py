# Class description of a Quadcopter object
#
# By: Patrick Ledzian
# Date: 14 Apr 2020

"""
Class descriptor for a quadcopter object. Includes trajectory, controller, and dynamical equation modules. Calculates
the state update in a single method which is integrated in main.py
"""

# External Libraries
import numpy as np


# noinspection SpellCheckingInspection
class Quadcopter:
	"""
	Class descriptor for a quadcopter object
	"""
	def __init__(self, initial_state=np.zeros(6), desired_state=np.zeros(3)):
		"""
		Class constructor, define the properties inherent to a quadcopter
		:param initial_state: so the initial position and orientation (quaternion) can be set
		:param desired_state: desired position, assumes desired orientation is level
		"""
		# TODO: could make a physical properties dictionary and a quadcopter properties dictionary... maybe idk yet
		self._quad_properties = {
			"mass": 0.030,  																		# mass in kg
			"gravity": 9.81,  																		# gravitational force
			"inertia": np.array([[1.43e-5, 0.0, 0.0], [0.0, 1.43e-5, 0.0], [0.0, 0.0, 2.89e-5]]), 	# inertial tensor m^2 kg
			"invI": np.zeros((3, 3)),
			"length": 0.046,  																		# quadcopter arm length in meters
			"max_force": 0.0,  																		# max force allowed in Newtons
			"min_force": 0.0,
			"limitsA": np.zeros((4, 3)),
			"limitsB": np.zeros((3, 4))
		}
		self._quad_properties.update({
			"invI": np.linalg.inv(self._quad_properties["inertia"]),
			"max_force": (2.5 * self._quad_properties["mass"] * self._quad_properties["gravity"]),
			"min_force": (-0.95 * self._quad_properties["mass"] * self._quad_properties["gravity"]),
			"limitsA": np.array([
				[0.25, 0.0, -0.5 / self._quad_properties["length"]],
				[0.25, 0.5 / self._quad_properties["length"], 0.0],
				[0.25, 0.0, 0.5 / self._quad_properties["length"]],
				[0.25, -0.5 / self._quad_properties["length"], 0.0]
			]),
			"limitsB": np.array([
				[1, 1, 1, 1],
				[0.0, self._quad_properties["length"], 0.0, -self._quad_properties["length"]],
				[-self._quad_properties["length"], 0.0, self._quad_properties["length"], 0.0]
			])
		})
		self._state = Quadcopter.set_initial_state(initial_state)
		self._desired_state = Quadcopter.set_desired_state(desired_state)
		self._goal = np.array(self._desired_state) 							# necessary since by default python assigns by reference
	#
	# Helper methods
	#
	@staticmethod
	def set_initial_state(s):
		# x, y, z, xd, yd, zd, qw, qx, qy, qz, p, q, r
		return np.array([s[0], s[1], s[2], 0.0, 0.0, 0.0, 1.0, s[3], s[4], s[5], 0.0, 0.0, 0.0])

	@staticmethod
	def set_desired_state(s):
		# x, y, z, xd, yd, zd, xdd, ydd, zdd, yaw, yawd
		return np.array([s[0], s[1], s[2], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

	# TODO: encapsulte in quaternion class
	@staticmethod
	def quat2rot(quat):
		"""
		A quaternion describes some rotation in 3-D space to get to a specific position, this function converts that
		to a euler angle parameterized rotation matrix
		:param quat: 1x4 quaternion
		:return: 3x3 rotation matrix bRw
		"""
		q_norm = quat / np.linalg.norm(quat) 										# Frobenius / 2-norm is default
		assert np.shape(q_norm) != "(4,)", "Quaternion vector dimension error"
		q_hat = np.array([
			[0, -q_norm[3], q_norm[2]],
			[q_norm[3], 0, -q_norm[1]],
			[-q_norm[2], q_norm[1], 0]
			])
		R = np.identity(3) + 2*np.matmul(q_hat, q_hat) + 2*q_norm[0]*q_hat

		return R

	@staticmethod
	def quat2euler(quat):
		"""
		Converts a quaternion to euler angles of roll, pitch, and yaw
		:param quat: 1x4 quaternion
		:return: 1x3 euler angle orientation
		"""
		assert str(np.shape(quat)) == "(4,)", "Not a valid quaterion in quat2euler"
		R = Quadcopter.quat2rot(quat)
		phi = np.arcsin(R[1, 2])
		# TODO: assert for imaginary components that can appear in arctan2, compare real to magnitue w/ error bound
		psi = np.arctan2((-R[1, 0]/np.cos(phi)), (R[1, 1]/np.cos(phi)))
		theta = np.arctan2((-R[0, 2]/np.cos(phi)), (R[2, 2]/np.cos(phi)))

		return np.array([phi, theta, psi]) 		# roll, pitch, yaw

	#
	# Behavioral methods
	#
	def simple_line_trajectory(self, cur_pos, stop_pos, finish_time, cur_time):
		"""
		Creates a trajectory by updating the desired state iteratively in the simulation. Doesn't take into account
		the vehicle dynamics when generating
		:param cur_pos: 1x3 current vehicle position
		:param stop_pos: 1x3 desired vehicle position
		:param finish_time: time allowed to finish the movement
		:param cur_time: current time during the trajectory maneuver
		:return:
		"""
		# create desired pos, vel, acc at a given point in time for a linear trajectory
		vel_max = (stop_pos - cur_pos) * 2 / finish_time
		if cur_time >= 0.0:
			vel = vel_max * cur_time / finish_time  # think about having this decay as a broad explonential curve
			pos = cur_pos + (0.5) * cur_time * vel
			acc = np.zeros(3)
		assert cur_time >= 0, "Current time is less than zero in trajectory"
		new_des_state = np.concatenate((pos, vel, acc))

		return new_des_state

	def pid_controller(self):
		"""
		Nested PID controller, the inner-loop is the attitute controller and the outer-loop is the position
		:return: 1x1 desired sum of prop thrusts in body frame, 3x1 desired angular velocity in body frame
		"""
		# define the basic nested PID controller for the quadcopter
		Kp_pos = np.array([15, 15, 30])
		Kd_pos = np.array([12, 12, 10])

		Kp_ang = np.ones(3)*3000
		Kd_ang = np.ones(3)*300

		# Linear acceleration
		acc_des = self._desired_state[6:9] + Kd_pos*(self._desired_state[3:6] - self._state[3:6]) + Kp_pos*(self._desired_state[0:3] - self._state[0:3]) 	# 3x1

		# Build desired roll, pitch, and yaw
		des_yaw = self._desired_state[9]
		phi_des = (1/self._quad_properties["gravity"]) * (acc_des[0]*np.sin(des_yaw) - acc_des[1]*np.cos(des_yaw))
		theta_des = (1/self._quad_properties["gravity"]) * (acc_des[0]*np.cos(des_yaw) + acc_des[1]*np.sin(des_yaw))
		psi_des = des_yaw

		euler_des = np.array([phi_des, theta_des, psi_des])
		pqr_des = np.array([0, 0, self._desired_state[10]])

		quat = self._state[6:10] 					# 4x1
		euler = Quadcopter.quat2euler(quat) 		# 3x1

		thrust = self._quad_properties["mass"] * (self._quad_properties["gravity"] + acc_des[2]) 		# 1x1
		moment = np.matmul(self._quad_properties["inertia"], (Kd_ang*(pqr_des - self._state[10:13]) + Kp_ang*(euler_des - euler))) 		# 3x1

		return thrust, moment

	def equations_of_motion(self, controller_thrust, angular_force):
		"""
		Calculates the rate of change of the vehicle outputting the derivative of the state
		:param controller_thrust: 1x1 desired sum of prop thrusts in body frame
		:param angular_force: 3x1 desired angular velocity in body frame
		:return: 13x1 statedot, rate of change of current state due to controller inputs
		"""
		# define the quadcopter dynamics time step update
		assert str(np.shape(angular_force)) == "(3,)", "Incorrect moment matrix dimensions"
		forces = np.array([controller_thrust, angular_force[0], angular_force[1]]) 					# excludes the yaw force from physical limitations
		prop_thrust = np.matmul(self._quad_properties["limitsA"], forces) 																		# 4x1
		prop_thrust_limited = np.maximum(np.minimum(prop_thrust, self._quad_properties["max_force"]), self._quad_properties["min_force"]) 		# 4x1
		new_thrust = np.matmul(self._quad_properties["limitsB"][0, :], prop_thrust_limited) 													# 1x1
		new_ang_force = np.append(np.matmul(self._quad_properties["limitsB"][1:3], prop_thrust_limited), angular_force[2]) 						# 3x1

		# assign variables from the state array
		state = self._state
		x = state[0]
		y = state[1]
		z = state[2]
		xd = state[3]
		yd = state[4]
		zd = state[5]
		qw = state[6]
		qx = state[7]
		qy = state[8]
		qz = state[9]
		p = state[10]
		q = state[11]
		r = state[12]

		# Orientation
		quat = np.array([qw, qx, qy, qz])
		bRw = Quadcopter.quat2rot(quat) 											# 3x3
		wRb = np.transpose(bRw) 													# 3x3, transforms body frame to world frame

		# Linear Acceleration
		z_thrust = np.array([0, 0, new_thrust]) 									# 3x1
		downward_force = np.array([0, 0, (self._quad_properties["mass"]*self._quad_properties["gravity"])]) 	# 3x1
		linear_accel = 1 / self._quad_properties["mass"] * (np.matmul(wRb, z_thrust) - downward_force) 			# 3x1, in world frame

		# Constraint quaternion (must have norm of 1)
		quat_err = 1 - np.sum(np.square(quat)) 										# 1x1
		# TODO: implement skew symmetric matrix check
		# TODO: encapsulte in quaternion class
		# symmetric check: (arr.transpose() == arr).all()
		# skew symmetric check: (arr.transpose() == -arr).all()

		q_special_form = np.array([
			[0, -p, -q, -r],
			[p, 0, -r, q],
			[q, r, 0, -p],
			[r, -q, p, 0]
		])
		qdot = -0.5*np.matmul(q_special_form, quat) + 2*quat_err*quat

		# Angular Acceleration
		# TODO: validate the size of these numbers, they seem large
		# TODO: implement check for diagonal omegadot, off-diagonal values should be 0
		omega = np.array([p, q, r]) 												# 3x1
		omegadot = self._quad_properties["invI"] * (new_ang_force - np.cross(omega, (np.matmul(self._quad_properties["inertia"], omega)))) 		# 3x3

		# Create state derivative
		statedot = np.array([
			xd,
			yd,
			zd,
			linear_accel[0],
			linear_accel[1],
			linear_accel[2],
			qdot[0],
			qdot[1],
			qdot[2],
			qdot[3],
			omegadot[0, 0],
			omegadot[1, 1],
			omegadot[2, 2]
		])
		assert str(np.shape(statedot)) == "(13,)", "statedot is of the wrong dimension"

		return statedot 					# 13x1

	def simulation_step(self, s=0, t=0, total_sim_time=0):
		"""
		Combines a trajectory with a controller with a definition of dynamics to return a rate of change of the current
		state for integration in main.py to get the new state s.
		:param s: 13x1 current state
		:param t: current time step for the odeint() solver
		:param total_sim_time: total time allowed to finish the trajectory
		:return: 13x1 statedot, rate of change of current state due to controller inputs
		"""
		self._state = s
		assert str(np.shape(self._state)) == "(13,)", "Incorrect state vector size in simulation_step: {}".format(np.shape(self._state))
		assert str(np.shape(self._desired_state)) == "(11,)", "Incorrect desired state vector size in simulation_step: {}".format(np.shape(self._desired_state))
		assert str(np.shape(self._goal)) == "(11,)", "Incorrect goal vector size in simulation_step: {}".format(np.shape(self._goal))

		# This is where the lego blocks go together. Change trajectories, controllers, and dynamics here.
		new_des_state = self.simple_line_trajectory(self._state[0:3], self._goal[0:3], total_sim_time, t)
		self._desired_state[0:9] = new_des_state
		thrust, moment = self.pid_controller()
		statedot = self.equations_of_motion(thrust, moment)

		return statedot