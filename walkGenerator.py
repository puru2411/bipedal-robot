import numpy as np
import time
import motorController
import trajectoryGenerator


class WalkGenerator():
	def __init__(self,pelvic_interval, legUp_length, legDown_length, footJoint_to_bottom, hip_shift, sittinghip_shift, swayLength, stepTime, robot, physicsClient, motor_force):
		self._pelvic_interval = pelvic_interval   # distance between the two hip joints
		self._legUp_length = legUp_length        # hip joint to knee joint
		self._legDown_length = legDown_length     # knee joint to foot joint
		self._footJoint_to_bottom = footJoint_to_bottom # foot joint to bottom
		self._hip_shift = hip_shift         # hip_shift of lift of a foot #temp in original (temp)
		self._sittinghip_shift = sittinghip_shift   # sit hip_shift. increase this will make leg more fold. too high or too low makes an error  (z0)
		self._swayLength = swayLength         # foot sway length, length by which feet move in one step (x0)
		self._stepTime = stepTime             # time taken for one step (t0)

		self.trajectory = trajectoryGenerator.TrajectoryGenerator(pelvic_interval, legUp_length, legDown_length, footJoint_to_bottom)  # initializing trajectory generator
		self.trajectory.setTrajectoryParameters(hip_shift, sittinghip_shift, swayLength, stepTime)                  # setting up walk/trajectory parameters
		self.controller = motorController.MotorController(robot, physicsClient, motor_force)  # setting up motor controller to control motor positions

	# Inverse Kinematics (to be updated...)
	def getInverseKinematics(self, Phip, Pfoot):
		l1 = self._legUp_length
		l2 = self._legDown_length
		l3 = self._footJoint_to_bottom
		
		x = Pfoot[0]- Phip[0]
		y = Pfoot[1]- Phip[1]
		z = Phip[2] - Pfoot[2] - l3

		q1 = 0
		q2 = np.arctan(y/z)
		q6 = q2

		l11 = l1*np.cos(q2)
		l21 = l2*np.cos(q2)
		r = np.sqrt(z**2 + x**2)

		phi1 = np.arctan(x/z)
		phi2 = np.arccos((l11**2 + r**2 - l21**2)/(2*l11*r))
		q3 = phi1+phi2

		phi3 = np.arccos((l11**2 + l21**2 - r**2)/(2*l11*l21))
		q4 = np.pi - phi3
		q5 = q4-q3
		return [q1, q2, q3, q4, q5, q6]

	def sit_stand_motion(self, d):
		t0 = self._stepTime

		t = 0
		while t<=t0/4:
			if d == 'sit':
				p1 , p2, p3, p4 = self.trajectory.getSittingTrajectory(t)
			elif d == 'sta':
				p1 , p2, p3, p4 = self.trajectory.getStandingTrajectory(t)
			
			targetMotorPositons = self.getInverseKinematics(p2, p1) + self.getInverseKinematics(p3, p4)
			self.controller.setMotorPositions(targetMotorPositons)
			t+=.00015
			time.sleep(.0001)

	def start_stop_motions(self, d, swayLength):
		t0 = self._stepTime
		t = 0
		while t<=t0:
			if d == 'lst':
				p1 , p2, p3, p4 = self.trajectory.getLeftLegStartingTrajectory(t,swayLength)
			elif d == 'rst':
				p1 , p2, p3, p4 = self.trajectory.getRightLegStartingTrajectory(t,swayLength)
			elif d == 'ls':
				p1 , p2, p3, p4 = self.trajectory.getLeftLegStoppingTrajectory(t,swayLength)
			elif d == 'rs':
				p1 , p2, p3, p4 = self.trajectory.getRightLegStoppingTrajectory(t,swayLength)

			targetMotorPositons = self.getInverseKinematics(p2, p1) + self.getInverseKinematics(p3, p4)
			self.controller.setMotorPositions(targetMotorPositons)
			t+=.00015
			time.sleep(.0001)

	def walk(self, d, swayLength, turn, angle):
		t0 = self._stepTime
		t = 0
		while t<=t0:
			if d == 'l':
				p1 , p2, p3, p4 = self.trajectory.getLeftLegWalkingTrajectory(turn,t,swayLength)

				targetMotorPositons = self.getInverseKinematics(p2, p1) + self.getInverseKinematics(p3, p4)
				if(turn == "l"):
					targetMotorPositons[6] = -((t-2*t0/10)/(6*t0/10))*angle/2
					targetMotorPositons[0] = ((t-2*t0/10)/(6*t0/10))*angle/2
				elif(turn == "r"):
					targetMotorPositons[6] = -(1-(t-2*t0/10)/(6*t0/10))*angle/2
					targetMotorPositons[0] = (1-(t-2*t0/10)/(6*t0/10))*angle/2

			elif d == 'r':
				p1 , p2, p3, p4 = self.trajectory.getRightLegWalkingTrajectory(turn,t,swayLength)

				targetMotorPositons = self.getInverseKinematics(p2, p1) + self.getInverseKinematics(p3, p4)

				if(turn == "l"):
					targetMotorPositons[6] = -(1-(t-2*t0/10)/(6*t0/10))*angle/2
					targetMotorPositons[0] = (1-(t-2*t0/10)/(6*t0/10))*angle/2
				elif(turn == "r"):
					targetMotorPositons[6] = -((t-2*t0/10)/(6*t0/10))*angle/2
					targetMotorPositons[0] = ((t-2*t0/10)/(6*t0/10))*angle/2

			self.controller.setMotorPositions(targetMotorPositons)
			t+=.00015
			time.sleep(.0001)
