# Code By:-
# Purushotam Kumar Agrawal { Github ---> PURU2411 }
# Vibhanshu Vaibhav { Github ---> VibhanshuV }
# Project GitHub: ---> https://github.com/puru2411/bipedal-robot.git

import pybullet as p
import time
from time import sleep
import pybullet_data
import numpy as np
import math
import os
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

import motorController
import trajectoryGenerator


# Robot Body Parameters 
l1 = 0.11		# Hip to Knee Length
l2 = 0.11		# Knee to Ankle Length
l3 = 0.031  	# Foot to Ankle Length
l0 = 0.0705     # Pelvic Interval

t0 = 1.0        # time taken for a single step
z0 = 0.2   		# sitting position height
temp = 0.6*l0   # leg lift 
x0 = 0.2        # sway length (distance coverd in one step of walking)

"""
x0 = (0, 0.2] for forward 
x0 = (0, -0.2] for backward
x0 = 0.0000001 for no walklength (just to turn left-right on its origial postion) 
""" 
motor_force = 5   # maximum torque by motors

#initialiing the environment
physicsClient = p.connect(p.GUI) #connecting to the physics simulation
p.setGravity(0, 0, -9.8)         #setting up gravity
fixedTimeStep = 1.0/800.0        #Timestep 800Hz 
p.setTimeStep(timeStep=fixedTimeStep, physicsClientId=physicsClient)
useRealTimeSimulation = 1
p.setRealTimeSimulation(useRealTimeSimulation)
# p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=10, cameraPitch=-5, cameraTargetPosition=[0, 0, 0.1], physicsClientId=physicsClient)

# loading the groung
p.setAdditionalSearchPath(pybullet_data.getDataPath())  
planeId = p.loadURDF('plane.urdf') 

# spawning the track
trackStartPos = [0, 0, 0.06]
trackStartOrientation = p.getQuaternionFromEuler([np.pi,0,0])
trackId = p.loadURDF("track.urdf",trackStartPos, trackStartOrientation, useFixedBase = 1)

# spawing the biped
cubeStartPos = [0,-0.05,0.5]
cubeStartOrientation = p.getQuaternionFromEuler([np.pi/2,0,np.pi/2])
robot = p.loadURDF("biped_model.urdf",cubeStartPos, cubeStartOrientation, useFixedBase = 0)

"""
#To get info of the robot
print("Join Info: ")
for i in range(p.getNumJoints(robot)):
 	print(p.getJointInfo(robot, i))

print("Link Info: ")
for i in range(p.getNumJoints(robot)+1):
           print(p.getLinkStates(robot, [i]))
"""           

trajectory = trajectoryGenerator.TrajectoryGenerator(l0,l1,l2,l3)  # initializing trajectory generator
trajectory.setTrajectoryParameters(temp,z0,x0,t0)                  # setting up walk/trajectory parameters

controller = motorController.MotorController(robot, physicsClient, motor_force)  # setting up motor controller to control motor positions

# Inverse Kinematics (to be updated...)
def getInverseKinematics(Phip, Pfoot):
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

def walk(d):
	t = 0
	while t<=t0:
		if d == 'sit':
			p1 , p2, p3, p4 = trajectory.getSittingTrajectory(t)
		elif d == 'sta':
			p1 , p2, p3, p4 = trajectory.getStandingTrajectory(t)
		elif d == 'lst':
			p1 , p2, p3, p4 = trajectory.getLeftLegStartingTrajectory(t,temp)
		elif d == 'rst':
			p1 , p2, p3, p4 = trajectory.getRightLegStartingTrajectory(t,temp)
		elif d == 'l':
			turn = 's'
			p1 , p2, p3, p4 = trajectory.getLeftLegWalkingTrajectory(turn,t,temp)
		elif d == 'r':
			turn = 's'
			p1 , p2, p3, p4 = trajectory.getRightLegWalkingTrajectory(turn,t,temp)
		elif d == 'ls':
			p1 , p2, p3, p4 = trajectory.getLeftLegStoppingTrajectory(t,temp)
		elif d == 'rs':
			p1 , p2, p3, p4 = trajectory.getRightLegStoppingTrajectory(t,temp)

		targetMotorPositons = getInverseKinematics(p2, p1) + getInverseKinematics(p3, p4)
		controller.setMotorPositions(targetMotorPositons)
		t+=.00015
		time.sleep(.0001)


# def turn(d ,turn ,angle):
# 	t = 0
# 	while t<=t0:
# 	  	if d == 'r' :
# 	  		p1 , p2, p3, p4 = trajectory.getRightLegWalkingTrajectory(turn,t,temp)
# 	  		targetMotorPositons = getInverseKinematics(p2, p1) + getInverseKinematics(p3, p4)
# 	  		if(turn == "l"):
# 	  			targetMotorPositons[6] = (1-(t-2*t0/10)/(6*t0/10))*angle/2
# 	  			targetMotorPositons[0] = -(1-(t-2*t0/10)/(6*t0/10))*angle/2
# 	  		elif(turn == "r"):
# 	  			targetMotorPositons[6] = ((t-2*t0/10)/(6*t0/10))*angle/2
# 	  			targetMotorPositons[0] = -((t-2*t0/10)/(6*t0/10))*angle/2
# 	  		controller.setMotorPositions(targetMotorPositons)

# 	  	elif turn == 'l':
# 	  		p1 , p2, p3, p4 = trajectory.getLeftLegWalkingTrajectory(turn,t,temp)
# 	  		targetMotorPositons = getInverseKinematics(p2, p1) + getInverseKinematics(p3, p4)
# 	  		if(turn == "l"):
# 	  			targetMotorPositons[6] = ((t-2*t0/10)/(6*t0/10))*angle/2
# 	  			targetMotorPositons[0] = -((t-2*t0/10)/(6*t0/10))*angle/2
# 	  		elif(turn == "r"):
# 	  			targetMotorPositons[6] = (1-(t-2*t0/10)/(6*t0/10))*angle/2
# 	  			targetMotorPositons[0] = -(1-(t-2*t0/10)/(6*t0/10))*angle/2
# 	  		controller.setMotorPositions(targetMotorPositons)

# 	  	t+=.00015
# 	  	time.sleep(.0001)



# Setting Up Motors
def sit_at_height():
	t =0
	while t<=t0/4:
		p1 , p2, p3, p4 = trajectory.getSittingTrajectory(t)

		targetMotorPositons = getInverseKinematics(p2, p1) + getInverseKinematics(p3, p4)
		controller.setMotorPositions(targetMotorPositons)
		t+=.00015
		time.sleep(.0001)

def stand_upright():
	t =0
	while t<=t0/4:
		p1 , p2, p3, p4 = trajectory.getStandingTrajectory(t)

		targetMotorPositons = getInverseKinematics(p2, p1) + getInverseKinematics(p3, p4)
		controller.setMotorPositions(targetMotorPositons)
		t+=.00015
		time.sleep(.0001)

def walk_by_leftLeg(turn, angle):
	t =0
	while t<=t0:
		p1 , p2, p3, p4 = trajectory.getLeftLegWalkingTrajectory(turn,t,temp)

		targetMotorPositons = getInverseKinematics(p2, p1) + getInverseKinematics(p3, p4)
		if(turn == "l"):
			targetMotorPositons[6] = -((t-2*t0/10)/(6*t0/10))*angle/2
			targetMotorPositons[0] = ((t-2*t0/10)/(6*t0/10))*angle/2
		elif(turn == "r"):
			targetMotorPositons[6] = -(1-(t-2*t0/10)/(6*t0/10))*angle/2
			targetMotorPositons[0] = (1-(t-2*t0/10)/(6*t0/10))*angle/2

		controller.setMotorPositions(targetMotorPositons)
		t+=.00015
		time.sleep(.0001)


def walk_by_rightLeg(turn, angle):
	t =0
	while t<=t0:
		p1 , p2, p3, p4 = trajectory.getRightLegWalkingTrajectory(turn,t,temp)

		targetMotorPositons = getInverseKinematics(p2, p1) + getInverseKinematics(p3, p4)

		if(turn == "l"):
			targetMotorPositons[6] = -(1-(t-2*t0/10)/(6*t0/10))*angle/2
			targetMotorPositons[0] = (1-(t-2*t0/10)/(6*t0/10))*angle/2
		elif(turn == "r"):
			targetMotorPositons[6] = -((t-2*t0/10)/(6*t0/10))*angle/2
			targetMotorPositons[0] = ((t-2*t0/10)/(6*t0/10))*angle/2

		controller.setMotorPositions(targetMotorPositons)
		t+=.00015
		time.sleep(.0001)


#################################################################################################################################################################
################################################### What your Robot should do for you, write below! #############################################################
#################################################################################################################################################################

time.sleep(1.5)
sit_at_height()
time.sleep(1.5)


# forward motion
x0 = 0.2
# going stright
walk('lst')

for _ in range(2):
	walk('r')
	walk('l')

walk('rs')


# turning right
walk('lst')

for _ in range(2):
	walk_by_rightLeg(turn = "r", angle = np.pi/8)
	walk_by_leftLeg(turn = "r", angle = np.pi/8)

walk('rs')


# turning left
walk('rst')

for _ in range(2):

	walk_by_leftLeg(turn = "l", angle = np.pi/8)  # ("L" --> left, "R" --> rigth, "S" --> stright) --> when only 5 dof of leg is used
	walk_by_rightLeg(turn = "l", angle = np.pi/8)  # ("l" --> left, "r" --> rigth, "s" --> stright) --> when all 6 dof of leg is used
	
walk('ls')


# going stright
walk('lst')

for _ in range(2):
	walk('r')
	walk('l')

walk('rs')


# turning left
walk('rst')

for _ in range(2):

	walk_by_leftLeg(turn = "l", angle = np.pi/8)  # ("L" --> left, "R" --> rigth, "S" --> stright) --> when only 5 dof of leg is used
	walk_by_rightLeg(turn = "l", angle = np.pi/8)  # ("l" --> left, "r" --> rigth, "s" --> stright) --> when all 6 dof of leg is used
	
walk('ls')

# turning right
walk('lst')

for _ in range(2):
	walk_by_rightLeg(turn = "r", angle = np.pi/8)
	walk_by_leftLeg(turn = "r", angle = np.pi/8)

walk('rs')


# going stright
walk('lst')

for _ in range(2):
	walk_by_rightLeg(turn = "s", angle = 0)
	walk_by_leftLeg(turn = "s", angle = 0)

walk('rs')

stand_upright()

# x0 = -0.2
# trajectory.setTrajectoryParameters(temp,z0,x0,t0)

# # going stright
# walk('lst')

# for _ in range(2):
# 	walk('r')
# 	walk('l')

# walk('rs')


# # turning right
# walk('lst')

# for _ in range(2):
# 	walk_by_rightLeg(turn = "r", angle = np.pi/8)
# 	walk_by_leftLeg(turn = "r", angle = np.pi/8)

# walk('rs')


# # turning left
# walk('rst')

# for _ in range(2):

# 	walk_by_leftLeg(turn = "l", angle = np.pi/8)  # ("L" --> left, "R" --> rigth, "S" --> stright) --> when only 5 dof of leg is used
# 	walk_by_rightLeg(turn = "l", angle = np.pi/8)  # ("l" --> left, "r" --> rigth, "s" --> stright) --> when all 6 dof of leg is used
	
# walk('ls')


# # going stright
# walk('lst')

# for _ in range(2):
# 	walk('r')
# 	walk('l')

# walk('rs')


# # turning left
# walk('rst')

# for _ in range(2):

# 	walk_by_leftLeg(turn = "l", angle = np.pi/8)  # ("L" --> left, "R" --> rigth, "S" --> stright) --> when only 5 dof of leg is used
# 	walk_by_rightLeg(turn = "l", angle = np.pi/8)  # ("l" --> left, "r" --> rigth, "s" --> stright) --> when all 6 dof of leg is used
	
# walk('ls')

# # turning right
# walk('lst')

# for _ in range(2):
# 	walk_by_rightLeg(turn = "r", angle = np.pi/8)
# 	walk_by_leftLeg(turn = "r", angle = np.pi/8)

# walk('rs')


# # going stright
# walk('lst')

# for _ in range(2):
# 	walk_by_rightLeg(turn = "s", angle = 0)
# 	walk_by_leftLeg(turn = "s", angle = 0)

# walk('rs')

# stand_upright()

time.sleep(10)
p.disconnect()


###################################################################### ROBOTICS CLUB, MNNIT ALLAHABAD #########################################################
