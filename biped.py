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

import walkGenerator


# Robot Body Parameters 
l1 = 0.11		# Hip to Knee Length
l2 = 0.11		# Knee to Ankle Length
l3 = 0.031  	# Foot to Ankle Length
l0 = 0.0705     # Pelvic Interval

t0 = 1.0        # time taken for a single step
sittinghip_shift = 0.2   		# sitting position height
hip_shift = 0.6*l0   # leg lift 
swayLength = 0.2        # sway length (distance coverd in one step of walking)

"""
swayLength = (0, 0.2] for forward 
swayLength = (0, -0.2] for backward
swayLength = 0.0000001 for no walklength (just to turn left-right on its origial postion) 
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
cubeStartPos = [0,-0.05,0.55]
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

walk_gen = walkGenerator.WalkGenerator(l0, l1, l2, l3, hip_shift, sittinghip_shift, swayLength, t0, 
											robot, physicsClient, motor_force)


#################################################################################################################################################################
################################################### What your Robot should do for you, write below! #############################################################
#################################################################################################################################################################




# time.sleep(1.5)
# walk_gen.sit_stand_motion(d = "sit")
# time.sleep(1.5)


# # forward motion
# swayLength = 0.2
# print(p.getLinkStates(robot, [0])[0][0])
# print(p.getEulerFromQuaternion(p.getLinkStates(robot, [0])[0][1]))

# # going stright
# walk_gen.start_stop_motions(d ='lst', swayLength = swayLength)
# print(p.getLinkStates(robot, [0])[0][0])
# print(p.getEulerFromQuaternion(p.getLinkStates(robot, [0])[0][1]))

# for _ in range(2):
# 	# walk_by_rightLeg(turn = "r", angle = np.pi/8)
# 	walk_gen.walk(d = "r", swayLength = swayLength, turn = "r", angle = np.pi/4 *3/4)
# 	# walk_by_leftLeg(turn = "r", angle = np.pi/8)
# 	walk_gen.walk(d = "l", swayLength = swayLength, turn = "r", angle = np.pi/4 *3/4)
# 	print(p.getLinkStates(robot, [0])[0][0])
# 	print(p.getEulerFromQuaternion(p.getLinkStates(robot, [0])[0][1]))
	


# walk_gen.start_stop_motions(d='rs', swayLength = swayLength)
# print(p.getLinkStates(robot, [0])[0][0])
# print(p.getEulerFromQuaternion(p.getLinkStates(robot, [0])[0][1]))


# time.sleep(10)
# p.disconnect()




time.sleep(1.5)
walk_gen.sit_stand_motion(d = "sit")
time.sleep(1.5)


# forward motion
swayLength = 0.2

# going stright
walk_gen.start_stop_motions(d ='lst', swayLength = swayLength)

for _ in range(2):
	walk_gen.walk(d = "r", swayLength = swayLength, turn = "s", angle = 0)
	walk_gen.walk(d = "l", swayLength = swayLength, turn = "s", angle = 0)


walk_gen.start_stop_motions(d='rs', swayLength = swayLength)


# turning right
walk_gen.start_stop_motions(d='lst', swayLength = swayLength)

for _ in range(2):
	walk_gen.walk(d = "r", swayLength = swayLength, turn = "r", angle = np.pi/8)
	walk_gen.walk(d = "l", swayLength = swayLength, turn = "r", angle = np.pi/8)

walk_gen.start_stop_motions(d = 'rs', swayLength = swayLength)


# turning left
walk_gen.start_stop_motions(d = 'rst', swayLength = swayLength)

for _ in range(2):

	walk_gen.walk(d = "l", swayLength = swayLength, turn = "l", angle = np.pi/8)  # ("L" --> left, "R" --> rigth, "S" --> stright) --> when only 5 dof of leg is used
	walk_gen.walk(d = "r", swayLength = swayLength, turn = "l", angle = np.pi/8)  # ("l" --> left, "r" --> rigth, "s" --> stright) --> when all 6 dof of leg is used
	
walk_gen.start_stop_motions(d = 'ls', swayLength = swayLength)

ang = np.pi/2-p.getEulerFromQuaternion(p.getLinkStates(robot, [0])[0][1])[2]
# print(p.getEulerFromQuaternion(p.getLinkStates(robot, [0])[0][1])[2], p.getEulerFromQuaternion(p.getLinkStates(robot, [0])[0][1]))
if (ang > 0.01):
	print("turingn left : ", ang)
	walk_gen.walk(d = "l", swayLength = 0.000001, turn = "l", angle = abs(ang)*3/4)  # ("L" --> left, "R" --> rigth, "S" --> stright) --> when only 5 dof of leg is used
	walk_gen.walk(d = "r", swayLength = 0.000001, turn = "l", angle = abs(ang)*3/4)  # ("l" --> left, "r" --> rigth, "s" --> stright) --> when all 6 dof of leg is used
elif(ang < -0.01):
	print("turingn right : ", ang)
	walk_gen.walk(d = "r", swayLength = 0.000001, turn = "r", angle = abs(ang)*3/4)
	walk_gen.walk(d = "l", swayLength = 0.000001, turn = "r", angle = abs(ang)*3/4)


# going stright
walk_gen.start_stop_motions(d = 'lst', swayLength = swayLength)

for _ in range(2):
	walk_gen.walk(d = "r", swayLength = swayLength, turn = "s", angle = 0)
	walk_gen.walk(d = "l", swayLength = swayLength, turn = "s", angle = 0)

walk_gen.start_stop_motions(d = 'rs', swayLength = swayLength)


# turning left
walk_gen.start_stop_motions(d = 'rst', swayLength = swayLength)

for _ in range(2):

	walk_gen.walk(d = "l", swayLength = swayLength, turn = "l", angle = np.pi/8)  # ("L" --> left, "R" --> rigth, "S" --> stright) --> when only 5 dof of leg is used
	walk_gen.walk(d = "r", swayLength = swayLength, turn = "l", angle = np.pi/8)  # ("l" --> left, "r" --> rigth, "s" --> stright) --> when all 6 dof of leg is used
	
walk_gen.start_stop_motions(d = 'ls', swayLength = swayLength)

# turning right
walk_gen.start_stop_motions(d = 'lst', swayLength = swayLength)

for _ in range(2):
	walk_gen.walk(d = "r", swayLength = swayLength, turn = "r", angle = np.pi/8)  # ("L" --> left, "R" --> rigth, "S" --> stright) --> when only 5 dof of leg is used
	walk_gen.walk(d = "l", swayLength = swayLength, turn = "r", angle = np.pi/8)  # ("l" --> left, "r" --> rigth, "s" --> stright) --> when all 6 dof of leg is used
	
walk_gen.start_stop_motions(d = 'rs', swayLength = swayLength)

ang = np.pi/2-p.getEulerFromQuaternion(p.getLinkStates(robot, [0])[0][1])[2]
if (ang > 0.01):
	print("turingn left : ", ang)
	walk_gen.walk(d = "l", swayLength = 0.000001, turn = "l", angle = abs(ang)*3/4)  # ("L" --> left, "R" --> rigth, "S" --> stright) --> when only 5 dof of leg is used
	walk_gen.walk(d = "r", swayLength = 0.000001, turn = "l", angle = abs(ang)*3/4)  # ("l" --> left, "r" --> rigth, "s" --> stright) --> when all 6 dof of leg is used
elif(ang < -0.01):
	print("turingn right : ", ang)
	walk_gen.walk(d = "r", swayLength = 0.000001, turn = "r", angle = abs(ang)*3/4)
	walk_gen.walk(d = "l", swayLength = 0.000001, turn = "r", angle = abs(ang)*3/4)


# going stright
walk_gen.start_stop_motions(d = 'lst', swayLength = swayLength)

for _ in range(2):
	walk_gen.walk(d = "r", swayLength = swayLength, turn = "s", angle = 0)
	walk_gen.walk(d = "l", swayLength = swayLength, turn = "s", angle = 0)

walk_gen.start_stop_motions(d = 'rs', swayLength = swayLength)




for _ in (range(3)):
	walk_gen.walk(d = "l", swayLength = 0.000001, turn = "l", angle = np.pi/6*3/4)  # ("L" --> left, "R" --> rigth, "S" --> stright) --> when only 5 dof of leg is used
	walk_gen.walk(d = "r", swayLength = 0.000001, turn = "l", angle = np.pi/6*3/4)  # ("l" --> left, "r" --> rigth, "s" --> stright) --> when all 6 dof of leg is used

dis = 0.000 - p.getLinkStates(robot, [0])[0][0][1]
# swayLength = (2.0/3.0)*dis
if(abs(dis) >0.05 ):
	swayLength = dis/1.50
	print(swayLength)
	# going stright
	walk_gen.start_stop_motions(d ='lst', swayLength = swayLength)
	for _ in range(1):
		walk_gen.walk(d = "r", swayLength = swayLength, turn = "s", angle = 0)
		walk_gen.walk(d = "l", swayLength = swayLength, turn = "s", angle = 0)
	walk_gen.start_stop_motions(d='rs', swayLength = swayLength)

for _ in (range(3)):
	walk_gen.walk(d = "l", swayLength = 0.000001, turn = "l", angle = np.pi/6*3/4)  # ("L" --> left, "R" --> rigth, "S" --> stright) --> when only 5 dof of leg is used
	walk_gen.walk(d = "r", swayLength = 0.000001, turn = "l", angle = np.pi/6*3/4)  # ("l" --> left, "r" --> rigth, "s" --> stright) --> when all 6 dof of leg is used

ang = -np.pi/2-p.getEulerFromQuaternion(p.getLinkStates(robot, [0])[0][1])[2]
print(p.getEulerFromQuaternion(p.getLinkStates(robot, [0])[0][1])[2], p.getEulerFromQuaternion(p.getLinkStates(robot, [0])[0][1]))

if (ang > 0.01):
	print("turingn left : ", ang)
	walk_gen.walk(d = "l", swayLength = 0.000001, turn = "l", angle = abs(ang)*3/4)  # ("L" --> left, "R" --> rigth, "S" --> stright) --> when only 5 dof of leg is used
	walk_gen.walk(d = "r", swayLength = 0.000001, turn = "l", angle = abs(ang)*3/4)  # ("l" --> left, "r" --> rigth, "s" --> stright) --> when all 6 dof of leg is used
elif(ang < -0.01):
	print("turingn right : ", ang)
	walk_gen.walk(d = "r", swayLength = 0.000001, turn = "r", angle = abs(ang)*3/4)
	walk_gen.walk(d = "l", swayLength = 0.000001, turn = "r", angle = abs(ang)*3/4)





# forward motion
swayLength = 0.2

# going stright
walk_gen.start_stop_motions(d ='lst', swayLength = swayLength)

for _ in range(2):
	walk_gen.walk(d = "r", swayLength = swayLength, turn = "s", angle = 0)
	walk_gen.walk(d = "l", swayLength = swayLength, turn = "s", angle = 0)


walk_gen.start_stop_motions(d='rs', swayLength = swayLength)


# turning right
walk_gen.start_stop_motions(d='lst', swayLength = swayLength)

for _ in range(2):
	walk_gen.walk(d = "r", swayLength = swayLength, turn = "r", angle = np.pi/8)
	walk_gen.walk(d = "l", swayLength = swayLength, turn = "r", angle = np.pi/8)

walk_gen.start_stop_motions(d = 'rs', swayLength = swayLength)


# turning left
walk_gen.start_stop_motions(d = 'rst', swayLength = swayLength)

for _ in range(2):

	walk_gen.walk(d = "l", swayLength = swayLength, turn = "l", angle = np.pi/8)  # ("L" --> left, "R" --> rigth, "S" --> stright) --> when only 5 dof of leg is used
	walk_gen.walk(d = "r", swayLength = swayLength, turn = "l", angle = np.pi/8)  # ("l" --> left, "r" --> rigth, "s" --> stright) --> when all 6 dof of leg is used
	
walk_gen.start_stop_motions(d = 'ls', swayLength = swayLength)

ang = -np.pi/2-p.getEulerFromQuaternion(p.getLinkStates(robot, [0])[0][1])[2]
# print(p.getEulerFromQuaternion(p.getLinkStates(robot, [0])[0][1])[2], p.getEulerFromQuaternion(p.getLinkStates(robot, [0])[0][1]))
if (ang > 0.01):
	print("turingn left : ", ang)
	walk_gen.walk(d = "l", swayLength = 0.000001, turn = "l", angle = abs(ang)*3/4)  # ("L" --> left, "R" --> rigth, "S" --> stright) --> when only 5 dof of leg is used
	walk_gen.walk(d = "r", swayLength = 0.000001, turn = "l", angle = abs(ang)*3/4)  # ("l" --> left, "r" --> rigth, "s" --> stright) --> when all 6 dof of leg is used
elif(ang < -0.01):
	print("turingn right : ", ang)
	walk_gen.walk(d = "r", swayLength = 0.000001, turn = "r", angle = abs(ang)*3/4)
	walk_gen.walk(d = "l", swayLength = 0.000001, turn = "r", angle = abs(ang)*3/4)


# going stright
walk_gen.start_stop_motions(d = 'lst', swayLength = swayLength)

for _ in range(2):
	# walk_by_rightLeg(turn = "r", angle = np.pi/8)
	walk_gen.walk(d = "r", swayLength = swayLength, turn = "s", angle = 0)
	# walk_by_leftLeg(turn = "r", angle = np.pi/8)
	walk_gen.walk(d = "l", swayLength = swayLength, turn = "s", angle = 0)

walk_gen.start_stop_motions(d = 'rs', swayLength = swayLength)


# turning left
walk_gen.start_stop_motions(d = 'rst', swayLength = swayLength)

for _ in range(2):

	walk_gen.walk(d = "l", swayLength = swayLength, turn = "l", angle = np.pi/8)  # ("L" --> left, "R" --> rigth, "S" --> stright) --> when only 5 dof of leg is used
	walk_gen.walk(d = "r", swayLength = swayLength, turn = "l", angle = np.pi/8)  # ("l" --> left, "r" --> rigth, "s" --> stright) --> when all 6 dof of leg is used
	
walk_gen.start_stop_motions(d = 'ls', swayLength = swayLength)

# turning right
walk_gen.start_stop_motions(d = 'lst', swayLength = swayLength)

for _ in range(2):
	walk_gen.walk(d = "r", swayLength = swayLength, turn = "r", angle = np.pi/8)  # ("L" --> left, "R" --> rigth, "S" --> stright) --> when only 5 dof of leg is used
	walk_gen.walk(d = "l", swayLength = swayLength, turn = "r", angle = np.pi/8)  # ("l" --> left, "r" --> rigth, "s" --> stright) --> when all 6 dof of leg is used
	
walk_gen.start_stop_motions(d = 'rs', swayLength = swayLength)

ang = -np.pi/2-p.getEulerFromQuaternion(p.getLinkStates(robot, [0])[0][1])[2]
if (ang > 0.01):
	print("turingn left : ", ang)
	walk_gen.walk(d = "l", swayLength = 0.000001, turn = "l", angle = abs(ang)*3/4)  # ("L" --> left, "R" --> rigth, "S" --> stright) --> when only 5 dof of leg is used
	walk_gen.walk(d = "r", swayLength = 0.000001, turn = "l", angle = abs(ang)*3/4)  # ("l" --> left, "r" --> rigth, "s" --> stright) --> when all 6 dof of leg is used
elif(ang < -0.01):
	print("turingn right : ", ang)
	walk_gen.walk(d = "r", swayLength = 0.000001, turn = "r", angle = abs(ang)*3/4)
	walk_gen.walk(d = "l", swayLength = 0.000001, turn = "r", angle = abs(ang)*3/4)


# going stright
walk_gen.start_stop_motions(d = 'lst', swayLength = swayLength)

for _ in range(2):
	walk_gen.walk(d = "r", swayLength = swayLength, turn = "s", angle = 0)
	walk_gen.walk(d = "l", swayLength = swayLength, turn = "s", angle = 0)

walk_gen.start_stop_motions(d = 'rs', swayLength = swayLength)



walk_gen.sit_stand_motion(d = "sta")

time.sleep(10)
p.disconnect()


###################################################################### ROBOTICS CLUB, MNNIT ALLAHABAD #########################################################
