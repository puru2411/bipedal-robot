# code by:-
# purushotam kumar agrawal { git ---> PURU2411 }
# Vibhanshu Vaibhav { git ---> VibhanshuV }

import pybullet as p
import time
from time import sleep
import pybullet_data
import numpy as np
import math
import os

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt

###################################################################################################################
# declaring some vlues and initiating the environment

# robot leg length 
l1 = 0.11
l2 = 0.11
l3 = 0.031
l0 = 0.0705

t0 = 1.0
z0 = 0.2   # sitting position height

# distance coverd in one step of walking.
# x0 = (0, 0.2] for forward 
# x0 = (0, -0.2] for backward
# x0 = 0.0000001 for no walklength (just to turn left-right on its origial postion) 
x0 = 0.2   

motor_force = 2   # maximum torque by motors

# # motor setting
# motor_kp = 0.5
# motor_kd = 0.5
# motor_torque = 2
# motor_max_velocity = 10.0

# physics parameter setting
physicsClient = p.connect(p.GUI)

fixedTimeStep = 1.0/800.0
p.setTimeStep(timeStep=fixedTimeStep, physicsClientId=physicsClient)
# numSolverIterations = 200
# p.setPhysicsEngineParameter(numSolverIterations=numSolverIterations)

useRealTimeSimulation = 1
p.setRealTimeSimulation(useRealTimeSimulation)

p.setGravity(0, 0, -9.8)
# p.resetDebugVisualizerCamera(cameraDistance=1, cameraYaw=10, cameraPitch=-5, cameraTargetPosition=[0, 0, 0.1], physicsClientId=physicsClient)

################################################^^^^^^^^^^^^^^^^^^^################################################


###################################################################################################################
# spawning and setting the robot

p.setAdditionalSearchPath(pybullet_data.getDataPath())  # to load ground
planeId = p.loadURDF('plane.urdf')  # or p.loadURDF('samurai.urdf')  # p.loadURDF('plane.urdf')

trackStartPos = [0,0,0.06]
trackStartOrientation = p.getQuaternionFromEuler([np.pi,0,0])
trackId = p.loadURDF("track.urdf",trackStartPos, trackStartOrientation, useFixedBase = 1)

##################### NEW MODEL ##########################

cubeStartPos = [0,0,0.6]
cubeStartOrientation = p.getQuaternionFromEuler([np.pi/2,0,np.pi/2])
robot = p.loadURDF("biped_model.urdf",cubeStartPos, cubeStartOrientation, useFixedBase = 1)
rightLegMotor = [1,4,7,10,13,16]  #from top to bottom
leftLegMotor = [19,22,25,28,31,34] # from top to bottom
for i in range(len(rightLegMotor)):
	p.setJointMotorControl2( bodyIndex=robot, jointIndex=rightLegMotor[i], controlMode=p.POSITION_CONTROL, targetPosition= 0)
	p.setJointMotorControl2( bodyIndex=robot, jointIndex=leftLegMotor[i], controlMode=p.POSITION_CONTROL, targetPosition= 0)

##########################################################


# cubeStartPos1 = [0.5,0,1]

# boxId1 = p.loadURDF("sphere2red.urdf",cubeStartPos1, globalScaling= 0.05)

# robot = p.loadURDF(os.path.abspath(os.path.dirname(__file__)) + '/humanoid_leg_12dof.8.urdf', [0, 0, 0.360],
#                    p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=0, globalScaling= 1 )
#stairs
# stairsStartPos = [-2,-0.08,0]
# stairsStartOrientation = p.getQuaternionFromEuler([0,0,np.pi/2])
# robotId = p.loadURDF("stairs.urdf",stairsStartPos, stairsStartOrientation, useFixedBase = 1, 
#                    # useMaximalCoordinates=1, ## New feature in Pybullet
#                    flags=p.URDF_USE_INERTIA_FROM_FILE)


# rightLegMotor = [1, 2, 3, 4, 5, 6]
# leftLegMotor = [17, 18, 19, 20 ,21, 22]

rightLegMotorPosition = [0, 0, 0, 0, 0, 0]
leftLegMotorPosition = [0, 0, 0, 0, 0, 0]

# stting the motor to zero positon initially
for i in range(len(rightLegMotor)):
	p.setJointMotorControl2( bodyIndex=robot, jointIndex=rightLegMotor[i], controlMode=p.POSITION_CONTROL, targetPosition=rightLegMotorPosition[i])
	p.setJointMotorControl2( bodyIndex=robot, jointIndex=leftLegMotor[i], controlMode=p.POSITION_CONTROL, targetPosition=leftLegMotorPosition[i])

# print("the joint info of robot : ")
# for i in range(p.getNumJoints(robot)):
# 	print(p.getJointInfo(robot, i))

# print("the link info of robot : ")
# for i in range(p.getNumJoints(robot)+1):
#   print(p.getLinkStates(robot, [i]))


################################################^^^^^^^^^^^^^^^^^^^################################################


###################################################################################################################
# the trajectory generator

def sitting(t):
	xi, yi = 0, 0

	x2 = xi
	y2 = yi-l0/2
	z2 = (l1+l2+l3) + 3*(z0 - (l1+l2+l3))*((t)**2)/((t0/4)**2) - 2*(z0 - (l1+l2+l3))*((t)**3)/((t0/4)**3)

	x3 = x2
	y3 = y2+l0
	z3 = z2

	x1 = x2
	y1 = y2
	z1 = 0

	x4 = x2
	y4 = y2+l0
	z4 = 0

	return [x1, y1, z1],  [x2, y2, z2], [x3, y3, z3], [x4, y4, z4]    # 1-> right heal, 2-> right west, 3-> left west, 4-> left heal


temp = 0.6*l0


def standing(t):
	xi, yi = 0, 0

	x2 = xi
	y2 = yi-l0/2
	z2 = z0 - 3*(z0 - (l1+l2+l3))*((t)**2)/((t0/4)**2) + 2*(z0 - (l1+l2+l3))*((t)**3)/((t0/4)**3)
	

	x3 = x2
	y3 = y2+l0
	z3 = z2

	x1 = x2
	y1 = y2
	z1 = 0

	x4 = x2
	y4 = y2+l0
	z4 = 0

	return [x1, y1, z1],  [x2, y2, z2], [x3, y3, z3], [x4, y4, z4] 


def standing_at_height(t,h):
	xi, yi = 0, 0

	x2 = xi
	y2 = yi-l0/2
	z2 = z0 - 3*(z0 - h)*((t)**2)/((t0/4)**2) + 2*(z0 - h)*((t)**3)/((t0/4)**3)
	

	x3 = x2
	y3 = y2+l0
	z3 = z2

	x1 = x2
	y1 = y2
	z1 = 0

	x4 = x2
	y4 = y2+l0
	z4 = 0

	return [x1, y1, z1],  [x2, y2, z2], [x3, y3, z3], [x4, y4, z4]


def leftLegStarting(t, xi, yi):
	# xi, yi = 0, 0
	t1 = 4*t0/10
	t2 = t0-t1
	if(t < t1):
		x2 = xi
		y2 = (yi-l0/2) + 3*(-temp)*((t)**2)/((t1)**2) - 2*(-temp)*((t)**3)/((t1)**3)
	elif(t < t2):
		x2 = xi
		y2 = yi-l0/2 - temp
	else:
		x2 = xi + 3*(x0/4)*((t-t2)**2)/((t0-t2)**2) - 2*(x0/4)*((t-t2)**3)/((t0-t2)**3)
		y2 = (yi - l0/2) - (temp)*np.sqrt(abs(1-((x2-xi)/(x0/4))**2))
	z2 = z0


	x3 = x2
	y3 = y2+l0
	z3 = z2


	x1 = xi
	y1 = yi - l0/2
	z1 = 0


	t3 = 3*t0/10
	t4 = t0-t3
	if(t < t3):
		x4 = xi
	elif(t < t4):
		x4 = xi + 3*(x0/2)*((t-t3)**2)/((t0-2*t3)**2) - 2*(x0/2)*((t-t3)**3)/((t0-2*t3)**3)
	else:
		x4 = xi + x0/2
	y4 = yi+l0/2
	z4 = (.015)*np.sqrt(abs(1-((x4-(xi + x0/4))/(x0/4))**2))


	return [x1, y1, z1],  [x2, y2, z2], [x3, y3, z3], [x4, y4, z4]


def rightLegStarting(t, xi, yi):
	# xi, yi = 0, 0
	t1 = 4*t0/10
	t2 = t0-t1
	if(t < t1):
		x2 = xi
		y2 = ((yi-l0/2) + 3*(temp)*((t)**2)/((t1)**2) - 2*(temp)*((t)**3)/((t1)**3))
	elif(t < t2):
		x2 = xi
		y2 = (yi-l0/2 + temp) 
	else:
		x2 = xi + 3*(x0/4)*((t-t2)**2)/((t0-t2)**2) - 2*(x0/4)*((t-t2)**3)/((t0-t2)**3)
		y2 = ((yi - l0/2) + (temp)*np.sqrt(abs(1-((x2-xi)/(x0/4))**2))) 
	z2 = z0


	x3 = x2
	y3 = y2+l0
	z3 = z2


	x4 = xi
	y4 = yi + l0/2 
	z4 = 0


	t3 = 3*t0/10
	t4 = t0-t3
	if(t < t3):
		x1 = xi
	elif(t < t4):
		x1 = xi + 3*(x0/2)*((t-t3)**2)/((t0-2*t3)**2) - 2*(x0/2)*((t-t3)**3)/((t0-2*t3)**3)
	else:
		x1 = xi + x0/2
	y1 = yi-l0/2 
	z1 = (.015)*np.sqrt(abs(1-((x1-(xi + x0/4))/(x0/4))**2))


	return [x1, y1, z1],  [x2, y2, z2], [x3, y3, z3], [x4, y4, z4]


def leftLegStopping(t, xi, yi):
	# xi, yi = 0, 0
	t1 = 5*t0/10
	t2 = t0-t1
	if(t < t1):
		x2 = xi + 3*(x0/4)*((t)**2)/((t1)**2) - 2*(x0/4)*((t)**3)/((t1)**3)
		y2 = (yi - l0/2) - (temp)*np.sqrt(abs(1-((x2-(xi + x0/4))/(x0/4))**2))
	elif(t < t2):
		x2 = xi + x0/4
		y2 = yi-l0/2-temp
	else:
		x2 = xi + x0/4
		y2 = (yi-l0/2-temp) + 3*(temp)*((t-t2)**2)/((t0-t2)**2) - 2*(temp)*((t-t2)**3)/((t0-t2)**3)

	z2 = z0


	x3 = x2
	y3 = y2+l0
	z3 = z2


	x1 = xi + x0/4
	y1 = yi - l0/2
	z1 = 0

	t3 = 3*t0/10
	t4 = t0-t3
	if(t < t3):
		x4 = xi-x0/4 
	elif(t < t4):
		x4 = xi-x0/4 + 3*(x0/2)*((t-t3)**2)/((t0-2*t3)**2) - 2*(x0/2)*((t-t3)**3)/((t0-2*t3)**3)
	else:
		x4 = xi + x0/4
	y4 = yi+l0/2
	z4 = (0.015)*np.sqrt(abs(1-((x4-xi)/(x0/4))**2))

	return [x1, y1, z1],  [x2, y2, z2], [x3, y3, z3], [x4, y4, z4]


def rightLegStopping(t, xi, yi):
	# xi, yi = 0, 0
	t1 = 5*t0/10
	t2 = t0-t1
	if(t < t1):
		x2 = xi + 3*(x0/4)*((t)**2)/((t1)**2) - 2*(x0/4)*((t)**3)/((t1)**3)
		y2 = (yi-l0/2) + (temp)*np.sqrt(abs(1-((x2-(xi+x0/4))/(x0/4))**2))
	elif(t < t2):
		x2 = xi + x0/4
		y2 = yi-l0/2+temp
	else:
		x2 = xi + x0/4
		y2 = ((yi-l0/2+temp) + 3*(-temp)*((t-t2)**2)/((t0-t2)**2) - 2*(-temp)*((t-t2)**3)/((t0-t2)**3))
	z2 = z0


	x3 = x2
	y3 = y2+l0
	z3 = z2


	x4 = xi + x0/4
	y4 = yi+l0/2 
	z4 = 0

	t3 = 3*t0/10
	t4 = t0-t3
	if(t < t3):
		x1 = xi-x0/4 
	elif(t < t4):
		x1 = xi-x0/4 + 3*(x0/2)*((t-t3)**2)/((t0-2*t3)**2) - 2*(x0/2)*((t-t3)**3)/((t0-2*t3)**3)
	else:
		x1 = xi + x0/4
	y1 = yi - l0/2
	z1 = (0.015)*np.sqrt(abs(1-((x1-(xi))/(x0/4))**2))

	return [x1, y1, z1],  [x2, y2, z2], [x3, y3, z3], [x4, y4, z4]


def rightLegWalking(turn, t, xi, yi):
	# xi, yi = x0/2, 0
	temp = 0.6*l0
	if(turn == "R"):
		temp = 0.3*l0

	t1 = 5*t0/10
	t2 = t0-t1
	if(t < t1):
		x2 = xi + 3*(x0/4)*((t)**2)/((t1)**2) - 2*(x0/4)*((t)**3)/((t1)**3)
	elif(t < t2):
		x2 = xi + x0/4
	else:
		x2 = (xi + x0/4) + 3*(x0/4)*((t-t2)**2)/((t1)**2) - 2*(x0/4)*((t-t2)**3)/((t1)**3)
	y2 = (yi-l0/2) + (temp)*np.sqrt(abs(1-((x2-(xi+x0/4))/(x0/4))**2))
	z2 = z0


	x3 = x2
	y3 = y2+l0
	z3 = z2


	x4 = xi + x0/4
	y4 = yi+l0/2 
	z4 = 0

	t3 = 2*t0/10
	t4 = t0-t3
	if(t < t3):
		x1 = xi-x0/4 
	elif(t < t4):
		x1 = xi-x0/4 + 3*(x0)*((t-t3)**2)/((t0-2*t3)**2) - 2*(x0)*((t-t3)**3)/((t0-2*t3)**3)
	else:
		x1 = xi + 3*x0/4
	y1 = yi - l0/2
	z1 = (0.015)*np.sqrt(abs(1-((x1-(xi+x0/4))/(x0/2))**2))
	if(turn == "R"):
		z1 = 0

	return [x1, y1, z1],  [x2, y2, z2], [x3, y3, z3], [x4, y4, z4]


def leftLegWalking(turn, t, xi, yi):
	# xi, yi = 0, 0
	temp = 0.6*l0
	if(turn == "L"):
		temp = 0.3*l0

	t1 = 5*t0/10
	t2 = t0-t1
	if(t < t1):
		x2 = xi + 3*(x0/4)*((t)**2)/((t1)**2) - 2*(x0/4)*((t)**3)/((t1)**3)
	elif(t < t2):
		x2 = xi + x0/4
	else:
		x2 = (xi + x0/4) + 3*(x0/4)*((t-t2)**2)/((t1)**2) - 2*(x0/4)*((t-t2)**3)/((t1)**3)
	
	y2 = (yi - l0/2) - (temp)*np.sqrt(abs(1-((x2-(xi + x0/4))/(x0/4))**2))
	z2 = z0


	x3 = x2
	y3 = y2+l0
	z3 = z2


	x1 = xi + x0/4
	y1 = yi - l0/2
	z1 = 0

	t3 = 2*t0/10
	t4 = t0-t3
	if(t < t3):
		x4 = xi-x0/4 
	elif(t < t4):
		x4 = xi-x0/4 + 3*(x0)*((t-t3)**2)/((t0-2*t3)**2) - 2*(x0)*((t-t3)**3)/((t0-2*t3)**3)
	else:
		x4 = xi + 3*x0/4
	y4 = yi+l0/2
	z4 = (0.015)*np.sqrt(abs(1-((x4-(xi + x0/4))/(x0/2))**2))
	if(turn == "L"):
		z4 = 0

	return [x1, y1, z1],  [x2, y2, z2], [x3, y3, z3], [x4, y4, z4]



################################################^^^^^^^^^^^^^^^^^^^################################################


###################################################################################################################
# to print the trejectory
# just to visualise the trajectory in matplotlib. no use in simulation

def create_plot():
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('x axis')
    ax.set_ylabel('y axis')
    ax.set_zlabel('z axis')
    ax.set_autoscale_on(False)
    # fig.canvas.draw()
    # plt.show()
    return fig, ax


def update_plot(X11, Y11, Z11, X12, Y12, Z12, X13, Y13, Z13, X14, Y14, Z14, fig, ax):
    ax.cla()
    
    ax.plot_wireframe(X11, Y11, Z11, color = 'b')
    ax.plot_wireframe(X12, Y12, Z12, color = 'r')
    ax.plot_wireframe(X13, Y13, Z13, color = 'r')
    ax.plot_wireframe(X14, Y14, Z14, color = 'b')

    # ax.plot(X11, Y11, Z11, '*-')
    # ax.plot(X12, Y12, Z12, '*-')
    # ax.plot(X13, Y13, Z13, '*-')
    # ax.plot(X14, Y14, Z14, '*-')
    # print('in update ', X1, Y1, Z1)
    plt.draw()
    ax.set_xlabel('x axis')
    ax.set_ylabel('y axis')
    ax.set_zlabel('z axis')
    ax.set_autoscale_on(False)
    fig.canvas.draw()
    fig.canvas.flush_events()
    plt.pause(.00001)
    # ax.cla()
    # ax.plot_wireframe(Z,Y,X,color='r')
    # plt.pause(.5)
    # ax.plot_wireframe(Z,Y,X,color='b')
    # plt.pause(3)
    # plt.show()

# just to visualise the trajectory in matplotlib. no use in simulation
def plot_trajectry():
	t =0
	i=0
	X1 = []; Y1 = []; Z1 = []
	X2 = []; Y2 = []; Z2 = []
	X3 = []; Y3 = []; Z3 = []
	X4 = []; Y4 = []; Z4 = []

	fig, ax = create_plot()
	while(t<=4*t0):
		if(t<=t0):
			p1 , p2, p3, p4 = leftLegStarting(t, 0, 0)
			# print('1 : ',i , ' : ',t , ' : ', p2)
			# print('1 : ',i , ' : ',t , ' : ', p3)
			# print(p3)
			X1.append(p1[0]); Y1.append(p1[1]); Z1.append(p1[2])
			X2.append(p2[0]); Y2.append(p2[1]); Z2.append(p2[2])
			X3.append(p3[0]); Y3.append(p3[1]); Z3.append(p3[2])
			X4.append(p4[0]); Y4.append(p4[1]); Z4.append(p4[2])

			X11 = X1[:]; Y11 = Y1[:]; Z11 = Z1[:]
			X11 = np.reshape(X1, (1, i + 1))
			Y11 = np.reshape(Y1, (1, i + 1))
			Z11 = np.reshape(Z1, (1, i + 1))

			X12 = X2[:]; Y12 = Y2[:]; Z12 = Z2[:]
			X12 = np.reshape(X2, (1, i + 1))
			Y12 = np.reshape(Y2, (1, i + 1))
			Z12 = np.reshape(Z2, (1, i + 1))

			X13 = X3[:]; Y13 = Y3[:]; Z13 = Z3[:]
			X13 = np.reshape(X3, (1, i + 1))
			Y13 = np.reshape(Y3, (1, i + 1))
			Z13 = np.reshape(Z3, (1, i + 1))

			X14 = X4[:]; Y14 = Y4[:]; Z14 = Z4[:]
			X14 = np.reshape(X4, (1, i + 1))
			Y14 = np.reshape(Y4, (1, i + 1))
			Z14 = np.reshape(Z4, (1, i + 1))

			update_plot(X11, Y11, Z11, X12, Y12, Z12, X13, Y13, Z13, X14, Y14, Z14, fig, ax)
		elif(t<= 2*t0):
			p1 , p2, p3, p4 = rightLegWalking(t-t0, 0, 0)
			print('2 : ',i , ' : ',t-t0 , ' : ', p2)
			print('2 : ',i , ' : ',t-t0 , ' : ', p3)
			X1.append(p1[0]); Y1.append(p1[1]); Z1.append(p1[2])
			X2.append(p2[0]); Y2.append(p2[1]); Z2.append(p2[2])
			X3.append(p3[0]); Y3.append(p3[1]); Z3.append(p3[2])
			X4.append(p4[0]); Y4.append(p4[1]); Z4.append(p4[2])

			X11 = X1[:]; Y11 = Y1[:]; Z11 = Z1[:]
			X11 = np.reshape(X1, (1, i + 1))
			Y11 = np.reshape(Y1, (1, i + 1))
			Z11 = np.reshape(Z1, (1, i + 1))

			X12 = X2[:]; Y12 = Y2[:]; Z12 = Z2[:]
			X12 = np.reshape(X2, (1, i + 1))
			Y12 = np.reshape(Y2, (1, i + 1))
			Z12 = np.reshape(Z2, (1, i + 1))

			X13 = X3[:]; Y13 = Y3[:]; Z13 = Z3[:]
			X13 = np.reshape(X3, (1, i + 1))
			Y13 = np.reshape(Y3, (1, i + 1))
			Z13 = np.reshape(Z3, (1, i + 1))

			X14 = X4[:]; Y14 = Y4[:]; Z14 = Z4[:]
			X14 = np.reshape(X4, (1, i + 1))
			Y14 = np.reshape(Y4, (1, i + 1))
			Z14 = np.reshape(Z4, (1, i + 1))

			update_plot(X11, Y11, Z11, X12, Y12, Z12, X13, Y13, Z13, X14, Y14, Z14, fig, ax)

		elif(t<=3*t0):
			p1 , p2, p3, p4 = leftLegWalking(t-2.0*t0, x0, 0)
			# print('3 : ',i , ' : ',t-2.0*t0 , ' : ', p2)
			# print('3 : ',i , ' : ',t-2.0*t0 , ' : ', p3)
			X1.append(p1[0]); Y1.append(p1[1]); Z1.append(p1[2])
			X2.append(p2[0]); Y2.append(p2[1]); Z2.append(p2[2])
			X3.append(p3[0]); Y3.append(p3[1]); Z3.append(p3[2])
			X4.append(p4[0]); Y4.append(p4[1]); Z4.append(p4[2])

			X11 = X1[:]; Y11 = Y1[:]; Z11 = Z1[:]
			X11 = np.reshape(X1, (1, i + 1))
			Y11 = np.reshape(Y1, (1, i + 1))
			Z11 = np.reshape(Z1, (1, i + 1))

			X12 = X2[:]; Y12 = Y2[:]; Z12 = Z2[:]
			X12 = np.reshape(X2, (1, i + 1))
			Y12 = np.reshape(Y2, (1, i + 1))
			Z12 = np.reshape(Z2, (1, i + 1))

			X13 = X3[:]; Y13 = Y3[:]; Z13 = Z3[:]
			X13 = np.reshape(X3, (1, i + 1))
			Y13 = np.reshape(Y3, (1, i + 1))
			Z13 = np.reshape(Z3, (1, i + 1))

			X14 = X4[:]; Y14 = Y4[:]; Z14 = Z4[:]
			X14 = np.reshape(X4, (1, i + 1))
			Y14 = np.reshape(Y4, (1, i + 1))
			Z14 = np.reshape(Z4, (1, i + 1))

			update_plot(X11, Y11, Z11, X12, Y12, Z12, X13, Y13, Z13, X14, Y14, Z14, fig, ax)
		elif(t<= 4*t0):
			p1 , p2, p3, p4 = rightLegWalking(t-3.0*t0, 3.0*x0/2.0, 0)
			# print('4 : ',i , ' : ',t-3.0*t0 , ' : ', p2)
			# print('4 : ',i , ' : ',t-3.0*t0 , ' : ', p3)
			X1.append(p1[0]); Y1.append(p1[1]); Z1.append(p1[2])
			X2.append(p2[0]); Y2.append(p2[1]); Z2.append(p2[2])
			X3.append(p3[0]); Y3.append(p3[1]); Z3.append(p3[2])
			X4.append(p4[0]); Y4.append(p4[1]); Z4.append(p4[2])

			X11 = X1[:]; Y11 = Y1[:]; Z11 = Z1[:]
			X11 = np.reshape(X1, (1, i + 1))
			Y11 = np.reshape(Y1, (1, i + 1))
			Z11 = np.reshape(Z1, (1, i + 1))

			X12 = X2[:]; Y12 = Y2[:]; Z12 = Z2[:]
			X12 = np.reshape(X2, (1, i + 1))
			Y12 = np.reshape(Y2, (1, i + 1))
			Z12 = np.reshape(Z2, (1, i + 1))

			X13 = X3[:]; Y13 = Y3[:]; Z13 = Z3[:]
			X13 = np.reshape(X3, (1, i + 1))
			Y13 = np.reshape(Y3, (1, i + 1))
			Z13 = np.reshape(Z3, (1, i + 1))

			X14 = X4[:]; Y14 = Y4[:]; Z14 = Z4[:]
			X14 = np.reshape(X4, (1, i + 1))
			Y14 = np.reshape(Y4, (1, i + 1))
			Z14 = np.reshape(Z4, (1, i + 1))

			update_plot(X11, Y11, Z11, X12, Y12, Z12, X13, Y13, Z13, X14, Y14, Z14, fig, ax)

		t+= 0.01
		i+=1

	# t = 1
	# p1 , p2, p3, p4 = rightLegWalking(t, 0, 0)
	# print(t-3.0*t0 , ' : ', p2)
	# print(t-3.0*t0 , ' : ', p3)

	plt.show()


################################################^^^^^^^^^^^^^^^^^^^################################################


###################################################################################################################
# inverse kinamatics (updating real soon)

def get_inv_kin_angles(Phip, Pfoot):
	x = Pfoot[0]- Phip[0]
	y = Pfoot[1]- Phip[1]
	z = Phip[2] - Pfoot[2] - l3

	q1 = 0
	q2 = np.arctan(y/z)
	q6 = -q2

	l11 = l1*np.cos(q2)
	l21 = l2*np.cos(q2)
	r = np.sqrt(z**2 + x**2)

	phi1 = np.arctan(x/z)
	phi2 = np.arccos((l11**2 + r**2 - l21**2)/(2*l11*r))
	q3 = phi1+phi2

	phi3 = np.arccos((l11**2 + l21**2 - r**2)/(2*l11*l21))
	q4 = phi3-np.pi

	q5 = -(q4+q3)

	return [q1, q2, -q3, -q4, -q5, q6]


################################################^^^^^^^^^^^^^^^^^^^################################################


###################################################################################################################

def sit_at_height():
	t =0
	while t<=t0/4:
		p1 , p2, p3, p4 = sitting(t)

		rightLegMotorPosition = get_inv_kin_angles(p2, p1)
		leftLegMotorPosition = get_inv_kin_angles(p3, p4)
		# print("p2, p1 : ", p2, p1)
		# print("rightLegMotorPosition : ", rightLegMotorPosition)
		# print("p3, p4 : ", p3, p4)
		# print("leftLegMotorPosition : ", leftLegMotorPosition)
		for i in range(len(rightLegMotor)):
			p.setJointMotorControl2( bodyIndex=robot, jointIndex=rightLegMotor[i], controlMode=p.POSITION_CONTROL, targetPosition=rightLegMotorPosition[i], force=motor_force)
			p.setJointMotorControl2( bodyIndex=robot, jointIndex=leftLegMotor[i], controlMode=p.POSITION_CONTROL, targetPosition=leftLegMotorPosition[i], force=motor_force)

		t+=.0002
		# time.sleep(.0001)


def stand_at_height(h):
	t =0
	while t<=t0/4:
		p1 , p2, p3, p4 = standing(t,h)

		rightLegMotorPosition = get_inv_kin_angles(p2, p1)
		leftLegMotorPosition = get_inv_kin_angles(p3, p4)
		# print("p2, p1 : ", p2, p1)
		# print("rightLegMotorPosition : ", rightLegMotorPosition)
		# print("p3, p4 : ", p3, p4)
		# print("leftLegMotorPosition : ", leftLegMotorPosition)
		for i in range(len(rightLegMotor)):
			p.setJointMotorControl2( bodyIndex=robot, jointIndex=rightLegMotor[i], controlMode=p.POSITION_CONTROL, targetPosition=rightLegMotorPosition[i], force=motor_force)
			p.setJointMotorControl2( bodyIndex=robot, jointIndex=leftLegMotor[i], controlMode=p.POSITION_CONTROL, targetPosition=leftLegMotorPosition[i], force=motor_force)

		t+=.0002
		# time.sleep(.0001)


def stand_upright():
	t =0
	while t<=t0/4:
		p1 , p2, p3, p4 = standing(t)

		rightLegMotorPosition = get_inv_kin_angles(p2, p1)
		leftLegMotorPosition = get_inv_kin_angles(p3, p4)
		# print("p2, p1 : ", p2, p1)
		# print("rightLegMotorPosition : ", rightLegMotorPosition)
		# print("p3, p4 : ", p3, p4)
		# print("leftLegMotorPosition : ", leftLegMotorPosition)
		for i in range(len(rightLegMotor)):
			p.setJointMotorControl2( bodyIndex=robot, jointIndex=rightLegMotor[i], controlMode=p.POSITION_CONTROL, targetPosition=rightLegMotorPosition[i], force=motor_force)
			p.setJointMotorControl2( bodyIndex=robot, jointIndex=leftLegMotor[i], controlMode=p.POSITION_CONTROL, targetPosition=leftLegMotorPosition[i], force=motor_force)

		t+=.0002
		# time.sleep(.0001)


def start_by_leftLeg():
	t =0
	while t<=t0:
		p1 , p2, p3, p4 = leftLegStarting(t, 0, 0)

		rightLegMotorPosition = get_inv_kin_angles(p2, p1)
		leftLegMotorPosition = get_inv_kin_angles(p3, p4)
		# print("p2, p1 : ", p2, p1)
		# print("rightLegMotorPosition : ", rightLegMotorPosition)
		# print("p3, p4 : ", p3, p4)
		# print("leftLegMotorPosition : ", leftLegMotorPosition)
		for i in range(len(rightLegMotor)):
			p.setJointMotorControl2( bodyIndex=robot, jointIndex=rightLegMotor[i], controlMode=p.POSITION_CONTROL, targetPosition=rightLegMotorPosition[i], force=motor_force)
			p.setJointMotorControl2( bodyIndex=robot, jointIndex=leftLegMotor[i], controlMode=p.POSITION_CONTROL, targetPosition=leftLegMotorPosition[i], force=motor_force)

		t+=.0002
		# time.sleep(.0001)


def start_by_rightLeg():
	t =0
	while t<=t0:
		p1 , p2, p3, p4 = rightLegStarting(t, 0, 0)

		rightLegMotorPosition = get_inv_kin_angles(p2, p1)
		leftLegMotorPosition = get_inv_kin_angles(p3, p4)
		# print("p2, p1 : ", p2, p1)
		# print("rightLegMotorPosition : ", rightLegMotorPosition)
		# print("p3, p4 : ", p3, p4)
		# print("leftLegMotorPosition : ", leftLegMotorPosition)
		for i in range(len(rightLegMotor)):
			p.setJointMotorControl2( bodyIndex=robot, jointIndex=rightLegMotor[i], controlMode=p.POSITION_CONTROL, targetPosition=rightLegMotorPosition[i], force=motor_force)
			p.setJointMotorControl2( bodyIndex=robot, jointIndex=leftLegMotor[i], controlMode=p.POSITION_CONTROL, targetPosition=leftLegMotorPosition[i], force=motor_force)

		t+=.0002
		# time.sleep(.0001)


def walk_by_leftLeg(turn, angle):
	t =0
	while t<=t0:
		p1 , p2, p3, p4 = leftLegWalking(turn, t, 0, 0)

		rightLegMotorPosition = get_inv_kin_angles(p2, p1)
		leftLegMotorPosition = get_inv_kin_angles(p3, p4)
		if(turn == "l"):
			leftLegMotorPosition[0] = ((t-2*t0/10)/(6*t0/10))*angle/2
			rightLegMotorPosition[0] = -((t-2*t0/10)/(6*t0/10))*angle/2
		elif(turn == "r"):
			leftLegMotorPosition[0] = (1-(t-2*t0/10)/(6*t0/10))*angle/2
			rightLegMotorPosition[0] = -(1-(t-2*t0/10)/(6*t0/10))*angle/2

		# print("p2, p1 : ", p2, p1)
		# print("rightLegMotorPosition : ", rightLegMotorPosition)
		# print("p3, p4 : ", p3, p4)
		# print("leftLegMotorPosition : ", leftLegMotorPosition)
		for i in range(len(rightLegMotor)):
			p.setJointMotorControl2( bodyIndex=robot, jointIndex=rightLegMotor[i], controlMode=p.POSITION_CONTROL, targetPosition=rightLegMotorPosition[i], force=motor_force)
			p.setJointMotorControl2( bodyIndex=robot, jointIndex=leftLegMotor[i], controlMode=p.POSITION_CONTROL, targetPosition=leftLegMotorPosition[i], force=motor_force)

		t+=.0002
		# time.sleep(.0001)


def walk_by_rightLeg(turn, angle):
	t =0
	while t<=t0:
		p1 , p2, p3, p4 = rightLegWalking(turn, t, 0, 0)

		rightLegMotorPosition = get_inv_kin_angles(p2, p1)
		leftLegMotorPosition = get_inv_kin_angles(p3, p4)
		if(turn == "l"):
			leftLegMotorPosition[0] = (1-(t-2*t0/10)/(6*t0/10))*angle/2
			rightLegMotorPosition[0] = -(1-(t-2*t0/10)/(6*t0/10))*angle/2
		elif(turn == "r"):
			leftLegMotorPosition[0] = ((t-2*t0/10)/(6*t0/10))*angle/2
			rightLegMotorPosition[0] = -((t-2*t0/10)/(6*t0/10))*angle/2

		# print("p2, p1 : ", p2, p1)
		# print("rightLegMotorPosition : ", rightLegMotorPosition)
		# print("p3, p4 : ", p3, p4)
		# print("leftLegMotorPosition : ", leftLegMotorPosition)
		for i in range(len(rightLegMotor)):
			p.setJointMotorControl2( bodyIndex=robot, jointIndex=rightLegMotor[i], controlMode=p.POSITION_CONTROL, targetPosition=rightLegMotorPosition[i], force=motor_force)
			p.setJointMotorControl2( bodyIndex=robot, jointIndex=leftLegMotor[i], controlMode=p.POSITION_CONTROL, targetPosition=leftLegMotorPosition[i], force=motor_force)

		t+=.0002
		# time.sleep(.0001)


def stop_by_leftLeg():
	t =0
	while t<=t0:
		p1 , p2, p3, p4 = leftLegStopping(t, 0, 0)

		rightLegMotorPosition = get_inv_kin_angles(p2, p1)
		leftLegMotorPosition = get_inv_kin_angles(p3, p4)
		# print("p2, p1 : ", p2, p1)
		# print("rightLegMotorPosition : ", rightLegMotorPosition)
		# print("p3, p4 : ", p3, p4)
		# print("leftLegMotorPosition : ", leftLegMotorPosition)
		for i in range(len(rightLegMotor)):
			p.setJointMotorControl2( bodyIndex=robot, jointIndex=rightLegMotor[i], controlMode=p.POSITION_CONTROL, targetPosition=rightLegMotorPosition[i], force=motor_force)
			p.setJointMotorControl2( bodyIndex=robot, jointIndex=leftLegMotor[i], controlMode=p.POSITION_CONTROL, targetPosition=leftLegMotorPosition[i], force=motor_force)

		t+=.0002
		# time.sleep(.0001)


def stop_by_rightLeg():
	t =0
	while t<=t0:
		p1 , p2, p3, p4 = rightLegStopping(t, 0, 0)

		rightLegMotorPosition = get_inv_kin_angles(p2, p1)
		leftLegMotorPosition = get_inv_kin_angles(p3, p4)
		# print("p2, p1 : ", p2, p1)
		# print("rightLegMotorPosition : ", rightLegMotorPosition)
		# print("p3, p4 : ", p3, p4)
		# print("leftLegMotorPosition : ", leftLegMotorPosition)
		for i in range(len(rightLegMotor)):
			p.setJointMotorControl2( bodyIndex=robot, jointIndex=rightLegMotor[i], controlMode=p.POSITION_CONTROL, targetPosition=rightLegMotorPosition[i], force=motor_force)
			p.setJointMotorControl2( bodyIndex=robot, jointIndex=leftLegMotor[i], controlMode=p.POSITION_CONTROL, targetPosition=leftLegMotorPosition[i], force=motor_force)

		t+=.0002
		# time.sleep(.0001)


################################################^^^^^^^^^^^^^^^^^^^################################################


###################################################################################################################
###################################################################################################################
# what your robot should do for you. write bellow


time.sleep(10)
# sit_at_height()
# time.sleep(1.5)


# # forward motion
# x0 = 0.2
# # going stright
# start_by_leftLeg()

# for _ in range(2):
# 	walk_by_rightLeg(turn = "s", angle = 0)
# 	walk_by_leftLeg(turn = "s", angle = 0)

# stop_by_rightLeg()


# # turning right
# start_by_leftLeg()

# for _ in range(2):
# 	walk_by_rightLeg(turn = "r", angle = np.pi/8)
# 	walk_by_leftLeg(turn = "r", angle = np.pi/8)

# stop_by_rightLeg()


# # turning left
# start_by_rightLeg()

# for _ in range(2):

# 	walk_by_leftLeg(turn = "l", angle = np.pi/8)  # ("L" --> left, "R" --> rigth, "S" --> stright) --> when only 5 dof of leg is used
# 	walk_by_rightLeg(turn = "l", angle = np.pi/8)  # ("l" --> left, "r" --> rigth, "s" --> stright) --> when all 6 dof of leg is used
	
# stop_by_leftLeg()


# # going stright
# start_by_leftLeg()

# for _ in range(2):
# 	walk_by_rightLeg(turn = "s", angle = 0)
# 	walk_by_leftLeg(turn = "s", angle = 0)

# stop_by_rightLeg()


# # turning left
# start_by_rightLeg()

# for _ in range(2):

# 	walk_by_leftLeg(turn = "l", angle = np.pi/8)  # ("L" --> left, "R" --> rigth, "S" --> stright) --> when only 5 dof of leg is used
# 	walk_by_rightLeg(turn = "l", angle = np.pi/8)  # ("l" --> left, "r" --> rigth, "s" --> stright) --> when all 6 dof of leg is used
	
# stop_by_leftLeg()

# # turning right
# start_by_leftLeg()

# for _ in range(2):
# 	walk_by_rightLeg(turn = "r", angle = np.pi/8)
# 	walk_by_leftLeg(turn = "r", angle = np.pi/8)

# stop_by_rightLeg()


# # going stright
# start_by_leftLeg()

# for _ in range(2):
# 	walk_by_rightLeg(turn = "s", angle = 0)
# 	walk_by_leftLeg(turn = "s", angle = 0)

# stop_by_rightLeg()


# # backward motion
# x0 = -0.2
# # going stright
# start_by_leftLeg()

# for _ in range(2):
# 	walk_by_rightLeg(turn = "s", angle = 0)
# 	walk_by_leftLeg(turn = "s", angle = 0)

# stop_by_rightLeg()


# # turning right
# start_by_leftLeg()

# for _ in range(2):
# 	walk_by_rightLeg(turn = "r", angle = np.pi/8)
# 	walk_by_leftLeg(turn = "r", angle = np.pi/8)

# stop_by_rightLeg()


# # turning left
# start_by_rightLeg()

# for _ in range(2):

# 	walk_by_leftLeg(turn = "l", angle = np.pi/8)  # ("L" --> left, "R" --> rigth, "S" --> stright) --> when only 5 dof of leg is used
# 	walk_by_rightLeg(turn = "l", angle = np.pi/8)  # ("l" --> left, "r" --> rigth, "s" --> stright) --> when all 6 dof of leg is used
	
# stop_by_leftLeg()


# stand_upright()


rightLegMotorPosition = [0, 0, 0, 0, 0, -np.pi/4]
leftLegMotorPosition = [0, 0, 0, 0, 0, np.pi/4]

# stting the motor to zero positon initially
for i in range(len(rightLegMotor)):
	p.setJointMotorControl2( bodyIndex=robot, jointIndex=rightLegMotor[i], controlMode=p.POSITION_CONTROL, targetPosition=rightLegMotorPosition[i])
	p.setJointMotorControl2( bodyIndex=robot, jointIndex=leftLegMotor[i], controlMode=p.POSITION_CONTROL, targetPosition=leftLegMotorPosition[i])


time.sleep(50)
p.disconnect()



################################################^^^^^^^^^^^^^^^^^^^################################################
################################################^^^^^^^^^^^^^^^^^^^################################################
