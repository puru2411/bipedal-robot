import pybullet as p
import time
import pybullet_data
import numpy as np

physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-9.8)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,0.6]
cubeStartOrientation = p.getQuaternionFromEuler([np.pi/2,0,np.pi/2])
robotId = p.loadURDF("COM.urdf",cubeStartPos, cubeStartOrientation, 
                   # useMaximalCoordinates=1, ## New feature in Pybullet
                   flags=p.URDF_USE_INERTIA_FROM_FILE)

#[1,4,7,10,13,16,19,22,25,28,31,34]

j = 0
for i in range(p.getNumJoints(robotId)):
    if (p.getJointInfo(robotId,i)[2] == 0):
        print("Revolute Joint ",i)
        print(" ")
        print(p.getJointInfo(robotId,i))
        j+=1
        
print("Total no. of rev joints: ", j)

#for i in range(i):
    #if (p.getJointInfo(robotId,i)[2] == 0):
        #p.setJointMotorControl2( bodyIndex=robotId, jointIndex=i, controlMode=p.POSITION_CONTROL, targetPosition=0)
        #p.setJointMotorControl2( bodyIndex=robotId, jointIndex=i, controlMode=p.POSITION_CONTROL, targetPosition=0)

for i in range (10000):
    p.stepSimulation()
    #p.setJointMotorControl2( bodyIndex=robotId, jointIndex=34, controlMode=p.POSITION_CONTROL, targetPosition=np.pi/18)
    time.sleep(1./240.)
cubePos, cubeOrn = p.getBasePositionAndOrientation(robotId)
print(cubePos,cubeOrn)
p.disconnect()

