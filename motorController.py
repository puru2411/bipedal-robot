"""
This is used to control the position of motors in the robot
"""

import numpy as np
import time
import pybullet as p


class MotorController:

    def __init__(self, robot, physicsClientId, motor_force):
        self._robot = robot
        self._physicsClientId = physicsClientId
        self._joint_id_list = [1, 17, 2, 18, 3, 19, 4, 20, 5, 21, 6, 22]  # 1,2,3,4,5,6 -> right leg, 17,18,19,20,21,22 -> left leg
        joint_pos_list = [] 
        for i in range(len(self._joint_id_list)):
            joint_pos_list.append(p.getJointState(self._robot, self._joint_id_list[i], physicsClientId=self._physicsClientId)[0])
       
        self._joint_targetPosRight = np.array([0, 0, 0, 0, 0, 0])
        self._joint_targetPosLeft = np.array([0, 0, 0, 0, 0, 0])
        self._joint_currentPos = np.array(joint_pos_list, dtype=np.float)  # values stored from right to left leg
        # self._kp = kp
        # self._kd = kd
        self._motor_force = motor_force

    def getMotorAngles(self):
        for i in range(len(self._joint_id_list)):
            self._joint_currentPos[i] = p.getJointState(self._robot, self._joint_id_list[i], physicsClientId=self._physicsClientId)[0]
        return self._joint_currentPos

        # to set joint positions
    def setMotorPositions(self, targetMotorPositions):  # goes from right leg to left leg
        #arranging motors positions in proper sequence (r,l,r,l,r,l...)
        rightLegMotorPosition = targetMotorPositions[:6]
        leftLegMotorPosition = targetMotorPositions[6:]
        targetMotorPosition = []
        for i in range(len(rightLegMotorPosition)):
            targetMotorPosition.append(rightLegMotorPosition[i])
            targetMotorPosition.append(leftLegMotorPosition[i])
        
        #setting up motor positions
        for i in range(len(self._joint_id_list)):
            p.setJointMotorControl2(
                bodyIndex=self._robot,
                jointIndex=self._joint_id_list[i],
                controlMode=p.POSITION_CONTROL,
                targetPosition=targetMotorPosition[i],
                # positionGain=self._kp,
                # velocityGain=self._kd,
                force=self._motor_force,
                physicsClientId=self._physicsClientId)