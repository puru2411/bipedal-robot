import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import math

"""

xi, yi -> center of mass of robot
x1, y1 -> left foot
x2, y2 -> left hip joint
x3, y3 -> right hip joint
x4, y4 -> right foot

"""

class TrajectoryGenerator():
        def _init_(self):
                self._pelvic_interval = 0.0705    # distance between the two hip joints
                self._legUp_length = 0.11         # hip joint to knee joint
                self._legDown_length = 0.11      # knee joint to foot joint
                self._footJoint_to_bottom = 0.031 # foot joint to bottom

        def setTrajectoryParameters(self, liftHeight, sittingHeight, swayLength, stepTime):
                self._liftHeight = liftHeight         # height of lift of a foot #temp in original
                self._sittingHeight = sittingHeight   # sit height. increase this will make leg more fold. too high or too low makes an error   
                self._swayLength = swayLength         # foot sway length, length by which feet move in one step
                self._stepTime = stepTime             # time taken for one step

        def getSittingTrajectory(self,t):       # t is a parametric constant (here ,time)
                l0 = self._pelvic_interval
                l1 = self._legUp_length
                l2 = self._legDown_length
                l3 = self._footJoint_to_bottom

                xi, yi = 0, 0
                #hip joints
                x2 = xi
                y2 = yi - l0/2
                z2 = (l1+l2+l3) + 3*(z0 - (l1+l2+l3))*((t)**2)/((t0/4)**2) - 2*(z0 - (l1+l2+l3))*((t)**3)/((t0/4)**3)
                x3 = x2
                y3 = y2 + l0
                z3 = z2
                #foot
                x1 = x2
                y1 = y2
                z1 = 0
                x4 = x2
                y4 = y2 + l0
                z4 = 0
                return [x1, y1, z1], [x2, y2, z2], [x3, y3, z3], [x4, y4, z4]

        def getStandingTrajectory(self,t):
                l0 = self._pelvic_interval
                l1 = self._legUp_length
                l2 = self._legDown_length
                l3 = self._footJoint_to_bottom

                xi, yi = 0, 0
                #hip joints
                x2 = xi
                y2 = yi - l0/2
                z2 = z0 - 3*(z0 - (l1+l2+l3))*((t)**2)/((t0/4)**2) + 2*(z0 - (l1+l2+l3))*((t)**3)/((t0/4)**3)
                x3 = x2
                y3 = y2 + l0
                z3 = z2
                #foot
                x1 = x2
                y1 = y2
                z1 = 0
                x4 = x2
                y4 = y2 + l0
                z4 = 0

                return [x1, y1, z1], [x2, y2, z2], [x3, y3, z3], [x4, y4, z4]

        def getLeftLegStartingTrajectory(self,t,height):
                temp = height # 0.6*l0
                xi, yi = 0, 0
                t0 = self._stepTime
                t1 = 4*t0/10
                t2 = t0-t1
                #hip
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
                #foot
                x1 = xi
                y1 = yi-l0/2
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

        def getRightLegStartingTrajectory(self,t,height):
                temp = height # 0.6*l0
                xi, yi = 0, 0
                t0 = self._stepTime
                t1 = 4*t0/10
                t2 = t0-t1
                #hip
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
                #foot
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

        def getLeftLegStoppingTrajectory(self,t,height):
                temp = height # 0.6*l0
                xi, yi = 0, 0
                t0 = self._stepTime
                t1 = 5*t0/10
                t2 = t0-t1
                #hip
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
                #foot
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

        def getRightLegStoppingTrajectory(self,t,height):
                temp = height # 0.6*l0
                xi, yi = 0, 0
                t0 = self._stepTime
                t1 = 5*t0/10
                t2 = t0-t1
                #hip
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
                #foot
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

        def getRightLegWalkingTrajectory(self,turn,t,height):
                temp = height # 0.6*l0
                xi, yi = 0, 0
                if(turn == "R"):
                        temp = height/2
                t0 = self._stepTime
                t1 = 5*t0/10
                t2 = t0-t1
                #hip
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
                #foot
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

        def getLeftLegWalkingTrajectory(self,turn,t,height):
                temp = height # 0.6*l0
                xi, yi = 0, 0
                if(turn == "L"):
                        temp = height/2
                t0 = self._stepTime
                t1 = 5*t0/10
                t2 = t0-t1
                #hip
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
                #foot
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

        def printTrajectories(self):
                pass
                

        



        

        
                        
                
                
                
                
                
                
                
                
        
                
