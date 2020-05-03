from math import cos, acos, sin, atan, atan2, atanh
from math import degrees
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from math import sqrt
from math import pi


class drawArm:
    def __init__(self, jointAngles=[0, 0]):
        self.shoulder = np.array([0, 0])
        self.torso = np.array([(0, 0), (-1, 0)])
        self.beamLengths = [2, 2]
        #self.updateJointsFK(jointAngles)
        self.updateJointsIK(1, 0)
        self.showAnimation = True
        self.angle1 = 0
        self.angle2 = 0
        self.T1 = 0
        self.T0 = 0




    def findAngleElbow(self):

        vect1 = self.wrist - self.elbow
        vect2 = self.shoulder - self.elbow

        cosAngle = np.dot(vect1, vect2) / np.linalg.norm(vect1) * np.linalg.norm(vect2)



        angle = np.arccos(cosAngle)


        self.angle1 = np.degrees(angle)
        #return np.degrees(angle)


    def findAngleShoulder(self):

        vect1 = self.elbow - self.shoulder
        vect2 = self.torso - self.shoulder

        cosAngle = np.dot(vect1, vect2) / np.linalg.norm(vect1) * np.linalg.norm(vect2)


        angle = np.arccos(cosAngle)


        self.angle2 = np.degrees(angle)
        #return np.degrees(angle)

    def getAngleElbow(self):
        self.findAngleElbow()
        return self.angle1

    def getAngleShoulder(self):
        self.findAngleShoulder()
        return self.angle2



    def updateJointsFK(self, jointAngles):
        self.jointAngles = jointAngles
        self.theta0 = self.jointAngles[0]
        self.theta1 = self.jointAngles[1]
        self.forwardKinematics()
        self.checkAngle()

    def updateJointsIK(self, x, y):
        x1 = x
        y1 = y
        self.inverseKinematics(x1, y1)
        self.checkAngle()

    def checkAngle(self):


        length0 = self.beamLengths[0]
        length1 = self.beamLengths[1]
        print("FIRST: ")
        print (self.getAngleShoulder())

        print (self.findSlope(self.shoulder[0], self.shoulder[1], self.elbow[0], self.elbow[1]))
        print (self.findSlope(self.elbow[0], self.elbow[1], self.wrist[0], self.wrist[1]))

        if self.elbow[0] < 0 and self.elbow[1] > 0:
            self.elbow[0] = 0
            self.elbow[1] = 2
            theta0 = self.T0
            theta1 = self.T1
            self.wrist = self.elbow + np.array([length1 * cos(theta0 + theta1), length1 * sin(theta0 + theta1)])
            print ("IF")

        elif self.elbow[0] < 0 and self.elbow[1] == 1:
            self.elbow[0] = 0
            self.elbow[1] = 1
            self.wrist = self.elbow + np.array([self.length1 * cos(theta0 + theta1), length1 * sin(theta0 + theta1)])
            print ("ELIF 1")


        elif any(self.getAngleShoulder()) == 180 and self.wrist[1] <= .1 and self.wrist[0] <= 0:
            print ("ELIF 2")
            self.wrist[0] = -.1
            self.wrist[1] =  0

        elif any(self.getAngleShoulder() == 180) and self.wrist[1] <= .1 and self.wrist[0] >= 0:
            print ("ELIF 3")
            self.wrist[1] = 0
            self.wrist[0] = -.1

        elif any(self.getAngleShoulder() > 50) and self.elbow[0] < -.5 and self.elbow[1] < 0:
            
            while all(self.getAngleShoulder() > 50):
                print ("ELIF 4 WHILE 1")
                print (self.getAngleShoulder())
                self.elbow[0] = self.elbow[0] + .01
                self.elbow[1] = self.elbow[1] - .01
                theta0 = self.T0
                theta1 = self.T1
               
                
                self.wrist = self.elbow + np.array([length1 * cos(theta0 + theta1), length1 * sin(theta0 + theta1)])
                print (self.angle2)
                print (self.angle1)
            

    def findSlope(self,x1,y1,x2,y2):

        slope = ((y2 - y1) / (x2 - x1))
        return slope

    def update(frame):
        xdata.append(frame)
        ydata.append(np.exp(-frame ** 2))
        ln.set_data(xdata, ydata)
        return ln




    def forwardKinematics(self):

        theta0 = self.jointAngles[0]
        theta1 = self.jointAngles[1]
        length0 = self.beamLengths[0]
        length1 = self.beamLengths[1]
        self.elbow = self.shoulder + np.array([length0 * cos(theta0), length0 * sin(theta0)])
        self.wrist = self.elbow + np.array([length1 * cos(theta0 + theta1), length1 * sin(theta0 + theta1)])
        self.findAngleShoulder()
        self.findAngleElbow()
        self.checkAngle()
        self.findAngleShoulder()
        self.findAngleElbow()

    def inverseKinematics(self,x,y):
        x1 = x
        y1 = y
        length0 = self.beamLengths[0]
        length1 = self.beamLengths[1]

        

        theta1 = acos( ((x1)**2 + (y1)**2 - 2**2 - 2**2) / ( 2 * 2 * 2))
        theta0 = atan2(y1, x1) - atan2((length1 * sin(theta1)) , (length0 + length1 * cos(theta1)))
        self.T1 = theta1
        self.T0 = theta0
        print(((-x1**2) + (y1**2) -2**2 - 2**2))
        print(acos(0))
        print("theta1")
        print(theta1)
        print("theta0")
        print(theta0)
        if theta1 < 0:
            theta1 = 0
        elif theta1 > 3:
            theta1 = 3
        self.elbow = self.shoulder + np.array([length0 * cos(theta0), length0 * sin(theta0)])
        self.wrist = self.elbow + np.array([length1 * cos(theta0 + theta1), length1 * sin(theta0 + theta1)])
        self.findAngleShoulder()
        self.findAngleElbow()
        self.checkAngle()
        self.findAngleShoulder()
        self.findAngleElbow()

    def plot(self):

        plt.plot([self.shoulder[0], self.elbow[0]],
                 [self.shoulder[1], self.elbow[1]],
                 'r-')
        plt.plot([self.elbow[0], self.wrist[0]],
                 [self.elbow[1], self.wrist[1]],
                 '-')
        plt.plot(self.shoulder[0], self.shoulder[1], 'o')
        plt.plot(self.elbow[0], self.elbow[1], 'ko')
        plt.plot(self.wrist[0], self.wrist[1], 'bo')

        plt.plot(self.torso[0], self.torso[1])
        self.findAngleShoulder()
        self.findAngleElbow()

arm = drawArm()
wristX = 4
wristY = 0
'''
theta0 = -1
theta1 = 1 #cannot be less than 0


if theta1 < 0:
    theta1 = 0
if theta1 > 3:
    theta1 = 3
    '''
#arm.updateJointsFK([theta0, theta1])

arm.updateJointsIK(wristX, wristY)



arm.plot()

plt.annotate("Shoulder", xy=(arm.shoulder[0], arm.shoulder[1]), xytext=(0.15, 0.5),
             arrowprops=dict(facecolor='black', shrink=0.05))
plt.annotate("Elbow", xy=(arm.elbow[0], arm.elbow[1]), xytext=(1.25, 0.25),
             arrowprops=dict(facecolor='black', shrink=0.05))
plt.annotate("Wrist", xy=(arm.wrist[0], arm.wrist[1]), xytext=(1, 1.75),
             arrowprops=dict(facecolor='black', shrink=0.05))
arm.checkAngle()

plt.ylim(-4, 4)
plt.xlim(-4, 4)

plt.show()
