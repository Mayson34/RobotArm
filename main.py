# Coded by Mason Fisher
# Y00743561

# This progaram is my attempt to get a robotic arm to only move within the confines of normal human motion. This is a 2 dimensional side view of a standing person. 
# The arm moves up and down, rotating both along the shoulder and and elbow. Since there is no Z axis, this program would not control the rotation of the shoulder 
# so there is no full rotation of the arm.

from math import cos, acos, sin, atan, atan2, atanh
from math import degrees
import numpy as np
import matplotlib.pyplot as plt
from math import sqrt
from math import pi

# Both classes used for exception handling user inputs for the end effector postion
class Error(Exception):
   "Base class for  exceptions"
   pass

class InvalidValue(Error):
   "Raised when an invalid value is entered"
   pass


class drawArm:

    # Initializes the default values 
    def __init__(self, jointAngles=[0, 0]):
        self.shoulder = np.array([0, 0]) #array used for the shoulder
        self.torso = np.array([(0, 0), (-1, 0)]) #array used to plot the torso so it can be used to calculate the angle of the upper arm 
        self.beamLengths = [2, 2]
        #self.updateJointsFK(jointAngles) # Can be uncommented out if you would like to use forward kinematics to just move the joints around
        self.updateJointsIK(4, 0) # Inverse kinematics use to move the end effector to the desired position
        self.angle1 = 0 # Varibles for angle calculations
        self.angle2 = 0
        self.T1 = 0
        self.T0 = 0



        


    def findAngleShoulder(self):

        # Method used to get the angle of the upper arm using vectors
        vect1 = self.elbow - self.shoulder
        vect2 = self.torso - self.shoulder

        cosAngle = np.dot(vect1, vect2) / np.linalg.norm(vect1) * np.linalg.norm(vect2)


        angle = np.arccos(cosAngle)


        self.angle2 = np.degrees(angle)
        



    def getAngleShoulder(self):

        # Getter mehtod for upper arm angle
        self.findAngleShoulder()
        return self.angle2



    def updateJointsFK(self, jointAngles):

        # Update the position of the joints using forward kinematics
        self.jointAngles = jointAngles
        self.theta0 = self.jointAngles[0]
        self.theta1 = self.jointAngles[1]
        self.forwardKinematics()
        self.checkAngle()

    def updateJointsIK(self, x, y):

        # Update the position of the joints using inverse kinematics
        x1 = x
        y1 = y
        self.inverseKinematics(x1, y1)
        self.checkAngle()

    def checkAngle(self):
        
        # Method used to ensure that the arm moves like a human arm
        length0 = self.beamLengths[0]
        length1 = self.beamLengths[1]
        theta0 = self.T0
        theta1 = self.T1
       

        if self.elbow[0] < 0 and self.elbow[1] > 0:
            
            # This IF statement makes sure that the upper arm can't move baackwards if its pointing up as if it was next to your head
            self.elbow[0] = 0
            self.elbow[1] = 2
           
            # Ensure that the wrist point is updated with the elbow point
            self.wrist = self.elbow + np.array([length1 * cos(theta0 + theta1), length1 * sin(theta0 + theta1)])
            

        elif self.elbow[0] < 0 and self.elbow[1] == 2:
            
            # This ELIF statement makes sure that the upper arm can't move baackwards if its pointing up as if it was next to the head
            self.elbow[0] = 0
            self.elbow[1] = 2
            # Ensure that the wrist point is updated with the elbow point
            self.wrist = self.elbow + np.array([self.length1 * cos(theta0 + theta1), length1 * sin(theta0 + theta1)])
            


        elif any(self.getAngleShoulder()) == 180 and self.wrist[1] <= .1 and self.wrist[0] <= 0:
            
            # This ELIF statement makes sure that the forearm doesn't  move past a particular point after it goes behind the head with a wrist X value of less than or equal to 0
            self.wrist[0] = -.1
            self.wrist[1] =  0
            
        elif any(self.getAngleShoulder() == 180) and self.wrist[1] <= .1 and self.wrist[0] >= 0:
            
            # This ELIF statement makes sure that the forearm doesn't  move past a particular point after it goes behind the head with a wrist X value of greater than or equal to 0
            self.wrist[1] = 0
            self.wrist[0] = -.1

        elif any(self.getAngleShoulder() > 50) and self.elbow[0] < -.5 and self.elbow[1] < 0:
            
            # This ELIF statement makes sure that the upper arm can't move more than 50 degrees away from the torso if its pointing down and behind the body
            while all(self.getAngleShoulder() > 50):
                
                # The while statement moves the upper arm back towards the torso if it tries to go past 50 degrees away from the torso
                self.elbow[0] = self.elbow[0] + .01
                self.elbow[1] = self.elbow[1] - .01

                self.wrist = self.elbow + np.array([length1 * cos(theta0 + theta1), length1 * sin(theta0 + theta1)])
            if self.wrist[1] > -2 and self.wrist[0] < -2:
                while self.wrist[1] > -2:
                    self.wrist[0] = self.wrist[0] + .1
                    self.wrist[1] = self.wrist[1] - .1
                
                
           
                

    def forwardKinematics(self):

        # Method used to calculate forward kinematics
        
        # The theta values are used in the forward kinematics calculations to control how much the arm moves
        theta0 = self.jointAngles[0] 
        theta1 = self.jointAngles[1]
        
        # The lengths are also used in the forward kinematics calculations to find the next position
        length0 = self.beamLengths[0] 
        length1 = self.beamLengths[1]

        # The forward kinematics equation using arrays, trig fucntions and the values of the joints
        self.elbow = self.shoulder + np.array([length0 * cos(theta0), length0 * sin(theta0)])
        self.wrist = self.elbow + np.array([length1 * cos(theta0 + theta1), length1 * sin(theta0 + theta1)])
        self.findAngleShoulder()
        
        self.checkAngle()
        self.findAngleShoulder()
        

    def inverseKinematics(self,x,y):

        # Method used to calculate inverse kinematics using an x and y value taken from user input
        x1 = x
        y1 = y
        length0 = self.beamLengths[0]
        length1 = self.beamLengths[1]

        
        # The theta values are calculated using geometry and trig functions. The forearm position is calculated first becuase the result is needed to find the position of the upper arm 
        

        numerator = ((x1)**2 + (y1)**2 - 2**2 - 2**2)
        denominator = ( 2 * 2 * 2)
        frac = numerator/denominator

        if frac < -1:
            frac = -1
        
        elif frac > 1:
            frac = 1
        
        theta1 = acos(frac)

        # The IF/ELIF block is an extra check used to make sure that the AI doesn't move the forearm to an invalid position
        if theta1 < 0:
            theta1 = 0
        elif theta1 > 3:
            theta1 = 3

        theta0 = atan2(y1, x1) - atan2((length1 * sin(theta1)) , (length0 + length1 * cos(theta1)))

        # Assigning the T1 and T0 variables so the theta values can be used in other methods such as checkAngle
        self.T1 = theta1
        self.T0 = theta0

        
       
        
        # Updating the position and checking to make sure that it is a valid one
        self.elbow = self.shoulder + np.array([length0 * cos(theta0), length0 * sin(theta0)])
        self.wrist = self.elbow + np.array([length1 * cos(theta0 + theta1), length1 * sin(theta0 + theta1)])
        self.findAngleShoulder()
        self.checkAngle()
        self.findAngleShoulder()
        

    def plot(self):

        # Method to plot the arm on to the X,Y plane 

        # Plot the upper arm
        plt.plot([self.shoulder[0], self.elbow[0]],
                 [self.shoulder[1], self.elbow[1]],
                 'r-')
        
        # Plot the forearm
        plt.plot([self.elbow[0], self.wrist[0]],
                 [self.elbow[1], self.wrist[1]],
                 '-')

        # Change the colors of the lines used 
        plt.plot(self.shoulder[0], self.shoulder[1], 'o')
        plt.plot(self.elbow[0], self.elbow[1], 'ko')
        plt.plot(self.wrist[0], self.wrist[1], 'bo')

        # Plot the torso
        plt.plot(self.torso[0], self.torso[1])


# Create the drawArm object to be used to call methods
arm = drawArm()

# Exception handling
while True:
    try:

        
        wristX = int(input("Enter the X value: "))
        
        # Make sure that the input X value is in the range of the coordinate plane
        if wristX < -4 or wristX > 4:
            raise InvalidValue
        
        # Make sure that the input X value is an integer
        elif isinstance(wristX,int) == False:
            raise ValueError
        break

    # Catch custom exception
    except InvalidValue:
        print("Invalid X value, try again.")
    
    # Catch ValueError exception
    except ValueError:
        print("Value needs to be an integer, try again.")

while True:
    try:

        wristY = int(input("Enter the Y value: "))
        
        # Make sure that the input Y value is in the range of the coordinate plane
        if wristY < -4 or wristY > 4:
            raise InvalidValue
        
        # Make sure that the input Y value is an integer
        elif isinstance(wristY,int) == False:
            raise ValueError
        break

     # Catch custom exception
    except InvalidValue:
        print("Invalid Y value, try again.")
    
    # Catch ValueError exception
    except ValueError:
        print("Value needs to be an integer, try again.")
        
'''
Can be uncommented to use forward kinematics to move the arm around with no specific end effector point

theta0 = -1
theta1 = 1 


if theta1 < 0:
    theta1 = 0
if theta1 > 3:
    theta1 = 3

# Calling the forward kinematics update function    
arm.updateJointsFK([theta0, theta1])
'''

# Calling the inverse kinematics update function
arm.updateJointsIK(wristX, wristY)


# Plotting the arm
arm.plot()

# Pointing out the Shoulder, Elbow and Wrist on the arm
plt.annotate("Shoulder", xy=(arm.shoulder[0], arm.shoulder[1]), xytext=(0.15, 0.5),
             arrowprops=dict(facecolor='black', shrink=0.05))
plt.annotate("Elbow", xy=(arm.elbow[0], arm.elbow[1]), xytext=(1.25, 0.25),
             arrowprops=dict(facecolor='black', shrink=0.05))
plt.annotate("Wrist", xy=(arm.wrist[0], arm.wrist[1]), xytext=(1, 1.75),
             arrowprops=dict(facecolor='black', shrink=0.05))
arm.checkAngle()

# Setting the X and Y limits of the coordinate plane
plt.ylim(-4, 4)
plt.xlim(-4, 4)

# Show the final position on the coordinate plane
plt.show()
