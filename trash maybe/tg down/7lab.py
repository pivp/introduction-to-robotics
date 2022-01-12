#!/usr/bin/env python3
from ev3dev.ev3 import *
import numpy

motor1 = LargeMotor('outA') # first motor declaration
motor2 = LargeMotor('outB') # second motor declaration
motor3 = LargeMotor('outC') # third motor declaration

a1 = 0.06
a2 = 0.15
a3 = 0.145

#alpha1 = 
#alpha2 = 
#alpha3 = 

d1 = 0.163
#d2 = 
#d3 = 

#a = [a1, a2, a3]
#alpha = [alpha1, alpha2, alpha3]
#d = [d1, d2, d3]
#tetta = [tetta1, tetta2, tetta3]

startCords = [0, 0, 0]
x = 1
y = 1
z = 1
goalCords = [x, y, z]

tetta1 = arctg(goalCords[1] / goalCords[0])
tetta2 = arctg(((goalCords[2] - d1) / sqrt((goalCords[0]**2) + (goalCords[1]**2))) + arccos(((a2**2) + (goalCords[0]**2) + (goalCords[1]**2) + ((goalCords[2] - d1)**2) - a3**2)) / (2 * a2 * sqrt((goalCords[0]**2) + (goalCords[1]**2) + ((goalCords[2] - d1)**2))))
tetta3 = pi - arccos(((a2**2) + (a3**2) - (goalCords[0]**2) - (goalCords[1]**2) - ((goalCords[2] - d1)**2)) / (2 * a2 * sqrt((goalCords[0]**2) + (goalCords[1]**2) + ((goalCords[2] - d1)**2))))

t01 = np.array([cos(tetta1), 0, sin(tetta1), a1 * cos(tetta1)], 
			   [sin(tetta1), 0, -cos(tetta1), a1 * sin(tetta1)],
			   [0, 1, 0, d1],
			   [0, 0, 0, 1])

t12 = np.array([cos(tetta2 + pi/2), 0, -sin(tetta2 + pi/2), a2 * cos(tetta2 + pi/2)], 
			   [sin(tetta2 + pi/2), 0, cos(tetta2 + pi/2), a2 * sin(tetta2 + pi/2)],
			   [0, 0, 1, 0],
			   [0, 0, 0, 1])

t23 = np.array([cos(tetta3), 0, -sin(tetta3), a3 * cos(tetta3)], 
			   [sin(tetta3), 0, cos(tetta3), a3 * sin(tetta3)],
			   [0, 0, 1, 0],
			   [0, 0, 0, 1])

result = np.linalg.inv(np.dot(t01, t02, t03))

def run(startCords, goalCords):
	# create file for output
	dataOutput = open("7LabMeasurements [" + str(round(startCords[0])) + " " + str(round(startCords[1])) + " " + str(round(startCords[2])) + "] -> [" + str(goalCords[0]) + " " + str(goalCords[1]) + " " + str(goalCords[1]) + "].txt","w+")

	motor1.position = 0 # in grad
	motor2.position = 0 # in grad
	motor3.position = 0 # in grad
	angle1 = 0 # angle of rotation of the first motor in radians
	angle2 = 0 # angle of rotation of the second motor in radians
	angle3 = 0 # angle of rotation of the third motor in radians

	prevPosition = [startCords[0], startCords[1]]
