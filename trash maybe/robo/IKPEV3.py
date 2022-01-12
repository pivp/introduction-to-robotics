#!/usr/bin/env python3
from math import pi, sin, cos, acos, asin, atan2, atan, sqrt
import time
from ev3dev.ev3 import *

kp = [10, 10, 10] # proportional gain
ki = [1, 1, 1] # integral gain
kd = [10, 10, 10] # differential gain
geerRatio = [1, 2.3, 2.3] # geer ratio

uMax = 7 # max voltage

motor1 = LargeMotor('outA') # first motor declaration
motor2 = LargeMotor('outB') # second motor declaration
motor3 = LargeMotor('outC') # third motor declaration

# system DH-parameters
a = [0.01, 0.17, 0.095]
alpha = [pi/2, 0, 0]
d = [0.165, 0, 0]
tetta = [0, pi/2 - 0, -1 * 0]

def sign(x):
	if x > 0:
		return 1
	elif x < 0:
		return -1
	else:
		return 0

def getGoalTettaByGoalCords(goalCords):

	goalTetta = [0, 0, 0]

	goalTetta[0] = atan2(goalCords[1], goalCords[0])

	goalCords[0] = goalCords[0] + a[0] * cos(goalTetta[0])
	goalCords[1] = goalCords[1] + a[0] * sin(goalTetta[0])

	aaa = atan2((goalCords[2] - d[0]), sqrt((goalCords[0]**2) + (goalCords[1]**2)))
	bbb = a[1]**2 + goalCords[0]**2 + goalCords[1]**2 + (goalCords[2] - d[0])**2 - a[2]**2
	ccc = 2 * a[1] * sqrt(goalCords[0]**2 + goalCords[1]**2 + (goalCords[2] - d[0])**2)

	if bbb > ccc:
		ddd = 1
	else:
		ddd = bbb / ccc

	goalTetta[1] = - (pi/2 - (aaa + acos(ddd)))

	goalTetta[2] = pi - acos((a[1]**2 + a[2]**2 - goalCords[0]**2 - goalCords[1]**2 - (goalCords[2] - d[0])**2) / (2 * a[1] * a[2]))

	return goalTetta

def run(curCords, goalCords, curTetta):

	# create file for output
	dataOutput = open("7LabMeasurementsIKP [" + str(round(curCords[0])) + " " + str(round(curCords[1])) + " " + str(round(curCords[2])) + "] -> [" + str(goalCords[0]) + " " + str(goalCords[1]) + " " + str(goalCords[2]) + "].txt","w+")

	goalTetta = getGoalTettaByGoalCords(goalCords)

	motor1.position = 0 # in grad
	motor2.position = 0 # in grad
	motor3.position = 0 # in grad
	angle1 = 0 # angle of rotation of the first motor in radians
	angle2 = 0 # angle of rotation of the second motor in radians
	angle3 = 0 # angle of rotation of the third motor in radians

	prevTetta = [0, 0, 0]
	curAngleError = [0, 0, 0]
	prevAngleError = [0, 0, 0]
	integralSum = [0, 0, 0]
	uIntegral = [0, 0, 0] # uIntegral - integral component of voltage

	totalAngleError = 0
	for i in range(3):
		curAngleError[i] = goalTetta[i] - curTetta[i]
		totalAngleError += abs(curAngleError[i])

	currentTime = time.time()
	prevTime = time.time()

	# if total angle error less than 10 degrees, then we consider that the manipulator has achieved his goal
	while (totalAngleError > 0.174):

		prevTime = currentTime
		currentTime = time.time()
		dt = currentTime - prevTime

		for i in range(3):
			prevAngleError[i] = curAngleError[i]

		angle1 = geerRatio[0] * motor1.position * pi / 180
		angle2 = geerRatio[1] * motor2.position * pi / 180
		angle3 = geerRatio[2] * motor3.position * pi / 180

		prevTetta[0] = curTetta[0]
		prevTetta[1] = curTetta[1]
		prevTetta[2] = curTetta[2]

		curTetta[0] = prevTetta[0] + angle1
		curTetta[1] = prevTetta[1] + angle2
		curTetta[2] = prevTetta[2] + angle3

		for i in range(3):
			curAngleError[i] = goalTetta[i] - curTetta[i]

			integralSum[i] += curAngleError[i] * dt
			uIntegral[i] = ki[i] * integralSum[i]

			# anti-windup for integral component (limited to 15% of maximum power)
			if abs(uIntegral[i] / uMax * 100) > 15:
				uIntegral[i] = sign(uIntegral[i]) * uMax * 0.15

		u1 = kp[0] * curAngleError[0] + uIntegral[0] + kd[0] * (curAngleError[0] - prevAngleError[0]) / dt
		u2 = kp[1] * curAngleError[1] + uIntegral[1] + kd[1] * (curAngleError[1] - prevAngleError[1]) / dt
		u3 = kp[2] * curAngleError[2] + uIntegral[2] + kd[2] * (curAngleError[2] - prevAngleError[2]) / dt

		if abs(u1) > uMax:
			voltage1 = sign(u1) * 100
		else:
			voltage1 = u1 / uMax * 100

		if abs(u2) > uMax:
			voltage2 = sign(u2) * 100
		else:
			voltage2 = u2 / uMax * 100

		if abs(u3) > uMax:
			voltage3 = sign(u3) * 100
		else:
			voltage3 = u3 / uMax * 100

		# power supply to the motors
		motor1.run_direct(duty_cycle_sp = (voltage1))
		motor2.run_direct(duty_cycle_sp = (voltage2))
		motor3.run_direct(duty_cycle_sp = (voltage3))

		# output positions
		dataOutput.write(str(curTetta[0]) + '\t' + str(curTetta[1]) + '\t' + str(curTetta[2]) + '\n')

		totalAngleError = 0
		for i in range(3):
			totalAngleError += abs(goalTetta[i] - curTetta[i])

	dataOutput.close()

	motor1.stop(stop_action = 'brake')
	motor2.stop(stop_action = 'brake')
	motor3.stop(stop_action = 'brake')

	return goalCords, curTetta

run([-0.01, 0, 0.43], [0.255, 0, 0.165], [0, 0, 0])