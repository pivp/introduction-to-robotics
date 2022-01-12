#!/usr/bin/env python3
from ev3dev.ev3 import *
from math import atan2, sqrt, pi, cos, sin, tanh
import time

wNls = 14.64 # max angular speed reached by the motor
r = 0.028 # wheel radius
# vMax = wNls * r # max linear speed of our robot
vMax = 70 # percentage of the maximum power the maximum when achieved max angular speed
wheelDist = 0.151 # distance between wheels
# wMax = 2 * r * wNls / wheelDist # max angular speed of our robot
wMax = 30 # percentage of the maximum power the maximum when achieved max angular speed

battery = PowerSupply() # battery declaration
uMax = battery.measured_volts # max voltage depending on the current battery charge

sideLength = 0.5 # the length of the side of the square along which the robot will move

# from the equation: |wMax| = kW * (pi / 4) + 0.5 * vMax
# we can find coefficient kW

kW = 7 # coefficient for calculating the required angular velocity

kiS = 3 # integral gain for linear speed
kiR = 4 # integral gain for angular speed

kdS = 7 # differential gain for linear speed
kdR = 9 # differential dain for angular speed

motorR = LargeMotor('outA') # right motor declaration
motorL = LargeMotor('outB') # left motor declaration

# stopButton = TouchSensor("in1") # robot's stop button declaration

def sign(x):
	if x > 0:
		return 1
	elif x < 0:
		return -1
	else:
		return 0

def run(curPosition, goal, tetta):

	# create file for output
	dataOutput = open("5LabPIDMeasurements [" + str(round(curPosition[0])) + " " + str(round(curPosition[1])) + "] -> [" + str(goal[0]) + " " + str(goal[1]) + "].txt","w+")

	motorR.position = 0 # in grad
	motorL.position = 0 # in grad
	angleR = 0 # angle of rotation of the right motor in radians
	angleL = 0 # angle of rotation of the left motor in radians

	prevPosition = [curPosition[0], curPosition[1]]

	distanceToGoal = sqrt(((goal[0] - curPosition[0])**2) + ((goal[1] - curPosition[1])**2))

	azimut = atan2(goal[1] - curPosition[1], goal[0] - curPosition[0]) # in radians
	headingAngle = azimut - tetta

	intSumDistanceToGoal = 0 # integral sum for distance to goal
	intSumHeadingAngle = 0 # integral sum for heading angle

	currentTime = time.time()
	prevTime = time.time()

	while distanceToGoal > 0.01:

		prevDistanceToGoal = distanceToGoal
		prevHeadingAngle = headingAngle

		prevTime = currentTime
		currentTime = time.time()
		dt = currentTime - prevTime

		angleRPrev = angleR
		angleLPrev = angleL
		angleR = motorR.position * pi / 180
		angleL = motorL.position * pi / 180
		difAngleR = angleR - angleRPrev
		difAngleL = angleL - angleLPrev

		prevPosition[0] = curPosition[0]
		prevPosition[1] = curPosition[1]

		tettaPrev = tetta

		tetta = tettaPrev + (difAngleR - difAngleL) * r / wheelDist # in radians

		curPosition[0] = prevPosition[0] + cos(tetta) * (difAngleR + difAngleL) / 2 * r
		curPosition[1] = prevPosition[1] + sin(tetta) * (difAngleR + difAngleL) / 2 * r

		distanceToGoal = sqrt(((goal[0] - curPosition[0])**2) + ((goal[1] - curPosition[1])**2))

		azimut = atan2(goal[1] - curPosition[1], goal[0] - curPosition[0]) # in radians
		headingAngle = azimut - tetta

		# shortest corner turn
		if abs(headingAngle) > pi:
			headingAngle = headingAngle - sign(headingAngle) * 2 * pi

		intSumHeadingAngle += headingAngle * dt
		intSumDistanceToGoal += distanceToGoal * dt

		# linear velocity
		linVel = vMax * tanh(distanceToGoal) * cos(headingAngle)

		# angular velocity
		angVel = kW * headingAngle + vMax * (tanh(distanceToGoal) / distanceToGoal) * sin(headingAngle) * cos(headingAngle)

		# uSI - integral component of voltage for angular speed
		uSI = kiS * intSumDistanceToGoal
		# anti-windup for integral component (limited to 15% of maximum power)
		if abs(uSI / uMax * 100) > 15:
			uSI = sign(uSI) * uMax * 0.15

		# voltage for linear speed and its limitation
		uS = linVel + uSI + kdS * (distanceToGoal - prevDistanceToGoal) / dt
		if abs(uS / uMax * 100) > 70:
			uS = sign(uS) * uMax * 0.7

		# uRI - integral component of voltage for angular speed
		uRI = kiR * intSumHeadingAngle
		# anti-windup for integral component (limited to 15% of maximum power)
		if abs(uRI / uMax * 100) > 15:
			uRI = sign(uRI) * uMax * 0.15

		# voltage for angular speed and its limitation
		uR = angVel + uRI + kdR * (headingAngle - prevHeadingAngle) / dt
		if abs(uR / uMax * 100) > 30:
			uR = sign(uR) * uMax * 0.3

		# conversion of the applied voltage to the right and left motors in percent and its limitation
		if abs(uS + uR) > uMax:
			voltageR = sign(uS + uR) * 100
		else:
			voltageR = (uS + uR) / uMax * 100

		if abs(uS - uR) > uMax:
			voltageL = sign(uS - uR) * 100
		else:
			voltageL = (uS - uR) / uMax * 100

		# power supply to the motors
		motorR.run_direct(duty_cycle_sp = (voltageR))
		motorL.run_direct(duty_cycle_sp = (voltageL))

		# output positions
		dataOutput.write(str(curPosition[0]) + '\t' + str(curPosition[1]) + '\n')

		'''
		# to use the stop button
		if stopButton.is_pressed:
			motorR.run_direct(duty_cycle_sp = 0)
			motorL.run_direct(duty_cycle_sp = 0)
			break
		'''

	dataOutput.close()

	motorR.stop(stop_action = 'brake')
	motorL.stop(stop_action = 'brake')

	return curPosition, tetta

curPosition, tetta = run([0, 0], [sideLength, sideLength], 0)
curPosition, tetta = run(curPosition, [-1 * sideLength, sideLength], tetta)
curPosition, tetta = run(curPosition, [-1 * sideLength, -1 * sideLength], tetta)
curPosition, tetta = run(curPosition, [sideLength, -1 * sideLength], tetta)
curPosition, tetta = run(curPosition, [sideLength, sideLength], tetta)