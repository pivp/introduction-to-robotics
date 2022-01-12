#!/usr/bin/env python3
from ev3dev.ev3 import *
from math import atan2, sqrt, pi, cos, sin

kS = 51 # proportional gain for distance error
kR = 5 # proportional gain for angle error

r = 0.028 # wheel radius
wheelDist = 0.151 # distance between wheels

battery = PowerSupply() # battery declaration
uMax = battery.measured_volts # max voltage depending on the current battery charge

sideLength = 0.5 # the length of the side of the square along which the robot will move

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
	dataOutput = open("4LabMeasurements [" + str(curPosition[0]) + " " + str(curPosition[1]) + "] -> [" + str(goal[0]) + " " + str(goal[1]) + "].txt","w+")

	motorR.position = 0 # in grad
	motorL.position = 0 # in grad
	angleR = 0 # angle of rotation of the right motor in radians
	angleL = 0 # angle of rotation of the left motor in radians

	prevPosition = [curPosition[0], curPosition[1]]

	distanceToGoal = sqrt(((goal[0] - curPosition[0])**2) + ((goal[1] - curPosition[1])**2))

	while distanceToGoal > 0.05:

		angleRPrev = angleR
		angleLPrev = angleL
		angleR = motorR.position * pi / 180
		angleL = motorL.position * pi / 180
		difAngleR = angleR - angleRPrev
		difAngleL = angleL - angleLPrev

		prevPosition[0] = curPosition[0]
		prevPosition[1] = curPosition[1]

		tettaPrev = tetta

		azimut = atan2(goal[1] - curPosition[1], goal[0] - curPosition[0]) # in radians
		tetta = tettaPrev + (difAngleR - difAngleL) * r / wheelDist # in radians
		curPosition[0] = prevPosition[0] + cos(tetta) * (difAngleR + difAngleL) / 2 * r
		curPosition[1] = prevPosition[1] + sin(tetta) * (difAngleR + difAngleL) / 2 * r

		headingAngle = azimut - tetta
		distanceToGoal = sqrt(((goal[0] - curPosition[0])**2) + ((goal[1] - curPosition[1])**2))

		# shortest corner turn
		if abs(headingAngle) > pi:
			headingAngle = headingAngle - sign(headingAngle) * 2 * pi

		uS = kS * distanceToGoal # voltage for linear speed and its limitation
		if abs(uS / uMax * 100) > 70:
			uS = sign(uS) * uMax * 0.7

		uR = kR * headingAngle # voltage for angular speed and its limitation
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