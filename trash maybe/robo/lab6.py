#!/usr/bin/env python3
from ev3dev.ev3 import *
from time import time
from math import pi

battery = PowerSupply() # battery declaration
uMax = battery.measured_volts # max voltage depending on the current battery charge

# initialization coefficient

k1 = -109.57289382265577
k2 = -1.9164815400990134
k3 = -14.653675349530525
# initialization motors

motorR = LargeMotor('outA')
motorL = LargeMotor('outB')


motorR.position = 0
motorL.position = 0

angleR = 0
angleL = 0

TimeStart = time()
currentTime = time()

# initialization gyro sensors

Gyrosensor_for_angle = GyroSensor('in2')
Gyrosensor_for_rate = GyroSensor('in3')
startangle = Gyrosensor_for_angle.angle * pi / 180
startrate = Gyrosensor_for_rate.rate * pi / 180
# initialization file

dataOutput = open("data_of_6labs.txt","w+")

# start

while currentTime - TimeStart < 7:

    prevTime = currentTime
    currentTime = time()
    dt = currentTime - prevTime

    angleRPrev = angleR
    angleLPrev = angleL
    angleR = motorR.position * pi / 180
    angleL = motorL.position * pi / 180
    difAngleR = angleR - angleRPrev
    difAngleL = angleL - angleLPrev
    difAngleTetta = (difAngleL/dt + difAngleR/dt)/2

    MeanAngle = Gyrosensor_for_angle.angle * pi / 180 - startangle
    MeanRotationAngle = Gyrosensor_for_rate.rate * pi / 180 - startrate


    ErrDifAngleTetta = -difAngleTetta
    ErrAngle = -MeanAngle
    ErrRotate = - MeanRotationAngle

    u = ErrAngle*k1 + ErrDifAngleTetta*k2 + ErrRotate*k3
    if u >= uMax:
        u = uMax
    if u <= -uMax:
         u = -uMax

    motorR.run_direct(duty_cycle_sp=(int(u/uMax*100)))
    motorL.run_direct(duty_cycle_sp=(int(u/uMax*100)))
    dataOutput.write(str(u/uMax*100) + "\t" + str(MeanAngle) + "\t" + str(MeanRotationAngle) + "\t" + str(difAngleTetta) + "\n")
    print(str(MeanAngle) + "\t" + str(MeanRotationAngle) + "\t" + str(u/uMax*100))

dataOutput.close()
motorR.stop(stop_action='brake')
motorL.stop(stop_action='brake')