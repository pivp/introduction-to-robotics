#!/usr/bin/env python3
from math import pi, cos, sin

from ev3dev.ev3 import *
import time

motorA = LargeMotor('outA')
motorA.position = 0

motorB = LargeMotor('outB')
motorB.position = 0

motorC = LargeMotor('outC')
motorC.position = 0

data_1 = open('test_7_1.txt', 'w')
data_2 = open('test_7_2.txt', 'w')
data_3 = open('test_7_3.txt', 'w')
coordinates = open('coordinates.txt', 'w')

kp1 = 1
kp2 = 1
kp3 = 1
ki1 = 0.012
ki2 = 0.015
ki3 = 0.01

k_1 = 0.6
k_2 = 0.21
k_3 = 0.18

timeStart = time.time()

integral_1 = 0
integral_2 = 0
integral_3 = 0

a1 = 1
a2 = 15.6
a3 = 10.2
d1 = 18.9

while time.time() - timeStart < 5:
    theta_1 = k_1 * motorA.position
    theta_2 = k_2 * motorB.position
    theta_3 = k_3 * motorC.position

    x2 = a3 * cos(theta_3)
    y2 = a3 * sin(theta_3)
    z2 = 0

    x1 = cos(theta_2 + pi / 2) * x2 - sin(theta_2 + pi / 2) * y2 + a2 * cos(theta_2 + pi / 2)
    y1 = sin(theta_2 + pi / 2) * x2 + cos(theta_2 + pi / 2) * y2 + a2 * sin(theta_2 + pi / 2)
    z1 = 0

    x0 = cos(theta_1) * x1 + a1 * cos(theta_1)
    y0 = sin(theta_1) * x1 + a1 * sin(theta_1)
    z0 = y1 + d1

    t = time.time()

    error_1 = 90 - theta_1
    error_2 = 90 - theta_2
    error_3 = 90 - theta_3

    integral_1 = integral_1 + error_1
    integral_2 = integral_2 + error_2
    integral_3 = integral_3 + error_3

    U_1 = kp1 * error_1 + ki1 * integral_1
    U_2 = kp2 * error_2 + ki2 * integral_2
    U_3 = kp3 * error_3 + ki3 * integral_3

    if U_1 > 100:
        U_1 = 100
    if U_1 < -100:
        U_1 = -100

    if U_2 > 100:
        U_2 = 100
    if U_2 < -100:
        U_2 = -100

    if U_3 > 100:
        U_3 = 100
    if U_3 < -100:
        U_3 = -100

    theta_last_1 = theta_1
    theta_last_2 = theta_2
    theta_last_3 = theta_3

    motorA.run_direct(duty_cycle_sp=U_1)
    motorB.run_direct(duty_cycle_sp=U_2)
    motorC.run_direct(duty_cycle_sp=U_3)

    data_1.write(str(theta_1) + " " + str(t - timeStart) + "\n")
    data_2.write(str(theta_2) + " " + str(t - timeStart) + "\n")
    data_3.write(str(theta_3) + " " + str(t - timeStart) + "\n")
    coordinates.write(str(x0) + " " + str(y0) + " " + str(z0) + "\n")

motorA.run_direct(duty_cycle_sp=0)
motorA.stop(stop_action='brake')

motorB.run_direct(duty_cycle_sp=0)
motorB.stop(stop_action='brake')

motorC.run_direct(duty_cycle_sp=0)
motorC.stop(stop_action='brake')

data_1.close()
data_2.close()
data_2.close()
coordinates.close()
