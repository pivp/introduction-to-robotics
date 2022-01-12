import numpy as np
from math import sin, cos, acos, asin, atan, sqrt, pi

a1 = 0.06
a2 = 0.15
a3 = 0.145
d1 = 0.163
curTetta = [1, 0.5, pi]

t01 = np.array([[cos(curTetta[0]), 0, sin(curTetta[0]), a1 * cos(curTetta[0])], 
	[sin(curTetta[0]), 0, -cos(curTetta[0]), a1 * sin(curTetta[0])], 
	[0, 1, 0, d1], 
	[0, 0, 0, 1]])

t12 = np.array([[cos(curTetta[1] + pi/2), -sin(curTetta[1] + pi/2), 0, a2 * cos(curTetta[1] + pi/2)], 
			   [sin(curTetta[1] + pi/2), cos(curTetta[1] + pi/2), 0, a2 * sin(curTetta[1] + pi/2)],
			   [0, 0, 1, 0],
			   [0, 0, 0, 1]])

t23 = np.array([[cos(curTetta[2]), -sin(curTetta[2]), 0, a3 * cos(curTetta[2])], 
			   [sin(curTetta[2]), cos(curTetta[2]), 0, a3 * sin(curTetta[2])],
			   [0, 0, 1, 0],
			   [0, 0, 0, 1]])

result = np.dot(t01, t12, t23)
result = np.linalg.det(result)
curTetta = np.dot(result, curTetta)

print(result)