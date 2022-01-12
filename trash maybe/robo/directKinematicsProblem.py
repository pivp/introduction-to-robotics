import numpy as np
from math import sin, cos, pi

"""
	when tetta[0] = 0 the axis x0 is codirectional with projections of 2 and 3 edges on this axis
	when tetta[1] = 0 the second edge of the manipulator is codirectional to the first
	when tetta[2] = 0 there is no bend between the second and third edge

	all distances are in meters and angles are in radians
"""

# system DH-parameters
a = [0.06, 0.16, 0.10]
alpha = [pi/2, 0, 0]
d = [0.133, 0, 0]
tetta = [0, pi/2 + 0, -1 * 0]
tetta = [3.141592653589793, pi/2-0.1275010612350045, -1.3602121062422476]
# vector in the base (initial) coordinate system
k0 = np.matrix([0, 0, 0, 1])

# transformation matrix
T03 = np.identity(4)

for i in range(3):
	# rotation matrix about an axis z by an angle tetta
	Rz = np.matrix([[cos(tetta[i]), -sin(tetta[i]), 0, 0],
		[sin(tetta[i]), cos(tetta[i]), 0, 0],
		[0, 0, 1, 0],
		[0, 0, 0, 1]])
	T03 = T03.dot(Rz)

	# offset matrix by a and d
	if i == 0:
		Tad = np.matrix([[1, 0, 0, -a[i]],
			[0, 1, 0, 0],
			[0, 0, 1, d[i]], 
			[0, 0, 0, 1]])
	else:
		Tad = np.matrix([[1, 0, 0, a[i]],
			[0, 1, 0, 0],
			[0, 0, 1, d[i]], 
			[0, 0, 0, 1]])
	T03 = T03.dot(Tad)

	# rotation matrix about an axis x by an angle alpha
	Rx = np.matrix([[1, 0, 0, 0],
		[0, cos(alpha[i]), -sin(alpha[i]), 0],
		[0, sin(alpha[i]), cos(alpha[i]), 0],
		[0, 0, 0, 1]])
	T03 = T03.dot(Rx)

# vector in the result coordinate system
k03 = np.dot(T03, k0.transpose())

print(k03)