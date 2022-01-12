from math import pi, sin, cos, acos, asin, atan2, atan, sqrt

def sign(x):
	if x > 0:
		return 1
	elif x < 0:
		return -1
	else:
		return 0

# system DH-parameters
a = [0.01, 0.17, 0.095]
alpha = [pi/2, 0, 0]
d = [0.165, 0, 0]
tetta = [0, pi/2 - 0, -1 * 0]

goalCords = [0.255, 0, 0.165]

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
print(goalTetta)