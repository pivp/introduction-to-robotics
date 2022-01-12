import numpy as np

# engine parameters

km = ke = 0.48756937411725393
R = 8.183683911882799
J = 0.002437632

# acceleration of gravity

g = 9.81

# robot parameters

mt = 0.551 # body mass
jt = 0.001255 # moment of inertia of a body
mk = 0.023 # wheel mass
r = 0.0275 # wheel radius
jk = 0.00000869 # moment of inertia of a wheel
l =  0.122 # distance from the center of the wheel to the center of mass of the body [14,5;15]
x = mt*l*r*(mt*l*r-2*J)-(mt*l*l+jt)*(mt*r*r+2*mk*r*r+2*jk+2*J) # parameters for calculating coefficients
print(x)
stp = 6.3

# transient time standard 

# elements of the matrix Α

a22 = 2*km*ke*(mt*l*r+mt*l*l+jt)/R/x
a21 = mt*mt*g*l*l*r/x
a32 = -2*km*ke*(mt*l*r+mt*r*r+2*mk*r*r+2*jk)/R/x
a31 = -mt*g*l*(mt*r*r+2*mk*r*r+2*jk+2*J)/x

print("a22 = " + str(a22))
print("a21 = " + str(a21))
print("a32 = " + str(a32))
print("a31 = " + str(a31))

# elements of the matrix B

b2 = -2*km*(mt*l*r+mt*l*l+jt)/R/x
b3 = 2*km*(mt*l*r+mt*r*r+2*mk*r*r+2*jk)/R/x

print("b2 = " + str(b2))
print("b3 = " + str(b3))

# custom variables

tp = 0.7 # transient time

w0 = stp/tp

# elements of the matrix C

C = np.array([[0, b2, b3], [b3, 0, a32*b2-a22*b3], [a32*b2-a22*b3, a21*b3-a31*b2, 0]])
C = np.linalg.inv(C)

# elements of the matrix D

D = np.array([[3*w0+a22], [3*w0**2+a31], [w0**3-a22*a31+a21*a32]])

# elements of the matrix K
K = C.dot(D)
k1 = K[0][0]
k2 = K[1][0]
k3 = K[2][0]
print("k1 = "+str(k1))
print("k2 = "+str(k2))
print("k3 = "+str(k3))
input()
