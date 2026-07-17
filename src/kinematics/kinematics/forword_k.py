import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import math

#-------------------CONSTANTS------------------#
sqrt3 = np.sqrt(3)
pi=3.141592653
sin30=0.5
tan30=1/sqrt3
tan60=sqrt3
sin120=sqrt3/2
cos120=-0.5

#-------------------LENGTHS------------------#
# f=0.519615 #Base 
# e=0.1732 #EndEffector
# rf=0.3     #Bicep
# re=0.600      #Forarm
f=0.519615 #Base 300 dia
e=0.1732 #EndEffector 100 dia
rf=0.25    #Bicep
re=0.600      #Forarm


#Forword kinematics calculations

def forward(theta1, theta2, theta3):

    t = (f - e) * tan30 / 2
    dtr = pi / float(180.0)

    theta1 *= dtr
    theta2 *= dtr
    theta3 *= dtr

    y1 = -(t + rf*math.cos(theta1))
    z1 = -rf*math.sin(theta1)

    y2 = (t + rf*math.cos(theta2)) * sin30
    x2 = y2 * tan60
    z2 = -rf*math.sin(theta2)

    y3 = (t + rf*math.cos(theta3)) * sin30
    x3 = -y3 * tan60
    z3 = -rf*math.sin(theta3)

    dnm = (y2 - y1)*x3 - (y3 - y1)*x2

    w1 = y1*y1 + z1*z1
    w2 = x2*x2 + y2*y2 + z2*z2
    w3 = x3*x3 + y3*y3 + z3*z3

    a1 = (z2 - z1)*(y3 - y1) - (z3 - z1)*(y2 - y1)
    b1 = -((w2 - w1)*(y3 - y1) - (w3 - w1)*(y2 - y1)) / 2.0

    a2 = -(z2 - z1)*x3 + (z3 - z1)*x2
    b2 = ((w2 - w1)*x3 - (w3 - w1)*x2) / 2.0

    a = a1*a1 + a2*a2 + dnm*dnm
    b = 2*(a1*b1 + a2*(b2 - y1*dnm) - z1*dnm*dnm)
    c = (b2 - y1*dnm)*(b2 - y1*dnm) + b1*b1 + dnm*dnm*(z1*z1 - re*re)

    d = b*b - 4.0*a*c
    if d < 0:
        return None  # non-existing point

    z0 = -0.5*(b + math.sqrt(d)) / a
    x0 = (a1*z0 + b1) / dnm
    y0 = (a2*z0 + b2) / dnm

    return x0, y0, z0


if __name__ == "__main__":
    print(forward(0.573634505894663, 71.56391808362446, 7.09158554287998))