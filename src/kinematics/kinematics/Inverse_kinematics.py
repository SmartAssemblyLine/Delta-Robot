import math
import numpy as np

# ------------------- CONSTANTS -------------------
sqrt3 = math.sqrt(3.0)
pi = math.pi
sin120 = sqrt3 / 2.0
cos120 = -0.5
tan60 = sqrt3
sin30 = 0.5
tan30 = 1 / sqrt3

# ------------------- GEOMETRY (mm) -------------------
# MUST match FK exactly
f=0.519615 #Base 300 dia
e=0.1732 #EndEffector 100 dia
rf=0.25    #Bicep
re=0.600      #Forarm
# f=0.439075 /2  #Base 
# e=0.200918 /2  #EndEffector
# rf=0.100      #Bicep
# re=0.200      #Forarm

#stepperdelta



# ====================================================
# Helper: IK for one arm (YZ plane)
# ====================================================
def delta_calcAngleYZ(x0, y0, z0):
    """
    Calculates theta for one arm.
    Returns theta in degrees, or None if unreachable.
    """

    y1 = -0.5 * 0.57735 * f      # f/2 * tan30
    y0 -= 0.5 * 0.57735 * e      # shift center to edge

    # z = a + b*y
    a = (x0*x0 + y0*y0 + z0*z0 + rf*rf - re*re - y1*y1) / (2*z0)
    b = (y1 - y0) / z0

    # discriminant
    d = -(a + b*y1)*(a + b*y1) + rf*(b*b*rf + rf)
    if d < 0:
        return None  # non-existing position

    yj = (y1 - a*b - math.sqrt(d)) / (b*b + 1)
    zj = a + b*yj

    theta = math.degrees(math.atan(-zj / (y1 - yj)))
    if yj > y1:
        theta += 180.0

    return theta


# ====================================================
# Full inverse kinematics
# (x0, y0, z0) → (theta1, theta2, theta3)
# ====================================================
def inverse(x0, y0, z0):
    """
    Returns (theta1, theta2, theta3) in degrees
    or None if point is unreachable.
    """

    theta1 = delta_calcAngleYZ(x0, y0, z0)
    if theta1 is None:
        return None

    # rotate +120°
    x1 = x0*cos120 + y0*sin120
    y1 = y0*cos120 - x0*sin120
    theta2 = delta_calcAngleYZ(x1, y1, z0)
    if theta2 is None:
        return None

    # rotate -120°
    x2 = x0*cos120 - y0*sin120
    y2 = y0*cos120 + x0*sin120
    theta3 = delta_calcAngleYZ(x2, y2, z0)
    if theta3 is None:
        return None
    if theta1 < -40 or theta2 < -40 or theta3 < -40:
        print(theta1)
        print(theta2)
        print(theta3)
        return None
    return theta1, theta2, theta3


thatas=inverse(-0.25,-0.0,-0.5)
print(thatas)