from math import sqrt, atan2

import numpy as np
import skfuzzy.control as ctrl
from skfuzzy.control import ControlSystem
from skfuzzy.membership import *

step = 0.001

# wheel diameter
R = 0.085

# Distance between 2 wheels
L = 0.265

# robot position and orientation
x, y, theta = 1000, 2000, 90

# target position and orientation
x_d, y_d, theta_d = 5000, 10000, 20

# Distance from the center of the robot to the target, in [0, 20]
p = sqrt(pow(x_d - x, 2) + pow(y_d - y, 2))
prev_p = 0

# angle between the robot heading and the vector connecting the robot center with the target, alpha in [-pi, +pi]
a = atan2(y_d - y, x_d - x) - theta

# the movement of the robot with the target, in [-1, 1]
ed = p - prev_p

# u Linear velocity, w angular velocity
# u in [0, 1.3] m/s
# w in [-4.3, 4.3] rad/s

# distance to obstacles measured by sensor i
d = [1, 2, 3, 4, 5, 6, 7, 8]
dr = min(d[0], d[1], d[2])
df = min(d[3], d[4])
dl = min(d[5], d[6], d[7])

fuzzy_system = FuzzySystem()
while True:
    u, w = fuzzy_system.run(dl, df, dr, a, p, ed)