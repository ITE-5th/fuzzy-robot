import numpy as np
import math
from math import atan2, cos, sin, hypot

from fuzzy_system.moo_fuzzy_system import MooFuzzySystem

goal_threshold = 1


def success(xo, yo, xd, yd):
    dist = hypot(xd - xo, yd - yo)
    return dist < goal_threshold


if __name__ == '__main__':
    step = 0.001

    # robot position and orientation
    x, y, theta = 0, 0, 0

    # target position and orientation
    # x_d, y_d, theta_d = 5, 0, 0
    x_d, y_d, theta_d = 0, 5, 0
    # x_d, y_d, theta_d = 5, 5, 0

    # Distance from the center of the robot to the target, in [0, 20]
    p = hypot(x_d - x, y_d - y)
    prev_p = 0

    # angle between the robot heading and the vector connecting the robot center with the target, alpha in [-pi, +pi]
    a = atan2(y_d - y, x_d - x) - theta

    # the movement of the robot with the target, in [-1, 1]
    ed = 0  # p - prev_p
    # u Linear velocity, w angular velocity
    # u in [0, 1.3] m/s
    # w in [-4.3, 4.3] rad/s

    # distance to obstacles measured by sensor i
    # d = [1.2, 2.2, 3.2, 4.2, 5.2, 2.2, 7.2, 8.2]
    # dr = min(d[0], d[1], d[2])
    # df = min(d[3], d[4])
    # dl = min(d[5], d[6], d[7])
    fuzzy_system = MooFuzzySystem(True)
    dl = 4
    df = 4
    dr = 4
    # a = max(min(a, 4), -4)
    p = max(min(p, 20), 0)
    ed = max(min(ed, 1), -1)
    # msg = {'dl': 1.21, 'df': 1.51, 'dr': 1.22, 'alpha': 0.0, 'p': 2.0, 'ed': 1}
    msg = {'dl': 3.21, 'df': 3.51, 'dr': 3.22, 'alpha': a, 'p': p, 'ed': ed}

    degree = 10
    goal_reached = success(x, y, x_d, y_d)
    while not goal_reached:
        # print(f"dl:{msg['dl']}, df:{msg['df']}, dr:{msg['dr']}, a:{msg['alpha']}, p:{msg['p']}, ed:{msg['ed']}")
        # dl = round(random() * 4, 2)
        # dr = round(random() * 4, 2)
        # df = round(random() * 4, 2)
        u, w = fuzzy_system.run(msg)
        print(f"u: {u}, w: {w}")

        theta += w
        theta = ((-theta + np.pi) % (2.0 * np.pi) - np.pi) * -1.0

        x += u * cos(theta)
        y += u * sin(theta)

        a = round(atan2(y_d - y, x_d - x) - theta, degree)
        a = ((-a + np.pi) % (2.0 * np.pi) - np.pi) * -1.0

        p_current = hypot(x_d - x, y_d - y)
        ed = round(p_current - p, degree)
        p = round(p_current, degree)
        msg = {'dl': 3.21, 'df': 3.51, 'dr': 3.22, 'alpha': a, 'p': p, 'ed': ed}
        # print(f"u = {round(u,degree)}, w = {round(w,degree)}")
        print(
            f"x:{round(x,degree)}, "
            f"y:{round(y,degree)}, "
            # f"t = {round(atan2(y_d - y, x_d - x), degree)}, "
            f"theta = {round(theta, degree)}, "
            f"alpha:{round(a,degree)}, "
            f"p: {msg['p']}, "
            f"ed: {msg['ed']}, "
        )
        goal_reached = success(x, y, x_d, y_d)

    if goal_reached:
        print('GOAL REACHED')
