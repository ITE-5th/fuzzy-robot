from math import sqrt, atan2, radians, cos, sin, hypot

from fuzzy_controller.fuzzy_system import FuzzySystem

goal_threshold = 1


def success(xo, yo, xd, yd):
    dist = hypot(xd - xo, yd - yo)
    return dist < goal_threshold


if __name__ == '__main__':
    step = 0.001

    # robot position and orientation
    x, y, theta = 0, 0, 0

    # target position and orientation
    x_d, y_d, theta_d = 2, 0, 0

    # wheel diameter
    R = 0.085

    # Distance between 2 wheels
    L = 0.265

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
    # d = [1.2, 2.2, 3.2, 4.2, 5.2, 2.2, 7.2, 8.2]
    # dr = min(d[0], d[1], d[2])
    # df = min(d[3], d[4])
    # dl = min(d[5], d[6], d[7])
    fuzzy_system = FuzzySystem(False)
    dl = 4
    df = 4
    dr = 4
    a = max(min(a, 4), -4)
    p = max(min(p, 20), 0)
    ed = max(min(ed, 1), -1)
    print(f"dl:{dl}, df:{df}, dr:{dr}, a:{a}, p:{p}, ed:{ed}")
    # msg = {'dl': 1.21, 'df': 1.51, 'dr': 1.22, 'alpha': 0.0, 'p': 2.0, 'ed': 1}
    # dl = msg['dl']
    # df = msg['df']
    # dr = msg['dr']
    # a = msg['alpha']
    # p = msg['p']
    # ed = msg['ed']
    goal_reached = success(x, y, x_d, y_d)
    while not goal_reached:
        u, w = fuzzy_system.run(dl, df, dr, a, p, ed)
        theta += w
        theta = theta % radians(360)

        x = u * cos(theta)
        y = u * sin(theta)
        a = round(atan2(y_d - y, x_d - x) - theta, 2)
        p_current = sqrt(pow(x_d - x, 2) + pow(y_d - y, 2))
        ed = round(p_current - p, 2)
        p = round(p_current, 2)
        print(f"u = {round(u,2)}, w = {round(w,2)}")
        print(f"position is : x:{round(x,2)},y:{round(y,2)},theta:{round(theta,2)}")
        goal_reached = success(x, y, x_d, y_d)
    if goal_reached:
        print('GOAL REACHED')
