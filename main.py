from math import sqrt

from fuzzy_controller.fuzzy_system import FuzzySystem

if __name__ == '__main__':
    step = 0.001

    # wheel diameter
    R = 0.085

    # Distance between 2 wheels
    L = 0.265

    # robot position and orientation
    x, y, theta = 10, 10, 90

    # target position and orientation
    x_d, y_d, theta_d = 12, 12, 20

    # Distance from the center of the robot to the target, in [0, 20]
    p = sqrt(pow(x_d - x, 2) + pow(y_d - y, 2))
    prev_p = 0

    # angle between the robot heading and the vector connecting the robot center with the target, alpha in [-pi, +pi]
    # a = atan2(y_d - y, x_d - x) - theta
    a = -2

    # the movement of the robot with the target, in [-1, 1]
    ed = p - prev_p
    print(ed)
    # u Linear velocity, w angular velocity
    # u in [0, 1.3] m/s
    # w in [-4.3, 4.3] rad/s

    # distance to obstacles measured by sensor i
    d = [1.2, 2.2, 3.2, 4.2, 5.2, 2.2, 7.2, 8.2]
    dr = min(d[0], d[1], d[2])
    df = min(d[3], d[4])
    dl = min(d[5], d[6], d[7])
    fuzzy_system = FuzzySystem(False)
    dl = max(min(dl, 4), 0)
    df = max(min(df, 4), 0)
    dr = max(min(dr, 4), 0)
    a = max(min(a, 4), -4)
    p = max(min(p, 20), 0)
    ed = max(min(ed, 1), -1)
    print('{} {} {} {} {} {}'.format(dl, df, dr, a, p, ed))
    msg = {'dl': 1.21, 'df': 1.51, 'dr': 1.22, 'alpha': 0.0, 'p': 2.0, 'ed': 1}
    dl = msg['dl']
    df = msg['df']
    dr = msg['dr']
    a = msg['alpha']
    p = msg['p']
    ed = msg['ed']
    while True:
        u, w = fuzzy_system.run(dl, df, dr, a, p, ed)

        print(f"u = {u}, w = {w}")
