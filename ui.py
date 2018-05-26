import os
import sys
import time
from math import hypot, atan2, cos, sin

import numpy as np
import pyqtgraph as pg
from PyQt5 import uic, QtWidgets
from PyQt5.QtCore import QRectF
from PyQt5.QtWidgets import QGraphicsPixmapItem, QApplication
from qtconsole.qt import QtGui

from fuzzy_system.moo_fuzzy_system import MooFuzzySystem

os.environ['PYQTGRAPH_QT_LIB'] = "PyQt5"
FormClass = uic.loadUiType("ui.ui")[0]


class Ui(QtWidgets.QMainWindow, FormClass):
    background_color = (255, 255, 255, 1)
    max_range = 25
    incremental = True
    time_between_plot = 0.2
    goal_threshold = 1

    def __init__(self, parent=None):
        QtWidgets.QMainWindow.__init__(self, parent)
        self.setupUi(self)
        self.plotWidget.setBackground(Ui.background_color)
        self.plotWidget.setXRange(0, Ui.max_range)
        self.plotWidget.setYRange(0, Ui.max_range)
        self.plotWidget.setRange(rect=QRectF(-10, -10, 20, 20))

        self.plotWidget.hideAxis(axis='left')
        self.plotWidget.hideAxis(axis='bottom')

        self.fuzzy_system = MooFuzzySystem(True)
        self.pixel_map = QtGui.QPixmap("city.png")
        self.image_scale = 0.02

    @staticmethod
    def success(xo, yo, xd, yd):
        dist = hypot(xd - xo, yd - yo)
        return dist < Ui.goal_threshold

    def solve(self):
        # robot position and orientation
        x, y, theta = 0, 0, 0

        # target position and orientation
        # x_d, y_d, theta = -10, 0, 0
        # x_d, y_d, theta = 0, -10, 0
        x_d, y_d, theta = 10, 15, 1.5
        # Distance from the center of the robot to the target, in [0, 20]
        p = hypot(x_d - x, y_d - y)

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
        # a = max(min(a, 4), -4)
        p = max(min(p, 20), 0)
        ed = max(min(ed, 1), -1)

        dl, df, dr = 0.2, 0.2, 3.2
        # msg = {'dl': 1.21, 'df': 1.51, 'dr': 1.22, 'alpha': 0.0, 'p': 2.0, 'ed': 1}
        msg = {'dl': dl, 'df': df, 'dr': dr, 'alpha': a, 'p': p, 'ed': ed}

        degree = 10
        goal_reached = self.success(x, y, x_d, y_d)

        print(
            f"x:{round(x,degree)}, "
            f"y:{round(y,degree)}, "
            # f"t = {round(atan2(y_d - y, x_d - x), degree)}, "
            f"theta = {round(theta, degree)}, "
            f"alpha:{round(a,degree)}, "
            f"p: {msg['p']}, "
            f"ed: {msg['ed']}, "
        )

        self.show_solution([x, y], [x_d, y_d])
        while not goal_reached:
            # print(f"dl:{msg['dl']}, df:{msg['df']}, dr:{msg['dr']}, a:{msg['alpha']}, p:{msg['p']}, ed:{msg['ed']}")
            # dl = round(random() * 4, 2)
            # dr = round(random() * 4, 2)
            # df = round(random() * 4, 2)
            u, w = self.fuzzy_system.run(msg)
            # u /= 4
            # w /= 4
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

            p = max(min(p, 20), 0)
            ed = max(min(ed, 1), -1)

            msg = {'dl': dl, 'df': df, 'dr': dr, 'alpha': a, 'p': p, 'ed': ed}
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
            goal_reached = self.success(x, y, x_d, y_d)
            time.sleep(Ui.time_between_plot)
            self.show_solution([x, y], [x_d, y_d])

        if goal_reached:
            print('GOAL REACHED')

    def show_solution(self, robot, target):
        # plot_path(robot)
        self.plot([robot, target])
        QApplication.processEvents()

    def plot(self, position):
        xs, ys = np.asarray(position).T

        self.plotWidget.clear()
        item = pg.ScatterPlotItem([xs[0]], [ys[0]], pen=pg.mkPen(color=(0, 0, 0), width=2))
        self.plotWidget.addItem(item)

        image = QGraphicsPixmapItem(self.pixel_map)
        image.scale(self.image_scale, -self.image_scale)
        image.setPos(xs[1] - self.pixel_map.width() * self.image_scale / 2,
                     ys[1] + self.pixel_map.height() * self.image_scale / 2)
        self.plotWidget.addItem(image)


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)

    ui = Ui()
    ui.setWindowTitle("Robot")
    ui.show()
    ui.solve()
    app.exec_()
