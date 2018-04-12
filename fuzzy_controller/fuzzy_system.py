from multiprocessing import cpu_count

import numpy as np
import skfuzzy.control as ctrl
from platypus import NSGAII, Problem, Real
from skfuzzy.membership import *

from fuzzy_controller.goal_reaching_controller import GoalReachingController
from fuzzy_controller.local_minimum_avoidance_controller import LocalMinimunAvoidanceController
from fuzzy_controller.obstacle_avoidance_controller import ObstacleAvoidanceController


class FuzzySystem:
    def __init__(self):
        self.step = 0.001
        self.input_dl, self.input_df, self.input_dr, self.input_a, self.input_p, self.input_ed = self.build_inputs()
        self.output_u, self.output_w = self.build_outputs()
        self.oa_controller = ObstacleAvoidanceController(self.input_dl, self.input_df, self.input_dr, self.input_a,
                                                         self.input_p, self.input_ed,
                                                         self.output_u, self.output_w)
        self.lma_controller = LocalMinimunAvoidanceController(self.input_dl, self.input_df, self.input_dr, self.input_a,
                                                              self.input_p, self.input_ed,
                                                              self.output_u, self.output_w)
        self.gr_controller = GoalReachingController(self.input_dl, self.input_df, self.input_dr, self.input_a,
                                                    self.input_p, self.input_ed, self.output_u,
                                                    self.output_w)
        self.controller = ctrl.ControlSystem(
            self.oa_controller.rules + self.lma_controller.rules + self.gr_controller.rules)
        self.controller = ctrl.ControlSystemSimulation(self.controller)

    def run(self, dl, df, dr, a, p, ed):
        self.controller.inputs(
            {"input_dl": dl, "input_df": df, "input_dr": dr, "input_a": a, "input_p": p, "input_ed": ed})
        # not yet :3
        self.controller.compute()
        return self.solve_problems()

    def solve_problems(self):
        u_problem, w_problem = self.build_problems()
        algorithm = NSGAII(u_problem)
        algorithm.run(1000)
        u = algorithm.result[0].objectives
        algorithm = NSGAII(w_problem)
        algorithm.run(1000)
        w = algorithm.result[0].objectives
        return u, w

    def u_function(self, x):
        return []

    def w_function(self, x):
        return []

    def build_problems(self):
        u_problem, w_problem = Problem(1, cpu_count()), Problem(1, cpu_count())
        u_problem.types[:] = Real(0, 2)
        w_problem.types[:] = Real(-5, 5)
        u_problem.function, w_problem.function = self.u_function, self.w_function
        return u_problem, w_problem

    def build_inputs(self):
        input_dl = ctrl.Antecedent(np.arange(0, 4, self.step), "input_dl")  # meters
        input_df = ctrl.Antecedent(np.arange(0, 4, self.step), "input_df")  # meters
        input_dr = ctrl.Antecedent(np.arange(0, 4, self.step), "input_dr")  # meters
        input_a = ctrl.Antecedent(np.arange(-4, +4, self.step), "input_a")  # rad
        input_p = ctrl.Antecedent(np.arange(0, 20, self.step), "input_p")  # meters
        input_ed = ctrl.Antecedent(np.arange(-1, 1, self.step), "input_ed")  # meters

        input_dl["N"] = zmf(input_dl.universe, 0.22, 1.22)
        input_df["N"] = zmf(input_df.universe, 0.22, 1.22)
        input_dr["N"] = zmf(input_dr.universe, 0.22, 1.22)

        input_dl["M"] = gaussmf(input_dl.universe, 0.88, 0.25)
        input_df["M"] = gaussmf(input_df.universe, 0.88, 0.25)
        input_dr["M"] = gaussmf(input_dr.universe, 0.88, 0.25)

        input_dl["F"] = smf(input_dl.universe, 0.65, 1.7)
        input_df["F"] = smf(input_df.universe, 0.65, 1.7)
        input_dr["F"] = smf(input_dr.universe, 0.65, 1.7)

        input_a["LN"] = zmf(input_a.universe, -2.88, -0.92)
        input_a["N"] = gaussmf(input_a.universe, -0.95, 0.4)
        input_a["Z"] = gaussmf(input_a.universe, 0, 0.15)
        input_a["P"] = gaussmf(input_a.universe, 0.95, 0.4)
        input_a["LP"] = smf(input_a.universe, 0.92, 2.88)

        input_p['N'] = zmf(input_p.universe, 0, 3.5)
        input_p['M'] = gaussmf(input_p.universe, 2, 0.6)
        input_p['F'] = smf(input_p.universe, 0, 4)

        input_ed['NT'] = zmf(input_ed.universe, -0.1, 0.05)
        input_ed['ZT'] = gaussmf(input_ed.universe, 0, 0.015)
        input_ed['PT'] = smf(input_ed.universe, -0.05, 0.1)
        return input_dl, input_df, input_dr, input_a, input_p, input_ed

    def build_outputs(self):
        output_u = ctrl.Consequent(np.arange(0, 2, self.step), "output_u")  # m/s
        output_w = ctrl.Consequent(np.arange(-5, +5, self.step), "output_w")  # rad/s

        output_u["S"] = zmf(output_u.universe, 0.16, 0.6)
        output_u["M"] = gaussmf(output_u.universe, 0.5, 0.12)
        output_u["L"] = smf(output_u.universe, 0.42, 0.95)
        # oa_u.view()

        output_w["LNO"] = zmf(output_w.universe, -4, -1.5)
        output_w["NO"] = gaussmf(output_w.universe, -1.8, 0.6)
        output_w["ZO"] = gaussmf(output_w.universe, 0.1, 0.2)
        output_w["PO"] = gaussmf(output_w.universe, 1.8, 0.6)
        output_w["LPO"] = smf(output_w.universe, 1.5, 4)
        return output_u, output_w
