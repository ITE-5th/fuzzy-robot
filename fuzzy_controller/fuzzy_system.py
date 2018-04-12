import numpy as np
import skfuzzy.control as ctrl
from platypus import NSGAII, Problem
from platypus import Subset
from skfuzzy.membership import *

from fuzzy_controller.goal_reaching_controller import GoalReachingController
from fuzzy_controller.local_minimum_avoidance_controller import LocalMinimunAvoidanceController
from fuzzy_controller.obstacle_avoidance_controller import ObstacleAvoidanceController
from misc.lexicographic import Lexicographic


class FuzzySystem:
    def __init__(self, use_lex=False):
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
        self.u1, self.w1 = None, None
        self.u2, self.w2 = None, None
        self.u3, self.w3 = None, None
        self.use_lex = use_lex

    def run(self, dl, df, dr, a, p, ed):
        # temp = {"input_dl": dl, "input_df": df, "input_dr": dr, "input_a": a, "input_p": p, "input_ed": ed}
        self.u1, self.w1 = self.oa_controller.compute(dl, df, dr, a, p, ed)
        self.u2, self.w2 = self.lma_controller.compute(dl, df, dr, a, p, ed)
        self.u3, self.w3 = self.gr_controller.compute(dl, df, dr, a, p, ed)
        if self.use_lex:
            return self.solve_lexicographic()
        return self.solve_problems()

    def solve_lexicographic(self):
        w1, u1 = self.w1.mfx.max(), self.u1.mfx.max()
        w2, u2 = self.w2.mfx.max(), self.u2.mfx.max()
        w3, u3 = self.w3.mfx.max(), self.u3.mfx.max()

        a = Lexicographic([w1, u1], maximize=[True, False])
        b = Lexicographic([w2, u2], maximize=[True, False])
        c = Lexicographic([w3, u3], maximize=[True, False])

        if a > b:
            return self.w1.find_x(w1), self.u1.find_x(u1)

        if b > c:
            return self.w2.find_x(w2), self.u2.find_x(u2)

        return self.w3.find_x(w3), self.u3.find_x(u3)

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
        x = x[0]
        return [self.u1.find_mfx(x[0]), self.u2.find_mfx(x[0]), self.u3.find_mfx(x[0])]

    def w_function(self, x):
        x = x[0]
        return [self.w1.find_mfx(x[0]), self.w2.find_mfx(x[0]), self.w3.find_mfx(x[0])]

    def build_problems(self):
        u_problem, w_problem = Problem(1, 3), Problem(1, 3)
        u_problem.types[:] = Subset(self.output_u.universe, 1)
        w_problem.types[:] = Subset(self.output_w.universe, 1)
        u_problem.directions[:] = Problem.MAXIMIZE
        w_problem.directions[:] = Problem.MAXIMIZE
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
