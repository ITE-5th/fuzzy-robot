import numpy as np
import skfuzzy.control as ctrl
from skfuzzy.membership import *

from fuzzy_system.fuzzy_system import FuzzySystem
from fuzzy_system.goal_reaching_controller import GoalReachingController
from fuzzy_system.local_minimum_avoidance_controller import LocalMinimumAvoidanceController
from fuzzy_system.obstacle_avoidance_controller import ObstacleAvoidanceController
from solver.lexicographic_solver import LexicographicSolver
from solver.multi_objective_optimization_solver import MultiObjectiveOptimizationSolver


class MooFuzzySystem(FuzzySystem):
    def __init__(self, use_lex, step=0.001, iterations=1000):
        self.step = step
        self.use_lex = use_lex
        self.input_dl, self.input_df, self.input_dr, self.input_a, self.input_p, self.input_ed = self.build_inputs()
        self.output_u, self.output_w = self.build_outputs()
        self.oa_controller = ObstacleAvoidanceController(self.input_dl, self.input_df, self.input_dr, self.input_a,
                                                         self.input_p, self.input_ed,
                                                         self.output_u, self.output_w)
        self.lma_controller = LocalMinimumAvoidanceController(self.input_dl, self.input_df, self.input_dr, self.input_a,
                                                              self.input_p, self.input_ed,
                                                              self.output_u, self.output_w)
        self.gr_controller = GoalReachingController(self.input_dl, self.input_df, self.input_dr, self.input_a,
                                                    self.input_p, self.input_ed, self.output_u,
                                                    self.output_w)
        self.lex_solver = LexicographicSolver()
        self.moo_solver = MultiObjectiveOptimizationSolver(iterations)

    def run(self, values: dict):
        dl, df, dr, a, p, ed = values["dl"], values["df"], values["dr"], values["alpha"], values["p"], values["ed"]
        u1, w1, inf11, inf12 = self.oa_controller.compute(dl, df, dr, a, p, ed)
        u2, w2, inf21, inf22 = self.lma_controller.compute(dl, df, dr, a, p, ed)
        u3, w3, inf31, inf32 = self.gr_controller.compute(dl, df, dr, a, p, ed)

        self.moo_solver.set_variables(u1, u2, u3, w1, w2, w3, inf11, inf12, inf21, inf22, inf31, inf32)
        self.lex_solver.set_variables(u1, u2, u3, w1, w2, w3, inf11, inf12, inf21, inf22, inf31, inf32)
        return self.lex_solver.solve() if self.use_lex else self.moo_solver.solve()

    def build_inputs(self):
        input_dl = ctrl.Antecedent(np.arange(0, 4, self.step), "input_dl")  # meters
        input_df = ctrl.Antecedent(np.arange(0, 4, self.step), "input_df")  # meters
        input_dr = ctrl.Antecedent(np.arange(0, 4, self.step), "input_dr")  # meters
        input_a = ctrl.Antecedent(np.arange(-4, 4, self.step), "input_a")  # rad
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
        output_w = ctrl.Consequent(np.arange(-4, 4, self.step), "output_w")  # rad/s
        # output_w.terms.values()

        output_u["S"] = zmf(output_u.universe, 0.16, 0.6)
        output_u["M"] = gaussmf(output_u.universe, 0.5, 0.12)
        output_u["L"] = smf(output_u.universe, 0.42, 0.95)

        # output_w["LNO"] = zmf(output_w.universe, -math.pi, -math.pi/2)
        # output_w["NO"] = gaussmf(output_w.universe, -math.pi/2, 0.6)
        # output_w["ZO"] = gaussmf(output_w.universe, 0, 0.15)
        # output_w["PO"] = gaussmf(output_w.universe, math.pi/2, 0.6)
        # output_w["LPO"] = smf(output_w.universe, math.pi/2, math.pi)
        output_w["LNO"] = zmf(output_w.universe, -4, -1.5)
        output_w["NO"] = gaussmf(output_w.universe, -1.8, 0.6)
        output_w["ZO"] = gaussmf(output_w.universe, 0.1, 0.2)
        output_w["PO"] = gaussmf(output_w.universe, 1.8, 0.6)
        output_w["LPO"] = smf(output_w.universe, 1.5, 4)
        return output_u, output_w
