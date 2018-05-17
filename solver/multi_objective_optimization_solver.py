from multiprocessing.pool import Pool

import numpy as np
from platypus import Problem, Subset, NSGAII

from solver.solver import Solver


class MultiObjectiveOptimizationSolver(Solver):
    def __init__(self, output_u, output_w, iterations=1000):
        super().__init__()
        self.u_universe, self.w_universe = output_u.universe, output_w.universe
        self.iterations = iterations
        self.us = []
        self.ws = []

    def solve(self):
        u_problem, w_problem = self.build_problems()
        with Pool(2) as p:
            u, w = p.map(self._solve, [(u_problem, self.iterations), (w_problem, self.iterations)])
        return u, w

    @staticmethod
    def _solve(args):
        problem, iterations = args
        algorithm = NSGAII(problem)
        algorithm.run(iterations)
        result = algorithm.result[0]
        result = result.objectives[0]
        return result

    def u_function(self, x):
        x = x[0][0]
        u1, u2, u3 = [self.us[i].find_mfx(x) if self.used_components[i] else None for i in
                      range(len(self.used_components))]
        result = np.asarray([u1, u2, u3])
        result = result[self.used_components].reshape(-1)
        result = result.tolist()
        return result

    def w_function(self, x):
        x = x[0][0]
        w1, w2, w3 = [self.ws[i].find_mfx(x) if self.used_components[i] else None for i in
                      range(len(self.used_components))]
        result = np.asarray([w1, w2, w3])
        result = result[self.used_components].reshape(-1)
        result = result.tolist()
        return result

    def build_problems(self):
        self.us = [self.u1, self.u2, self.u3]
        self.ws = [self.w1, self.w2, self.w3]
        temp = sum(self.used_components)
        u_problem, w_problem = Problem(1, temp), Problem(1, temp)
        u_problem.types[:] = Subset(self.u_universe, 1)
        w_problem.types[:] = Subset(self.w_universe, 1)
        u_problem.directions[:] = w_problem.directions[:] = Problem.MAXIMIZE
        u_problem.function, w_problem.function = self.u_function, self.w_function
        return u_problem, w_problem
