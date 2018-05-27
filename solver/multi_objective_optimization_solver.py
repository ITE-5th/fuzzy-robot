from multiprocessing import cpu_count
from multiprocessing.pool import Pool

import numpy as np
from platypus import Problem, Subset, NSGAII

from solver.solver import Solver


class MultiObjectiveOptimizationSolver(Solver):
    def __init__(self, iterations=1000):
        super().__init__()
        self.iterations = iterations
        self.us = []
        self.ws = []

    def solve(self):
        print(self.used_components)
        u, w = self.build_problems()
        if isinstance(u, np.float):
            return u, w
        with Pool(cpu_count()) as p:
            u, w = p.map(self._solve, [(u, self.iterations, True), (w, self.iterations, False)])
        return u, w

    @staticmethod
    def _solve(args):
        problem, iterations, use_max = args
        algorithm = NSGAII(problem)
        algorithm.run(iterations)
        feasible_solutions = [s.objectives[0] for s in algorithm.result if s.feasible]
        return max(feasible_solutions) if use_max else min(feasible_solutions)

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
        if temp == 1:
            return self.defuzz_not_none()
        u_problem, w_problem = Problem(1, temp), Problem(1, temp)
        u_universe, w_universe = self.universe_not_none()
        u_problem.types[:] = Subset(u_universe, 1)
        w_problem.types[:] = Subset(w_universe, 1)
        u_problem.directions[:] = Problem.MAXIMIZE
        w_problem.directions[:] = Problem.MAXIMIZE
        u_problem.function, w_problem.function = self.u_function, self.w_function
        return u_problem, w_problem

    def universe_not_none(self):
        if self.u1 is not None:
            return self.u1.x, self.w1.x
        if self.u2 is not None:
            return self.u2.x, self.w2.x
        return self.u3.x, self.w3.x

    def defuzz_not_none(self):
        if self.u1 is not None and self.used_components[0]:
            return self.inf11.defuzz(), self.inf12.defuzz()
        if self.u2 is not None and self.used_components[1]:
            return self.inf21.defuzz(), self.inf22.defuzz()
        # return self.inf31.sim.output['output_u'], self.inf31.sim.output['output_w']
        return self.inf31.defuzz(), self.inf32.defuzz()
