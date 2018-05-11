from multiprocessing.pool import Pool

from platypus import Problem, Subset, NSGAII

from solver.solver import Solver


class MultiObjectiveOptimizationSolver(Solver):
    def __init__(self, output_u, output_w, iterations=10000):
        super().__init__()
        self.u_universe, self.w_universe = output_u.universe, output_w.universe
        self.iterations = iterations

    def solve(self):
        u_problem, w_problem = self.build_problems()
        with Pool(2) as p:
            u, w = p.map(self._solve, [(u_problem, self.iterations), (w_problem, self.iterations)])
            p.join()
        return u, w

    @staticmethod
    def _solve(args):
        problem, iterations = args
        algorithm = NSGAII(problem)
        algorithm.run(iterations)
        return algorithm.result[0].objectives

    def u_function(self, x):
        x = x[0][0]
        return [self.u1.find_mfx(x), self.u2.find_mfx(x), self.u3.find_mfx(x)]

    def w_function(self, x):
        x = x[0][0]
        return [self.w1.find_mfx(x), self.w2.find_mfx(x), self.w3.find_mfx(x)]

    def build_problems(self):
        u_problem, w_problem = Problem(1, 3), Problem(1, 3)
        u_problem.types[:] = Subset(self.u_universe, 1)
        w_problem.types[:] = Subset(self.w_universe, 1)
        u_problem.directions[:] = w_problem.directions[:] = Problem.MAXIMIZE
        u_problem.function, w_problem.function = self.u_function, self.w_function
        return u_problem, w_problem
