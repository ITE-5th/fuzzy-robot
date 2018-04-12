from platypus import Problem, Subset, NSGAII

from solver.solver import Solver


class MultiObjectiveOptimizationSolver(Solver):
    def __init__(self, output_u, output_w):
        super().__init__()
        self.u_universe, self.w_universe = output_u.universe, output_w.universe

    def solve(self, u1, u2, u3, w1, w2, w3):
        u_problem, w_problem = self.build_problems()
        algorithm = NSGAII(u_problem)
        algorithm.run(1000)
        u = algorithm.result[0].objectives
        algorithm = NSGAII(w_problem)
        algorithm.run(1000)
        w = algorithm.result[0].objectives
        return u, w

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
        u_problem.directions[:] = Problem.MAXIMIZE
        w_problem.directions[:] = Problem.MAXIMIZE
        u_problem.function, w_problem.function = self.u_function, self.w_function
        return u_problem, w_problem
