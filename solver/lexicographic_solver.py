from misc.lexicographic import Lexicographic
from solver.solver import Solver


class LexicographicSolver(Solver):
    def solve(self):
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
