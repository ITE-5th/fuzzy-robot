from misc.lexicographic import Lexicographic
from solver.solver import Solver


class LexicographicSolver(Solver):

    @staticmethod
    def get_if_not_none(u, w):
        if u is not None:
            return u.mfx.max(), w.mfx.max()
        return -1000000, -100000

    def solve(self):
        u1, w1 = self.get_if_not_none(self.u1, self.w1)
        u2, w2 = self.get_if_not_none(self.u2, self.w2)
        u3, w3 = self.get_if_not_none(self.u3, self.w3)
        temp = [False, True]
        a = Lexicographic([u1, w1], maximize=temp)
        b = Lexicographic([u2, w2], maximize=temp)
        c = Lexicographic([u3, w3], maximize=temp)

        if a > b:
            return self.u1.find_x(u1), self.w1.find_x(w1)

        if b > c:
            return self.u2.find_x(u2), self.w2.find_x(w2)

        return self.u3.find_x(u3), self.w3.find_x(w3)
