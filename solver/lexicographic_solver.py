import numpy as np

from misc.lexicographic import Lexicographic
from solver.solver import Solver


class LexicographicSolver(Solver):

    @staticmethod
    def get_if_not_none(u, w):
        if u is not None:
            return u.mfx.max(), w.mfx.max()
        return -1000000, -100000

    def solve2(self):
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

    def solve(self):
        print(self.used_components)

        # u1: lm, u2: oa, u3: gr
        # if self.u1 is None or self.u2 is None or self.u3 is None:
        #     print(f"{self.u1}, {self.u2}, {self.u3}")
        # u = self._solve(self.u2, self.u1, self.u3)
        # w = self._solve(self.w2, self.w1, self.w3)
        u = []
        LexicographicSolver.append(u, self.u1)
        LexicographicSolver.append(u, self.u2)
        LexicographicSolver.append(u, self.u3)

        u = LexicographicSolver.unify_length2(u)
        u = self._solve(u, u=True)

        w = []
        LexicographicSolver.append(w, self.w1)
        LexicographicSolver.append(w, self.w2)
        LexicographicSolver.append(w, self.w3)
        w = LexicographicSolver.unify_length2(w)

        w = self._solve(w, u=False)

        return u, w

    @staticmethod
    def append(t, v):
        if v is not None:
            t.append(v)
        return t

    @staticmethod
    def _solve(arrays, u=False):

        mfx1 = arrays[0].mfx
        ind1 = LexicographicSolver.max1(mfx1)

        for ar in arrays:
            mfx2 = ar.mfx
            ind2 = LexicographicSolver.max2(mfx2, ind1)
            if len(ind2) == 1:
                return ar.x[ind2[0]]
                # return ar.find_x(mfx2[ind2])

            ind1 = ind2

        temp = []
        for i in ind2:
            x = ar.x[i]
            # x = ar.find_x(mfx2[i])
            # x = ar.find_x(i)
            temp.append(x)

        temp1 = sorted(temp)
        return temp1[-1] if u else temp[0]

    @staticmethod
    def _solve2(t1, t2, t3):
        mfx1, mfx2, mfx3 = LexicographicSolver.unify_length(t1.mfx, t2.mfx, t3.mfx)

        ind1 = LexicographicSolver.max1(mfx1)
        ind2 = LexicographicSolver.max2(mfx2, ind1)
        if len(ind2) == 1:
            return t2.find_x(mfx2[ind2])

        ind3 = LexicographicSolver.max2(mfx3, ind2)
        if len(ind3) == 1:
            return t3.find_x(mfx3[ind3])

        temp = []
        for i in ind3:
            x = t3.find_x(i)
            temp.append(x)

        return sorted(temp)[-1]

    @staticmethod
    def max1(ar):
        return np.where(ar == ar.max())[0]

    @staticmethod
    def max2(ar, indices):
        return np.where(ar == ar[indices].max())[0]

    @staticmethod
    def unify_length(t1, t2, t3, value=-1):
        max_length = max(max(len(t1), len(t2)), len(t3))
        t1 = np.append(t1, [value] * (max_length - len(t1)))
        t2 = np.append(t2, [value] * (max_length - len(t2)))
        t3 = np.append(t3, [value] * (max_length - len(t3)))
        return t1, t2, t3

    @staticmethod
    def unify_length2(t, value=-1):
        if len(t) == 1:
            return t

        max_length = len(t[0].mfx)
        for temp in t:
            if len(temp.mfx) > max_length:
                max_length = len(temp.mfx)

        new_t = []
        for temp in t:
            temp.mfx = np.append(temp.mfx, [value] * (max_length - len(temp.mfx) + 1))
            new_t.append(temp)
        return new_t
