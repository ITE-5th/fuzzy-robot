from abc import abstractmethod, ABCMeta


class Solver(metaclass=ABCMeta):
    def __init__(self):
        self.u1 = self.u2 = self.u3 = self.w1 = self.w2 = self.w3 = self.inf11 = self.inf12 = self.inf21 = self.inf22 = self.inf31 = self.inf32 = None
        self.used_components = [True, True, True]

    def set_variables(self, u1, u2, u3, w1, w2, w3, inf11, inf12, inf21, inf22, inf31, inf32):
        self.u1, self.u2, self.u3, self.w1, self.w2, self.w3, self.inf11, self.inf12, self.inf21, self.inf22, self.inf31, self.inf32 = u1, u2, u3, w1, w2, w3, inf11, inf12, inf21, inf22, inf31, inf32
        self.used_components = [self.u1 is not None, self.u2 is not None, self.u3 is not None]

        # TODO: Remove this line when activating all problems
        # self.used_components = [False, False, True]
        # self.used_components = [True, False, False]
        # self.used_components = [True, False, True]

    @abstractmethod
    def solve(self):
        raise NotImplementedError()
