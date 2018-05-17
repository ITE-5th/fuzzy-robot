from abc import abstractmethod, ABCMeta


class Solver(metaclass=ABCMeta):
    def __init__(self):
        self.u1 = self.u2 = self.u3 = self.w1 = self.w2 = self.w3 = None
        self.used_components = [True, True, True]

    def set_variables(self, u1, u2, u3, w1, w2, w3):
        self.u1, self.u2, self.u3, self.w1, self.w2, self.w3 = u1, u2, u3, w1, w2, w3
        self.used_components = [self.u1 is not None, self.u2 is not None, self.u3 is not None]

    @abstractmethod
    def solve(self):
        raise NotImplementedError()
