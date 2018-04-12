from abc import abstractmethod, ABCMeta


class Solver(metaclass=ABCMeta):
    def __init__(self):
        self.u1 = self.u2 = self.u3 = self.w1 = self.w2 = self.w3 = None

    def set_variables(self, u1, u2, u3, w1, w2, w3):
        self.u1, self.u2, self.u3, self.w1, self.w2, self.w3 = u1, u2, u3, w1, w2, w3

    @abstractmethod
    def solve(self):
        raise NotImplementedError()
