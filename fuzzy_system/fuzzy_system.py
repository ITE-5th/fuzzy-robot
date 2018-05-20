from abc import ABCMeta, abstractmethod


class FuzzySystem(metaclass=ABCMeta):

    @abstractmethod
    def run(self, values: dict):
        raise NotImplementedError()
