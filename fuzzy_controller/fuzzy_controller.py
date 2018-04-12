from abc import ABCMeta, abstractmethod

import skfuzzy.control as ctrl


class FuzzyController(metaclass=ABCMeta):
    def __init__(self, input_dl, input_df, input_dr, input_a, input_p, input_ed, output_u, output_w):
        self.input_dl, self.input_df, self.input_dr, self.input_a, self.input_p, self.input_ed = input_dl, input_df, input_dr, input_a, input_p, input_ed
        self.output_u, self.output_w = output_u, output_w
        self.controller = ctrl.ControlSystem(self.build_rules())
        self.controller = ctrl.ControlSystemSimulation(self.controller)

    @abstractmethod
    def build_rules(self):
        raise NotImplementedError()

    def compute(self, inputs):
        self.controller.inputs(inputs)
        self.controller.compute()
        # :/
        return None, None
