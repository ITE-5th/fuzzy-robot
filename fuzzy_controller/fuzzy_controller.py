from abc import ABCMeta, abstractmethod

import skfuzzy.control as ctrl
from skfuzzy.control.controlsystem import CrispValueCalculator


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
        u_universe, mfu, _ = CrispValueCalculator(self.output_u, self.controller).find_memberships()
        w_universe, mfw, _ = CrispValueCalculator(self.output_w, self.controller).find_memberships()
        return MfMapping(u_universe, mfu), MfMapping(w_universe, mfw)


class MfMapping:
    def __init__(self, x, mfx):
        self.indices = {t: i for i, t in enumerate(x)}
        self.mfx = mfx

    def find(self, x):
        return self.mfx[self.indices[x]]
