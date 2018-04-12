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

    @abstractmethod
    def inputs(self, dl, df, dr, a, p, ed):
        raise NotImplementedError()

    def compute(self, dl, df, dr, a, p, ed):
        temp = self.inputs(dl, df, dr, a, p, ed)
        self.controller.inputs(temp)
        self.controller.compute()
        u_universe, mfu, _ = CrispValueCalculator(self.output_u, self.controller).find_memberships()
        w_universe, mfw, _ = CrispValueCalculator(self.output_w, self.controller).find_memberships()
        return MfMapping(u_universe, mfu), MfMapping(w_universe, mfw)


class MfMapping:
    # not working
    def __init__(self, x, mfx):
        self.x = x
        self.mfx = mfx
        self.x_indices = {t: i for i, t in enumerate(self.x)}
        self.mfx_indices = {t: i for i, t in enumerate(self.mfx)}

    def find_mfx(self, x):
        temp = self.x_indices[x]
        return self.mfx[temp]

    def find_x(self, mfx):
        temp = self.mfx_indices[mfx]
        return self.x[temp]
