from abc import ABCMeta, abstractmethod

import skfuzzy.control as ctrl
from skfuzzy import fuzzy_or
from skfuzzy.control.controlsystem import CrispValueCalculator


class MooFuzzyController(metaclass=ABCMeta):
    def __init__(self, input_dl, input_df, input_dr, input_a, input_p, input_ed, output_u, output_w):
        self.input_dl, self.input_df, self.input_dr, self.input_a, self.input_p, self.input_ed = input_dl, input_df, input_dr, input_a, input_p, input_ed
        self.output_u, self.output_w = output_u, output_w
        self.rules = self.build_rules()

    @abstractmethod
    def build_rules(self):
        raise NotImplementedError()

    @abstractmethod
    def inputs(self, dl, df, dr, a, p, ed):
        raise NotImplementedError()

    def validate(self, values):
        for key, value in values.items():
            x = getattr(self, key)
            universe = x.universe
            maxi, mini = universe.max(), universe.min()
            if not (mini <= value <= maxi):
                return False
        return True

    def compute(self, dl, df, dr, a, p, ed):
        temp = self.inputs(dl, df, dr, a, p, ed)
        if not self.validate(temp):
            return None, None, None, None
        try:
            controller = ctrl.ControlSystem(self.rules)
            controller = ctrl.ControlSystemSimulation(controller)
            controller.inputs(temp)
            controller.compute()
            u, t1 = self.get_consequent_membership_function(self.output_u, controller)
            w, t2 = self.get_consequent_membership_function(self.output_w, controller)
            return u, w, t1, t2
        except:
            return None, None, None, None

    @staticmethod
    def get_consequent_membership_function(consequent: ctrl.Consequent, controller):
        # t = CrispValueCalculator(consequent, controller)
        # _, mfx, _ = t.find_memberships()
        # y = MfMapping(consequent.universe, mfx)
        # return y, t
        terms = list(consequent.terms.keys())
        universe = consequent.universe
        _, result, _ = CrispValueCalculator(consequent[terms[0]], controller)
        for i in range(1, len(terms)):
            term = terms[i]
            _, temp, _ = CrispValueCalculator(consequent[term], controller)
            _, result = fuzzy_or(universe, result, universe, temp)
        return MfMapping(universe, result), CrispValueCalculator(consequent, controller)


class MfMapping:
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
