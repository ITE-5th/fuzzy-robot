import traceback

import numpy as np
import skfuzzy.control as ctrl
from skfuzzy import fuzzy_or, trimf, pimf

from fuzzy_system.fuzzy_system import FuzzySystem


class SimpleFuzzySystem(FuzzySystem):

    def __init__(self, step=0.01):
        self.step = step
        self.input_front, self.input_left, self.input_right, self.input_velocity = self.build_inputs()
        self.output_velocity, self.output_angle = self.build_outputs()
        self.rules = self.build_rules()

    def run(self, values: dict):
        try:
            values = {f"input_{k}": v for k, v in values.items() if k != "velocity"}
            controller = ctrl.ControlSystem(self.rules)
            controller = ctrl.ControlSystemSimulation(controller)
            controller.inputs(values)
            controller.compute()
            return controller.output["output_velocity"], controller.output["output_angle"]
        except Exception as e:
            print(e)
            print(traceback.format_exc())
            return 50, 0

    def build_rules(self):
        return [
            # the first rule
            ctrl.Rule(self.input_front['far'], self.output_velocity['fast']),
            ctrl.Rule(self.input_front['far'], self.output_angle['front']),
            # the second rule
            ctrl.Rule(self.input_front['close'] & self.input_right["far"], self.output_velocity['fast']),
            ctrl.Rule(self.input_front['close'] & self.input_right["far"], self.output_angle['right']),
            # the third rule
            ctrl.Rule(self.input_front['close'] & self.input_right["close"] & self.input_left["far"],
                      self.output_velocity['fast']),
            ctrl.Rule(self.input_front['close'] & self.input_right["close"] & self.input_left["far"],
                      self.output_angle['left']),
            # the fourth rule
            ctrl.Rule(self.input_front['close'] & self.input_right["close"] & self.input_left["close"],
                      self.output_velocity['fast']),
            ctrl.Rule(self.input_front['close'] & self.input_right["close"] & self.input_left["close"],
                      self.output_angle['right']),
        ]

    def build_inputs(self):
        # min_distance, max_distance = 10, 140
        min_distance, max_distance = 0, 70
        min_velocity, max_velocity = 0, 100
        input_front = ctrl.Antecedent(np.arange(min_distance, max_distance, self.step), 'input_front')
        input_left = ctrl.Antecedent(np.arange(min_distance, max_distance, self.step), 'input_left')
        input_right = ctrl.Antecedent(np.arange(min_distance, max_distance, self.step), 'input_right')
        input_velocity = ctrl.Antecedent(np.arange(min_velocity, max_velocity, self.step), 'input_velocity')

        input_front = self.tri_pi_tri(input_front, "close", [0, 0, 30, 40])
        input_front = self.tri_pi_tri(input_front, "far", [30, 40, 70, 70])

        input_left = self.tri_pi_tri(input_left, "close", [0, 0, 30, 40])
        input_left = self.tri_pi_tri(input_left, "far", [30, 40, 70, 70])

        input_right = self.tri_pi_tri(input_right, "close", [0, 0, 30, 40])
        input_right = self.tri_pi_tri(input_right, "far", [30, 40, 70, 70])

        input_velocity = self.tri_pi_tri(input_velocity, "slow", [0, 0, 5, 15])
        input_velocity = self.tri_pi_tri(input_velocity, "medium", [10, 20, 30, 40])
        input_velocity = self.tri_pi_tri(input_velocity, "fast", [35, 40, 50, 55])
        input_velocity = self.tri_pi_tri(input_velocity, "very_fast", [50, 60, 70, 80])
        input_velocity = self.tri_pi_tri(input_velocity, "stop", [70, 85, 100, 100])

        return input_front, input_left, input_right, input_velocity

    def build_outputs(self):
        min_velocity, max_velocity = 0, 100
        theta = 180
        output_velocity = ctrl.Consequent(np.arange(min_velocity, max_velocity, self.step), "output_velocity")
        output_angle = ctrl.Consequent(np.arange(-theta, theta, self.step), "output_angle")

        output_velocity = self.tri_pi_tri(output_velocity, "slow", [0, 0, 5, 15])
        output_velocity = self.tri_pi_tri(output_velocity, "medium", [10, 20, 30, 40])
        output_velocity = self.tri_pi_tri(output_velocity, "fast", [35, 40, 50, 55])
        output_velocity = self.tri_pi_tri(output_velocity, "very_fast", [50, 60, 70, 80])
        output_velocity = self.tri_pi_tri(output_velocity, "stop", [70, 85, 100, 100])

        output_angle["left"] = trimf(output_angle.universe, [-90, -90, 25])
        output_angle["front"] = trimf(output_angle.universe, [-25, 0, 25])
        output_angle["right"] = trimf(output_angle.universe, [-25, 90, 90])

        return output_velocity, output_angle

    @staticmethod
    def tri_pi_tri(var, term: str, values):
        var[term] = fuzzy_or(
            var.universe, trimf(var.universe, [values[0], values[1], values[1]]),
            var.universe, pimf(var.universe, values[1], values[1], values[2], values[2])
        )[1]

        var[term] = fuzzy_or(
            var.universe, var[term].mf,
            var.universe, trimf(var.universe, [values[2], values[2], values[3]])
        )[1]

        return var
