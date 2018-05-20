import numpy as np
import skfuzzy.control as ctrl

from fuzzy_system.fuzzy_system import FuzzySystem


class SimpleFuzzySystem(FuzzySystem):

    def __init__(self, step=0.01):
        self.step = step
        self.rules = self.build_rules()
        self.input_front, self.input_left, self.input_right, self.input_velocity = self.build_inputs()
        self.output_velocity, self.output_angle = self.build_outputs()

    def run(self, values: dict):
        try:
            values = {f"input_{k}": v for k, v in values.items()}
            controller = ctrl.ControlSystem(self.rules)
            controller = ctrl.ControlSystemSimulation(controller)
            controller.inputs(values)
            controller.compute()
            return controller.output["output_velocity"], controller.output["output_angle"]
        except:
            return 50, 0

    def build_rules(self):
        return [
            # the first rule
            ctrl.Rule(self.input_front['far'], self.output_velocity['fast']),
            ctrl.Rule(self.input_front['far'], self.output_angle['front']),
            # the second rule
            ctrl.Rule(self.input_front['close'] & self.input_right["far"], self.output_velocity['fast']),
            ctrl.Rule(self.input_front['close'] & self.input_right["far"], self.output_angle['front']),
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
        min_distance, max_distance = 10, 140
        min_velocity, max_velocity = 0, 100
        input_front = ctrl.Antecedent(np.arange(min_distance, max_distance, self.step), 'input_front')
        input_left = ctrl.Antecedent(np.arange(min_distance, max_distance, self.step), 'input_left')
        input_right = ctrl.Antecedent(np.arange(min_distance, max_distance, self.step), 'input_right')
        input_velocity = ctrl.Antecedent(np.arange(min_velocity, max_velocity, self.step), 'input_velocity')
        return input_front, input_left, input_right, input_velocity

    def build_outputs(self):
        min_velocity, max_velocity = 0, 100
        theta = 180
        output_velocity = ctrl.Consequent(np.arange(min_velocity, max_velocity, self.step), "output_velocity")
        output_angle = ctrl.Consequent(np.arange(-theta, theta, self.step), "output_angle")
        return output_velocity, output_angle
