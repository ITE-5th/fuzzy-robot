import skfuzzy.control as ctrl

from fuzzy_system.moo_fuzzy_controller import MooFuzzyController


class LocalMinimumAvoidanceController(MooFuzzyController):

    def inputs(self, dl, df, dr, a, p, ed):
        return {
            "input_dl": dl,
            "input_df": df,
            "input_dr": dr,
            "input_a": a,
            "input_ed": ed
        }

    def build_rules(self):
        return [
            ctrl.Rule(self.input_dl['N'] & self.input_df['N'] & self.input_dr['N'] & self.input_a['Z'],
                      self.output_u['S']),
            ctrl.Rule(self.input_dl['N'] & self.input_df['N'] & self.input_dr['N'] & self.input_a['Z'],
                      self.output_w['PO']),
            ctrl.Rule(self.input_dl['F'] & self.input_df['N'] & self.input_dr['N'] & self.input_a['P'],
                      self.output_u['M']),
            ctrl.Rule(self.input_dl['F'] & self.input_df['N'] & self.input_dr['N'] & self.input_a['P'],
                      self.output_w['PO']),
            ctrl.Rule(self.input_dl['F'] & self.input_df['N'] & self.input_dr['N'] & self.input_a['LP'],
                      self.output_u['S']),
            ctrl.Rule(self.input_dl['F'] & self.input_df['N'] & self.input_dr['N'] & self.input_a['LP'],
                      self.output_w['LPO']),
            ctrl.Rule(
                self.input_dl['F'] & self.input_df['F'] & self.input_dr['N'] & self.input_ed['PT'] & self.input_a['P'],
                self.output_u['M']),
            ctrl.Rule(
                self.input_dl['F'] & self.input_df['F'] & self.input_dr['N'] & self.input_ed['PT'] & self.input_a['P'],
                self.output_w['ZO']),
            ctrl.Rule(
                self.input_dl['F'] & self.input_df['F'] & self.input_dr['N'] & self.input_ed['PT'] & self.input_a['LP'],
                self.output_u['M']),
            ctrl.Rule(
                self.input_dl['F'] & self.input_df['F'] & self.input_dr['N'] & self.input_ed['PT'] & self.input_a['LP'],
                self.output_w['ZO']),
            ctrl.Rule(
                self.input_dl['F'] & self.input_df['F'] & self.input_dr['F'] & self.input_ed['PT'] & self.input_a['P'],
                self.output_u['M']),
            ctrl.Rule(
                self.input_dl['F'] & self.input_df['F'] & self.input_dr['F'] & self.input_ed['PT'] & self.input_a['P'],
                self.output_w['ZO']),
            ctrl.Rule(
                self.input_dl['F'] & self.input_df['F'] & self.input_dr['F'] & self.input_ed['PT'] & self.input_a['LP'],
                self.output_u['M']),
            ctrl.Rule(
                self.input_dl['F'] & self.input_df['F'] & self.input_dr['F'] & self.input_ed['PT'] & self.input_a['LP'],
                self.output_w['LNO']),
        ]
