import skfuzzy.control as ctrl

from fuzzy_system.moo_fuzzy_controller import MooFuzzyController


class ObstacleAvoidanceController(MooFuzzyController):

    def validate(self, values):
        t = 1.5
        temp = super().validate(values)
        temp2 = (values["input_dl"] <= t or values["input_df"] <= t or values["input_dr"] <= t)
        return temp and temp2

    def inputs(self, dl, df, dr, a, p, ed):
        return {
            "input_dl": dl,
            "input_df": df,
            "input_dr": dr,
            "input_a": a
        }

    def build_rules(self):
        return [
            # ctrl.Rule(self.input_dl['N'] & self.input_df['N'] & self.input_dr['F'], self.output_u['S']),
            # ctrl.Rule(self.input_dl['N'] & self.input_df['N'] & self.input_dr['F'], self.output_w['PO']),
            ctrl.Rule(self.input_dl['N'] & self.input_df['F'] & self.input_dr['N'], self.output_u['M']),
            ctrl.Rule(self.input_dl['N'] & self.input_df['F'] & self.input_dr['N'], self.output_w['ZO']),
            ctrl.Rule(self.input_dl['F'] & self.input_df['N'] & self.input_dr['N'], self.output_u['M']),
            ctrl.Rule(self.input_dl['F'] & self.input_df['N'] & self.input_dr['N'], self.output_w['PO']),
            ctrl.Rule(self.input_dl['F'] & self.input_df['N'] & self.input_dr['N'], self.output_w['LPO']),
            ctrl.Rule(self.input_dl['M'] & self.input_df['N'] & self.input_dr['N'], self.output_u['M']),
            ctrl.Rule(self.input_dl['M'] & self.input_df['N'] & self.input_dr['N'], self.output_w['PO']),
            ctrl.Rule(self.input_dl['F'] & self.input_df['N'] & self.input_dr['M'], self.output_u['M']),
            ctrl.Rule(self.input_dl['F'] & self.input_df['N'] & self.input_dr['M'], self.output_w['LPO']),
            ctrl.Rule(self.input_dl['N'] & self.input_df['N'] & self.input_dr['F'], self.output_u['M']),
            ctrl.Rule(self.input_dl['N'] & self.input_df['N'] & self.input_dr['F'], self.output_w['LNO']),
            ctrl.Rule(self.input_dl['N'] & self.input_df['N'] & self.input_dr['M'], self.output_u['M']),
            ctrl.Rule(self.input_dl['N'] & self.input_df['N'] & self.input_dr['M'], self.output_w['NO']),
            ctrl.Rule(self.input_dl['F'] & self.input_df['N'] & self.input_dr['F'], self.output_u['M']),
            ctrl.Rule(self.input_dl['F'] & self.input_df['N'] & self.input_dr['F'], self.output_w['LPO']),
            ctrl.Rule(self.input_dl['M'] & self.input_df['N'] & self.input_dr['F'], self.output_u['M']),
            ctrl.Rule(self.input_dl['M'] & self.input_df['N'] & self.input_dr['F'], self.output_w['NO']),
            ctrl.Rule(self.input_dl['F'] & self.input_df['N'] & self.input_dr['M'], self.output_u['M']),
            ctrl.Rule(self.input_dl['F'] & self.input_df['N'] & self.input_dr['M'], self.output_w['PO']),
            ctrl.Rule(self.input_dl['N'] & self.input_df['M'] & self.input_dr['N'], self.output_u['M']),
            ctrl.Rule(self.input_dl['N'] & self.input_df['M'] & self.input_dr['N'], self.output_w['PO']),
            ctrl.Rule(self.input_dl['N'] & self.input_df['N'] & self.input_dr['N'], self.output_u['S']),
            ctrl.Rule(self.input_dl['N'] & self.input_df['N'] & self.input_dr['N'], self.output_w['PO']),
            ctrl.Rule(self.input_dl['M'] & self.input_df['N'] & self.input_dr['M'], self.output_u['S']),
            ctrl.Rule(self.input_dl['M'] & self.input_df['N'] & self.input_dr['M'], self.output_w['PO']),
            ctrl.Rule(self.input_dl['N'] & self.input_df['M'] & self.input_dr['M'], self.output_u['M']),
            ctrl.Rule(self.input_dl['N'] & self.input_df['M'] & self.input_dr['M'], self.output_w['NO']),
            ctrl.Rule(self.input_dl['N'] & self.input_df['M'] & self.input_dr['F'], self.output_u['M']),
            ctrl.Rule(self.input_dl['N'] & self.input_df['M'] & self.input_dr['F'], self.output_w['NO']),
            ctrl.Rule(self.input_dl['N'] & self.input_df['F'] & self.input_dr['M'], self.output_u['M']),
            # ctrl.Rule(self.input_dl['N'] & self.input_df['F'] & self.input_dr['M'], self.output_w['NO']),
            ctrl.Rule(self.input_dl['N'] & self.input_df['F'] & self.input_dr['M'], self.output_w['ZO']),
            ctrl.Rule(self.input_dl['N'] & self.input_df['F'] & self.input_dr['F'] & self.input_a['LN'],
                      self.output_u['S']),
            ctrl.Rule(self.input_dl['N'] & self.input_df['F'] & self.input_dr['F'] & self.input_a['LN'],
                      self.output_w['LNO']),
            ctrl.Rule(self.input_dl['N'] & self.input_df['F'] & self.input_dr['F'] & self.input_a['N'],
                      self.output_u['S']),
            ctrl.Rule(self.input_dl['N'] & self.input_df['F'] & self.input_dr['F'] & self.input_a['N'],
                      self.output_w['NO']),
            ctrl.Rule(self.input_dl['N'] & self.input_df['F'] & self.input_dr['F'] & self.input_a['Z'],
                      self.output_u['L']),
            ctrl.Rule(self.input_dl['N'] & self.input_df['F'] & self.input_dr['F'] & self.input_a['Z'],
                      self.output_w['ZO']),
            # ctrl.Rule(self.input_dl['N'] & self.input_df['N'] & self.input_dr['N'] & self.input_a['LP'],
            #           self.output_u['L']),
            # ctrl.Rule(self.input_dl['N'] & self.input_df['N'] & self.input_dr['N'] & self.input_a['LP'],
            #           self.output_w['ZO']),
            ctrl.Rule(self.input_dl['N'] & self.input_df['N'] & self.input_dr['N'] & self.input_a['LP'],
                      self.output_u['S']),
            ctrl.Rule(self.input_dl['N'] & self.input_df['N'] & self.input_dr['N'] & self.input_a['LP'],
                      self.output_w['PO']),
            ctrl.Rule(self.input_dl['N'] & self.input_df['F'] & self.input_dr['F'] & self.input_a['P'],
                      self.output_u['L']),
            ctrl.Rule(self.input_dl['N'] & self.input_df['F'] & self.input_dr['F'] & self.input_a['P'],
                      self.output_w['ZO']),
            ctrl.Rule(self.input_dl['M'] & self.input_df['M'] & self.input_dr['N'], self.output_u['M']),
            ctrl.Rule(self.input_dl['M'] & self.input_df['M'] & self.input_dr['N'], self.output_w['PO']),
            ctrl.Rule(self.input_dl['F'] & self.input_df['M'] & self.input_dr['N'], self.output_u['M']),
            ctrl.Rule(self.input_dl['F'] & self.input_df['M'] & self.input_dr['N'], self.output_w['PO']),
            ctrl.Rule(self.input_dl['M'] & self.input_df['F'] & self.input_dr['N'], self.output_u['M']),
            # ctrl.Rule(self.input_dl['M'] & self.input_df['F'] & self.input_dr['N'], self.output_w['PO']),
            ctrl.Rule(self.input_dl['M'] & self.input_df['F'] & self.input_dr['N'], self.output_w['ZO']),
            ctrl.Rule(self.input_dl['F'] & self.input_df['F'] & self.input_dr['N'] & self.input_a['LN'],
                      self.output_u['L']),
            ctrl.Rule(self.input_dl['F'] & self.input_df['F'] & self.input_dr['N'] & self.input_a['LN'],
                      self.output_w['ZO']),
            ctrl.Rule(self.input_dl['F'] & self.input_df['F'] & self.input_dr['N'] & self.input_a['N'],
                      self.output_u['L']),
            ctrl.Rule(self.input_dl['F'] & self.input_df['F'] & self.input_dr['N'] & self.input_a['N'],
                      self.output_w['ZO']),
            ctrl.Rule(self.input_dl['F'] & self.input_df['F'] & self.input_dr['N'] & self.input_a['Z'],
                      self.output_u['L']),
            ctrl.Rule(self.input_dl['F'] & self.input_df['F'] & self.input_dr['N'] & self.input_a['Z'],
                      self.output_w['ZO']),
            ctrl.Rule(self.input_dl['F'] & self.input_df['F'] & self.input_dr['N'] & self.input_a['LP'],
                      self.output_u['S']),
            ctrl.Rule(self.input_dl['F'] & self.input_df['F'] & self.input_dr['N'] & self.input_a['LP'],
                      self.output_w['LPO']),
            ctrl.Rule(self.input_dl['F'] & self.input_df['F'] & self.input_dr['N'] & self.input_a['P'],
                      self.output_u['S']),
            ctrl.Rule(self.input_dl['F'] & self.input_df['F'] & self.input_dr['N'] & self.input_a['P'],
                      self.output_w['LPO']),
        ]
