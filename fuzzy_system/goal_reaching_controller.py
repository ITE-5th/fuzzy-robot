import skfuzzy.control as ctrl

from fuzzy_system.moo_fuzzy_controller import MooFuzzyController


class GoalReachingController(MooFuzzyController):

    def inputs(self, dl, df, dr, a, p, ed):
        return {
            "input_p": p,
            "input_a": a
        }

    def build_rules(self):
        return [
            ctrl.Rule(self.input_p['N'] & self.input_a['Z'], self.output_u['S']),
            ctrl.Rule(self.input_p['N'] & self.input_a['Z'], self.output_w['ZO']),
            ctrl.Rule(self.input_p['N'] & self.input_a['N'], self.output_u['S']),
            ctrl.Rule(self.input_p['N'] & self.input_a['N'], self.output_w['NO']),
            ctrl.Rule(self.input_p['N'] & self.input_a['LN'], self.output_u['S']),
            ctrl.Rule(self.input_p['N'] & self.input_a['LN'], self.output_w['LNO']),
            ctrl.Rule(self.input_p['N'] & self.input_a['P'], self.output_u['S']),
            ctrl.Rule(self.input_p['N'] & self.input_a['P'], self.output_w['PO']),
            ctrl.Rule(self.input_p['N'] & self.input_a['LP'], self.output_u['S']),
            ctrl.Rule(self.input_p['N'] & self.input_a['LP'], self.output_w['LPO']),
            # ctrl.Rule(self.input_p['M'] & self.input_a['Z'], self.output_u['M']),
            # ctrl.Rule(self.input_p['M'] & self.input_a['Z'], self.output_w['ZO']),
            ctrl.Rule(self.input_p['M'] & self.input_a['Z'], self.output_u['S']),
            ctrl.Rule(self.input_p['M'] & self.input_a['Z'], self.output_w['PO']),
            ctrl.Rule(self.input_p['M'] & self.input_a['N'], self.output_u['M']),
            ctrl.Rule(self.input_p['M'] & self.input_a['N'], self.output_w['NO']),
            ctrl.Rule(self.input_p['M'] & self.input_a['LN'], self.output_u['M']),
            ctrl.Rule(self.input_p['M'] & self.input_a['LN'], self.output_w['LNO']),
            ctrl.Rule(self.input_p['M'] & self.input_a['P'], self.output_u['M']),
            ctrl.Rule(self.input_p['M'] & self.input_a['P'], self.output_w['PO']),
            ctrl.Rule(self.input_p['M'] & self.input_a['LP'], self.output_u['M']),
            ctrl.Rule(self.input_p['M'] & self.input_a['LP'], self.output_w['LPO']),
            ctrl.Rule(self.input_p['F'] & self.input_a['Z'], self.output_u['L']),
            ctrl.Rule(self.input_p['F'] & self.input_a['Z'], self.output_w['ZO']),
            ctrl.Rule(self.input_p['F'] & self.input_a['N'], self.output_u['L']),
            ctrl.Rule(self.input_p['F'] & self.input_a['N'], self.output_w['NO']),
            ctrl.Rule(self.input_p['F'] & self.input_a['LN'], self.output_u['L']),
            ctrl.Rule(self.input_p['F'] & self.input_a['LN'], self.output_w['LNO']),
            ctrl.Rule(self.input_p['F'] & self.input_a['P'], self.output_u['L']),
            ctrl.Rule(self.input_p['F'] & self.input_a['P'], self.output_w['PO']),
            ctrl.Rule(self.input_p['F'] & self.input_a['LP'], self.output_u['L']),
            ctrl.Rule(self.input_p['F'] & self.input_a['LP'], self.output_w['LPO']),
        ]
