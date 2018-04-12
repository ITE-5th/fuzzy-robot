from math import sqrt, atan2

import numpy as np
import skfuzzy.control as ctrl
from skfuzzy.control import ControlSystem
from skfuzzy.membership import *

step = 0.001

# wheel diameter
R = 0.085

# Distance between 2 wheels
L = 0.265

# robot position and orientation
x, y, theta = 1000, 2000, 90

# target position and orientation
x_d, y_d, theta_d = 5000, 10000, 20

# Distance from the center of the robot to the target, in [0, 20]
p = sqrt(pow(x_d - x, 2) + pow(y_d - y, 2))
prev_p = 0

# angle between the robot heading and the vector connecting the robot center with the target, alpha in [-pi, +pi]
a = atan2(y_d - y, x_d - x) - theta

# the movement of the robot with the target, in [-1, 1]
e_d = p - prev_p

# u Linear velocity, w angular velocity
# u in [0, 1.3] m/s
# w in [-4.3, 4.3] rad/s

# distance to obstacles measured by sensor i
d = [1, 2, 3, 4, 5, 6, 7, 8]
dr = min(d[0], d[1], d[2])
df = min(d[3], d[4])
dl = min(d[5], d[6], d[7])

# i) reaching the target from an arbitrary position
# ii) avoiding obstacles
# iii) escaping local minimum regions


# Inputs of Obstacle Avoidance
input_dl = ctrl.Antecedent(np.arange(0, 4, step), "input_dl")  # meters
input_df = ctrl.Antecedent(np.arange(0, 4, step), "input_df")  # meters
input_dr = ctrl.Antecedent(np.arange(0, 4, step), "input_dr")  # meters
input_a = ctrl.Antecedent(np.arange(-4, +4, step), "input_a")  # rad
input_p = ctrl.Antecedent(np.arange(0, 20, step), "input_p")  # meters
input_ed = ctrl.Antecedent(np.arange(-1, 1, step), "input_ed")  # meters

input_dl["N"] = zmf(input_dl.universe, 0.22, 1.22)
input_df["N"] = zmf(input_df.universe, 0.22, 1.22)
input_dr["N"] = zmf(input_dr.universe, 0.22, 1.22)

input_dl["M"] = gaussmf(input_dl.universe, 0.88, 0.25)
input_df["M"] = gaussmf(input_df.universe, 0.88, 0.25)
input_dr["M"] = gaussmf(input_dr.universe, 0.88, 0.25)

input_dl["F"] = smf(input_dl.universe, 0.65, 1.7)
input_df["F"] = smf(input_df.universe, 0.65, 1.7)
input_dr["F"] = smf(input_dr.universe, 0.65, 1.7)

input_a["LN"] = zmf(input_a.universe, -2.88, -0.92)
input_a["N"] = gaussmf(input_a.universe, -0.95, 0.4)
input_a["Z"] = gaussmf(input_a.universe, 0, 0.15)
input_a["P"] = gaussmf(input_a.universe, 0.95, 0.4)
input_a["LP"] = smf(input_a.universe, 0.92, 2.88)

input_p['N'] = zmf(input_p.universe, 0, 3.5)
input_p['M'] = gaussmf(input_p.universe, 2, 0.6)
input_p['F'] = smf(input_p.universe, 0, 4)

input_ed['NT'] = zmf(input_ed.universe, -0.1, 0.05)
input_ed['ZT'] = gaussmf(input_ed.universe, 0, 0.015)
input_ed['PT'] = smf(input_ed.universe, -0.05, 0.1)

# input_dl.view()
# input_df.view()
# input_dr.view()
# input_p.view()
input_ed.view()

# Outputs of Obstacle Avoidance
output_u = ctrl.Consequent(np.arange(0, 2, step), "output_u")  # m/s
output_w = ctrl.Consequent(np.arange(-5, +5, step), "output_w")  # rad/s

output_u["S"] = zmf(output_u.universe, 0.16, 0.6)
output_u["M"] = gaussmf(output_u.universe, 0.5, 0.12)
output_u["L"] = smf(output_u.universe, 0.42, 0.95)
# oa_u.view()

output_w["LNO"] = zmf(output_w.universe, -4, -1.5)
output_w["NO"] = gaussmf(output_w.universe, -1.8, 0.6)
output_w["ZO"] = gaussmf(output_w.universe, 0.1, 0.2)
output_w["PO"] = gaussmf(output_w.universe, 1.8, 0.6)
output_w["LPO"] = smf(output_w.universe, 1.5, 4)
# oa_w.view()

oa_rules = [
    ctrl.Rule(input_dl['N'] & input_df['N'] & input_dr['F'], output_u['S']),
    ctrl.Rule(input_dl['N'] & input_df['N'] & input_dr['F'], output_w['PO']),
    ctrl.Rule(input_dl['F'] & input_df['N'] & input_dr['N'], output_u['M']),
    ctrl.Rule(input_dl['F'] & input_df['N'] & input_dr['N'], output_w['PO']),
    ctrl.Rule(input_dl['M'] & input_df['N'] & input_dr['N'], output_u['M']),
    ctrl.Rule(input_dl['M'] & input_df['N'] & input_dr['N'], output_w['PO']),
    ctrl.Rule(input_dl['F'] & input_df['N'] & input_dr['M'], output_u['M']),
    ctrl.Rule(input_dl['F'] & input_df['N'] & input_dr['M'], output_w['LPO']),
    ctrl.Rule(input_dl['N'] & input_df['N'] & input_dr['F'], output_u['M']),
    ctrl.Rule(input_dl['N'] & input_df['N'] & input_dr['F'], output_w['LNO']),
    ctrl.Rule(input_dl['N'] & input_df['N'] & input_dr['M'], output_u['M']),
    ctrl.Rule(input_dl['N'] & input_df['N'] & input_dr['M'], output_w['NO']),
    ctrl.Rule(input_dl['F'] & input_df['N'] & input_dr['F'], output_u['M']),
    ctrl.Rule(input_dl['F'] & input_df['N'] & input_dr['F'], output_w['LPO']),
    ctrl.Rule(input_dl['M'] & input_df['N'] & input_dr['F'], output_u['M']),
    ctrl.Rule(input_dl['M'] & input_df['N'] & input_dr['F'], output_w['NO']),
    ctrl.Rule(input_dl['F'] & input_df['N'] & input_dr['M'], output_u['M']),
    ctrl.Rule(input_dl['F'] & input_df['N'] & input_dr['M'], output_w['PO']),
    ctrl.Rule(input_dl['N'] & input_df['M'] & input_dr['N'], output_u['M']),
    ctrl.Rule(input_dl['N'] & input_df['M'] & input_dr['N'], output_w['PO']),
    ctrl.Rule(input_dl['N'] & input_df['N'] & input_dr['N'], output_u['S']),
    ctrl.Rule(input_dl['N'] & input_df['N'] & input_dr['N'], output_w['PO']),
    ctrl.Rule(input_dl['M'] & input_df['N'] & input_dr['M'], output_u['S']),
    ctrl.Rule(input_dl['M'] & input_df['N'] & input_dr['M'], output_w['PO']),
    ctrl.Rule(input_dl['N'] & input_df['M'] & input_dr['M'], output_u['M']),
    ctrl.Rule(input_dl['N'] & input_df['M'] & input_dr['M'], output_w['NO']),
    ctrl.Rule(input_dl['N'] & input_df['M'] & input_dr['F'], output_u['M']),
    ctrl.Rule(input_dl['N'] & input_df['M'] & input_dr['F'], output_w['NO']),
    ctrl.Rule(input_dl['N'] & input_df['F'] & input_dr['M'], output_u['M']),
    ctrl.Rule(input_dl['N'] & input_df['F'] & input_dr['M'], output_w['NO']),
    ctrl.Rule(input_dl['N'] & input_df['F'] & input_dr['F'] & input_a['LN'], output_u['S']),
    ctrl.Rule(input_dl['N'] & input_df['F'] & input_dr['F'] & input_a['LN'], output_w['LNO']),
    ctrl.Rule(input_dl['N'] & input_df['F'] & input_dr['F'] & input_a['N'], output_u['S']),
    ctrl.Rule(input_dl['N'] & input_df['F'] & input_dr['F'] & input_a['N'], output_w['NO']),
    ctrl.Rule(input_dl['N'] & input_df['F'] & input_dr['F'] & input_a['Z'], output_u['L']),
    ctrl.Rule(input_dl['N'] & input_df['F'] & input_dr['F'] & input_a['Z'], output_w['ZO']),
    ctrl.Rule(input_dl['N'] & input_df['N'] & input_dr['N'] & input_a['LP'], output_u['L']),
    ctrl.Rule(input_dl['N'] & input_df['N'] & input_dr['N'] & input_a['LP'], output_w['ZO']),
    ctrl.Rule(input_dl['N'] & input_df['F'] & input_dr['F'] & input_a['P'], output_u['L']),
    ctrl.Rule(input_dl['N'] & input_df['F'] & input_dr['F'] & input_a['P'], output_w['ZO']),
    ctrl.Rule(input_dl['M'] & input_df['M'] & input_dr['N'], output_u['M']),
    ctrl.Rule(input_dl['M'] & input_df['M'] & input_dr['N'], output_w['PO']),
    ctrl.Rule(input_dl['F'] & input_df['M'] & input_dr['N'], output_u['M']),
    ctrl.Rule(input_dl['F'] & input_df['M'] & input_dr['N'], output_w['PO']),
    ctrl.Rule(input_dl['M'] & input_df['F'] & input_dr['N'], output_u['M']),
    ctrl.Rule(input_dl['M'] & input_df['F'] & input_dr['N'], output_w['PO']),
    ctrl.Rule(input_dl['F'] & input_df['F'] & input_dr['N'] & input_a['LN'], output_u['L']),
    ctrl.Rule(input_dl['F'] & input_df['F'] & input_dr['N'] & input_a['LN'], output_w['ZO']),
    ctrl.Rule(input_dl['F'] & input_df['F'] & input_dr['N'] & input_a['N'], output_u['L']),
    ctrl.Rule(input_dl['F'] & input_df['F'] & input_dr['N'] & input_a['N'], output_w['ZO']),
    ctrl.Rule(input_dl['F'] & input_df['F'] & input_dr['N'] & input_a['Z'], output_u['L']),
    ctrl.Rule(input_dl['F'] & input_df['F'] & input_dr['N'] & input_a['Z'], output_w['ZO']),
    ctrl.Rule(input_dl['F'] & input_df['F'] & input_dr['N'] & input_a['LP'], output_u['S']),
    ctrl.Rule(input_dl['F'] & input_df['F'] & input_dr['N'] & input_a['LP'], output_w['LPO']),
    ctrl.Rule(input_dl['F'] & input_df['F'] & input_dr['N'] & input_a['P'], output_u['S']),
    ctrl.Rule(input_dl['F'] & input_df['F'] & input_dr['N'] & input_a['P'], output_w['LPO']),
]

gr_rules = [
    ctrl.Rule(input_p['N'] & input_a['Z'], output_u['S']),
    ctrl.Rule(input_p['N'] & input_a['Z'], output_w['ZO']),
    ctrl.Rule(input_p['N'] & input_a['N'], output_u['S']),
    ctrl.Rule(input_p['N'] & input_a['N'], output_w['NO']),
    ctrl.Rule(input_p['N'] & input_a['LN'], output_u['S']),
    ctrl.Rule(input_p['N'] & input_a['LN'], output_w['LNO']),
    ctrl.Rule(input_p['N'] & input_a['P'], output_u['S']),
    ctrl.Rule(input_p['N'] & input_a['P'], output_w['PO']),
    ctrl.Rule(input_p['N'] & input_a['LP'], output_u['S']),
    ctrl.Rule(input_p['N'] & input_a['LP'], output_w['LPO']),
    ctrl.Rule(input_p['M'] & input_a['Z'], output_u['M']),
    ctrl.Rule(input_p['M'] & input_a['Z'], output_w['ZO']),
    ctrl.Rule(input_p['M'] & input_a['N'], output_u['M']),
    ctrl.Rule(input_p['M'] & input_a['N'], output_w['NO']),
    ctrl.Rule(input_p['M'] & input_a['LN'], output_u['M']),
    ctrl.Rule(input_p['M'] & input_a['LN'], output_w['LNO']),
    ctrl.Rule(input_p['M'] & input_a['P'], output_u['M']),
    ctrl.Rule(input_p['M'] & input_a['P'], output_w['PO']),
    ctrl.Rule(input_p['M'] & input_a['LP'], output_u['M']),
    ctrl.Rule(input_p['M'] & input_a['LP'], output_w['LPO']),
    ctrl.Rule(input_p['F'] & input_a['Z'], output_u['L']),
    ctrl.Rule(input_p['F'] & input_a['Z'], output_w['ZO']),
    ctrl.Rule(input_p['F'] & input_a['N'], output_u['L']),
    ctrl.Rule(input_p['F'] & input_a['N'], output_w['NO']),
    ctrl.Rule(input_p['F'] & input_a['LN'], output_u['L']),
    ctrl.Rule(input_p['F'] & input_a['LN'], output_w['LNO']),
    ctrl.Rule(input_p['F'] & input_a['P'], output_u['L']),
    ctrl.Rule(input_p['F'] & input_a['P'], output_w['PO']),
    ctrl.Rule(input_p['F'] & input_a['LP'], output_u['L']),
    ctrl.Rule(input_p['F'] & input_a['LP'], output_w['LPO']),
]

lm_rules = [
    ctrl.Rule(input_dl['N'] & input_df['N'] & input_dr['N'] & input_a['Z'], output_u['S']),
    ctrl.Rule(input_dl['N'] & input_df['N'] & input_dr['N'] & input_a['Z'], output_w['PO']),
    ctrl.Rule(input_dl['F'] & input_df['N'] & input_dr['N'] & input_a['P'], output_u['M']),
    ctrl.Rule(input_dl['F'] & input_df['N'] & input_dr['N'] & input_a['P'], output_w['PO']),
    ctrl.Rule(input_dl['F'] & input_df['N'] & input_dr['N'] & input_a['LP'], output_u['S']),
    ctrl.Rule(input_dl['F'] & input_df['N'] & input_dr['N'] & input_a['LP'], output_w['LPO']),
    ctrl.Rule(input_dl['F'] & input_df['F'] & input_dr['N'] & input_ed['PT'] & input_a['P'], output_u['M']),
    ctrl.Rule(input_dl['F'] & input_df['F'] & input_dr['N'] & input_ed['PT'] & input_a['P'], output_w['ZO']),
    ctrl.Rule(input_dl['F'] & input_df['F'] & input_dr['N'] & input_ed['PT'] & input_a['LP'], output_u['M']),
    ctrl.Rule(input_dl['F'] & input_df['F'] & input_dr['N'] & input_ed['PT'] & input_a['LP'], output_w['ZO']),
    ctrl.Rule(input_dl['F'] & input_df['F'] & input_dr['F'] & input_ed['PT'] & input_a['P'], output_u['M']),
    ctrl.Rule(input_dl['F'] & input_df['F'] & input_dr['F'] & input_ed['PT'] & input_a['P'], output_w['ZO']),
    ctrl.Rule(input_dl['F'] & input_df['F'] & input_dr['F'] & input_ed['PT'] & input_a['LP'], output_u['M']),
    ctrl.Rule(input_dl['F'] & input_df['F'] & input_dr['F'] & input_ed['PT'] & input_a['LP'], output_w['LNO']),
]

# oa_cs = ControlSystem(oa_rules)
# simulation = ctrl.ControlSystemSimulation(oa_cs)

gr_cs = ControlSystem(gr_rules)
# simulation = ctrl.ControlSystemSimulation(gr_cs)

lm_cs = ControlSystem(lm_rules)
gr_cs.view()
