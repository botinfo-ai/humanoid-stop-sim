# A walking G1 under Unitree's published policy: five stops at 20 moments over one gait cycle (simulation)
numpy 2.0.2, mujoco 3.3.7
config g1.yaml: kps [100.0, 100.0, 100.0, 150.0, 40.0, 40.0, 100.0, 100.0, 100.0, 150.0, 40.0, 40.0] kds [2.0, 2.0, 2.0, 4.0, 2.0, 2.0, 2.0, 2.0, 2.0, 4.0, 2.0, 2.0]
default_angles [-0.1, 0.0, 0.0, 0.3, -0.2, 0.0, -0.1, 0.0, 0.0, 0.3, -0.2, 0.0] action_scale 0.25 cmd_init [0.5, 0.0, 0.0] cmd_scale [2.0, 2.0, 0.25] dt 0.002 decimation 10 obs 47
create_damping_cmd kd (command_helper.py): 8
torchscript archive: 31 entries, root 'policy_lstm_1', data.pkl policy_lstm_1/data.pkl
policy: LSTM 64 + actor 64 -> 32 -> 12 (ELU between layers)
model: nq 19, nu 12, mass 32.11 kg, timestep 0.002
colliding shapes on the pelvis body: ['head_link', 'left_elbow_link', 'left_shoulder_yaw_link', 'left_wrist_roll_rubber_hand', 'logo_link', 'pelvis cylinder #11', 'pelvis cylinder #13', 'pelvis cylinder #21', 'pelvis cylinder #23', 'pelvis_contour_link', 'right_elbow_link', 'right_shoulder_yaw_link', 'right_wrist_roll_rubber_hand', 'torso_link_23dof_rev_1_0']

control: walking with no stop to 8.76 s: non-foot floor contacts 0, upright at end True, end tilt 2.3 deg

## stops at 20 moments, 4.0-4.76 s (phase 0-0.95 of the policy's 0.8 s clock); 4 s after each stop
upright at end = pelvis z > 0.6 m and tilt < 30 deg at the last frame; speeds are the touching shape's centre
t stop | phase | feet down at stop | mode | first non-foot touch: shape, s, m/s | torso shape: s, m/s | head shape: s, m/s | upright at end | end pelvis z m | travel m
4.0 | 0.0 | right | stopmove | none | - | - | True | 0.779 | 0.199
4.0 | 0.0 | right | hold_last | right_wrist_roll_rubber_hand, 0.962, 3.0 | 1.496, 0.8 | - | False | 0.062 | 1.243
4.0 | 0.0 | right | hold_default | right_wrist_roll_rubber_hand, 0.928, 3.03 | - | - | False | 0.311 | 0.854
4.0 | 0.0 | right | damp_kd8 | left_knee_link, 0.722, 0.68 | 1.394, 0.91 | - | False | 0.063 | 1.005
4.0 | 0.0 | right | zero_torque | right_knee_link, 0.292, 1.55 | 1.732, 0.75 | - | False | 0.064 | 0.544
4.04 | 0.05 | left+right | stopmove | none | - | - | True | 0.777 | 0.21
4.04 | 0.05 | left+right | hold_last | right_knee_link, 1.026, 1.14 | 1.354, 0.7 | 1.39, 0.32 | False | 0.061 | 1.071
4.04 | 0.05 | left+right | hold_default | right_wrist_roll_rubber_hand, 0.908, 3.04 | - | - | False | 0.311 | 0.827
4.04 | 0.05 | left+right | damp_kd8 | left_knee_link, 0.794, 0.69 | 1.54, 0.86 | - | False | 0.062 | 0.967
4.04 | 0.05 | left+right | zero_torque | right_knee_link, 0.278, 1.58 | 1.532, 0.82 | - | False | 0.061 | 0.484
4.08 | 0.1 | left | stopmove | none | - | - | True | 0.778 | 0.223
4.08 | 0.1 | left | hold_last | right_knee_link, 0.982, 1.07 | 1.592, 0.96 | - | False | 0.061 | 1.229
4.08 | 0.1 | left | hold_default | right_wrist_roll_rubber_hand, 0.902, 3.03 | - | - | False | 0.311 | 0.838
4.08 | 0.1 | left | damp_kd8 | right_knee_link, 0.74, 0.51 | 1.82, 0.66 | - | False | 0.065 | 0.909
4.08 | 0.1 | left | zero_torque | right_knee_link, 0.272, 1.53 | 2.216, 0.89 | - | False | 0.062 | 0.273
4.12 | 0.15 | left | stopmove | none | - | - | True | 0.78 | 0.232
4.12 | 0.15 | left | hold_last | right_knee_link, 0.914, 1.07 | 1.472, 0.9 | - | False | 0.061 | 1.226
4.12 | 0.15 | left | hold_default | right_wrist_roll_rubber_hand, 0.908, 3.06 | - | - | False | 0.312 | 0.862
4.12 | 0.15 | left | damp_kd8 | right_knee_link, 0.694, 0.57 | 3.334, 0.33 | - | False | 0.062 | 0.894
4.12 | 0.15 | left | zero_torque | right_knee_link, 0.306, 1.6 | 1.686, 1.21 | - | False | 0.066 | 0.488
4.16 | 0.2 | left | stopmove | none | - | - | True | 0.781 | 0.238
4.16 | 0.2 | left | hold_last | right_knee_link, 0.902, 1.09 | 1.424, 0.93 | - | False | 0.061 | 1.2
4.16 | 0.2 | left | hold_default | left_wrist_roll_rubber_hand, 0.9, 3.08 | - | - | False | 0.313 | 0.871
4.16 | 0.2 | left | damp_kd8 | right_knee_link, 0.684, 0.62 | - | - | False | 0.063 | 0.938
4.16 | 0.2 | left | zero_torque | left_knee_link, 0.312, 1.45 | 1.838, 0.93 | - | False | 0.061 | 0.372
4.2 | 0.25 | left | stopmove | none | - | - | True | 0.781 | 0.236
4.2 | 0.25 | left | hold_last | right_knee_link, 0.88, 1.13 | 1.244, 0.76 | 1.29, 0.45 | False | 0.061 | 1.147
4.2 | 0.25 | left | hold_default | left_wrist_roll_rubber_hand, 0.902, 3.02 | - | - | False | 0.312 | 0.89
4.2 | 0.25 | left | damp_kd8 | right_knee_link, 0.684, 0.66 | - | 1.422, 0.74 | False | 0.061 | 0.975
4.2 | 0.25 | left | zero_torque | left_knee_link, 0.306, 1.5 | 1.776, 1.61 | - | False | 0.062 | 0.18
4.24 | 0.3 | left | stopmove | none | - | - | True | 0.781 | 0.229
4.24 | 0.3 | left | hold_last | right_knee_link, 0.854, 1.12 | 1.17, 0.85 | 1.226, 0.41 | False | 0.061 | 1.083
4.24 | 0.3 | left | hold_default | left_wrist_roll_rubber_hand, 0.914, 3.01 | - | - | False | 0.311 | 0.898
4.24 | 0.3 | left | damp_kd8 | right_knee_link, 0.682, 0.69 | 1.506, 0.31 | 1.342, 0.91 | False | 0.063 | 0.96
4.24 | 0.3 | left | zero_torque | left_knee_link, 0.302, 1.53 | 1.808, 1.19 | - | False | 0.07 | 0.285
4.28 | 0.35 | left | stopmove | none | - | - | True | 0.781 | 0.212
4.28 | 0.35 | left | hold_last | right_knee_link, 0.848, 1.09 | 1.162, 0.81 | 1.188, 0.54 | False | 0.061 | 1.085
4.28 | 0.35 | left | hold_default | left_wrist_roll_rubber_hand, 0.938, 3.02 | - | - | False | 0.312 | 0.91
4.28 | 0.35 | left | damp_kd8 | right_knee_link, 0.68, 0.71 | 1.318, 0.78 | - | False | 0.062 | 0.967
4.28 | 0.35 | left | zero_torque | left_knee_link, 0.3, 1.56 | 1.384, 0.42 | 1.384, 0.52 | False | 0.062 | 0.162
4.32 | 0.4 | left | stopmove | none | - | - | True | 0.78 | 0.195
4.32 | 0.4 | left | hold_last | right_knee_link, 0.868, 1.15 | 1.158, 0.95 | - | False | 0.061 | 1.084
4.32 | 0.4 | left | hold_default | left_wrist_roll_rubber_hand, 0.954, 3.03 | - | - | False | 0.311 | 0.9
4.32 | 0.4 | left | damp_kd8 | right_knee_link, 0.678, 0.71 | 1.328, 0.9 | - | False | 0.062 | 0.998
4.32 | 0.4 | left | zero_torque | left_knee_link, 0.298, 1.56 | 1.256, 2.21 | 1.26, 2.62 | False | 0.078 | 0.221
4.36 | 0.45 | left | stopmove | none | - | - | True | 0.779 | 0.183
4.36 | 0.45 | left | hold_last | left_wrist_roll_rubber_hand, 0.886, 3.12 | - | - | False | 0.305 | 0.992
4.36 | 0.45 | left | hold_default | left_wrist_roll_rubber_hand, 0.96, 3.03 | - | - | False | 0.312 | 0.9
4.36 | 0.45 | left | damp_kd8 | right_knee_link, 0.69, 0.7 | 1.37, 0.81 | - | False | 0.061 | 1.022
4.36 | 0.45 | left | zero_torque | left_knee_link, 0.294, 1.58 | 1.48, 0.72 | - | False | 0.062 | 0.581
4.4 | 0.5 | left | stopmove | none | - | - | True | 0.776 | 0.185
4.4 | 0.5 | left | hold_last | left_wrist_roll_rubber_hand, 0.956, 2.95 | 1.57, 0.77 | - | False | 0.061 | 1.221
4.4 | 0.5 | left | hold_default | left_wrist_roll_rubber_hand, 0.952, 3.01 | - | - | False | 0.312 | 0.896
4.4 | 0.5 | left | damp_kd8 | right_knee_link, 0.74, 0.7 | 1.416, 0.81 | - | False | 0.061 | 1.023
4.4 | 0.5 | left | zero_torque | left_knee_link, 0.282, 1.58 | 2.198, 1.04 | - | False | 0.064 | 0.492
4.44 | 0.55 | left+right | stopmove | none | - | - | True | 0.773 | 0.193
4.44 | 0.55 | left+right | hold_last | left_knee_link, 1.062, 1.25 | 1.396, 0.76 | 1.44, 0.36 | False | 0.061 | 1.122
4.44 | 0.55 | left+right | hold_default | left_wrist_roll_rubber_hand, 0.912, 3.01 | - | - | False | 0.312 | 0.85
4.44 | 0.55 | left+right | damp_kd8 | right_knee_link, 0.798, 0.72 | 1.576, 0.66 | - | False | 0.061 | 1.0
4.44 | 0.55 | left+right | zero_torque | left_knee_link, 0.268, 1.58 | 1.234, 0.77 | 1.234, 0.84 | False | 0.064 | 0.449
4.48 | 0.6 | right | stopmove | none | - | - | True | 0.775 | 0.2
4.48 | 0.6 | right | hold_last | left_wrist_roll_rubber_hand, 0.934, 3.14 | 1.442, 0.71 | - | False | 0.061 | 1.097
4.48 | 0.6 | right | hold_default | left_wrist_roll_rubber_hand, 0.928, 3.04 | - | - | False | 0.312 | 0.851
4.48 | 0.6 | right | damp_kd8 | left_knee_link, 0.738, 0.5 | 1.622, 0.68 | - | False | 0.065 | 1.038
4.48 | 0.6 | right | zero_torque | left_knee_link, 0.29, 1.57 | - | 1.482, 0.9 | False | 0.108 | 0.255
4.52 | 0.65 | right | stopmove | none | - | - | True | 0.776 | 0.205
4.52 | 0.65 | right | hold_last | left_wrist_roll_rubber_hand, 0.858, 3.17 | 1.408, 0.8 | - | False | 0.061 | 1.077
4.52 | 0.65 | right | hold_default | left_wrist_roll_rubber_hand, 0.922, 3.04 | - | - | False | 0.312 | 0.874
4.52 | 0.65 | right | damp_kd8 | left_knee_link, 0.7, 0.55 | - | - | False | 0.149 | 0.888
4.52 | 0.65 | right | zero_torque | left_knee_link, 0.306, 1.59 | 1.676, 1.08 | - | False | 0.062 | 0.434
4.56 | 0.7 | right | stopmove | none | - | - | True | 0.777 | 0.209
4.56 | 0.7 | right | hold_last | left_wrist_roll_rubber_hand, 0.842, 3.15 | 1.384, 0.82 | - | False | 0.061 | 1.077
4.56 | 0.7 | right | hold_default | left_wrist_roll_rubber_hand, 0.9, 3.07 | - | - | False | 0.313 | 0.866
4.56 | 0.7 | right | damp_kd8 | left_knee_link, 0.682, 0.58 | - | - | False | 0.296 | 0.822
4.56 | 0.7 | right | zero_torque | left_knee_link, 0.314, 1.51 | 1.69, 1.06 | - | False | 0.061 | 0.446
4.6 | 0.75 | right | stopmove | none | - | - | True | 0.778 | 0.21
4.6 | 0.75 | right | hold_last | left_knee_link, 0.848, 1.09 | 1.31, 0.73 | - | False | 0.061 | 1.088
4.6 | 0.75 | right | hold_default | left_wrist_roll_rubber_hand, 0.884, 3.13 | - | - | False | 0.312 | 0.864
4.6 | 0.75 | right | damp_kd8 | left_knee_link, 0.67, 0.61 | 2.86, 0.37 | - | False | 0.061 | 0.933
4.6 | 0.75 | right | zero_torque | right_knee_link, 0.318, 1.46 | 1.952, 0.94 | - | False | 0.062 | 0.481
4.64 | 0.8 | right | stopmove | none | - | - | True | 0.778 | 0.208
4.64 | 0.8 | right | hold_last | left_knee_link, 0.868, 1.09 | 1.228, 0.79 | 1.25, 0.45 | False | 0.061 | 1.095
4.64 | 0.8 | right | hold_default | right_wrist_roll_rubber_hand, 0.878, 3.05 | - | - | False | 0.312 | 0.864
4.64 | 0.8 | right | damp_kd8 | left_knee_link, 0.664, 0.63 | 1.634, 0.47 | - | False | 0.063 | 0.959
4.64 | 0.8 | right | zero_torque | right_knee_link, 0.312, 1.48 | 1.9, 1.03 | - | False | 0.064 | 0.295
4.68 | 0.85 | right | stopmove | none | - | - | True | 0.779 | 0.21
4.68 | 0.85 | right | hold_last | left_knee_link, 0.886, 1.09 | 1.192, 0.9 | 1.216, 0.52 | False | 0.062 | 1.085
4.68 | 0.85 | right | hold_default | right_wrist_roll_rubber_hand, 0.88, 3.02 | - | - | False | 0.312 | 0.871
4.68 | 0.85 | right | damp_kd8 | left_knee_link, 0.662, 0.66 | 1.43, 0.55 | - | False | 0.062 | 0.963
4.68 | 0.85 | right | zero_torque | right_knee_link, 0.306, 1.5 | 2.638, 1.12 | - | False | 0.061 | 0.395
4.72 | 0.9 | right | stopmove | none | - | - | True | 0.779 | 0.212
4.72 | 0.9 | right | hold_last | left_knee_link, 0.904, 1.15 | 1.176, 1.16 | 1.224, 0.59 | False | 0.061 | 1.08
4.72 | 0.9 | right | hold_default | right_wrist_roll_rubber_hand, 0.892, 3.03 | - | - | False | 0.311 | 0.852
4.72 | 0.9 | right | damp_kd8 | left_knee_link, 0.666, 0.67 | 1.346, 0.63 | - | False | 0.062 | 0.937
4.72 | 0.9 | right | zero_torque | right_knee_link, 0.304, 1.53 | 2.428, 0.91 | - | False | 0.062 | 0.393
4.76 | 0.95 | right | stopmove | none | - | - | True | 0.78 | 0.204
4.76 | 0.95 | right | hold_last | left_wrist_roll_rubber_hand, 0.906, 3.2 | 1.508, 1.33 | - | False | 0.062 | 1.018
4.76 | 0.95 | right | hold_default | right_wrist_roll_rubber_hand, 0.91, 3.03 | - | - | False | 0.311 | 0.861
4.76 | 0.95 | right | damp_kd8 | left_knee_link, 0.684, 0.68 | 1.376, 0.7 | - | False | 0.062 | 0.992
4.76 | 0.95 | right | zero_torque | right_knee_link, 0.3, 1.54 | 2.28, 0.73 | - | False | 0.063 | 0.224

## per mode, over all stop moments
mode | upright at end | non-foot touch (within 1.5 s) | first touch s: min / median / max | first touch m/s: min / median / max | first shapes | torso shape touched: n, s min-max, m/s max | head shape touched: n, m/s max
stopmove | 20/20 | 0 (0) | - | - | - | 0 | 0
hold_last | 0/20 | 20 (20) | 0.842 / 0.894 / 1.062 | 1.07 / 1.145 / 3.2 | right_wrist_roll_rubber_hand x1, right_knee_link x8, left_wrist_roll_rubber_hand x6, left_knee_link x5 | 19, 1.158-1.592, 1.33 | 8, 0.59
hold_default | 0/20 | 20 (20) | 0.878 / 0.909 / 0.96 | 3.01 / 3.03 / 3.13 | right_wrist_roll_rubber_hand x8, left_wrist_roll_rubber_hand x12 | 0 | 0
damp_kd8 | 0/20 | 20 (20) | 0.662 / 0.684 / 0.798 | 0.5 / 0.665 / 0.72 | left_knee_link x10, right_knee_link x10 | 16, 1.318-3.334, 0.91 | 2, 0.91
zero_torque | 0/20 | 20 (20) | 0.268 / 0.301 / 0.318 | 1.45 / 1.545 / 1.6 | right_knee_link x9, left_knee_link x11 | 19, 1.234-2.638, 2.21 | 4, 2.62
