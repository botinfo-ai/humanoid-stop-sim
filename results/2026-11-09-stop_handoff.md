# A walking G1 under Unitree's published policy: zero velocity, then a hand-off to PD gains alone (simulation)
numpy 2.0.2, mujoco 3.3.7
config g1.yaml: kps [100.0, 100.0, 100.0, 150.0, 40.0, 40.0, 100.0, 100.0, 100.0, 150.0, 40.0, 40.0] kds [2.0, 2.0, 2.0, 4.0, 2.0, 2.0, 2.0, 2.0, 2.0, 4.0, 2.0, 2.0]
default_angles [-0.1, 0.0, 0.0, 0.3, -0.2, 0.0, -0.1, 0.0, 0.0, 0.3, -0.2, 0.0] action_scale 0.25 cmd_init [0.5, 0.0, 0.0] cmd_scale [2.0, 2.0, 0.25] dt 0.002 decimation 10 obs 47
create_damping_cmd kd (command_helper.py): 8
torchscript archive: 31 entries, root 'policy_lstm_1', data.pkl policy_lstm_1/data.pkl
policy: LSTM 64 + actor 64 -> 32 -> 12 (ELU between layers)
model: nq 19, nu 12, mass 32.11 kg, timestep 0.002
colliding shapes on the pelvis body: ['head_link', 'left_elbow_link', 'left_shoulder_yaw_link', 'left_wrist_roll_rubber_hand', 'logo_link', 'pelvis cylinder #11', 'pelvis cylinder #13', 'pelvis cylinder #21', 'pelvis cylinder #23', 'pelvis_contour_link', 'right_elbow_link', 'right_shoulder_yaw_link', 'right_wrist_roll_rubber_hand', 'torso_link_23dof_rev_1_0']

## 7 stops at 20 moments, 4.0-4.76 s (phase 0-0.95 of the policy's 0.8 s clock); 6 s after each stop; every mode starts with zero velocity
upright at end = pelvis z > 0.6 m and tilt < 30 deg at the last frame; hand-off = when the policy was switched off
t stop | mode | hand-off: s after stop, feet, pelvis m/s | first non-foot touch: shape, s, m/s | torso shape: s | upright at end | end pelvis z m | max tilt after hand-off deg | still after s | travel after stop m
4.0 | stopmove | - | none | - | True | 0.776 | 3.9 | None | 0.241
4.0 | zv1_hold_default | 1.0, left, 0.045 | left_wrist_roll_rubber_hand, 2.356, 2.98 | - | False | 0.312 | 78.9 | 2.96 | 1.004
4.0 | zv2_hold_default | 2.0, left, 0.127 | right_wrist_roll_rubber_hand, 3.352, 2.95 | 4.486 | False | 0.062 | 90.5 | None | 1.137
4.0 | zv_ds_hold_last | 0.792, left+right, 0.132 | left_hip_pitch_link, 2.34, 2.43 | 3.6 | False | 0.061 | 94.4 | 5.48 | 0.964
4.0 | zv_ds_hold_default | 0.792, left+right, 0.132 | left_wrist_roll_rubber_hand, 2.082, 2.95 | - | False | 0.311 | 78.0 | 2.84 | 1.007
4.0 | zv_ds_ramp_default | 0.792, left+right, 0.132 | left_wrist_roll_rubber_hand, 2.534, 3.04 | 2.756 | False | 0.061 | 90.8 | 4.6 | 1.064
4.0 | zv_ds_damp | 0.792, left+right, 0.132 | left_knee_link, 1.838, 0.68 | - | False | 0.064 | 100.9 | 4.16 | 0.436
4.04 | stopmove | - | none | - | True | 0.773 | 4.0 | None | 0.254
4.04 | zv1_hold_default | 1.0, left, 0.067 | left_wrist_roll_rubber_hand, 2.362, 2.98 | - | False | 0.311 | 78.7 | 2.96 | 1.021
4.04 | zv2_hold_default | 2.0, left+right, 0.129 | left_wrist_roll_rubber_hand, 3.368, 3.15 | - | False | 0.313 | 78.7 | 3.96 | 1.028
4.04 | zv_ds_hold_last | 0.754, left+right, 0.139 | left_hip_pitch_link, 2.274, 2.45 | - | False | 0.167 | 95.3 | 3.08 | 0.974
4.04 | zv_ds_hold_default | 0.754, left+right, 0.139 | left_wrist_roll_rubber_hand, 2.032, 2.95 | - | False | 0.31 | 78.7 | 3.36 | 1.0
4.04 | zv_ds_ramp_default | 0.754, left+right, 0.139 | left_wrist_roll_rubber_hand, 2.412, 3.03 | 2.632 | False | 0.061 | 91.5 | 4.52 | 1.064
4.04 | zv_ds_damp | 0.754, left+right, 0.139 | left_knee_link, 1.8, 0.68 | - | False | 0.064 | 101.0 | 4.08 | 0.437
4.08 | stopmove | - | none | - | True | 0.775 | 4.0 | None | 0.269
4.08 | zv1_hold_default | 1.0, left, 0.087 | left_wrist_roll_rubber_hand, 2.354, 2.97 | - | False | 0.311 | 78.6 | 2.96 | 1.036
4.08 | zv2_hold_default | 2.0, right, 0.079 | left_wrist_roll_rubber_hand, 3.378, 3.13 | - | False | 0.313 | 79.0 | 3.96 | 1.049
4.08 | zv_ds_hold_last | 0.716, left+right, 0.141 | left_hip_pitch_link, 2.154, 2.47 | - | False | 0.173 | 95.6 | 2.84 | 0.992
4.08 | zv_ds_hold_default | 0.716, left+right, 0.141 | left_wrist_roll_rubber_hand, 1.986, 2.96 | - | False | 0.31 | 76.9 | 4.04 | 1.04
4.08 | zv_ds_ramp_default | 0.716, left+right, 0.141 | left_wrist_roll_rubber_hand, 2.268, 3.02 | 2.498 | False | 0.061 | 92.7 | 4.04 | 1.088
4.08 | zv_ds_damp | 0.716, left+right, 0.141 | left_knee_link, 1.774, 0.68 | - | False | 0.064 | 100.2 | 3.84 | 0.427
4.12 | stopmove | - | none | - | True | 0.776 | 4.0 | None | 0.279
4.12 | zv1_hold_default | 1.0, left, 0.11 | left_wrist_roll_rubber_hand, 2.332, 2.99 | - | False | 0.311 | 78.8 | 2.92 | 1.037
4.12 | zv2_hold_default | 2.0, right, 0.05 | left_wrist_roll_rubber_hand, 3.386, 3.13 | - | False | 0.313 | 78.9 | 4.0 | 1.065
4.12 | zv_ds_hold_last | 0.678, left+right, 0.138 | left_hip_pitch_link, 2.028, 2.55 | - | False | 0.169 | 95.0 | 2.48 | 1.042
4.12 | zv_ds_hold_default | 0.678, left+right, 0.138 | left_wrist_roll_rubber_hand, 1.95, 2.97 | - | False | 0.31 | 77.4 | 3.28 | 1.065
4.12 | zv_ds_ramp_default | 0.678, left+right, 0.138 | left_hip_roll_link, 2.114, 2.27 | 2.372 | False | 0.061 | 93.9 | 3.96 | 1.105
4.12 | zv_ds_damp | 0.678, left+right, 0.138 | left_knee_link, 1.77, 0.72 | - | False | 0.064 | 95.6 | 3.72 | 0.4
4.16 | stopmove | - | none | - | True | 0.777 | 4.0 | None | 0.285
4.16 | zv1_hold_default | 1.0, left, 0.128 | left_wrist_roll_rubber_hand, 2.292, 3.04 | - | False | 0.312 | 79.2 | 2.88 | 1.034
4.16 | zv2_hold_default | 2.0, right, 0.031 | left_wrist_roll_rubber_hand, 3.39, 3.13 | - | False | 0.313 | 78.8 | 4.0 | 1.075
4.16 | zv_ds_hold_last | 0.642, left+right, 0.133 | left_wrist_roll_rubber_hand, 2.218, 2.99 | - | False | 0.323 | 84.9 | 2.88 | 1.037
4.16 | zv_ds_hold_default | 0.642, left+right, 0.133 | left_wrist_roll_rubber_hand, 1.922, 2.99 | 3.726 | False | 0.061 | 90.3 | 5.48 | 1.08
4.16 | zv_ds_ramp_default | 0.642, left+right, 0.133 | left_wrist_roll_rubber_hand, 2.108, 2.94 | - | False | 0.311 | 78.5 | 2.96 | 1.043
4.16 | zv_ds_damp | 0.642, left+right, 0.133 | left_hip_pitch_link, 1.788, 1.9 | - | False | 0.064 | 90.2 | 3.56 | 0.355
4.2 | stopmove | - | none | - | True | 0.778 | 3.9 | None | 0.284
4.2 | zv1_hold_default | 1.0, left, 0.129 | left_wrist_roll_rubber_hand, 2.248, 3.1 | - | False | 0.311 | 79.4 | 2.84 | 1.019
4.2 | zv2_hold_default | 2.0, right, 0.023 | left_wrist_roll_rubber_hand, 3.394, 3.12 | - | False | 0.313 | 78.8 | 4.0 | 1.072
4.2 | zv_ds_hold_last | 0.606, left+right, 0.129 | left_wrist_roll_rubber_hand, 2.056, 2.96 | 2.814 | False | 0.061 | 90.0 | 4.88 | 1.07
4.2 | zv_ds_hold_default | 0.606, left+right, 0.129 | left_wrist_roll_rubber_hand, 1.902, 2.99 | 2.708 | False | 0.061 | 90.4 | 4.48 | 1.063
4.2 | zv_ds_ramp_default | 0.606, left+right, 0.129 | left_wrist_roll_rubber_hand, 2.01, 2.98 | - | False | 0.31 | 77.3 | 3.52 | 1.068
4.2 | zv_ds_damp | 0.606, left+right, 0.129 | left_shoulder_yaw_link, 1.768, 2.92 | - | False | 0.064 | 90.3 | 3.16 | 0.322
4.24 | stopmove | - | none | - | True | 0.778 | 3.9 | None | 0.276
4.24 | zv1_hold_default | 1.0, left+right, 0.115 | left_wrist_roll_rubber_hand, 2.258, 3.04 | - | False | 0.311 | 79.2 | 2.84 | 1.013
4.24 | zv2_hold_default | 2.0, right, 0.032 | right_wrist_roll_rubber_hand, 3.39, 3.09 | - | False | 0.313 | 78.9 | 4.0 | 1.061
4.24 | zv_ds_hold_last | 0.568, left+right, 0.124 | left_wrist_roll_rubber_hand, 1.976, 2.99 | 2.306 | False | 0.061 | 89.9 | 4.44 | 1.145
4.24 | zv_ds_hold_default | 0.568, left+right, 0.124 | left_wrist_roll_rubber_hand, 1.874, 3.0 | 2.628 | False | 0.061 | 90.5 | 4.24 | 1.064
4.24 | zv_ds_ramp_default | 0.568, left+right, 0.124 | left_wrist_roll_rubber_hand, 1.946, 3.04 | 2.468 | False | 0.061 | 90.2 | 4.44 | 1.116
4.24 | zv_ds_damp | 0.568, left+right, 0.124 | pelvis cylinder #11, 1.744, 3.48 | - | False | 0.064 | 93.5 | 3.0 | 0.291
4.28 | stopmove | - | none | - | True | 0.779 | 3.9 | None | 0.261
4.28 | zv1_hold_default | 1.0, right, 0.07 | left_wrist_roll_rubber_hand, 2.3, 3.09 | - | False | 0.311 | 78.9 | 2.88 | 1.012
4.28 | zv2_hold_default | 2.0, right, 0.048 | right_wrist_roll_rubber_hand, 3.386, 3.11 | - | False | 0.313 | 79.0 | 4.0 | 1.038
4.28 | zv_ds_hold_last | 0.534, left+right, 0.111 | left_wrist_roll_rubber_hand, 2.012, 3.05 | 2.25 | False | 0.061 | 90.9 | 4.4 | 1.086
4.28 | zv_ds_hold_default | 0.534, left+right, 0.111 | left_wrist_roll_rubber_hand, 1.85, 3.0 | 2.632 | False | 0.061 | 90.4 | 4.24 | 1.065
4.28 | zv_ds_ramp_default | 0.534, left+right, 0.111 | left_wrist_roll_rubber_hand, 1.972, 3.11 | 2.33 | False | 0.061 | 90.3 | 4.2 | 1.123
4.28 | zv_ds_damp | 0.534, left+right, 0.111 | pelvis cylinder #11, 1.738, 3.41 | - | False | 0.065 | 93.9 | 2.84 | 0.273
4.32 | stopmove | - | none | - | True | 0.779 | 4.0 | None | 0.243
4.32 | zv1_hold_default | 1.0, right, 0.046 | left_wrist_roll_rubber_hand, 2.346, 3.11 | - | False | 0.311 | 78.9 | 2.96 | 1.004
4.32 | zv2_hold_default | 2.0, right, 0.067 | left_wrist_roll_rubber_hand, 3.384, 3.02 | - | False | 0.314 | 79.1 | 3.96 | 1.014
4.32 | zv_ds_hold_last | 0.5, left+right, 0.11 | right_wrist_roll_rubber_hand, 2.182, 2.9 | 2.438 | False | 0.061 | 91.4 | 4.32 | 1.121
4.32 | zv_ds_hold_default | 0.5, left+right, 0.11 | left_wrist_roll_rubber_hand, 1.804, 2.99 | - | False | 0.309 | 77.9 | 3.24 | 1.012
4.32 | zv_ds_ramp_default | 0.5, left+right, 0.11 | left_wrist_roll_rubber_hand, 2.07, 3.11 | - | False | 0.311 | 79.3 | 2.68 | 0.987
4.32 | zv_ds_damp | 0.5, left+right, 0.11 | pelvis cylinder #11, 1.742, 3.17 | - | False | 0.065 | 91.5 | 3.4 | 0.275
4.36 | stopmove | - | none | - | True | 0.78 | 4.0 | None | 0.228
4.36 | zv1_hold_default | 1.0, right, 0.032 | left_wrist_roll_rubber_hand, 2.36, 3.13 | - | False | 0.311 | 78.8 | 2.96 | 0.995
4.36 | zv2_hold_default | 2.0, right, 0.086 | left_wrist_roll_rubber_hand, 3.378, 2.95 | - | False | 0.314 | 79.0 | 3.96 | 1.0
4.36 | zv_ds_hold_last | 0.836, left+right, 0.139 | right_hip_pitch_link, 2.314, 2.38 | - | False | 0.172 | 95.1 | 2.68 | 0.972
4.36 | zv_ds_hold_default | 0.836, left+right, 0.139 | left_wrist_roll_rubber_hand, 2.164, 3.21 | - | False | 0.31 | 79.5 | 2.8 | 0.97
4.36 | zv_ds_ramp_default | 0.836, left+right, 0.139 | right_hip_pitch_link, 2.518, 2.53 | 3.562 | False | 0.061 | 94.7 | 5.36 | 1.062
4.36 | zv_ds_damp | 0.836, left+right, 0.139 | right_knee_link, 1.848, 0.79 | 4.026 | False | 0.065 | 104.9 | 4.12 | 0.521
4.4 | stopmove | - | none | - | True | 0.779 | 4.2 | None | 0.226
4.4 | zv1_hold_default | 1.0, right, 0.028 | left_wrist_roll_rubber_hand, 2.352, 3.14 | - | False | 0.311 | 78.7 | 2.96 | 0.992
4.4 | zv2_hold_default | 2.0, right, 0.091 | left_wrist_roll_rubber_hand, 3.352, 2.97 | - | False | 0.313 | 77.4 | 4.24 | 1.035
4.4 | zv_ds_hold_last | 0.794, left+right, 0.147 | right_hip_pitch_link, 2.33, 2.38 | - | False | 0.173 | 95.0 | 2.72 | 0.974
4.4 | zv_ds_hold_default | 0.794, left+right, 0.147 | right_wrist_roll_rubber_hand, 2.124, 2.93 | - | False | 0.31 | 79.2 | 2.76 | 0.978
4.4 | zv_ds_ramp_default | 0.794, left+right, 0.147 | right_hip_pitch_link, 2.502, 2.56 | 2.996 | False | 0.061 | 94.3 | 4.64 | 1.105
4.4 | zv_ds_damp | 0.794, left+right, 0.147 | right_knee_link, 1.816, 0.82 | 3.716 | False | 0.064 | 102.1 | 3.84 | 0.524
4.44 | stopmove | - | none | - | True | 0.777 | 4.1 | None | 0.229
4.44 | zv1_hold_default | 1.0, right, 0.037 | left_wrist_roll_rubber_hand, 2.33, 3.12 | - | False | 0.311 | 78.9 | 2.92 | 0.994
4.44 | zv2_hold_default | 2.0, left+right, 0.105 | left_wrist_roll_rubber_hand, 3.338, 2.97 | 4.46 | False | 0.061 | 90.4 | None | 1.17
4.44 | zv_ds_hold_last | 0.752, left+right, 0.155 | right_hip_pitch_link, 2.344, 2.39 | 3.888 | False | 0.062 | 94.7 | None | 0.954
4.44 | zv_ds_hold_default | 0.752, left+right, 0.155 | right_wrist_roll_rubber_hand, 2.066, 2.93 | - | False | 0.31 | 79.3 | 2.68 | 0.982
4.44 | zv_ds_ramp_default | 0.752, left+right, 0.155 | right_wrist_roll_rubber_hand, 2.468, 2.99 | 2.688 | False | 0.061 | 90.3 | 4.52 | 1.076
4.44 | zv_ds_damp | 0.752, left+right, 0.155 | right_knee_link, 1.764, 0.81 | 3.668 | False | 0.064 | 102.1 | 3.84 | 0.533
4.48 | stopmove | - | none | - | True | 0.778 | 4.0 | None | 0.232
4.48 | zv1_hold_default | 1.0, right, 0.056 | left_wrist_roll_rubber_hand, 2.318, 3.1 | - | False | 0.311 | 78.9 | 2.92 | 1.001
4.48 | zv2_hold_default | 2.0, left, 0.048 | left_wrist_roll_rubber_hand, 3.344, 2.97 | - | False | 0.314 | 79.1 | 3.92 | 1.01
4.48 | zv_ds_hold_last | 0.712, left+right, 0.151 | right_knee_link, 2.286, 1.01 | 3.678 | False | 0.061 | 94.9 | 5.56 | 0.984
4.48 | zv_ds_hold_default | 0.712, left+right, 0.151 | right_wrist_roll_rubber_hand, 2.024, 2.94 | - | False | 0.31 | 79.0 | 3.0 | 0.997
4.48 | zv_ds_ramp_default | 0.712, left+right, 0.151 | right_wrist_roll_rubber_hand, 2.338, 3.03 | 2.57 | False | 0.061 | 91.5 | 4.2 | 1.082
4.48 | zv_ds_damp | 0.712, left+right, 0.151 | right_knee_link, 1.744, 0.86 | 3.38 | False | 0.064 | 100.7 | 3.56 | 0.515
4.52 | stopmove | - | none | - | True | 0.78 | 3.9 | None | 0.235
4.52 | zv1_hold_default | 1.0, right, 0.079 | right_wrist_roll_rubber_hand, 2.306, 3.11 | - | False | 0.311 | 79.1 | 2.92 | 1.001
4.52 | zv2_hold_default | 2.0, left, 0.01 | left_wrist_roll_rubber_hand, 3.358, 2.98 | - | False | 0.314 | 79.1 | 3.96 | 1.022
4.52 | zv_ds_hold_last | 0.672, left+right, 0.149 | right_knee_link, 2.252, 1.07 | - | False | 0.173 | 94.7 | 2.88 | 1.039
4.52 | zv_ds_hold_default | 0.672, left+right, 0.149 | right_wrist_roll_rubber_hand, 1.984, 2.94 | 3.074 | False | 0.061 | 90.6 | 4.68 | 1.17
4.52 | zv_ds_ramp_default | 0.672, left+right, 0.149 | right_wrist_roll_rubber_hand, 2.242, 3.06 | 2.564 | False | 0.061 | 90.5 | 4.24 | 1.11
4.52 | zv_ds_damp | 0.672, left+right, 0.149 | right_hip_pitch_link, 1.71, 2.07 | 3.306 | False | 0.063 | 98.7 | 3.44 | 0.49
4.56 | stopmove | - | none | - | True | 0.781 | 3.9 | None | 0.239
4.56 | zv1_hold_default | 1.0, right, 0.102 | left_wrist_roll_rubber_hand, 2.296, 3.02 | - | False | 0.312 | 79.2 | 2.88 | 1.002
4.56 | zv2_hold_default | 2.0, left, 0.025 | left_wrist_roll_rubber_hand, 3.382, 2.97 | - | False | 0.314 | 79.1 | 3.96 | 1.032
4.56 | zv_ds_hold_last | 0.632, left+right, 0.151 | right_hip_pitch_link, 2.246, 2.51 | - | False | 0.173 | 95.5 | 2.76 | 1.087
4.56 | zv_ds_hold_default | 0.632, left+right, 0.151 | right_wrist_roll_rubber_hand, 1.944, 2.95 | 3.096 | False | 0.061 | 90.5 | 4.68 | 1.182
4.56 | zv_ds_ramp_default | 0.632, left+right, 0.151 | right_wrist_roll_rubber_hand, 2.184, 3.05 | 2.606 | False | 0.061 | 90.3 | 4.48 | 1.147
4.56 | zv_ds_damp | 0.632, left+right, 0.151 | right_hip_pitch_link, 1.67, 2.07 | 3.366 | False | 0.064 | 97.3 | 3.52 | 0.48
4.6 | stopmove | - | none | - | True | 0.781 | 3.9 | None | 0.239
4.6 | zv1_hold_default | 1.0, left+right, 0.11 | left_wrist_roll_rubber_hand, 2.288, 2.95 | - | False | 0.311 | 79.5 | 2.88 | 0.996
4.6 | zv2_hold_default | 2.0, left, 0.045 | left_wrist_roll_rubber_hand, 3.418, 2.97 | - | False | 0.313 | 78.8 | 4.0 | 1.042
4.6 | zv_ds_hold_last | 0.592, left+right, 0.153 | right_hip_pitch_link, 2.246, 2.54 | 3.008 | False | 0.061 | 95.1 | 4.84 | 1.124
4.6 | zv_ds_hold_default | 0.592, left+right, 0.153 | right_wrist_roll_rubber_hand, 1.898, 2.95 | 3.14 | False | 0.061 | 90.4 | 4.72 | 1.183
4.6 | zv_ds_ramp_default | 0.592, left+right, 0.153 | right_wrist_roll_rubber_hand, 2.148, 3.01 | 2.688 | False | 0.061 | 90.1 | 4.68 | 1.109
4.6 | zv_ds_damp | 0.592, left+right, 0.153 | right_hip_pitch_link, 1.632, 2.05 | 3.306 | False | 0.064 | 96.1 | 3.44 | 0.466
4.64 | stopmove | - | none | - | True | 0.781 | 3.9 | None | 0.238
4.64 | zv1_hold_default | 1.0, left, 0.094 | right_wrist_roll_rubber_hand, 2.31, 3.13 | - | False | 0.312 | 78.9 | 2.88 | 0.998
4.64 | zv2_hold_default | 2.0, left, 0.066 | left_wrist_roll_rubber_hand, 3.446, 2.97 | - | False | 0.313 | 78.5 | 4.08 | 1.045
4.64 | zv_ds_hold_last | 0.552, left+right, 0.156 | right_knee_link, 2.302, 1.11 | - | False | 0.173 | 95.1 | 2.92 | 1.078
4.64 | zv_ds_hold_default | 0.552, left+right, 0.156 | right_wrist_roll_rubber_hand, 1.834, 2.95 | 2.944 | False | 0.061 | 90.3 | 4.52 | 1.16
4.64 | zv_ds_ramp_default | 0.552, left+right, 0.156 | right_wrist_roll_rubber_hand, 2.15, 2.99 | 2.856 | False | 0.061 | 90.3 | 4.68 | 1.072
4.64 | zv_ds_damp | 0.552, left+right, 0.156 | right_hip_pitch_link, 1.596, 2.0 | 3.138 | False | 0.063 | 97.7 | 3.24 | 0.465
4.68 | stopmove | - | none | - | True | 0.781 | 4.0 | None | 0.241
4.68 | zv1_hold_default | 1.0, left, 0.043 | left_wrist_roll_rubber_hand, 2.31, 3.08 | - | False | 0.312 | 78.8 | 2.88 | 1.005
4.68 | zv2_hold_default | 2.0, left, 0.088 | left_wrist_roll_rubber_hand, 3.468, 2.98 | - | False | 0.313 | 78.5 | 4.04 | 1.047
4.68 | zv_ds_hold_last | 0.514, left+right, 0.17 | right_hip_pitch_link, 2.27, 2.34 | - | False | 0.173 | 95.0 | 2.68 | 0.961
4.68 | zv_ds_hold_default | 0.514, left+right, 0.17 | left_wrist_roll_rubber_hand, 1.748, 3.21 | - | False | 0.31 | 78.2 | 2.36 | 0.997
4.68 | zv_ds_ramp_default | 0.514, left+right, 0.17 | right_wrist_roll_rubber_hand, 2.228, 2.96 | 3.126 | False | 0.061 | 90.7 | 4.72 | 1.048
4.68 | zv_ds_damp | 0.514, left+right, 0.17 | right_knee_link, 1.498, 0.7 | 4.904 | False | 0.066 | 103.0 | 2.36 | 0.529
4.72 | stopmove | - | none | - | True | 0.78 | 4.0 | None | 0.245
4.72 | zv1_hold_default | 1.0, left, 0.015 | left_wrist_roll_rubber_hand, 2.31, 3.06 | - | False | 0.312 | 78.8 | 2.88 | 1.012
4.72 | zv2_hold_default | 2.0, left, 0.112 | left_wrist_roll_rubber_hand, 3.46, 3.01 | - | False | 0.314 | 79.0 | 4.04 | 1.04
4.72 | zv_ds_hold_last | 0.5, left+right, 0.196 | left_hip_pitch_link, 1.768, 2.48 | 2.846 | False | 0.061 | 93.5 | 4.0 | 1.156
4.72 | zv_ds_hold_default | 0.5, left+right, 0.196 | left_wrist_roll_rubber_hand, 1.698, 3.07 | - | False | 0.31 | 79.2 | 2.28 | 0.981
4.72 | zv_ds_ramp_default | 0.5, left+right, 0.196 | left_wrist_roll_rubber_hand, 1.912, 3.02 | 2.162 | False | 0.061 | 90.6 | 4.0 | 1.14
4.72 | zv_ds_damp | 0.5, left+right, 0.196 | right_knee_link, 1.448, 0.68 | 2.032 | False | 0.061 | 96.0 | 3.12 | 0.682
4.76 | stopmove | - | none | - | True | 0.779 | 3.9 | None | 0.241
4.76 | zv1_hold_default | 1.0, left, 0.025 | left_wrist_roll_rubber_hand, 2.334, 3.02 | - | False | 0.312 | 79.0 | 2.92 | 1.006
4.76 | zv2_hold_default | 2.0, left, 0.129 | left_wrist_roll_rubber_hand, 3.43, 3.11 | - | False | 0.313 | 78.8 | 4.04 | 1.026
4.76 | zv_ds_hold_last | 0.834, left+right, 0.129 | left_hip_pitch_link, 2.316, 2.53 | 3.272 | False | 0.061 | 95.0 | 5.12 | 0.917
4.76 | zv_ds_hold_default | 0.834, left+right, 0.129 | left_wrist_roll_rubber_hand, 2.12, 2.94 | - | False | 0.311 | 79.4 | 2.72 | 0.985
4.76 | zv_ds_ramp_default | 0.834, left+right, 0.129 | left_hip_pitch_link, 2.664, 2.5 | 3.616 | False | 0.061 | 94.2 | 5.4 | 1.039
4.76 | zv_ds_damp | 0.834, left+right, 0.129 | left_knee_link, 1.864, 0.66 | - | False | 0.072 | 103.4 | 2.76 | 0.504

## per mode, over all stop moments
mode | upright at end | hand-offs: n, s after stop min/median/max, pelvis m/s at hand-off max | non-foot touch: n, s min/median/max | first shapes | torso shape touched | end pelvis z m min-max | max tilt after hand-off deg max | still (0.48 s under 0.05 m/s) | travel after stop m min-max
stopmove | 20/20 | - | 0 | - | 0 | 0.773-0.781 | 4.2 | 0 | 0.226-0.285
zv1_hold_default | 0/20 | 20, 1.0/1.0/1.0, 0.129 | 20, 2.248/2.314/2.362 | left_wrist_roll_rubber_hand x18, right_wrist_roll_rubber_hand x2 | 0 | 0.311-0.312 | 79.5 | 20 | 0.992-1.037
zv2_hold_default | 0/20 | 20, 2.0/2.0/2.0, 0.129 | 20, 3.338/3.385/3.468 | right_wrist_roll_rubber_hand x3, left_wrist_roll_rubber_hand x17 | 2 | 0.061-0.314 | 90.5 | 18 | 1.0-1.17
zv_ds_hold_last | 0/20 | 20, 0.5/0.657/0.836, 0.196 | 20, 1.768/2.249/2.344 | left_hip_pitch_link x6, left_wrist_roll_rubber_hand x4, right_wrist_roll_rubber_hand x1, right_hip_pitch_link x6, right_knee_link x3 | 10 | 0.061-0.323 | 95.6 | 19 | 0.917-1.156
zv_ds_hold_default | 0/20 | 20, 0.5/0.657/0.836, 0.196 | 20, 1.698/1.947/2.164 | left_wrist_roll_rubber_hand x13, right_wrist_roll_rubber_hand x7 | 8 | 0.061-0.311 | 90.6 | 20 | 0.97-1.183
zv_ds_ramp_default | 0/20 | 20, 0.5/0.657/0.836, 0.196 | 20, 1.912/2.206/2.664 | left_wrist_roll_rubber_hand x9, left_hip_roll_link x1, right_hip_pitch_link x2, right_wrist_roll_rubber_hand x7, left_hip_pitch_link x1 | 17 | 0.061-0.311 | 94.7 | 20 | 0.987-1.147
zv_ds_damp | 0/20 | 20, 0.5/0.657/0.836, 0.196 | 20, 1.448/1.754/1.864 | left_knee_link x5, left_hip_pitch_link x1, left_shoulder_yaw_link x1, pelvis cylinder #11 x3, right_knee_link x6, right_hip_pitch_link x4 | 10 | 0.061-0.072 | 104.9 | 20 | 0.273-0.682
