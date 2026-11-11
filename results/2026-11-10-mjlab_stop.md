# Unitree's mjlab G1 velocity policy: what each stop does to a walking G1 (simulation)
numpy 2.0.2, mujoco 3.3.7

## inputs
unitreerobotics/unitree_rl_mjlab/deploy/robots/g1/config/config.yaml  1468 bytes  sha256 592d3e513cc4a4565a8c9d834bd25180372bc6b98363c18a2b4cf941e5aafe20
unitreerobotics/unitree_rl_mjlab/deploy/robots/g1/config/policy/velocity/v0/exported/policy.onnx  878421 bytes  sha256 2a66ca6336eadb3c0b34b557763f3e06d01ff8fcf6260dd4cedbd69d6093fc28
unitreerobotics/unitree_rl_mjlab/deploy/robots/g1/config/policy/velocity/v0/params/deploy.yaml  2289 bytes  sha256 01e1cf3f6ec44e9942494fbb4a9904df07201e14e5551a277f38d7c7d1bda28d
unitreerobotics/unitree_rl_mjlab/src/assets/robots/unitree_g1/xmls/g1.xml  21462 bytes  sha256 56539bc76eadb05dd439c47de94df52130ea8fa243d08bdddd9cbc32dd4c78a0

## deploy.yaml
step_dt 0.02, sim steps per policy step 4, gait period 0.6
stiffness [40.2, 99.1, 40.2, 99.1, 28.5, 28.5, 40.2, 99.1, 40.2, 99.1, 28.5, 28.5, 40.2, 28.5, 28.5, 14.3, 14.3, 14.3, 14.3, 14.3, 16.8, 16.8, 14.3, 14.3, 14.3, 14.3, 14.3, 16.8, 16.8]
damping [2.6, 6.3, 2.6, 6.3, 1.8, 1.8, 2.6, 6.3, 2.6, 6.3, 1.8, 1.8, 2.6, 1.8, 1.8, 0.9, 0.9, 0.9, 0.9, 0.9, 1.1, 1.1, 0.9, 0.9, 0.9, 0.9, 0.9, 1.1, 1.1]
default_joint_pos [-0.1, 0.0, 0.0, 0.3, -0.2, 0.0, -0.1, 0.0, 0.0, 0.3, -0.2, 0.0, 0.0, 0.0, 0.0, 0.35, 0.18, 0.0, 0.87, 0.0, 0.0, 0.0, 0.35, -0.18, 0.0, 0.87, 0.0, 0.0, 0.0]
action scale [0.55, 0.35, 0.55, 0.35, 0.44, 0.44, 0.55, 0.35, 0.55, 0.35, 0.44, 0.44, 0.55, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.44, 0.07, 0.07, 0.44, 0.44, 0.44, 0.44, 0.44, 0.07, 0.07]
action offset == default_joint_pos: True
observation terms, in order: ['base_ang_vel', 'projected_gravity', 'velocity_commands', 'gait_phase', 'joint_pos_rel', 'joint_vel_rel', 'last_action']

## config.yaml (the deploy's state machine)
Passive kd [3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0] (all 3: True)
FixStand kp [100.0, 100.0, 100.0, 150.0, 40.0, 40.0, 100.0, 100.0, 100.0, 150.0, 40.0, 40.0, 200.0, 200.0, 200.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0, 40.0] (as expected: True)
FixStand kd [2.0, 2.0, 2.0, 4.0, 2.0, 2.0, 2.0, 2.0, 2.0, 4.0, 2.0, 2.0, 5.0, 5.0, 5.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0] (as expected: True)
FixStand ramp 2.0 s to [-0.1, 0.0, 0.0, 0.3, -0.2, 0.0, -0.1, 0.0, 0.0, 0.3, -0.2, 0.0, 0.0, 0.0, 0.0, 0.35, 0.18, 0.0, 0.87, 0.0, 0.0, 0.0, 0.35, -0.18, 0.0, 0.87, 0.0, 0.0, 0.0] (== default_joint_pos: True)
Velocity state's exits: [('Passive', 'LT + B.on_pressed'), ('Mimic_Dance1_subject2', 'RB + A.on_pressed')]

## policy.onnx
ops ['Sub', 'Div', 'Gemm', 'Elu', 'Gemm', 'Elu', 'Gemm', 'Elu', 'Gemm']; normalizer (98,); layers 98 -> 512 -> 256 -> 128 -> 29 (Gemm transB, ELU alpha 1)
REPRO zero observation vs the 2026-11-10 recon run: max |diff| 0.000000 -> match True

## actuators (g1_constants.py, recomputed)
joints ['left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint', 'left_knee_joint', 'left_ankle_pitch_joint', 'left_ankle_roll_joint', 'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint', 'right_knee_joint', 'right_ankle_pitch_joint', 'right_ankle_roll_joint', 'waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint', 'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint', 'left_elbow_joint', 'left_wrist_roll_joint', 'left_wrist_pitch_joint', 'left_wrist_yaw_joint', 'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint', 'right_elbow_joint', 'right_wrist_roll_joint', 'right_wrist_pitch_joint', 'right_wrist_yaw_joint']
armature [0.010178, 0.025102, 0.010178, 0.025102, 0.007219, 0.007219, 0.010178, 0.025102, 0.010178, 0.025102, 0.007219, 0.007219, 0.010178, 0.007219, 0.007219, 0.00361, 0.00361, 0.00361, 0.00361, 0.00361, 0.00425, 0.00425, 0.00361, 0.00361, 0.00361, 0.00361, 0.00361, 0.00425, 0.00425]
effort limit [88.0, 139.0, 88.0, 139.0, 50.0, 50.0, 88.0, 139.0, 88.0, 139.0, 50.0, 50.0, 88.0, 50.0, 50.0, 25.0, 25.0, 25.0, 25.0, 25.0, 5.0, 5.0, 25.0, 25.0, 25.0, 25.0, 25.0, 5.0, 5.0]
armature x (2 pi 10 Hz)^2, to 0.1, == deploy.yaml stiffness: True; 2 x 2 x armature x 2 pi 10, to 0.1, == damping: True; 0.25 x effort / stiffness, to 0.01, == action scale: True
the actuators below use deploy.yaml's gains (what the robot is sent), with these armatures and effort limits

## bodies
mesh geoms removed (no mass: density 0 visuals): 35, classes ['visual']
g1.xml <option> as written: None
collision geoms set as FULL_COLLISION: feet 14 (condim 3, priority 1, friction 0.6), others 19 (condim 1)
[mjlab] nq 36 nv 35 nu 29, mass 33.341 kg, timestep 0.005, integrator 3, iterations 10/20; colliding geoms 33, on the feet 14; foot friction [0.6]
[mjlab] armature True, kp True, kd True, force range True; joint actuatorfrcrange [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
[menagerie] nq 36 nv 35 nu 29, mass 33.341 kg, timestep 0.005, integrator 3, iterations 10/20; colliding geoms 36, on the feet 10; foot friction [0.6, 1.0]
[menagerie] armature True, kp True, kd True, force range True; joint actuatorfrcrange [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
body masses, mjlab vs Menagerie: 30 bodies in common, max |diff| 0.0000 kg

## gate: stand 10 s at zero command from the default pose (pelvis dropped from 0.8 m)
[mjlab] upright at 10 s True; end pelvis z 0.784 m, tilt 0.7 deg, max tilt 0.9; drift 0.012 m; still after 0.02 s; lift-offs 0 (last None s); non-foot touch None; bad-qacc 0
[menagerie] upright at 10 s True; end pelvis z 0.784 m, tilt 0.9 deg, max tilt 0.9; drift 0.015 m; still after 0.02 s; lift-offs 0 (last None s); non-foot touch None; bad-qacc 0

## [mjlab] walking at 0.5 m/s: 2.0 s standing, then 4.0 s walking, then the stop at 30 moments 6.00-6.58 s
walking check: last 2 s moved 1.010 m forward, 0.118 m sideways (0.509 m/s); heading 7.1 deg; min pelvis z 0.775; no fall and no non-foot touch: True
mode | upright at end | first non-foot touch s after stop min/med/max | travel at end m (upright runs) | still after s | lift-offs after stop min/med/max | last lift-off s | legs over cap, max share | bad-qacc runs
continue | 30/30 | - | 3.078/3.079/3.08 (n 30) | - | 19/20.0/20 (n 30) | 5.7/5.84/5.98 (n 30) | 0.0 | 0
zero_cmd | 30/30 | - | 0.083/0.106/0.136 (n 30) | 1.1/1.16/1.46 (n 30) | 0/1.0/1 (n 30) | 0.04/0.18/0.38 (n 24) | 0.0 | 0
passive_kd3 | 0/30 | 0.4/0.46/0.54 (n 30) | - | - | 0/0.0/1 (n 30) | 0.36/0.36/0.36 (n 1) | 0.0 | 0
damp_kd8 | 0/30 | 0.58/0.71/0.8 (n 30) | - | - | 0/0.0/0 (n 30) | - | 0.0 | 0
zero_torque | 0/30 | 0.28/0.31/0.38 (n 30) | - | - | 0/1.0/3 (n 30) | 0.14/0.2/0.24 (n 19) | 0.0 | 0
hold_last | 0/30 | 0.74/0.8/1.26 (n 30) | - | - | 0/0.0/0 (n 30) | - | 0.0 | 0
fixstand | 0/30 | 0.7/0.83/1.24 (n 30) | - | - | 0/0.0/4 (n 30) | 0.06/0.66/1.02 (n 9) | 0.0 | 0
zero_cmd_then_passive | 0/30 | 2.66/2.68/2.7 (n 30) | - | - | 0/1.0/1 (n 30) | 0.04/0.18/0.38 (n 24) | 0.0 | 0
zero_cmd_then_fixstand | 0/30 | 3.56/4.08/5.34 (n 30) | - | - | 0/1.0/1 (n 30) | 0.04/0.18/0.38 (n 24) | 0.0 | 0
zero_cmd_then_passive: upright at the hand-off 30/30; speed then 0.009/0.011/0.017 (n 30) m/s; fell after the hand-off in 0.46/0.48/0.48 (n 30) s
zero_cmd_then_fixstand: upright at the hand-off 30/30; speed then 0.009/0.011/0.017 (n 30) m/s; fell after the hand-off in 1.14/1.64/2.94 (n 30) s
per moment, zero_cmd: moment, t_stop, speed at stop, upright, travel end m, still after s, lift-offs
  0 6.0 0.575 True 0.123 1.32 1
  1 6.02 0.502 True 0.117 1.24 1
  2 6.04 0.475 True 0.112 1.22 0
  3 6.06 0.493 True 0.104 1.16 0
  4 6.08 0.505 True 0.097 1.1 0
  5 6.1 0.51 True 0.091 1.1 0
  6 6.12 0.508 True 0.086 1.16 0
  7 6.14 0.501 True 0.089 1.12 0
  8 6.16 0.495 True 0.103 1.14 1
  9 6.18 0.495 True 0.106 1.14 1
  10 6.2 0.5 True 0.105 1.14 1
  11 6.22 0.512 True 0.106 1.14 1
  12 6.24 0.538 True 0.103 1.16 1
  13 6.26 0.569 True 0.102 1.12 1
  14 6.28 0.591 True 0.096 1.12 1
  15 6.3 0.579 True 0.083 1.14 1
  16 6.32 0.513 True 0.097 1.46 1
  17 6.34 0.491 True 0.118 1.24 1
  18 6.36 0.507 True 0.11 1.24 1
  19 6.38 0.515 True 0.105 1.2 1
  20 6.4 0.504 True 0.107 1.16 1
  21 6.42 0.489 True 0.114 1.16 1
  22 6.44 0.479 True 0.115 1.16 1
  23 6.46 0.477 True 0.119 1.16 1
  24 6.48 0.484 True 0.121 1.16 1
  25 6.5 0.499 True 0.128 1.2 1
  26 6.52 0.523 True 0.133 1.22 1
  27 6.54 0.55 True 0.136 1.26 1
  28 6.56 0.573 True 0.133 1.28 1
  29 6.58 0.58 True 0.129 1.32 1

## [mjlab] walking at 1.0 m/s: 2.0 s standing, then 4.0 s walking, then the stop at 30 moments 6.00-6.58 s
walking check: last 2 s moved 2.021 m forward, 0.195 m sideways (1.015 m/s); heading 4.8 deg; min pelvis z 0.769; no fall and no non-foot touch: True
mode | upright at end | first non-foot touch s after stop min/med/max | travel at end m (upright runs) | still after s | lift-offs after stop min/med/max | last lift-off s | legs over cap, max share | bad-qacc runs
continue | 30/30 | - | 5.987/5.989/5.994 (n 30) | - | 19/20.0/20 (n 30) | 5.68/5.84/5.98 (n 30) | 0.0 | 0
zero_cmd | 30/30 | - | 0.26/0.314/0.345 (n 30) | 0.84/1.4/2.0 (n 30) | 1/1.0/2 (n 30) | 0.08/0.29/0.52 (n 30) | 0.0 | 0
passive_kd3 | 0/30 | 0.36/0.4/0.46 (n 30) | - | - | 0/0.0/0 (n 30) | - | 0.0 | 0
damp_kd8 | 0/30 | 0.46/0.55/0.64 (n 30) | - | - | 0/0.0/0 (n 30) | - | 0.0 | 0
zero_torque | 0/30 | 0.24/0.32/0.36 (n 30) | - | - | 0/1.0/2 (n 30) | 0.12/0.16/0.22 (n 26) | 0.0 | 0
hold_last | 0/30 | 0.54/0.74/1.28 (n 30) | - | - | 0/0.0/1 (n 30) | 0.14/0.18/0.44 (n 5) | 0.0 | 0
fixstand | 0/30 | 0.52/0.72/1.54 (n 30) | - | - | 0/1.0/4 (n 30) | 0.08/0.48/1.08 (n 19) | 0.0233 | 0
zero_cmd_then_passive | 0/30 | 2.66/2.68/2.72 (n 30) | - | - | 1/1.0/2 (n 30) | 0.08/0.29/0.52 (n 30) | 0.0 | 0
zero_cmd_then_fixstand | 0/30 | 3.68/3.96/5.8 (n 30) | - | - | 1/1.0/2 (n 30) | 0.08/0.29/0.52 (n 30) | 0.0 | 0
zero_cmd_then_passive: upright at the hand-off 30/30; speed then 0.003/0.011/0.05 (n 30) m/s; fell after the hand-off in 0.46/0.46/0.48 (n 30) s
zero_cmd_then_fixstand: upright at the hand-off 30/30; speed then 0.003/0.011/0.05 (n 30) m/s; fell after the hand-off in 1.26/1.54/3.42 (n 30) s

## [menagerie] walking at 0.5 m/s: 2.0 s standing, then 4.0 s walking, then the stop at 30 moments 6.00-6.58 s
walking check: last 2 s moved 0.994 m forward, 0.140 m sideways (0.502 m/s); heading 8.4 deg; min pelvis z 0.774; no fall and no non-foot touch: True
mode | upright at end | first non-foot touch s after stop min/med/max | travel at end m (upright runs) | still after s | lift-offs after stop min/med/max | last lift-off s | legs over cap, max share | bad-qacc runs
continue | 30/30 | - | 3.014/3.015/3.017 (n 30) | - | 19/20.0/20 (n 30) | 5.7/5.84/5.98 (n 30) | 0.0 | 0
zero_cmd | 30/30 | - | 0.087/0.113/0.136 (n 30) | 1.12/1.22/1.34 (n 30) | 0/1.0/1 (n 30) | 0.04/0.19/0.4 (n 24) | 0.0 | 0
passive_kd3 | 0/30 | 0.4/0.46/0.52 (n 30) | - | - | 0/0.0/1 (n 30) | 0.34/0.34/0.34 (n 1) | 0.0 | 0
damp_kd8 | 0/30 | 0.58/0.69/0.8 (n 30) | - | - | 0/0.0/0 (n 30) | - | 0.0 | 0
zero_torque | 0/30 | 0.26/0.32/0.36 (n 30) | - | - | 0/0.0/2 (n 30) | 0.1/0.2/0.24 (n 13) | 0.0 | 0
hold_last | 0/30 | 0.72/0.84/1.26 (n 30) | - | - | 0/0.0/0 (n 30) | - | 0.0 | 0
fixstand | 0/30 | 0.72/0.86/1.36 (n 30) | - | - | 0/0.0/5 (n 30) | 0.06/0.58/1.02 (n 11) | 0.0 | 0
zero_cmd_then_passive | 0/30 | 2.66/2.68/2.68 (n 30) | - | - | 0/1.0/1 (n 30) | 0.04/0.19/0.4 (n 24) | 0.0 | 0
zero_cmd_then_fixstand | 0/30 | 4.24/4.68/5.68 (n 30) | - | - | 0/1.0/1 (n 30) | 0.04/0.19/0.4 (n 24) | 0.0 | 0
zero_cmd_then_passive: upright at the hand-off 30/30; speed then 0.003/0.008/0.012 (n 30) m/s; fell after the hand-off in 0.48/0.48/0.5 (n 30) s
zero_cmd_then_fixstand: upright at the hand-off 30/30; speed then 0.003/0.008/0.012 (n 30) m/s; fell after the hand-off in 1.8/2.24/3.28 (n 30) s

## clean start: 4 s standing at zero command, then the stop (6 s after it)
[mjlab] passive_kd3: upright at end False; first non-foot touch 0.7 s (left_shin_collision); end z 0.079, tilt 83.2; legs over cap 0.0, max request 24.3 N*m; bad-qacc 0
[mjlab] damp_kd8: upright at end False; first non-foot touch 1.16 s (head_collision); end z 0.08, tilt 84.3; legs over cap 0.0, max request 23.3 N*m; bad-qacc 0
[mjlab] zero_torque: upright at end False; first non-foot touch 0.4 s (left_hand_collision); end z 0.08, tilt 82.7; legs over cap 0.0, max request 0.0 N*m; bad-qacc 0
[mjlab] hold_last: upright at end False; first non-foot touch 3.6 s (pelvis_collision); end z 0.078, tilt 84.2; legs over cap 0.0, max request 42.4 N*m; bad-qacc 0
[mjlab] fixstand: upright at end False; first non-foot touch 2.9 s (left_shoulder_yaw_collision); end z 0.075, tilt 86.1; legs over cap 0.0, max request 40.9 N*m; bad-qacc 0
[mjlab] zero_cmd: upright at end True; first non-foot touch None s (None); end z 0.784, tilt 0.7; legs over cap 0.0, max request 5.3 N*m; bad-qacc 0
[menagerie] passive_kd3: upright at end False; first non-foot touch 0.7 s (left_knee_link geom 11); end z 0.152, tilt 128.0; legs over cap 0.0, max request 16.7 N*m; bad-qacc 0
[menagerie] damp_kd8: upright at end False; first non-foot touch 1.18 s (torso_link geom 41); end z 0.152, tilt 127.9; legs over cap 0.0, max request 23.3 N*m; bad-qacc 0
[menagerie] zero_torque: upright at end False; first non-foot touch 0.68 s (right_wrist_yaw_link geom 70); end z 0.067, tilt 86.2; legs over cap 0.0, max request 0.0 N*m; bad-qacc 0
[menagerie] hold_last: upright at end False; first non-foot touch 3.64 s (pelvis geom 3); end z 0.061, tilt 87.6; legs over cap 0.0, max request 34.4 N*m; bad-qacc 0
[menagerie] fixstand: upright at end False; first non-foot touch 3.42 s (pelvis geom 3); end z 0.061, tilt 87.9; legs over cap 0.0, max request 26.7 N*m; bad-qacc 0
[menagerie] zero_cmd: upright at end True; first non-foot touch None s (None); end z 0.784, tilt 0.9; legs over cap 0.0, max request 5.7 N*m; bad-qacc 0

saved 30 fps qpos (2 s before the stop + 6 s after, mjlab body, 0.5 m/s, moment 0): ['continue', 'damp_kd8', 'fixstand', 'hold_last', 'passive_kd3', 'zero_cmd', 'zero_cmd_then_fixstand', 'zero_cmd_then_passive', 'zero_torque']

## verdicts
seal 2c1d4b50 (mjlab policy keeps the G1 upright 10 s at zero command, numpy, sealed sim): True
REPRO zero-observation output matches the recon run: True
