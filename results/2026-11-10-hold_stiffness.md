# How stiff must a policy-off hold be to keep a G1 standing? (simulation)
numpy 2.0.2, mujoco 3.3.7
config g1.yaml: kps [100.0, 100.0, 100.0, 150.0, 40.0, 40.0, 100.0, 100.0, 100.0, 150.0, 40.0, 40.0] kds [2.0, 2.0, 2.0, 4.0, 2.0, 2.0, 2.0, 2.0, 2.0, 4.0, 2.0, 2.0] default_angles [-0.1, 0.0, 0.0, 0.3, -0.2, 0.0, -0.1, 0.0, 0.0, 0.3, -0.2, 0.0]
model: nq 19, nu 12, mass 32.11 kg, gravity 9.81, timestep 0.002
actuator ctrl range, N*m (unlimited = inf): left_hip_pitch_joint: inf, left_hip_roll_joint: inf, left_hip_yaw_joint: inf, left_knee_joint: inf, left_ankle_pitch_joint: inf, left_ankle_roll_joint: inf, right_hip_pitch_joint: inf, right_hip_roll_joint: inf, right_hip_yaw_joint: inf, right_knee_joint: inf, right_ankle_pitch_joint: inf, right_ankle_roll_joint: inf
actuator force range, N*m (unlimited = inf): left_hip_pitch_joint: inf, left_hip_roll_joint: inf, left_hip_yaw_joint: inf, left_knee_joint: inf, left_ankle_pitch_joint: inf, left_ankle_roll_joint: inf, right_hip_pitch_joint: inf, right_hip_roll_joint: inf, right_hip_yaw_joint: inf, right_knee_joint: inf, right_ankle_pitch_joint: inf, right_ankle_roll_joint: inf
joint torque caps (actuatorfrcrange), N*m: left_hip_pitch_joint: 88, left_hip_roll_joint: 139, left_hip_yaw_joint: 88, left_knee_joint: 139, left_ankle_pitch_joint: 50, left_ankle_roll_joint: 50, right_hip_pitch_joint: 88, right_hip_roll_joint: 139, right_hip_yaw_joint: 88, right_knee_joint: 139, right_ankle_pitch_joint: 50, right_ankle_roll_joint: 50
joint caps on: 'caps' model [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1], 'nocap' model [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
gains unitree: kp [100.0, 100.0, 100.0, 150.0, 40.0, 40.0, 100.0, 100.0, 100.0, 150.0, 40.0, 40.0] kd [2.0, 2.0, 2.0, 4.0, 2.0, 2.0, 2.0, 2.0, 2.0, 4.0, 2.0, 2.0]
gains ankle80: kp [100.0, 100.0, 100.0, 150.0, 80.0, 80.0, 100.0, 100.0, 100.0, 150.0, 80.0, 80.0] kd [2.0, 2.0, 2.0, 4.0, 4.0, 4.0, 2.0, 2.0, 2.0, 4.0, 4.0, 4.0]
gains ankle120: kp [100.0, 100.0, 100.0, 150.0, 120.0, 120.0, 100.0, 100.0, 100.0, 150.0, 120.0, 120.0] kd [2.0, 2.0, 2.0, 4.0, 6.0, 6.0, 2.0, 2.0, 2.0, 4.0, 6.0, 6.0]
gains ankle160: kp [100.0, 100.0, 100.0, 150.0, 160.0, 160.0, 100.0, 100.0, 100.0, 150.0, 160.0, 160.0] kd [2.0, 2.0, 2.0, 4.0, 8.0, 8.0, 2.0, 2.0, 2.0, 4.0, 8.0, 8.0]
gains ankle240: kp [100.0, 100.0, 100.0, 150.0, 240.0, 240.0, 100.0, 100.0, 100.0, 150.0, 240.0, 240.0] kd [2.0, 2.0, 2.0, 4.0, 12.0, 12.0, 2.0, 2.0, 2.0, 4.0, 12.0, 12.0]
gains ankle320: kp [100.0, 100.0, 100.0, 150.0, 320.0, 320.0, 100.0, 100.0, 100.0, 150.0, 320.0, 320.0] kd [2.0, 2.0, 2.0, 4.0, 16.0, 16.0, 2.0, 2.0, 2.0, 4.0, 16.0, 16.0]
gains legs_x2: kp [200.0, 200.0, 200.0, 300.0, 80.0, 80.0, 200.0, 200.0, 200.0, 300.0, 80.0, 80.0] kd [4.0, 4.0, 4.0, 8.0, 4.0, 4.0, 4.0, 4.0, 4.0, 8.0, 4.0, 4.0]
gains legs_x4: kp [400.0, 400.0, 400.0, 600.0, 160.0, 160.0, 400.0, 400.0, 400.0, 600.0, 160.0, 160.0] kd [8.0, 8.0, 8.0, 16.0, 8.0, 8.0, 8.0, 8.0, 8.0, 16.0, 8.0, 8.0]
colliding foot shapes: 10 (types [2, 7])

## Clean start: feet on the floor at the default angles, zero velocity, held 6 s
start: pelvis z 0.7847 m (lowest foot point was 0.0088 m before placing), mass 32.107 kg, centre of mass 0.6381 m above the ankle-pitch axes, 0.0463 m ahead of them, 0.0001 m left of their midpoint
gravity stiffness m*g*h about the ankle axes: 201.0 N*m/rad
model | hold | summed ankle-pitch kp N*m/rad | ratio to m*g*h | upright at end | first non-foot touch: shape, s, m/s | torso shape s | end pelvis z m | max tilt deg | still after s | share of steps with a requested torque over its cap | max |requested torque| at the ankles N*m (L pitch, L roll, R pitch, R roll) | max |applied torque| at the ankles N*m (same order) | bad-acceleration warnings
caps | policy on, zero command (control) | - | - | True | none | - | 0.776 | 4.9 | None | None | - | - | 0
caps | unitree | 80.0 | 0.398 | False | left_wrist_roll_rubber_hand, 1.252, 3.1 | - | 0.31 | 78.8 | 1.84 | 0.0 | [19.0, 0.8, 19.0, 0.8] | [19.0, 0.8, 19.0, 0.8] | 0
caps | ankle80 | 160.0 | 0.796 | False | left_wrist_roll_rubber_hand, 1.426, 3.21 | - | 0.309 | 75.8 | 1.76 | 0.0 | [20.0, 1.5, 20.0, 1.5] | [20.0, 1.5, 20.0, 1.5] | 0
caps | ankle120 | 240.0 | 1.194 | False | left_wrist_roll_rubber_hand, 1.586, 3.22 | - | 0.309 | 75.2 | 1.92 | 0.0 | [19.7, 1.6, 19.7, 1.6] | [19.7, 1.6, 19.7, 1.6] | 0
caps | ankle160 | 320.0 | 1.592 | False | left_wrist_roll_rubber_hand, 1.754, 3.24 | - | 0.309 | 75.2 | 2.04 | 0.0 | [19.4, 2.4, 19.4, 2.7] | [19.4, 2.4, 19.4, 2.7] | 0
caps | ankle240 | 480.0 | 2.388 | False | left_wrist_roll_rubber_hand, 2.192, 3.25 | - | 0.311 | 74.9 | 3.24 | 0.5987 | [34.8, 68.6, 29.1, 68.7] | [34.8, 50.0, 29.1, 50.0] | 0
caps | ankle320 | 640.0 | 3.184 | True | none | - | 0.782 | 8.5 | 1.08 | 0.0 | [17.8, 0.1, 17.8, 0.1] | [17.8, 0.1, 17.8, 0.1] | 0
caps | legs_x2 | 160.0 | 0.796 | False | left_wrist_roll_rubber_hand, 1.392, 3.18 | - | 0.316 | 76.6 | 1.72 | 0.0 | [20.1, 1.6, 20.1, 1.6] | [20.1, 1.6, 20.1, 1.6] | 0
caps | legs_x4 | 320.0 | 1.592 | False | left_wrist_roll_rubber_hand, 1.714, 3.22 | - | 0.32 | 75.9 | 1.96 | 0.0 | [23.5, 2.8, 23.5, 2.8] | [23.5, 2.8, 23.5, 2.8] | 0
nocap | policy on, zero command (control) | - | - | True | none | - | 0.776 | 4.9 | None | None | - | - | 0
nocap | unitree | 80.0 | 0.398 | False | left_wrist_roll_rubber_hand, 1.252, 3.1 | - | 0.31 | 78.8 | 1.84 | 0.0 | [19.0, 0.8, 19.0, 0.8] | [19.0, 0.8, 19.0, 0.8] | 0
nocap | ankle80 | 160.0 | 0.796 | False | left_wrist_roll_rubber_hand, 1.426, 3.21 | - | 0.309 | 75.8 | 1.76 | 0.0 | [20.0, 1.5, 20.0, 1.5] | [20.0, 1.5, 20.0, 1.5] | 0
nocap | ankle120 | 240.0 | 1.194 | False | left_wrist_roll_rubber_hand, 1.586, 3.22 | - | 0.309 | 75.2 | 1.92 | 0.0 | [19.7, 1.6, 19.7, 1.6] | [19.7, 1.6, 19.7, 1.6] | 0
nocap | ankle160 | 320.0 | 1.592 | False | left_wrist_roll_rubber_hand, 1.754, 3.24 | - | 0.309 | 75.2 | 2.04 | 0.0 | [19.4, 2.4, 19.4, 2.7] | [19.4, 2.4, 19.4, 2.7] | 0
nocap | ankle240 | 480.0 | 2.388 | True | left_wrist_roll_rubber_hand, 2.192, 3.25 | - | 0.778 | 96.5 | None | 0.0247 | [5173040.2, 4255664.9, 2424728.9, 6693567.8] | [1284231.7, 1731659.7, 45859.4, 74510.0] | 1
nocap | ankle320 | 640.0 | 3.184 | True | none | - | 0.782 | 8.5 | 1.08 | 0.0 | [17.8, 0.1, 17.8, 0.1] | [17.8, 0.1, 17.8, 0.1] | 0
nocap | legs_x2 | 160.0 | 0.796 | False | left_wrist_roll_rubber_hand, 1.392, 3.18 | - | 0.316 | 76.6 | 1.72 | 0.0 | [20.1, 1.6, 20.1, 1.6] | [20.1, 1.6, 20.1, 1.6] | 0
nocap | legs_x4 | 320.0 | 1.592 | False | left_wrist_roll_rubber_hand, 1.714, 3.22 | - | 0.32 | 75.9 | 1.96 | 0.0 | [23.5, 2.8, 23.5, 2.8] | [23.5, 2.8, 23.5, 2.8] | 0

## Hand-off: walking at cmd_init, stop at 20 moments 4.0-4.76 s, zero velocity 1 s, then policy off and each gain set pulls to the default angles; 6 s after the stop
model | t stop | gains | hand-off: feet, pelvis m/s, com above ankles m, com ahead m | first non-foot touch: shape, s, m/s | torso shape s | upright at end | end pelvis z m | max tilt after hand-off deg | still after s | travel m | share of steps with a requested torque over its cap | max |requested| / |applied| ankle torque N*m | walk: share of steps with a requested torque over its cap
caps | 4.0 | unitree | left, 0.045, 0.6213, 0.0431 | left_wrist_roll_rubber_hand, 2.356, 2.98 | - | False | 0.312 | 78.9 | 2.96 | 1.004 | 0.0 | 18.4 / 18.4 | 0.0
caps | 4.0 | ankle80 | left, 0.045, 0.6213, 0.0431 | right_wrist_roll_rubber_hand, 2.512, 3.23 | - | False | 0.311 | 76.0 | 2.88 | 1.007 | 0.0 | 19.5 / 19.5 | 0.0
caps | 4.0 | ankle120 | left, 0.045, 0.6213, 0.0431 | right_wrist_roll_rubber_hand, 2.646, 3.24 | - | False | 0.311 | 75.5 | 3.0 | 1.007 | 0.0 | 27.0 / 27.0 | 0.0
caps | 4.0 | ankle160 | left, 0.045, 0.6213, 0.0431 | left_wrist_roll_rubber_hand, 2.774, 3.21 | - | False | 0.311 | 75.2 | 3.16 | 1.008 | 0.0 | 36.0 / 36.0 | 0.0
caps | 4.0 | ankle240 | left, 0.045, 0.6213, 0.0431 | left_wrist_roll_rubber_hand, 3.036, 3.18 | - | False | 0.308 | 76.7 | 4.8 | 0.998 | 0.5828 | 68.6 / 50.0 | 0.0
caps | 4.0 | ankle320 | left, 0.045, 0.6213, 0.0431 | left_wrist_roll_rubber_hand, 3.782, 3.21 | - | False | 0.315 | 76.3 | None | 0.974 | 0.6292 | 122.1 / 50.0 | 0.0
caps | 4.0 | legs_x2 | left, 0.045, 0.6213, 0.0431 | left_wrist_roll_rubber_hand, 2.43, 3.21 | - | False | 0.316 | 76.5 | 2.76 | 1.007 | 0.0 | 21.0 / 21.0 | 0.0
caps | 4.0 | legs_x4 | left, 0.045, 0.6213, 0.0431 | left_wrist_roll_rubber_hand, 2.622, 3.21 | - | False | 0.32 | 76.0 | 2.88 | 1.003 | 0.0004 | 36.0 / 36.0 | 0.0
caps | 4.04 | unitree | left, 0.067, 0.6208, 0.0419 | left_wrist_roll_rubber_hand, 2.362, 2.98 | - | False | 0.311 | 78.7 | 2.96 | 1.021 | 0.0 | 18.2 / 18.2 | 0.0
caps | 4.04 | ankle80 | left, 0.067, 0.6208, 0.0419 | right_wrist_roll_rubber_hand, 2.516, 3.25 | - | False | 0.311 | 75.9 | 2.92 | 1.019 | 0.0 | 19.6 / 19.6 | 0.0
caps | 4.04 | ankle120 | left, 0.067, 0.6208, 0.0419 | left_wrist_roll_rubber_hand, 2.648, 3.21 | - | False | 0.31 | 75.5 | 3.04 | 1.018 | 0.0 | 28.1 / 28.1 | 0.0
caps | 4.04 | ankle160 | left, 0.067, 0.6208, 0.0419 | left_wrist_roll_rubber_hand, 2.776, 3.22 | - | False | 0.31 | 75.3 | 3.16 | 1.02 | 0.0 | 37.4 / 37.4 | 0.0
caps | 4.04 | ankle240 | left, 0.067, 0.6208, 0.0419 | left_wrist_roll_rubber_hand, 3.042, 3.18 | - | False | 0.31 | 77.7 | 4.2 | 1.017 | 0.5932 | 68.7 / 50.0 | 0.0
caps | 4.04 | ankle320 | left, 0.067, 0.6208, 0.0419 | left_wrist_roll_rubber_hand, 3.444, 3.23 | - | False | 0.311 | 75.0 | None | 0.961 | 0.5756 | 125.6 / 50.0 | 0.0
caps | 4.04 | legs_x2 | left, 0.067, 0.6208, 0.0419 | right_wrist_roll_rubber_hand, 2.432, 3.18 | - | False | 0.316 | 76.5 | 2.76 | 1.019 | 0.0 | 21.6 / 21.6 | 0.0
caps | 4.04 | legs_x4 | left, 0.067, 0.6208, 0.0419 | left_wrist_roll_rubber_hand, 2.626, 3.22 | - | False | 0.32 | 76.0 | 2.88 | 1.02 | 0.0008 | 37.4 / 37.4 | 0.0
caps | 4.08 | unitree | left, 0.087, 0.6211, 0.0417 | left_wrist_roll_rubber_hand, 2.354, 2.97 | - | False | 0.311 | 78.6 | 2.96 | 1.036 | 0.0 | 18.2 / 18.2 | 0.0
caps | 4.08 | ankle80 | left, 0.087, 0.6211, 0.0417 | right_wrist_roll_rubber_hand, 2.508, 3.28 | - | False | 0.311 | 75.8 | 2.88 | 1.032 | 0.0 | 19.9 / 19.9 | 0.0
caps | 4.08 | ankle120 | left, 0.087, 0.6211, 0.0417 | right_wrist_roll_rubber_hand, 2.64, 3.24 | - | False | 0.31 | 75.6 | 3.04 | 1.029 | 0.0 | 27.9 / 27.9 | 0.0
caps | 4.08 | ankle160 | left, 0.087, 0.6211, 0.0417 | left_wrist_roll_rubber_hand, 2.772, 3.21 | - | False | 0.31 | 75.3 | 3.16 | 1.031 | 0.0 | 37.2 / 37.2 | 0.0
caps | 4.08 | ankle240 | left, 0.087, 0.6211, 0.0417 | left_wrist_roll_rubber_hand, 3.05, 3.17 | - | False | 0.31 | 77.6 | 4.24 | 1.027 | 0.59 | 68.7 / 50.0 | 0.0
caps | 4.08 | ankle320 | left, 0.087, 0.6211, 0.0417 | left_wrist_roll_rubber_hand, 3.508, 3.21 | - | False | 0.311 | 75.8 | 4.84 | 0.977 | 0.5644 | 124.0 / 50.0 | 0.0
caps | 4.08 | legs_x2 | left, 0.087, 0.6211, 0.0417 | right_wrist_roll_rubber_hand, 2.432, 3.17 | - | False | 0.316 | 76.4 | 2.76 | 1.037 | 0.0 | 22.2 / 22.2 | 0.0
caps | 4.08 | legs_x4 | left, 0.087, 0.6211, 0.0417 | left_wrist_roll_rubber_hand, 2.628, 3.2 | - | False | 0.32 | 75.9 | 2.88 | 1.028 | 0.0008 | 37.2 / 37.2 | 0.0
caps | 4.12 | unitree | left, 0.11, 0.6222, 0.043 | left_wrist_roll_rubber_hand, 2.332, 2.99 | - | False | 0.311 | 78.8 | 2.92 | 1.037 | 0.0 | 18.7 / 18.7 | 0.0
caps | 4.12 | ankle80 | left, 0.11, 0.6222, 0.043 | right_wrist_roll_rubber_hand, 2.484, 3.3 | - | False | 0.31 | 75.8 | 2.88 | 1.032 | 0.0 | 19.8 / 19.8 | 0.0
caps | 4.12 | ankle120 | left, 0.11, 0.6222, 0.043 | right_wrist_roll_rubber_hand, 2.62, 3.25 | - | False | 0.31 | 75.5 | 3.04 | 1.03 | 0.0 | 24.4 / 24.4 | 0.0
caps | 4.12 | ankle160 | left, 0.11, 0.6222, 0.043 | left_wrist_roll_rubber_hand, 2.758, 3.22 | - | False | 0.31 | 75.3 | 3.16 | 1.031 | 0.0 | 32.6 / 32.6 | 0.0
caps | 4.12 | ankle240 | left, 0.11, 0.6222, 0.043 | left_wrist_roll_rubber_hand, 3.058, 3.18 | - | False | 0.309 | 76.4 | 4.12 | 1.025 | 0.5812 | 70.4 / 50.0 | 0.0
caps | 4.12 | ankle320 | left, 0.11, 0.6222, 0.043 | left_wrist_roll_rubber_hand, 3.644, 3.24 | - | False | 0.315 | 75.7 | None | 0.985 | 0.5416 | 128.1 / 50.0 | 0.0
caps | 4.12 | legs_x2 | left, 0.11, 0.6222, 0.043 | right_wrist_roll_rubber_hand, 2.424, 3.17 | - | False | 0.316 | 76.1 | 2.72 | 1.045 | 0.0 | 22.8 / 22.8 | 0.0
caps | 4.12 | legs_x4 | left, 0.11, 0.6222, 0.043 | left_wrist_roll_rubber_hand, 2.626, 3.21 | - | False | 0.32 | 76.0 | 2.88 | 1.035 | 0.0008 | 32.6 / 32.6 | 0.0
caps | 4.16 | unitree | left, 0.128, 0.6247, 0.0449 | left_wrist_roll_rubber_hand, 2.292, 3.04 | - | False | 0.312 | 79.2 | 2.88 | 1.034 | 0.0 | 19.8 / 19.8 | 0.0
caps | 4.16 | ankle80 | left, 0.128, 0.6247, 0.0449 | left_wrist_roll_rubber_hand, 2.45, 3.11 | - | False | 0.31 | 76.0 | 2.84 | 1.031 | 0.0 | 20.0 / 20.0 | 0.0
caps | 4.16 | ankle120 | left, 0.128, 0.6247, 0.0449 | left_wrist_roll_rubber_hand, 2.588, 3.16 | - | False | 0.31 | 75.3 | 2.96 | 1.029 | 0.0 | 19.5 / 19.5 | 0.0
caps | 4.16 | ankle160 | left, 0.128, 0.6247, 0.0449 | left_wrist_roll_rubber_hand, 2.72, 3.19 | - | False | 0.31 | 75.2 | 3.12 | 1.029 | 0.0 | 20.0 / 20.0 | 0.0
caps | 4.16 | ankle240 | left, 0.128, 0.6247, 0.0449 | left_wrist_roll_rubber_hand, 2.998, 3.21 | - | False | 0.31 | 75.4 | 4.08 | 1.022 | 0.5588 | 68.8 / 50.0 | 0.0
caps | 4.16 | ankle320 | left, 0.128, 0.6247, 0.0449 | right_wrist_roll_rubber_hand, 3.462, 3.24 | - | False | 0.314 | 75.8 | None | 0.992 | 0.58 | 123.8 / 50.0 | 0.0
caps | 4.16 | legs_x2 | left, 0.128, 0.6247, 0.0449 | right_wrist_roll_rubber_hand, 2.41, 3.17 | - | False | 0.316 | 76.2 | 3.0 | 1.045 | 0.0 | 23.1 / 23.1 | 0.0
caps | 4.16 | legs_x4 | left, 0.128, 0.6247, 0.0449 | left_wrist_roll_rubber_hand, 2.614, 3.21 | - | False | 0.32 | 76.0 | 2.84 | 1.038 | 0.0004 | 23.1 / 23.1 | 0.0
caps | 4.2 | unitree | left, 0.129, 0.629, 0.0438 | left_wrist_roll_rubber_hand, 2.248, 3.1 | - | False | 0.311 | 79.4 | 2.84 | 1.019 | 0.0 | 20.8 / 20.8 | 0.0
caps | 4.2 | ankle80 | left, 0.129, 0.629, 0.0438 | left_wrist_roll_rubber_hand, 2.414, 3.11 | - | False | 0.311 | 76.2 | 2.8 | 1.025 | 0.0 | 21.5 / 21.5 | 0.0
caps | 4.2 | ankle120 | left, 0.129, 0.629, 0.0438 | left_wrist_roll_rubber_hand, 2.558, 3.13 | - | False | 0.31 | 75.7 | 2.92 | 1.022 | 0.0 | 21.0 / 21.0 | 0.0
caps | 4.2 | ankle160 | left, 0.129, 0.629, 0.0438 | left_wrist_roll_rubber_hand, 2.696, 3.15 | - | False | 0.31 | 75.3 | 3.08 | 1.022 | 0.0 | 20.5 / 20.5 | 0.0
caps | 4.2 | ankle240 | left, 0.129, 0.629, 0.0438 | right_wrist_roll_rubber_hand, 3.002, 3.3 | - | False | 0.31 | 76.2 | 4.32 | 1.017 | 0.5784 | 70.4 / 50.0 | 0.0
caps | 4.2 | ankle320 | left, 0.129, 0.629, 0.0438 | left_wrist_roll_rubber_hand, 3.408, 3.22 | - | False | 0.312 | 77.3 | None | 1.01 | 0.5504 | 126.0 / 50.0 | 0.0
caps | 4.2 | legs_x2 | left, 0.129, 0.629, 0.0438 | right_wrist_roll_rubber_hand, 2.388, 3.19 | - | False | 0.316 | 75.9 | 3.08 | 1.035 | 0.0 | 22.9 / 22.9 | 0.0
caps | 4.2 | legs_x4 | left, 0.129, 0.629, 0.0438 | left_wrist_roll_rubber_hand, 2.574, 3.22 | - | False | 0.32 | 76.0 | 2.8 | 1.058 | 0.0 | 22.9 / 22.9 | 0.0
caps | 4.24 | unitree | left+right, 0.115, 0.6291, 0.0442 | left_wrist_roll_rubber_hand, 2.258, 3.04 | - | False | 0.311 | 79.2 | 2.84 | 1.013 | 0.0 | 19.7 / 19.7 | 0.0
caps | 4.24 | ankle80 | left+right, 0.115, 0.6291, 0.0442 | left_wrist_roll_rubber_hand, 2.432, 3.1 | - | False | 0.311 | 76.1 | 2.8 | 1.014 | 0.0 | 19.7 / 19.7 | 0.0
caps | 4.24 | ankle120 | left+right, 0.115, 0.6291, 0.0442 | right_wrist_roll_rubber_hand, 2.59, 3.29 | - | False | 0.31 | 75.3 | 2.96 | 1.013 | 0.0 | 20.2 / 20.2 | 0.0
caps | 4.24 | ankle160 | left+right, 0.115, 0.6291, 0.0442 | right_wrist_roll_rubber_hand, 2.75, 3.26 | - | False | 0.31 | 75.3 | 3.16 | 1.012 | 0.0 | 21.0 / 21.0 | 0.0
caps | 4.24 | ankle240 | left+right, 0.115, 0.6291, 0.0442 | left_wrist_roll_rubber_hand, 3.162, 3.23 | - | False | 0.313 | 75.3 | 5.2 | 1.016 | 0.5232 | 70.8 / 50.0 | 0.0
caps | 4.24 | ankle320 | left+right, 0.115, 0.6291, 0.0442 | left_wrist_roll_rubber_hand, 4.652, 3.18 | - | False | 0.312 | 76.9 | None | 0.977 | 0.374 | 124.3 / 50.0 | 0.0
caps | 4.24 | legs_x2 | left+right, 0.115, 0.6291, 0.0442 | right_wrist_roll_rubber_hand, 2.4, 3.17 | - | False | 0.316 | 76.2 | 3.0 | 1.028 | 0.0 | 23.2 / 23.2 | 0.0
caps | 4.24 | legs_x4 | left+right, 0.115, 0.6291, 0.0442 | left_wrist_roll_rubber_hand, 2.61, 3.21 | - | False | 0.32 | 76.0 | 2.84 | 1.033 | 0.0 | 23.1 / 23.1 | 0.0
caps | 4.28 | unitree | right, 0.07, 0.6258, 0.044 | left_wrist_roll_rubber_hand, 2.3, 3.09 | - | False | 0.311 | 78.9 | 2.88 | 1.012 | 0.0 | 19.5 / 19.5 | 0.0
caps | 4.28 | ankle80 | right, 0.07, 0.6258, 0.044 | left_wrist_roll_rubber_hand, 2.464, 3.18 | - | False | 0.311 | 76.0 | 2.8 | 1.015 | 0.0 | 20.1 / 20.1 | 0.0
caps | 4.28 | ankle120 | right, 0.07, 0.6258, 0.044 | right_wrist_roll_rubber_hand, 2.608, 3.21 | - | False | 0.311 | 75.4 | 2.96 | 1.015 | 0.0 | 23.7 / 23.7 | 0.0
caps | 4.28 | ankle160 | right, 0.07, 0.6258, 0.044 | left_wrist_roll_rubber_hand, 2.752, 3.27 | - | False | 0.311 | 75.3 | 3.08 | 1.016 | 0.0 | 31.6 / 31.6 | 0.0
caps | 4.28 | ankle240 | right, 0.07, 0.6258, 0.044 | left_wrist_roll_rubber_hand, 3.076, 3.26 | - | False | 0.311 | 75.5 | 4.72 | 1.014 | 0.5516 | 68.6 / 50.0 | 0.0
caps | 4.28 | ankle320 | right, 0.07, 0.6258, 0.044 | left_wrist_roll_rubber_hand, 3.52, 3.28 | - | False | 0.301 | 75.5 | None | 0.972 | 0.5952 | 125.8 / 50.0 | 0.0
caps | 4.28 | legs_x2 | right, 0.07, 0.6258, 0.044 | right_wrist_roll_rubber_hand, 2.408, 3.16 | - | False | 0.316 | 76.7 | 2.72 | 1.013 | 0.0 | 22.1 / 22.1 | 0.0
caps | 4.28 | legs_x4 | right, 0.07, 0.6258, 0.044 | left_wrist_roll_rubber_hand, 2.604, 3.21 | - | False | 0.32 | 75.9 | 2.84 | 1.035 | 0.0004 | 31.6 / 31.6 | 0.0
caps | 4.32 | unitree | right, 0.046, 0.6237, 0.0407 | left_wrist_roll_rubber_hand, 2.346, 3.11 | - | False | 0.311 | 78.9 | 2.96 | 1.004 | 0.0 | 19.3 / 19.3 | 0.0
caps | 4.32 | ankle80 | right, 0.046, 0.6237, 0.0407 | left_wrist_roll_rubber_hand, 2.502, 3.21 | - | False | 0.311 | 76.1 | 2.88 | 1.007 | 0.0 | 20.7 / 20.7 | 0.0
caps | 4.32 | ankle120 | right, 0.046, 0.6237, 0.0407 | left_wrist_roll_rubber_hand, 2.638, 3.28 | - | False | 0.311 | 75.5 | 3.0 | 1.007 | 0.0 | 27.7 / 27.7 | 0.0
caps | 4.32 | ankle160 | right, 0.046, 0.6237, 0.0407 | left_wrist_roll_rubber_hand, 2.768, 3.29 | - | False | 0.31 | 75.3 | 3.16 | 1.007 | 0.0 | 36.9 / 36.9 | 0.0
caps | 4.32 | ankle240 | right, 0.046, 0.6237, 0.0407 | left_wrist_roll_rubber_hand, 3.044, 3.28 | - | False | 0.311 | 75.9 | 4.2 | 1.002 | 0.5868 | 68.9 / 50.0 | 0.0
caps | 4.32 | ankle320 | right, 0.046, 0.6237, 0.0407 | left_wrist_roll_rubber_hand, 3.53, 3.23 | - | False | 0.303 | 76.0 | None | 0.956 | 0.5668 | 125.5 / 50.0 | 0.0
caps | 4.32 | legs_x2 | right, 0.046, 0.6237, 0.0407 | right_wrist_roll_rubber_hand, 2.432, 3.17 | - | False | 0.316 | 76.7 | 2.76 | 1.002 | 0.0 | 21.4 / 21.4 | 0.0
caps | 4.32 | legs_x4 | right, 0.046, 0.6237, 0.0407 | left_wrist_roll_rubber_hand, 2.616, 3.21 | - | False | 0.32 | 76.0 | 2.84 | 1.018 | 0.0004 | 36.9 / 36.9 | 0.0
caps | 4.36 | unitree | right, 0.032, 0.6228, 0.0382 | left_wrist_roll_rubber_hand, 2.36, 3.13 | - | False | 0.311 | 78.8 | 2.96 | 0.995 | 0.0 | 19.1 / 19.1 | 0.0
caps | 4.36 | ankle80 | right, 0.032, 0.6228, 0.0382 | right_wrist_roll_rubber_hand, 2.512, 3.17 | - | False | 0.311 | 76.2 | 2.92 | 0.996 | 0.0 | 20.8 / 20.8 | 0.0
caps | 4.36 | ankle120 | right, 0.032, 0.6228, 0.0382 | left_wrist_roll_rubber_hand, 2.646, 3.28 | - | False | 0.31 | 75.6 | 3.04 | 0.997 | 0.0 | 25.1 / 25.1 | 0.0
caps | 4.36 | ankle160 | right, 0.032, 0.6228, 0.0382 | left_wrist_roll_rubber_hand, 2.772, 3.29 | - | False | 0.31 | 75.3 | 3.16 | 0.997 | 0.0 | 33.5 / 33.5 | 0.0
caps | 4.36 | ankle240 | right, 0.032, 0.6228, 0.0382 | left_wrist_roll_rubber_hand, 3.032, 3.28 | - | False | 0.308 | 75.2 | 4.24 | 0.987 | 0.5532 | 68.7 / 50.0 | 0.0
caps | 4.36 | ankle320 | right, 0.032, 0.6228, 0.0382 | left_wrist_roll_rubber_hand, 3.276, 3.4 | - | False | 0.307 | 76.1 | None | 0.987 | 0.6764 | 123.8 / 50.0 | 0.0
caps | 4.36 | legs_x2 | right, 0.032, 0.6228, 0.0382 | right_wrist_roll_rubber_hand, 2.434, 3.17 | - | False | 0.316 | 76.6 | 2.76 | 0.992 | 0.0 | 21.1 / 21.1 | 0.0
caps | 4.36 | legs_x4 | right, 0.032, 0.6228, 0.0382 | left_wrist_roll_rubber_hand, 2.612, 3.2 | - | False | 0.32 | 76.0 | 2.84 | 1.001 | 0.0004 | 33.5 / 33.5 | 0.0
caps | 4.4 | unitree | right, 0.028, 0.6224, 0.0377 | left_wrist_roll_rubber_hand, 2.352, 3.14 | - | False | 0.311 | 78.7 | 2.96 | 0.992 | 0.0 | 18.9 / 18.9 | 0.0
caps | 4.4 | ankle80 | right, 0.028, 0.6224, 0.0377 | left_wrist_roll_rubber_hand, 2.504, 3.23 | - | False | 0.311 | 76.1 | 2.88 | 0.994 | 0.0 | 20.8 / 20.8 | 0.0
caps | 4.4 | ankle120 | right, 0.028, 0.6224, 0.0377 | left_wrist_roll_rubber_hand, 2.636, 3.27 | - | False | 0.31 | 75.5 | 3.0 | 0.994 | 0.0 | 23.5 / 23.5 | 0.0
caps | 4.4 | ankle160 | right, 0.028, 0.6224, 0.0377 | left_wrist_roll_rubber_hand, 2.762, 3.29 | - | False | 0.31 | 75.3 | 3.16 | 0.994 | 0.0 | 31.4 / 31.4 | 0.0
caps | 4.4 | ankle240 | right, 0.028, 0.6224, 0.0377 | left_wrist_roll_rubber_hand, 3.026, 3.25 | - | False | 0.311 | 77.6 | 4.52 | 0.989 | 0.6032 | 70.5 / 50.0 | 0.0
caps | 4.4 | ankle320 | right, 0.028, 0.6224, 0.0377 | left_wrist_roll_rubber_hand, 3.602, 3.34 | - | False | 0.314 | 76.9 | 5.2 | 1.006 | 0.67 | 121.9 / 50.0 | 0.0
caps | 4.4 | legs_x2 | right, 0.028, 0.6224, 0.0377 | right_wrist_roll_rubber_hand, 2.424, 3.18 | - | False | 0.316 | 76.7 | 2.76 | 0.988 | 0.0 | 21.0 / 21.0 | 0.0
caps | 4.4 | legs_x4 | right, 0.028, 0.6224, 0.0377 | left_wrist_roll_rubber_hand, 2.604, 3.21 | - | False | 0.32 | 75.8 | 2.84 | 1.002 | 0.0004 | 31.4 / 31.4 | 0.0
caps | 4.44 | unitree | right, 0.037, 0.6221, 0.0397 | left_wrist_roll_rubber_hand, 2.33, 3.12 | - | False | 0.311 | 78.9 | 2.92 | 0.994 | 0.0 | 19.0 / 19.0 | 0.0
caps | 4.44 | ankle80 | right, 0.037, 0.6221, 0.0397 | right_wrist_roll_rubber_hand, 2.482, 3.18 | - | False | 0.31 | 76.1 | 2.88 | 0.996 | 0.0 | 20.8 / 20.8 | 0.0
caps | 4.44 | ankle120 | right, 0.037, 0.6221, 0.0397 | left_wrist_roll_rubber_hand, 2.616, 3.27 | - | False | 0.31 | 75.5 | 3.0 | 0.997 | 0.0 | 23.2 / 23.2 | 0.0
caps | 4.44 | ankle160 | right, 0.037, 0.6221, 0.0397 | left_wrist_roll_rubber_hand, 2.744, 3.27 | - | False | 0.31 | 75.3 | 3.16 | 0.996 | 0.0 | 30.9 / 30.9 | 0.0
caps | 4.44 | ankle240 | right, 0.037, 0.6221, 0.0397 | left_wrist_roll_rubber_hand, 3.02, 3.27 | - | False | 0.311 | 77.1 | 4.56 | 0.993 | 0.59 | 72.8 / 50.0 | 0.0
caps | 4.44 | ankle320 | right, 0.037, 0.6221, 0.0397 | left_wrist_roll_rubber_hand, 3.386, 3.19 | 4.584 | False | 0.061 | 90.4 | None | 1.024 | 0.9132 | 124.3 / 50.0 | 0.0
caps | 4.44 | legs_x2 | right, 0.037, 0.6221, 0.0397 | left_wrist_roll_rubber_hand, 2.41, 3.2 | - | False | 0.316 | 76.4 | 2.72 | 0.998 | 0.0 | 20.8 / 20.8 | 0.0
caps | 4.44 | legs_x4 | right, 0.037, 0.6221, 0.0397 | left_wrist_roll_rubber_hand, 2.6, 3.21 | - | False | 0.32 | 75.7 | 2.84 | 1.003 | 0.0004 | 30.9 / 30.9 | 0.0
caps | 4.48 | unitree | right, 0.056, 0.6226, 0.0416 | left_wrist_roll_rubber_hand, 2.318, 3.1 | - | False | 0.311 | 78.9 | 2.92 | 1.001 | 0.0 | 19.4 / 19.4 | 0.0
caps | 4.48 | ankle80 | right, 0.056, 0.6226, 0.0416 | left_wrist_roll_rubber_hand, 2.472, 3.23 | - | False | 0.311 | 76.2 | 2.88 | 1.003 | 0.0 | 20.8 / 20.8 | 0.0
caps | 4.48 | ankle120 | right, 0.056, 0.6226, 0.0416 | right_wrist_roll_rubber_hand, 2.606, 3.2 | - | False | 0.31 | 75.5 | 3.0 | 1.002 | 0.0 | 22.3 / 22.3 | 0.0
caps | 4.48 | ankle160 | right, 0.056, 0.6226, 0.0416 | left_wrist_roll_rubber_hand, 2.74, 3.27 | - | False | 0.31 | 75.2 | 3.16 | 1.002 | 0.0 | 29.7 / 29.7 | 0.0
caps | 4.48 | ankle240 | right, 0.056, 0.6226, 0.0416 | left_wrist_roll_rubber_hand, 3.034, 3.28 | - | False | 0.309 | 75.3 | 4.48 | 0.998 | 0.564 | 68.7 / 50.0 | 0.0
caps | 4.48 | ankle320 | right, 0.056, 0.6226, 0.0416 | right_wrist_roll_rubber_hand, 3.33, 3.21 | - | False | 0.311 | 77.0 | 5.04 | 0.993 | 0.6504 | 122.3 / 50.0 | 0.0
caps | 4.48 | legs_x2 | right, 0.056, 0.6226, 0.0416 | left_wrist_roll_rubber_hand, 2.404, 3.19 | - | False | 0.316 | 76.7 | 2.72 | 0.998 | 0.0 | 20.5 / 20.5 | 0.0
caps | 4.48 | legs_x4 | right, 0.056, 0.6226, 0.0416 | left_wrist_roll_rubber_hand, 2.612, 3.2 | - | False | 0.32 | 76.0 | 2.88 | 1.004 | 0.0004 | 29.7 / 29.7 | 0.0
caps | 4.52 | unitree | right, 0.079, 0.6243, 0.0432 | right_wrist_roll_rubber_hand, 2.306, 3.11 | - | False | 0.311 | 79.1 | 2.92 | 1.001 | 0.0 | 19.8 / 19.8 | 0.0
caps | 4.52 | ankle80 | right, 0.079, 0.6243, 0.0432 | left_wrist_roll_rubber_hand, 2.462, 3.22 | - | False | 0.311 | 76.1 | 2.84 | 1.005 | 0.0 | 20.9 / 20.9 | 0.0
caps | 4.52 | ankle120 | right, 0.079, 0.6243, 0.0432 | left_wrist_roll_rubber_hand, 2.602, 3.26 | - | False | 0.31 | 75.5 | 3.0 | 1.005 | 0.0 | 20.6 / 20.6 | 0.0
caps | 4.52 | ankle160 | right, 0.079, 0.6243, 0.0432 | left_wrist_roll_rubber_hand, 2.74, 3.26 | - | False | 0.31 | 75.2 | 3.16 | 1.004 | 0.0 | 25.1 / 25.1 | 0.0
caps | 4.52 | ankle240 | right, 0.079, 0.6243, 0.0432 | left_wrist_roll_rubber_hand, 3.058, 3.26 | - | False | 0.311 | 75.2 | 3.96 | 1.003 | 0.5268 | 70.9 / 50.0 | 0.0
caps | 4.52 | ankle320 | right, 0.079, 0.6243, 0.0432 | left_wrist_roll_rubber_hand, 4.008, 3.2 | - | False | 0.305 | 77.2 | None | 0.953 | 0.4892 | 125.2 / 50.0 | 0.0
caps | 4.52 | legs_x2 | right, 0.079, 0.6243, 0.0432 | left_wrist_roll_rubber_hand, 2.406, 3.2 | - | False | 0.316 | 76.5 | 2.72 | 1.012 | 0.0 | 20.1 / 20.1 | 0.0
caps | 4.52 | legs_x4 | right, 0.079, 0.6243, 0.0432 | left_wrist_roll_rubber_hand, 2.63, 3.2 | - | False | 0.32 | 76.0 | 2.88 | 1.008 | 0.0004 | 25.1 / 25.1 | 0.0
caps | 4.56 | unitree | right, 0.102, 0.6277, 0.0428 | left_wrist_roll_rubber_hand, 2.296, 3.02 | - | False | 0.312 | 79.2 | 2.88 | 1.002 | 0.0 | 20.3 / 20.3 | 0.0
caps | 4.56 | ankle80 | right, 0.102, 0.6277, 0.0428 | right_wrist_roll_rubber_hand, 2.456, 3.17 | - | False | 0.311 | 76.1 | 2.84 | 1.005 | 0.0 | 21.5 / 21.5 | 0.0
caps | 4.56 | ankle120 | right, 0.102, 0.6277, 0.0428 | left_wrist_roll_rubber_hand, 2.602, 3.26 | - | False | 0.31 | 75.5 | 3.0 | 1.005 | 0.0 | 21.3 / 21.3 | 0.0
caps | 4.56 | ankle160 | right, 0.102, 0.6277, 0.0428 | left_wrist_roll_rubber_hand, 2.748, 3.27 | - | False | 0.31 | 75.3 | 3.16 | 1.005 | 0.0 | 20.6 / 20.6 | 0.0
caps | 4.56 | ankle240 | right, 0.102, 0.6277, 0.0428 | left_wrist_roll_rubber_hand, 3.102, 3.25 | - | False | 0.31 | 76.6 | 4.28 | 0.999 | 0.5716 | 68.8 / 50.0 | 0.0
caps | 4.56 | ankle320 | right, 0.102, 0.6277, 0.0428 | right_wrist_roll_rubber_hand, 4.342, 3.41 | - | False | 0.318 | 74.7 | 2.44 | 0.947 | 0.3796 | 124.0 / 50.0 | 0.0
caps | 4.56 | legs_x2 | right, 0.102, 0.6277, 0.0428 | left_wrist_roll_rubber_hand, 2.414, 3.2 | - | False | 0.316 | 76.4 | 2.72 | 1.015 | 0.0 | 20.4 / 20.4 | 0.0
caps | 4.56 | legs_x4 | right, 0.102, 0.6277, 0.0428 | left_wrist_roll_rubber_hand, 2.646, 3.19 | - | False | 0.32 | 75.9 | 2.88 | 1.015 | 0.0 | 22.1 / 22.1 | 0.0
caps | 4.6 | unitree | left+right, 0.11, 0.632, 0.039 | left_wrist_roll_rubber_hand, 2.288, 2.95 | - | False | 0.311 | 79.5 | 2.88 | 0.996 | 0.0 | 20.8 / 20.8 | 0.0
caps | 4.6 | ankle80 | left+right, 0.11, 0.632, 0.039 | right_wrist_roll_rubber_hand, 2.458, 3.18 | - | False | 0.311 | 76.1 | 2.84 | 1.002 | 0.0 | 22.0 / 22.0 | 0.0
caps | 4.6 | ankle120 | left+right, 0.11, 0.632, 0.039 | left_wrist_roll_rubber_hand, 2.612, 3.25 | - | False | 0.31 | 75.5 | 3.0 | 1.002 | 0.0 | 21.6 / 21.6 | 0.0
caps | 4.6 | ankle160 | left+right, 0.11, 0.632, 0.039 | left_wrist_roll_rubber_hand, 2.768, 3.26 | - | False | 0.31 | 75.4 | 3.16 | 1.001 | 0.0 | 20.8 / 20.8 | 0.0
caps | 4.6 | ankle240 | left+right, 0.11, 0.632, 0.039 | left_wrist_roll_rubber_hand, 3.172, 3.22 | - | False | 0.308 | 77.1 | 4.24 | 0.997 | 0.5488 | 68.6 / 50.0 | 0.0
caps | 4.6 | ankle320 | left+right, 0.11, 0.632, 0.039 | left_wrist_roll_rubber_hand, 4.842, 3.39 | - | False | 0.315 | 74.7 | 2.44 | 0.909 | 0.3012 | 123.2 / 50.0 | 0.0
caps | 4.6 | legs_x2 | left+right, 0.11, 0.632, 0.039 | left_wrist_roll_rubber_hand, 2.42, 3.19 | - | False | 0.316 | 76.6 | 2.76 | 1.007 | 0.0 | 20.9 / 20.9 | 0.0
caps | 4.6 | legs_x4 | left+right, 0.11, 0.632, 0.039 | left_wrist_roll_rubber_hand, 2.666, 3.21 | - | False | 0.32 | 76.0 | 2.92 | 1.015 | 0.0 | 22.8 / 22.8 | 0.0
caps | 4.64 | unitree | left, 0.094, 0.6317, 0.0398 | right_wrist_roll_rubber_hand, 2.31, 3.13 | - | False | 0.312 | 78.9 | 2.88 | 0.998 | 0.0 | 20.1 / 20.1 | 0.0
caps | 4.64 | ankle80 | left, 0.094, 0.6317, 0.0398 | left_wrist_roll_rubber_hand, 2.484, 3.22 | - | False | 0.311 | 76.1 | 2.88 | 1.001 | 0.0 | 20.5 / 20.5 | 0.0
caps | 4.64 | ankle120 | left, 0.094, 0.6317, 0.0398 | left_wrist_roll_rubber_hand, 2.64, 3.23 | - | False | 0.31 | 75.5 | 3.04 | 1.001 | 0.0 | 19.8 / 19.8 | 0.0
caps | 4.64 | ankle160 | left, 0.094, 0.6317, 0.0398 | left_wrist_roll_rubber_hand, 2.798, 3.23 | - | False | 0.31 | 75.4 | 3.2 | 1.0 | 0.0 | 20.0 / 20.0 | 0.0
caps | 4.64 | ankle240 | left, 0.094, 0.6317, 0.0398 | left_wrist_roll_rubber_hand, 3.172, 3.25 | - | False | 0.309 | 75.3 | 3.96 | 0.997 | 0.5252 | 69.5 / 50.0 | 0.0
caps | 4.64 | ankle320 | left, 0.094, 0.6317, 0.0398 | right_wrist_roll_rubber_hand, 4.498, 3.32 | - | False | 0.317 | 75.4 | 2.32 | 0.918 | 0.3668 | 122.4 / 50.0 | 0.0
caps | 4.64 | legs_x2 | left, 0.094, 0.6317, 0.0398 | left_wrist_roll_rubber_hand, 2.436, 3.18 | - | False | 0.316 | 76.6 | 2.76 | 1.008 | 0.0 | 21.1 / 21.1 | 0.0
caps | 4.64 | legs_x4 | left, 0.094, 0.6317, 0.0398 | left_wrist_roll_rubber_hand, 2.686, 3.21 | - | False | 0.32 | 75.9 | 2.92 | 1.016 | 0.0 | 23.3 / 23.3 | 0.0
caps | 4.68 | unitree | left, 0.043, 0.6274, 0.045 | left_wrist_roll_rubber_hand, 2.31, 3.08 | - | False | 0.312 | 78.8 | 2.88 | 1.005 | 0.0 | 19.4 / 19.4 | 0.0
caps | 4.68 | ankle80 | left, 0.043, 0.6274, 0.045 | left_wrist_roll_rubber_hand, 2.474, 3.2 | - | False | 0.311 | 76.1 | 2.88 | 1.006 | 0.0 | 20.3 / 20.3 | 0.0
caps | 4.68 | ankle120 | left, 0.043, 0.6274, 0.045 | left_wrist_roll_rubber_hand, 2.622, 3.21 | - | False | 0.31 | 75.4 | 3.0 | 1.008 | 0.0 | 27.7 / 27.7 | 0.0
caps | 4.68 | ankle160 | left, 0.043, 0.6274, 0.045 | left_wrist_roll_rubber_hand, 2.768, 3.22 | - | False | 0.31 | 75.3 | 3.16 | 1.007 | 0.0 | 36.9 / 36.9 | 0.0
caps | 4.68 | ankle240 | left, 0.043, 0.6274, 0.045 | left_wrist_roll_rubber_hand, 3.094, 3.23 | - | False | 0.314 | 76.5 | 4.2 | 1.011 | 0.5712 | 69.0 / 50.0 | 0.0
caps | 4.68 | ankle320 | left, 0.043, 0.6274, 0.045 | left_wrist_roll_rubber_hand, 3.384, 3.19 | - | False | 0.309 | 75.9 | None | 1.006 | 0.7192 | 125.7 / 50.0 | 0.0
caps | 4.68 | legs_x2 | left, 0.043, 0.6274, 0.045 | left_wrist_roll_rubber_hand, 2.418, 3.18 | - | False | 0.316 | 76.6 | 2.72 | 1.009 | 0.0 | 21.1 / 21.1 | 0.0
caps | 4.68 | legs_x4 | left, 0.043, 0.6274, 0.045 | left_wrist_roll_rubber_hand, 2.656, 3.22 | - | False | 0.32 | 76.0 | 2.88 | 1.017 | 0.0004 | 36.9 / 36.9 | 0.0
caps | 4.72 | unitree | left, 0.015, 0.6235, 0.0474 | left_wrist_roll_rubber_hand, 2.31, 3.06 | - | False | 0.312 | 78.8 | 2.88 | 1.012 | 0.0 | 19.2 / 19.2 | 0.0
caps | 4.72 | ankle80 | left, 0.015, 0.6235, 0.0474 | left_wrist_roll_rubber_hand, 2.468, 3.19 | - | False | 0.311 | 76.0 | 2.84 | 1.014 | 0.0 | 20.5 / 20.5 | 0.0
caps | 4.72 | ankle120 | left, 0.015, 0.6235, 0.0474 | left_wrist_roll_rubber_hand, 2.606, 3.19 | - | False | 0.311 | 75.4 | 2.96 | 1.015 | 0.0 | 28.8 / 28.8 | 0.0
caps | 4.72 | ankle160 | left, 0.015, 0.6235, 0.0474 | left_wrist_roll_rubber_hand, 2.744, 3.23 | - | False | 0.311 | 75.2 | 3.08 | 1.015 | 0.0 | 38.5 / 38.5 | 0.0
caps | 4.72 | ankle240 | left, 0.015, 0.6235, 0.0474 | right_wrist_roll_rubber_hand, 3.04, 3.26 | - | False | 0.309 | 78.1 | 4.84 | 1.006 | 0.5908 | 72.0 / 50.0 | 0.0
caps | 4.72 | ankle320 | left, 0.015, 0.6235, 0.0474 | left_wrist_roll_rubber_hand, 3.674, 3.24 | - | False | 0.307 | 76.7 | None | 0.995 | 0.5192 | 123.4 / 50.0 | 0.0
caps | 4.72 | legs_x2 | left, 0.015, 0.6235, 0.0474 | left_wrist_roll_rubber_hand, 2.406, 3.19 | - | False | 0.316 | 76.5 | 2.72 | 1.012 | 0.0 | 20.6 / 20.6 | 0.0
caps | 4.72 | legs_x4 | left, 0.015, 0.6235, 0.0474 | left_wrist_roll_rubber_hand, 2.622, 3.21 | - | False | 0.32 | 76.0 | 2.88 | 1.02 | 0.0004 | 38.5 / 38.5 | 0.0
caps | 4.76 | unitree | left, 0.025, 0.6218, 0.0462 | left_wrist_roll_rubber_hand, 2.334, 3.02 | - | False | 0.312 | 79.0 | 2.92 | 1.006 | 0.0 | 18.7 / 18.7 | 0.0
caps | 4.76 | ankle80 | left, 0.025, 0.6218, 0.0462 | left_wrist_roll_rubber_hand, 2.49, 3.18 | - | False | 0.311 | 76.0 | 2.84 | 1.01 | 0.0 | 20.2 / 20.2 | 0.0
caps | 4.76 | ankle120 | left, 0.025, 0.6218, 0.0462 | left_wrist_roll_rubber_hand, 2.626, 3.21 | - | False | 0.311 | 75.4 | 2.96 | 1.01 | 0.0 | 27.2 / 27.2 | 0.0
caps | 4.76 | ankle160 | left, 0.025, 0.6218, 0.0462 | left_wrist_roll_rubber_hand, 2.756, 3.23 | - | False | 0.311 | 75.3 | 3.08 | 1.011 | 0.0 | 36.3 / 36.3 | 0.0
caps | 4.76 | ankle240 | left, 0.025, 0.6218, 0.0462 | left_wrist_roll_rubber_hand, 3.034, 3.23 | - | False | 0.313 | 75.4 | 4.6 | 1.01 | 0.558 | 68.7 / 50.0 | 0.0
caps | 4.76 | ankle320 | left, 0.025, 0.6218, 0.0462 | left_wrist_roll_rubber_hand, 3.648, 3.22 | - | False | 0.311 | 76.8 | None | 0.983 | 0.6404 | 125.1 / 50.0 | 0.0
caps | 4.76 | legs_x2 | left, 0.025, 0.6218, 0.0462 | left_wrist_roll_rubber_hand, 2.416, 3.2 | - | False | 0.316 | 76.6 | 2.72 | 1.004 | 0.0 | 19.9 / 19.9 | 0.0
caps | 4.76 | legs_x4 | left, 0.025, 0.6218, 0.0462 | left_wrist_roll_rubber_hand, 2.618, 3.21 | - | False | 0.32 | 76.0 | 2.84 | 1.013 | 0.0004 | 36.3 / 36.3 | 0.0
nocap | 4.0 | unitree | left, 0.045, 0.6213, 0.0431 | left_wrist_roll_rubber_hand, 2.356, 2.98 | - | False | 0.312 | 78.9 | 2.96 | 1.004 | 0.0 | 18.4 / 18.4 | 0.0
nocap | 4.0 | ankle80 | left, 0.045, 0.6213, 0.0431 | right_wrist_roll_rubber_hand, 2.512, 3.23 | - | False | 0.311 | 76.0 | 2.88 | 1.007 | 0.0 | 19.5 / 19.5 | 0.0
nocap | 4.0 | ankle120 | left, 0.045, 0.6213, 0.0431 | right_wrist_roll_rubber_hand, 2.646, 3.24 | - | False | 0.311 | 75.5 | 3.0 | 1.007 | 0.0 | 27.0 / 27.0 | 0.0
nocap | 4.0 | ankle160 | left, 0.045, 0.6213, 0.0431 | left_wrist_roll_rubber_hand, 2.774, 3.21 | - | False | 0.311 | 75.2 | 3.16 | 1.008 | 0.0 | 36.0 / 36.0 | 0.0
nocap | 4.0 | ankle240 | left, 0.045, 0.6213, 0.0431 | pelvis_contour_link, 1.084, 6141.37 | 1.084 | True | 0.786 | 74.9 | None | 1.764 | 0.0484 | 5173040.2 / 1485232.9 | 0.0
nocap | 4.0 | ankle320 | left, 0.045, 0.6213, 0.0431 | pelvis_contour_link, 1.032, 5008.01 | 1.032 | True | 0.793 | 69.0 | None | 1.758 | 1.0 | 16635784.5 / 1136996.5 | 0.0
nocap | 4.0 | legs_x2 | left, 0.045, 0.6213, 0.0431 | left_wrist_roll_rubber_hand, 2.43, 3.21 | - | False | 0.316 | 76.5 | 2.76 | 1.007 | 0.0 | 21.0 / 21.0 | 0.0
nocap | 4.0 | legs_x4 | left, 0.045, 0.6213, 0.0431 | left_wrist_roll_rubber_hand, 2.622, 3.21 | - | False | 0.32 | 75.9 | 2.88 | 1.005 | 0.0004 | 36.0 / 36.0 | 0.0
nocap | 4.04 | unitree | left, 0.067, 0.6208, 0.0419 | left_wrist_roll_rubber_hand, 2.362, 2.98 | - | False | 0.311 | 78.7 | 2.96 | 1.021 | 0.0 | 18.2 / 18.2 | 0.0
nocap | 4.04 | ankle80 | left, 0.067, 0.6208, 0.0419 | right_wrist_roll_rubber_hand, 2.516, 3.25 | - | False | 0.311 | 75.9 | 2.92 | 1.019 | 0.0 | 19.6 / 19.6 | 0.0
nocap | 4.04 | ankle120 | left, 0.067, 0.6208, 0.0419 | left_wrist_roll_rubber_hand, 2.648, 3.21 | - | False | 0.31 | 75.5 | 3.04 | 1.018 | 0.0 | 28.1 / 28.1 | 0.0
nocap | 4.04 | ankle160 | left, 0.067, 0.6208, 0.0419 | left_wrist_roll_rubber_hand, 2.776, 3.22 | - | False | 0.31 | 75.3 | 3.16 | 1.02 | 0.0 | 37.4 / 37.4 | 0.0
nocap | 4.04 | ankle240 | left, 0.067, 0.6208, 0.0419 | pelvis_contour_link, 1.11, 2943.91 | 1.11 | True | 0.789 | 77.0 | None | 1.78 | 0.0544 | 5173040.2 / 400474.6 | 0.0
nocap | 4.04 | ankle320 | left, 0.067, 0.6208, 0.0419 | pelvis_contour_link, 1.034, 9212.57 | 1.034 | True | 0.793 | 69.0 | None | 1.777 | 1.0 | 16635784.5 / 922948.0 | 0.0
nocap | 4.04 | legs_x2 | left, 0.067, 0.6208, 0.0419 | right_wrist_roll_rubber_hand, 2.432, 3.18 | - | False | 0.316 | 76.5 | 2.76 | 1.019 | 0.0 | 21.6 / 21.6 | 0.0
nocap | 4.04 | legs_x4 | left, 0.067, 0.6208, 0.0419 | left_wrist_roll_rubber_hand, 2.626, 3.21 | - | False | 0.32 | 76.0 | 2.88 | 1.017 | 0.0004 | 37.4 / 37.4 | 0.0
nocap | 4.08 | unitree | left, 0.087, 0.6211, 0.0417 | left_wrist_roll_rubber_hand, 2.354, 2.97 | - | False | 0.311 | 78.6 | 2.96 | 1.036 | 0.0 | 18.2 / 18.2 | 0.0
nocap | 4.08 | ankle80 | left, 0.087, 0.6211, 0.0417 | right_wrist_roll_rubber_hand, 2.508, 3.28 | - | False | 0.311 | 75.8 | 2.88 | 1.032 | 0.0 | 19.9 / 19.9 | 0.0
nocap | 4.08 | ankle120 | left, 0.087, 0.6211, 0.0417 | right_wrist_roll_rubber_hand, 2.64, 3.24 | - | False | 0.31 | 75.6 | 3.04 | 1.029 | 0.0 | 27.9 / 27.9 | 0.0
nocap | 4.08 | ankle160 | left, 0.087, 0.6211, 0.0417 | left_wrist_roll_rubber_hand, 2.772, 3.21 | - | False | 0.31 | 75.3 | 3.16 | 1.031 | 0.0 | 37.2 / 37.2 | 0.0
nocap | 4.08 | ankle240 | left, 0.087, 0.6211, 0.0417 | left_wrist_roll_rubber_hand, 3.372, 3.26 | - | True | 0.785 | 76.1 | None | 1.8 | 0.0472 | 85095585.5 / 2783312.6 | 0.0
nocap | 4.08 | ankle320 | left, 0.087, 0.6211, 0.0417 | none | - | True | 0.793 | 69.0 | None | 1.793 | 1.0 | 25002268.9 / 2350561.2 | 0.0
nocap | 4.08 | legs_x2 | left, 0.087, 0.6211, 0.0417 | right_wrist_roll_rubber_hand, 2.432, 3.17 | - | False | 0.316 | 76.4 | 2.76 | 1.037 | 0.0 | 22.2 / 22.2 | 0.0
nocap | 4.08 | legs_x4 | left, 0.087, 0.6211, 0.0417 | left_wrist_roll_rubber_hand, 2.63, 3.21 | - | False | 0.32 | 75.9 | 2.88 | 1.034 | 0.0004 | 37.2 / 37.2 | 0.0
nocap | 4.12 | unitree | left, 0.11, 0.6222, 0.043 | left_wrist_roll_rubber_hand, 2.332, 2.99 | - | False | 0.311 | 78.8 | 2.92 | 1.037 | 0.0 | 18.7 / 18.7 | 0.0
nocap | 4.12 | ankle80 | left, 0.11, 0.6222, 0.043 | right_wrist_roll_rubber_hand, 2.484, 3.3 | - | False | 0.31 | 75.8 | 2.88 | 1.032 | 0.0 | 19.8 / 19.8 | 0.0
nocap | 4.12 | ankle120 | left, 0.11, 0.6222, 0.043 | right_wrist_roll_rubber_hand, 2.62, 3.25 | - | False | 0.31 | 75.5 | 3.04 | 1.03 | 0.0 | 24.4 / 24.4 | 0.0
nocap | 4.12 | ankle160 | left, 0.11, 0.6222, 0.043 | left_wrist_roll_rubber_hand, 2.758, 3.22 | - | False | 0.31 | 75.3 | 3.16 | 1.031 | 0.0 | 32.6 / 32.6 | 0.0
nocap | 4.12 | ankle240 | left, 0.11, 0.6222, 0.043 | pelvis_contour_link, 1.08, 10657.6 | 1.08 | True | 0.785 | 75.0 | None | 1.817 | 0.0468 | 5232794.7 / 222170.0 | 0.0
nocap | 4.12 | ankle320 | left, 0.11, 0.6222, 0.043 | none | - | True | 0.793 | 69.0 | None | 1.81 | 1.0 | 16635784.5 / 922948.0 | 0.0
nocap | 4.12 | legs_x2 | left, 0.11, 0.6222, 0.043 | right_wrist_roll_rubber_hand, 2.424, 3.17 | - | False | 0.316 | 76.1 | 2.72 | 1.045 | 0.0 | 22.8 / 22.8 | 0.0
nocap | 4.12 | legs_x4 | left, 0.11, 0.6222, 0.043 | left_wrist_roll_rubber_hand, 2.626, 3.2 | - | False | 0.32 | 76.0 | 2.84 | 1.038 | 0.0004 | 32.6 / 32.6 | 0.0
nocap | 4.16 | unitree | left, 0.128, 0.6247, 0.0449 | left_wrist_roll_rubber_hand, 2.292, 3.04 | - | False | 0.312 | 79.2 | 2.88 | 1.034 | 0.0 | 19.8 / 19.8 | 0.0
nocap | 4.16 | ankle80 | left, 0.128, 0.6247, 0.0449 | left_wrist_roll_rubber_hand, 2.45, 3.11 | - | False | 0.31 | 76.0 | 2.84 | 1.031 | 0.0 | 20.0 / 20.0 | 0.0
nocap | 4.16 | ankle120 | left, 0.128, 0.6247, 0.0449 | left_wrist_roll_rubber_hand, 2.588, 3.16 | - | False | 0.31 | 75.3 | 2.96 | 1.029 | 0.0 | 19.5 / 19.5 | 0.0
nocap | 4.16 | ankle160 | left, 0.128, 0.6247, 0.0449 | left_wrist_roll_rubber_hand, 2.72, 3.19 | - | False | 0.31 | 75.2 | 3.12 | 1.029 | 0.0 | 20.0 / 20.0 | 0.0
nocap | 4.16 | ankle240 | left, 0.128, 0.6247, 0.0449 | pelvis_contour_link, 1.102, 541.6 | - | True | 0.788 | 75.5 | None | 1.833 | 0.0512 | 7293316.3 / 79107.3 | 0.0
nocap | 4.16 | ankle320 | left, 0.128, 0.6247, 0.0449 | pelvis_contour_link, 1.042, 1278.75 | 1.042 | True | 1.063 | 69.0 | None | 1.813 | 0.9996 | 16635784.5 / 922948.0 | 0.0
nocap | 4.16 | legs_x2 | left, 0.128, 0.6247, 0.0449 | right_wrist_roll_rubber_hand, 2.41, 3.17 | - | False | 0.316 | 76.2 | 3.0 | 1.045 | 0.0 | 23.1 / 23.1 | 0.0
nocap | 4.16 | legs_x4 | left, 0.128, 0.6247, 0.0449 | left_wrist_roll_rubber_hand, 2.614, 3.21 | - | False | 0.32 | 76.0 | 2.84 | 1.038 | 0.0004 | 23.1 / 23.1 | 0.0
nocap | 4.2 | unitree | left, 0.129, 0.629, 0.0438 | left_wrist_roll_rubber_hand, 2.248, 3.1 | - | False | 0.311 | 79.4 | 2.84 | 1.019 | 0.0 | 20.8 / 20.8 | 0.0
nocap | 4.2 | ankle80 | left, 0.129, 0.629, 0.0438 | left_wrist_roll_rubber_hand, 2.414, 3.11 | - | False | 0.311 | 76.2 | 2.8 | 1.025 | 0.0 | 21.5 / 21.5 | 0.0
nocap | 4.2 | ankle120 | left, 0.129, 0.629, 0.0438 | left_wrist_roll_rubber_hand, 2.558, 3.13 | - | False | 0.31 | 75.7 | 2.92 | 1.022 | 0.0 | 21.0 / 21.0 | 0.0
nocap | 4.2 | ankle160 | left, 0.129, 0.629, 0.0438 | left_wrist_roll_rubber_hand, 2.696, 3.15 | - | False | 0.31 | 75.3 | 3.08 | 1.022 | 0.0 | 20.5 / 20.5 | 0.0
nocap | 4.2 | ankle240 | left, 0.129, 0.629, 0.0438 | left_wrist_roll_rubber_hand, 3.002, 3.2 | 3.154 | True | 0.782 | 75.4 | None | 1.839 | 0.0344 | 5173040.2 / 590806.2 | 0.0
nocap | 4.2 | ankle320 | left, 0.129, 0.629, 0.0438 | left_hip_yaw_link, 1.032, 2660.11 | - | True | 0.793 | 69.0 | None | 1.847 | 0.9992 | 16635784.5 / 922948.0 | 0.0
nocap | 4.2 | legs_x2 | left, 0.129, 0.629, 0.0438 | right_wrist_roll_rubber_hand, 2.388, 3.19 | - | False | 0.316 | 75.9 | 3.08 | 1.035 | 0.0 | 22.9 / 22.9 | 0.0
nocap | 4.2 | legs_x4 | left, 0.129, 0.629, 0.0438 | left_wrist_roll_rubber_hand, 2.574, 3.22 | - | False | 0.32 | 76.0 | 2.8 | 1.058 | 0.0 | 22.9 / 22.9 | 0.0
nocap | 4.24 | unitree | left+right, 0.115, 0.6291, 0.0442 | left_wrist_roll_rubber_hand, 2.258, 3.04 | - | False | 0.311 | 79.2 | 2.84 | 1.013 | 0.0 | 19.7 / 19.7 | 0.0
nocap | 4.24 | ankle80 | left+right, 0.115, 0.6291, 0.0442 | left_wrist_roll_rubber_hand, 2.432, 3.1 | - | False | 0.311 | 76.1 | 2.8 | 1.014 | 0.0 | 19.7 / 19.7 | 0.0
nocap | 4.24 | ankle120 | left+right, 0.115, 0.6291, 0.0442 | right_wrist_roll_rubber_hand, 2.59, 3.29 | - | False | 0.31 | 75.3 | 2.96 | 1.013 | 0.0 | 20.2 / 20.2 | 0.0
nocap | 4.24 | ankle160 | left+right, 0.115, 0.6291, 0.0442 | right_wrist_roll_rubber_hand, 2.75, 3.26 | - | False | 0.31 | 75.3 | 3.16 | 1.012 | 0.0 | 21.0 / 21.0 | 0.0
nocap | 4.24 | ankle240 | left+right, 0.115, 0.6291, 0.0442 | pelvis_contour_link, 1.194, 7503.96 | 1.194 | False | 0.326 | 117.0 | None | 1.007 | 0.0536 | 5173040.2 / 391347.7 | 0.0
nocap | 4.24 | ankle320 | left+right, 0.115, 0.6291, 0.0442 | pelvis_contour_link, 1.032, 613.98 | - | True | 0.793 | 69.0 | None | 1.866 | 0.9992 | 16635784.5 / 922948.0 | 0.0
nocap | 4.24 | legs_x2 | left+right, 0.115, 0.6291, 0.0442 | right_wrist_roll_rubber_hand, 2.4, 3.17 | - | False | 0.316 | 76.2 | 3.0 | 1.028 | 0.0 | 23.2 / 23.2 | 0.0
nocap | 4.24 | legs_x4 | left+right, 0.115, 0.6291, 0.0442 | left_wrist_roll_rubber_hand, 2.61, 3.21 | - | False | 0.32 | 76.0 | 2.84 | 1.033 | 0.0 | 23.1 / 23.1 | 0.0
nocap | 4.28 | unitree | right, 0.07, 0.6258, 0.044 | left_wrist_roll_rubber_hand, 2.3, 3.09 | - | False | 0.311 | 78.9 | 2.88 | 1.012 | 0.0 | 19.5 / 19.5 | 0.0
nocap | 4.28 | ankle80 | right, 0.07, 0.6258, 0.044 | left_wrist_roll_rubber_hand, 2.464, 3.18 | - | False | 0.311 | 76.0 | 2.8 | 1.015 | 0.0 | 20.1 / 20.1 | 0.0
nocap | 4.28 | ankle120 | right, 0.07, 0.6258, 0.044 | right_wrist_roll_rubber_hand, 2.608, 3.21 | - | False | 0.311 | 75.4 | 2.96 | 1.015 | 0.0 | 23.7 / 23.7 | 0.0
nocap | 4.28 | ankle160 | right, 0.07, 0.6258, 0.044 | left_wrist_roll_rubber_hand, 2.752, 3.27 | - | False | 0.311 | 75.3 | 3.08 | 1.016 | 0.0 | 31.6 / 31.6 | 0.0
nocap | 4.28 | ankle240 | right, 0.07, 0.6258, 0.044 | pelvis_contour_link, 1.19, 6083.87 | 1.19 | False | 0.327 | 120.0 | None | 1.026 | 0.0548 | 5173040.2 / 2539931.9 | 0.0
nocap | 4.28 | ankle320 | right, 0.07, 0.6258, 0.044 | none | - | True | 0.793 | 69.0 | None | 1.886 | 1.0 | 16635784.5 / 922948.0 | 0.0
nocap | 4.28 | legs_x2 | right, 0.07, 0.6258, 0.044 | right_wrist_roll_rubber_hand, 2.408, 3.16 | - | False | 0.316 | 76.7 | 2.72 | 1.013 | 0.0 | 22.1 / 22.1 | 0.0
nocap | 4.28 | legs_x4 | right, 0.07, 0.6258, 0.044 | left_wrist_roll_rubber_hand, 2.604, 3.21 | - | False | 0.32 | 75.8 | 2.84 | 1.036 | 0.0004 | 31.6 / 31.6 | 0.0
nocap | 4.32 | unitree | right, 0.046, 0.6237, 0.0407 | left_wrist_roll_rubber_hand, 2.346, 3.11 | - | False | 0.311 | 78.9 | 2.96 | 1.004 | 0.0 | 19.3 / 19.3 | 0.0
nocap | 4.32 | ankle80 | right, 0.046, 0.6237, 0.0407 | left_wrist_roll_rubber_hand, 2.502, 3.21 | - | False | 0.311 | 76.1 | 2.88 | 1.007 | 0.0 | 20.7 / 20.7 | 0.0
nocap | 4.32 | ankle120 | right, 0.046, 0.6237, 0.0407 | left_wrist_roll_rubber_hand, 2.638, 3.28 | - | False | 0.311 | 75.5 | 3.0 | 1.007 | 0.0 | 27.7 / 27.7 | 0.0
nocap | 4.32 | ankle160 | right, 0.046, 0.6237, 0.0407 | left_wrist_roll_rubber_hand, 2.768, 3.29 | - | False | 0.31 | 75.3 | 3.16 | 1.007 | 0.0 | 36.9 / 36.9 | 0.0
nocap | 4.32 | ankle240 | right, 0.046, 0.6237, 0.0407 | left_wrist_roll_rubber_hand, 3.048, 3.28 | - | True | 0.782 | 75.2 | None | 1.902 | 0.0352 | 5173040.2 / 623257.9 | 0.0
nocap | 4.32 | ankle320 | right, 0.046, 0.6237, 0.0407 | pelvis_contour_link, 1.04, 47231.9 | 1.04 | True | 0.793 | 164.4 | None | 1.904 | 1.0 | 16635784.5 / 922948.0 | 0.0
nocap | 4.32 | legs_x2 | right, 0.046, 0.6237, 0.0407 | right_wrist_roll_rubber_hand, 2.432, 3.17 | - | False | 0.316 | 76.7 | 2.76 | 1.002 | 0.0 | 21.4 / 21.4 | 0.0
nocap | 4.32 | legs_x4 | right, 0.046, 0.6237, 0.0407 | left_wrist_roll_rubber_hand, 2.616, 3.21 | - | False | 0.32 | 76.0 | 2.88 | 1.016 | 0.0004 | 36.9 / 36.9 | 0.0
nocap | 4.36 | unitree | right, 0.032, 0.6228, 0.0382 | left_wrist_roll_rubber_hand, 2.36, 3.13 | - | False | 0.311 | 78.8 | 2.96 | 0.995 | 0.0 | 19.1 / 19.1 | 0.0
nocap | 4.36 | ankle80 | right, 0.032, 0.6228, 0.0382 | right_wrist_roll_rubber_hand, 2.512, 3.17 | - | False | 0.311 | 76.2 | 2.92 | 0.996 | 0.0 | 20.8 / 20.8 | 0.0
nocap | 4.36 | ankle120 | right, 0.032, 0.6228, 0.0382 | left_wrist_roll_rubber_hand, 2.646, 3.28 | - | False | 0.31 | 75.6 | 3.04 | 0.997 | 0.0 | 25.1 / 25.1 | 0.0
nocap | 4.36 | ankle160 | right, 0.032, 0.6228, 0.0382 | left_wrist_roll_rubber_hand, 2.772, 3.29 | - | False | 0.31 | 75.3 | 3.16 | 0.997 | 0.0 | 33.5 / 33.5 | 0.0
nocap | 4.36 | ankle240 | right, 0.032, 0.6228, 0.0382 | left_wrist_roll_rubber_hand, 3.434, 3.26 | - | True | 0.793 | 75.4 | None | 1.925 | 0.0524 | 5173040.2 / 100605.0 | 0.0
nocap | 4.36 | ankle320 | right, 0.032, 0.6228, 0.0382 | none | - | True | 1.051 | 69.0 | None | 1.917 | 1.0 | 16635784.5 / 922948.0 | 0.0
nocap | 4.36 | legs_x2 | right, 0.032, 0.6228, 0.0382 | right_wrist_roll_rubber_hand, 2.434, 3.17 | - | False | 0.316 | 76.6 | 2.76 | 0.992 | 0.0 | 21.1 / 21.1 | 0.0
nocap | 4.36 | legs_x4 | right, 0.032, 0.6228, 0.0382 | left_wrist_roll_rubber_hand, 2.612, 3.2 | - | False | 0.32 | 76.0 | 2.84 | 1.001 | 0.0004 | 33.5 / 33.5 | 0.0
nocap | 4.4 | unitree | right, 0.028, 0.6224, 0.0377 | left_wrist_roll_rubber_hand, 2.352, 3.14 | - | False | 0.311 | 78.7 | 2.96 | 0.992 | 0.0 | 18.9 / 18.9 | 0.0
nocap | 4.4 | ankle80 | right, 0.028, 0.6224, 0.0377 | left_wrist_roll_rubber_hand, 2.504, 3.23 | - | False | 0.311 | 76.1 | 2.88 | 0.994 | 0.0 | 20.8 / 20.8 | 0.0
nocap | 4.4 | ankle120 | right, 0.028, 0.6224, 0.0377 | left_wrist_roll_rubber_hand, 2.636, 3.27 | - | False | 0.31 | 75.5 | 3.0 | 0.994 | 0.0 | 23.5 / 23.5 | 0.0
nocap | 4.4 | ankle160 | right, 0.028, 0.6224, 0.0377 | left_wrist_roll_rubber_hand, 2.762, 3.29 | - | False | 0.31 | 75.3 | 3.16 | 0.994 | 0.0 | 31.4 / 31.4 | 0.0
nocap | 4.4 | ankle240 | right, 0.028, 0.6224, 0.0377 | pelvis_contour_link, 1.072, 2869.22 | 1.072 | True | 0.785 | 75.5 | None | 1.952 | 0.0448 | 36790872.7 / 74510.0 | 0.0
nocap | 4.4 | ankle320 | right, 0.028, 0.6224, 0.0377 | pelvis_contour_link, 1.034, 517.36 | 1.034 | True | 0.793 | 69.0 | None | 1.946 | 1.0 | 16635784.5 / 1234790.5 | 0.0
nocap | 4.4 | legs_x2 | right, 0.028, 0.6224, 0.0377 | right_wrist_roll_rubber_hand, 2.424, 3.18 | - | False | 0.316 | 76.7 | 2.76 | 0.988 | 0.0 | 21.0 / 21.0 | 0.0
nocap | 4.4 | legs_x4 | right, 0.028, 0.6224, 0.0377 | left_wrist_roll_rubber_hand, 2.604, 3.21 | - | False | 0.32 | 75.9 | 2.84 | 0.999 | 0.0004 | 31.4 / 31.4 | 0.0
nocap | 4.44 | unitree | right, 0.037, 0.6221, 0.0397 | left_wrist_roll_rubber_hand, 2.33, 3.12 | - | False | 0.311 | 78.9 | 2.92 | 0.994 | 0.0 | 19.0 / 19.0 | 0.0
nocap | 4.44 | ankle80 | right, 0.037, 0.6221, 0.0397 | right_wrist_roll_rubber_hand, 2.482, 3.18 | - | False | 0.31 | 76.1 | 2.88 | 0.996 | 0.0 | 20.8 / 20.8 | 0.0
nocap | 4.44 | ankle120 | right, 0.037, 0.6221, 0.0397 | left_wrist_roll_rubber_hand, 2.616, 3.27 | - | False | 0.31 | 75.5 | 3.0 | 0.997 | 0.0 | 23.2 / 23.2 | 0.0
nocap | 4.44 | ankle160 | right, 0.037, 0.6221, 0.0397 | left_wrist_roll_rubber_hand, 2.744, 3.27 | - | False | 0.31 | 75.3 | 3.16 | 0.996 | 0.0 | 30.9 / 30.9 | 0.0
nocap | 4.44 | ankle240 | right, 0.037, 0.6221, 0.0397 | left_wrist_roll_rubber_hand, 3.37, 3.26 | - | True | 0.785 | 77.0 | None | 1.973 | 0.0448 | 5173040.2 / 466974.6 | 0.0
nocap | 4.44 | ankle320 | right, 0.037, 0.6221, 0.0397 | pelvis_contour_link, 1.048, 1761.06 | 1.048 | True | 1.051 | 69.0 | None | 1.958 | 1.0 | 33499210.7 / 922948.0 | 0.0
nocap | 4.44 | legs_x2 | right, 0.037, 0.6221, 0.0397 | left_wrist_roll_rubber_hand, 2.41, 3.2 | - | False | 0.316 | 76.4 | 2.72 | 0.998 | 0.0 | 20.8 / 20.8 | 0.0
nocap | 4.44 | legs_x4 | right, 0.037, 0.6221, 0.0397 | left_wrist_roll_rubber_hand, 2.6, 3.21 | - | False | 0.32 | 75.8 | 2.84 | 1.0 | 0.0004 | 30.9 / 30.9 | 0.0
nocap | 4.48 | unitree | right, 0.056, 0.6226, 0.0416 | left_wrist_roll_rubber_hand, 2.318, 3.1 | - | False | 0.311 | 78.9 | 2.92 | 1.001 | 0.0 | 19.4 / 19.4 | 0.0
nocap | 4.48 | ankle80 | right, 0.056, 0.6226, 0.0416 | left_wrist_roll_rubber_hand, 2.472, 3.23 | - | False | 0.311 | 76.2 | 2.88 | 1.003 | 0.0 | 20.8 / 20.8 | 0.0
nocap | 4.48 | ankle120 | right, 0.056, 0.6226, 0.0416 | right_wrist_roll_rubber_hand, 2.606, 3.2 | - | False | 0.31 | 75.5 | 3.0 | 1.002 | 0.0 | 22.3 / 22.3 | 0.0
nocap | 4.48 | ankle160 | right, 0.056, 0.6226, 0.0416 | left_wrist_roll_rubber_hand, 2.74, 3.27 | - | False | 0.31 | 75.2 | 3.16 | 1.002 | 0.0 | 29.7 / 29.7 | 0.0
nocap | 4.48 | ankle240 | right, 0.056, 0.6226, 0.0416 | left_knee_link, 1.246, 72.34 | 1.314 | False | 0.431 | 131.0 | None | 1.221 | 0.0812 | 6202231.5 / 218004.4 | 0.0
nocap | 4.48 | ankle320 | right, 0.056, 0.6226, 0.0416 | pelvis_contour_link, 1.032, 12385.36 | 1.032 | True | 0.793 | 69.0 | None | 1.985 | 1.0 | 16635784.5 / 1984960.8 | 0.0
nocap | 4.48 | legs_x2 | right, 0.056, 0.6226, 0.0416 | left_wrist_roll_rubber_hand, 2.404, 3.19 | - | False | 0.316 | 76.7 | 2.72 | 0.998 | 0.0 | 20.5 / 20.5 | 0.0
nocap | 4.48 | legs_x4 | right, 0.056, 0.6226, 0.0416 | left_wrist_roll_rubber_hand, 2.612, 3.19 | - | False | 0.32 | 76.0 | 2.84 | 1.006 | 0.0004 | 29.7 / 29.7 | 0.0
nocap | 4.52 | unitree | right, 0.079, 0.6243, 0.0432 | right_wrist_roll_rubber_hand, 2.306, 3.11 | - | False | 0.311 | 79.1 | 2.92 | 1.001 | 0.0 | 19.8 / 19.8 | 0.0
nocap | 4.52 | ankle80 | right, 0.079, 0.6243, 0.0432 | left_wrist_roll_rubber_hand, 2.462, 3.22 | - | False | 0.311 | 76.1 | 2.84 | 1.005 | 0.0 | 20.9 / 20.9 | 0.0
nocap | 4.52 | ankle120 | right, 0.079, 0.6243, 0.0432 | left_wrist_roll_rubber_hand, 2.602, 3.26 | - | False | 0.31 | 75.5 | 3.0 | 1.005 | 0.0 | 20.6 / 20.6 | 0.0
nocap | 4.52 | ankle160 | right, 0.079, 0.6243, 0.0432 | left_wrist_roll_rubber_hand, 2.74, 3.26 | - | False | 0.31 | 75.2 | 3.16 | 1.004 | 0.0 | 25.1 / 25.1 | 0.0
nocap | 4.52 | ankle240 | right, 0.079, 0.6243, 0.0432 | left_wrist_roll_rubber_hand, 3.408, 3.26 | - | True | 0.789 | 78.1 | None | 2.007 | 0.0528 | 20533397.9 / 250865.9 | 0.0
nocap | 4.52 | ankle320 | right, 0.079, 0.6243, 0.0432 | none | - | True | 0.793 | 69.0 | None | 2.003 | 1.0 | 16635784.5 / 922948.0 | 0.0
nocap | 4.52 | legs_x2 | right, 0.079, 0.6243, 0.0432 | left_wrist_roll_rubber_hand, 2.406, 3.2 | - | False | 0.316 | 76.5 | 2.72 | 1.012 | 0.0 | 20.1 / 20.1 | 0.0
nocap | 4.52 | legs_x4 | right, 0.079, 0.6243, 0.0432 | left_wrist_roll_rubber_hand, 2.63, 3.2 | - | False | 0.32 | 75.9 | 2.88 | 1.009 | 0.0004 | 25.1 / 25.1 | 0.0
nocap | 4.56 | unitree | right, 0.102, 0.6277, 0.0428 | left_wrist_roll_rubber_hand, 2.296, 3.02 | - | False | 0.312 | 79.2 | 2.88 | 1.002 | 0.0 | 20.3 / 20.3 | 0.0
nocap | 4.56 | ankle80 | right, 0.102, 0.6277, 0.0428 | right_wrist_roll_rubber_hand, 2.456, 3.17 | - | False | 0.311 | 76.1 | 2.84 | 1.005 | 0.0 | 21.5 / 21.5 | 0.0
nocap | 4.56 | ankle120 | right, 0.102, 0.6277, 0.0428 | left_wrist_roll_rubber_hand, 2.602, 3.26 | - | False | 0.31 | 75.5 | 3.0 | 1.005 | 0.0 | 21.3 / 21.3 | 0.0
nocap | 4.56 | ankle160 | right, 0.102, 0.6277, 0.0428 | left_wrist_roll_rubber_hand, 2.748, 3.27 | - | False | 0.31 | 75.3 | 3.16 | 1.005 | 0.0 | 20.6 / 20.6 | 0.0
nocap | 4.56 | ankle240 | right, 0.102, 0.6277, 0.0428 | left_wrist_roll_rubber_hand, 3.456, 3.26 | - | False | 0.332 | 149.8 | None | 1.157 | 0.0552 | 18993251.6 / 275917.8 | 0.0
nocap | 4.56 | ankle320 | right, 0.102, 0.6277, 0.0428 | pelvis_contour_link, 1.034, 3484.74 | 1.034 | True | 0.793 | 69.0 | None | 2.022 | 0.9996 | 16635784.5 / 922948.0 | 0.0
nocap | 4.56 | legs_x2 | right, 0.102, 0.6277, 0.0428 | left_wrist_roll_rubber_hand, 2.414, 3.2 | - | False | 0.316 | 76.4 | 2.72 | 1.015 | 0.0 | 20.4 / 20.4 | 0.0
nocap | 4.56 | legs_x4 | right, 0.102, 0.6277, 0.0428 | left_wrist_roll_rubber_hand, 2.646, 3.19 | - | False | 0.32 | 75.9 | 2.88 | 1.015 | 0.0 | 22.1 / 22.1 | 0.0
nocap | 4.6 | unitree | left+right, 0.11, 0.632, 0.039 | left_wrist_roll_rubber_hand, 2.288, 2.95 | - | False | 0.311 | 79.5 | 2.88 | 0.996 | 0.0 | 20.8 / 20.8 | 0.0
nocap | 4.6 | ankle80 | left+right, 0.11, 0.632, 0.039 | right_wrist_roll_rubber_hand, 2.458, 3.18 | - | False | 0.311 | 76.1 | 2.84 | 1.002 | 0.0 | 22.0 / 22.0 | 0.0
nocap | 4.6 | ankle120 | left+right, 0.11, 0.632, 0.039 | left_wrist_roll_rubber_hand, 2.612, 3.25 | - | False | 0.31 | 75.5 | 3.0 | 1.002 | 0.0 | 21.6 / 21.6 | 0.0
nocap | 4.6 | ankle160 | left+right, 0.11, 0.632, 0.039 | left_wrist_roll_rubber_hand, 2.768, 3.26 | - | False | 0.31 | 75.4 | 3.16 | 1.001 | 0.0 | 20.8 / 20.8 | 0.0
nocap | 4.6 | ankle240 | left+right, 0.11, 0.632, 0.039 | left_wrist_roll_rubber_hand, 3.172, 3.25 | 3.188 | True | 0.782 | 75.0 | None | 2.034 | 0.0304 | 9272261.6 / 409376.6 | 0.0
nocap | 4.6 | ankle320 | left+right, 0.11, 0.632, 0.039 | none | - | True | 0.792 | 69.0 | 2.44 | 2.041 | 0.3036 | 16635784.5 / 922948.0 | 0.0
nocap | 4.6 | legs_x2 | left+right, 0.11, 0.632, 0.039 | left_wrist_roll_rubber_hand, 2.42, 3.19 | - | False | 0.316 | 76.6 | 2.76 | 1.007 | 0.0 | 20.9 / 20.9 | 0.0
nocap | 4.6 | legs_x4 | left+right, 0.11, 0.632, 0.039 | left_wrist_roll_rubber_hand, 2.666, 3.21 | - | False | 0.32 | 76.0 | 2.92 | 1.015 | 0.0 | 22.8 / 22.8 | 0.0
nocap | 4.64 | unitree | left, 0.094, 0.6317, 0.0398 | right_wrist_roll_rubber_hand, 2.31, 3.13 | - | False | 0.312 | 78.9 | 2.88 | 0.998 | 0.0 | 20.1 / 20.1 | 0.0
nocap | 4.64 | ankle80 | left, 0.094, 0.6317, 0.0398 | left_wrist_roll_rubber_hand, 2.484, 3.22 | - | False | 0.311 | 76.1 | 2.88 | 1.001 | 0.0 | 20.5 / 20.5 | 0.0
nocap | 4.64 | ankle120 | left, 0.094, 0.6317, 0.0398 | left_wrist_roll_rubber_hand, 2.64, 3.23 | - | False | 0.31 | 75.5 | 3.04 | 1.001 | 0.0 | 19.8 / 19.8 | 0.0
nocap | 4.64 | ankle160 | left, 0.094, 0.6317, 0.0398 | left_wrist_roll_rubber_hand, 2.798, 3.23 | - | False | 0.31 | 75.4 | 3.2 | 1.0 | 0.0 | 20.0 / 20.0 | 0.0
nocap | 4.64 | ankle240 | left, 0.094, 0.6317, 0.0398 | left_wrist_roll_rubber_hand, 3.172, 3.25 | - | True | 0.782 | 81.1 | None | 2.057 | 0.0364 | 7739901.5 / 268837.4 | 0.0
nocap | 4.64 | ankle320 | left, 0.094, 0.6317, 0.0398 | pelvis_contour_link, 1.034, 966.01 | 1.034 | True | 0.793 | 69.0 | None | 2.059 | 0.9988 | 87697420.9 / 922948.0 | 0.0
nocap | 4.64 | legs_x2 | left, 0.094, 0.6317, 0.0398 | left_wrist_roll_rubber_hand, 2.436, 3.18 | - | False | 0.316 | 76.6 | 2.76 | 1.008 | 0.0 | 21.1 / 21.1 | 0.0
nocap | 4.64 | legs_x4 | left, 0.094, 0.6317, 0.0398 | left_wrist_roll_rubber_hand, 2.686, 3.21 | - | False | 0.32 | 75.9 | 2.92 | 1.016 | 0.0 | 23.3 / 23.3 | 0.0
nocap | 4.68 | unitree | left, 0.043, 0.6274, 0.045 | left_wrist_roll_rubber_hand, 2.31, 3.08 | - | False | 0.312 | 78.8 | 2.88 | 1.005 | 0.0 | 19.4 / 19.4 | 0.0
nocap | 4.68 | ankle80 | left, 0.043, 0.6274, 0.045 | left_wrist_roll_rubber_hand, 2.474, 3.2 | - | False | 0.311 | 76.1 | 2.88 | 1.006 | 0.0 | 20.3 / 20.3 | 0.0
nocap | 4.68 | ankle120 | left, 0.043, 0.6274, 0.045 | left_wrist_roll_rubber_hand, 2.622, 3.21 | - | False | 0.31 | 75.4 | 3.0 | 1.008 | 0.0 | 27.7 / 27.7 | 0.0
nocap | 4.68 | ankle160 | left, 0.043, 0.6274, 0.045 | left_wrist_roll_rubber_hand, 2.768, 3.22 | - | False | 0.31 | 75.3 | 3.16 | 1.007 | 0.0 | 36.9 / 36.9 | 0.0
nocap | 4.68 | ankle240 | left, 0.043, 0.6274, 0.045 | left_wrist_roll_rubber_hand, 3.414, 3.26 | - | True | 0.79 | 75.5 | None | 2.082 | 0.0556 | 5173040.2 / 134768.3 | 0.0
nocap | 4.68 | ankle320 | left, 0.043, 0.6274, 0.045 | none | - | True | 0.793 | 69.0 | None | 2.079 | 1.0 | 16635784.5 / 922948.0 | 0.0
nocap | 4.68 | legs_x2 | left, 0.043, 0.6274, 0.045 | left_wrist_roll_rubber_hand, 2.418, 3.18 | - | False | 0.316 | 76.6 | 2.72 | 1.009 | 0.0 | 21.1 / 21.1 | 0.0
nocap | 4.68 | legs_x4 | left, 0.043, 0.6274, 0.045 | left_wrist_roll_rubber_hand, 2.656, 3.22 | - | False | 0.32 | 76.0 | 2.88 | 1.018 | 0.0004 | 36.9 / 36.9 | 0.0
nocap | 4.72 | unitree | left, 0.015, 0.6235, 0.0474 | left_wrist_roll_rubber_hand, 2.31, 3.06 | - | False | 0.312 | 78.8 | 2.88 | 1.012 | 0.0 | 19.2 / 19.2 | 0.0
nocap | 4.72 | ankle80 | left, 0.015, 0.6235, 0.0474 | left_wrist_roll_rubber_hand, 2.468, 3.19 | - | False | 0.311 | 76.0 | 2.84 | 1.014 | 0.0 | 20.5 / 20.5 | 0.0
nocap | 4.72 | ankle120 | left, 0.015, 0.6235, 0.0474 | left_wrist_roll_rubber_hand, 2.606, 3.19 | - | False | 0.311 | 75.4 | 2.96 | 1.015 | 0.0 | 28.8 / 28.8 | 0.0
nocap | 4.72 | ankle160 | left, 0.015, 0.6235, 0.0474 | left_wrist_roll_rubber_hand, 2.744, 3.23 | - | False | 0.311 | 75.2 | 3.08 | 1.015 | 0.0 | 38.5 / 38.5 | 0.0
nocap | 4.72 | ankle240 | left, 0.015, 0.6235, 0.0474 | left_wrist_roll_rubber_hand, 3.43, 3.26 | - | True | 0.793 | 126.3 | None | 2.099 | 0.0548 | 6061670.2 / 74510.0 | 0.0
nocap | 4.72 | ankle320 | left, 0.015, 0.6235, 0.0474 | none | - | True | 1.063 | 73.5 | None | 2.084 | 1.0 | 16635784.5 / 922948.0 | 0.0
nocap | 4.72 | legs_x2 | left, 0.015, 0.6235, 0.0474 | left_wrist_roll_rubber_hand, 2.406, 3.19 | - | False | 0.316 | 76.5 | 2.72 | 1.012 | 0.0 | 20.6 / 20.6 | 0.0
nocap | 4.72 | legs_x4 | left, 0.015, 0.6235, 0.0474 | left_wrist_roll_rubber_hand, 2.622, 3.21 | - | False | 0.32 | 76.0 | 2.88 | 1.016 | 0.0004 | 38.5 / 38.5 | 0.0
nocap | 4.76 | unitree | left, 0.025, 0.6218, 0.0462 | left_wrist_roll_rubber_hand, 2.334, 3.02 | - | False | 0.312 | 79.0 | 2.92 | 1.006 | 0.0 | 18.7 / 18.7 | 0.0
nocap | 4.76 | ankle80 | left, 0.025, 0.6218, 0.0462 | left_wrist_roll_rubber_hand, 2.49, 3.18 | - | False | 0.311 | 76.0 | 2.84 | 1.01 | 0.0 | 20.2 / 20.2 | 0.0
nocap | 4.76 | ankle120 | left, 0.025, 0.6218, 0.0462 | left_wrist_roll_rubber_hand, 2.626, 3.21 | - | False | 0.311 | 75.4 | 2.96 | 1.01 | 0.0 | 27.2 / 27.2 | 0.0
nocap | 4.76 | ankle160 | left, 0.025, 0.6218, 0.0462 | left_wrist_roll_rubber_hand, 2.756, 3.23 | - | False | 0.311 | 75.3 | 3.08 | 1.011 | 0.0 | 36.3 / 36.3 | 0.0
nocap | 4.76 | ankle240 | left, 0.025, 0.6218, 0.0462 | pelvis_contour_link, 1.11, 43711.34 | 1.11 | True | 0.789 | 77.0 | None | 2.121 | 0.052 | 14855722.3 / 14855722.3 | 0.0
nocap | 4.76 | ankle320 | left, 0.025, 0.6218, 0.0462 | pelvis_contour_link, 1.046, 968.13 | 1.046 | True | 1.063 | 69.0 | None | 2.103 | 1.0 | 16635784.5 / 922948.0 | 0.0
nocap | 4.76 | legs_x2 | left, 0.025, 0.6218, 0.0462 | left_wrist_roll_rubber_hand, 2.416, 3.2 | - | False | 0.316 | 76.6 | 2.72 | 1.004 | 0.0 | 19.9 / 19.9 | 0.0
nocap | 4.76 | legs_x4 | left, 0.025, 0.6218, 0.0462 | left_wrist_roll_rubber_hand, 2.618, 3.21 | - | False | 0.32 | 76.0 | 2.84 | 1.013 | 0.0004 | 36.3 / 36.3 | 0.0

## per model and gain set
model | gains | clean start upright | clean start first touch s | hand-offs upright | non-foot touch: n, s min/median/max | torso touched | end pelvis z m min-max | max tilt deg max | still | share of steps with a requested torque over its cap, max | ankle torque after hand-off, max requested / max applied N*m | walking: share of steps with a requested torque over its cap, max; max requested ankle torque N*m | bad-acceleration warnings | com above ankles at hand-off m min-max
caps | unitree | False | 1.252 | 0/20 | 20, 2.248/2.314/2.362 | 0 | 0.311-0.312 | 79.5 | 20 | 0.0 | 20.8 / 20.8 | 0.0; 31.2 | 0 | 0.6208-0.632
caps | ankle80 | False | 1.426 | 0/20 | 20, 2.414/2.478/2.516 | 0 | 0.31-0.311 | 76.2 | 20 | 0.0 | 22.0 / 22.0 | 0.0; 31.2 | 0 | 0.6208-0.632
caps | ankle120 | False | 1.586 | 0/20 | 20, 2.558/2.618/2.648 | 0 | 0.31-0.311 | 75.7 | 20 | 0.0 | 28.8 / 28.8 | 0.0; 31.2 | 0 | 0.6208-0.632
caps | ankle160 | False | 1.754 | 0/20 | 20, 2.696/2.757/2.798 | 0 | 0.31-0.311 | 75.4 | 20 | 0.0 | 38.5 / 38.5 | 0.0; 31.2 | 0 | 0.6208-0.632
caps | ankle240 | False | 2.192 | 0/20 | 20, 2.998/3.043/3.172 | 0 | 0.308-0.314 | 78.1 | 20 | 0.6032 | 72.8 / 50.0 | 0.0; 31.2 | 0 | 0.6208-0.632
caps | ankle320 | True | none | 0/20 | 20, 3.276/3.566/4.842 | 1 | 0.061-0.318 | 90.4 | 6 | 0.9132 | 128.1 / 50.0 | 0.0; 31.2 | 0 | 0.6208-0.632
caps | legs_x2 | False | 1.392 | 0/20 | 20, 2.388/2.417/2.436 | 0 | 0.316-0.316 | 76.7 | 20 | 0.0 | 23.2 / 23.2 | 0.0; 31.2 | 0 | 0.6208-0.632
caps | legs_x4 | False | 1.714 | 0/20 | 20, 2.574/2.62/2.686 | 0 | 0.32-0.32 | 76.0 | 20 | 0.0008 | 38.5 / 38.5 | 0.0; 31.2 | 0 | 0.6208-0.632
nocap | unitree | False | 1.252 | 0/20 | 20, 2.248/2.314/2.362 | 0 | 0.311-0.312 | 79.5 | 20 | 0.0 | 20.8 / 20.8 | 0.0; 31.2 | 0 | 0.6208-0.632
nocap | ankle80 | False | 1.426 | 0/20 | 20, 2.414/2.478/2.516 | 0 | 0.31-0.311 | 76.2 | 20 | 0.0 | 22.0 / 22.0 | 0.0; 31.2 | 0 | 0.6208-0.632
nocap | ankle120 | False | 1.586 | 0/20 | 20, 2.558/2.618/2.648 | 0 | 0.31-0.311 | 75.7 | 20 | 0.0 | 28.8 / 28.8 | 0.0; 31.2 | 0 | 0.6208-0.632
nocap | ankle160 | False | 1.754 | 0/20 | 20, 2.696/2.757/2.798 | 0 | 0.31-0.311 | 75.4 | 20 | 0.0 | 38.5 / 38.5 | 0.0; 31.2 | 0 | 0.6208-0.632
nocap | ankle240 | True | 2.192 | 16/20 | 20, 1.072/3.025/3.456 | 10 | 0.326-0.793 | 149.8 | 0 | 0.0812 | 85095585.5 / 14855722.3 | 0.0; 31.2 | 20 | 0.6208-0.632
nocap | ankle320 | True | none | 20/20 | 12, 1.032/1.034/1.048 | 10 | 0.792-1.063 | 164.4 | 1 | 1.0 | 87697420.9 / 2350561.2 | 0.0; 31.2 | 20 | 0.6208-0.632
nocap | legs_x2 | False | 1.392 | 0/20 | 20, 2.388/2.417/2.436 | 0 | 0.316-0.316 | 76.7 | 20 | 0.0 | 23.2 / 23.2 | 0.0; 31.2 | 0 | 0.6208-0.632
nocap | legs_x4 | False | 1.714 | 0/20 | 20, 2.574/2.62/2.686 | 0 | 0.32-0.32 | 76.0 | 20 | 0.0004 | 38.5 / 38.5 | 0.0; 31.2 | 0 | 0.6208-0.632

## REPRO: 'caps' rows against the 2026-11-09 run (clean-start first touch s; hand-off first touch min/median/max s; hand-offs upright)
unitree: got (1.252, (2.248, 2.314, 2.362), 0) want (1.252, (2.248, 2.314, 2.362), 0) match True
ankle80: got (1.426, (2.414, 2.478, 2.516), 0) want (1.426, (2.414, 2.478, 2.516), 0) match True
ankle120: got (1.586, (2.558, 2.618, 2.648), 0) want (1.586, (2.558, 2.618, 2.648), 0) match True
ankle160: got (1.754, (2.696, 2.757, 2.798), 0) want (1.754, (2.696, 2.757, 2.798), 0) match True
ankle240: got (2.192, (2.998, 3.043, 3.172), 0) want (2.192, (2.998, 3.043, 3.172), 0) match True
ankle320: got (None, (3.276, 3.566, 4.842), 0) want (None, (3.276, 3.566, 4.842), 0) match True
legs_x2: got (1.392, (2.388, 2.417, 2.436), 0) want (1.392, (2.388, 2.417, 2.436), 0) match True
legs_x4: got (1.714, (2.574, 2.62, 2.686), 0) want (1.714, (2.574, 2.62, 2.686), 0) match True
REPRO caps rows = 2026-11-09 run: True
