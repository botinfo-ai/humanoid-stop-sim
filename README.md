# humanoid-stop-sim

What each kind of "stop" does to a **walking** Unitree G1, in simulation, under **Unitree's own published walking
policy**: seven modes stopped at one moment ([sim/stop_walk.py](sim/stop_walk.py)), five of them stopped at 20
moments spread over one whole gait cycle ([sim/stop_sweep.py](sim/stop_sweep.py)), and six two-stage stops (zero
velocity first, then the policy switched off) at the same 20 moments ([sim/stop_handoff.py](sim/stop_handoff.py)),
and eight policy-off holds of different stiffness, from a clean standing start and after a zero-velocity hand-off,
with the model's joint torque caps on and off ([sim/hold_stiffness.py](sim/hold_stiffness.py)). Some modes reproduce
Unitree's own stop commands; the others are ours, marked in the first table's "Origin" column and in each later
section. numpy and MuJoCo only (no torch).

> Simulation only. A run is what one model did in simulation, not what a real robot does. Nothing here has run on
> hardware, and none of it says whether any stop is right for a given setup.

## Why

When an AI agent's "stop" tool is mapped to one of Unitree's own stop calls, what happens to a G1 that is walking?
In Unitree's own code, "stop" can mean very different calls. In `unitree_sdk2` at
[69c04316](https://github.com/unitreerobotics/unitree_sdk2/blob/69c04316d81ac244c314e730106e24787b3e61b8/include/unitree/robot/g1/loco/g1_loco_client.hpp)
(`include/unitree/robot/g1/loco/g1_loco_client.hpp`):

| Call | Line | What it sends |
|---|---|---|
| `StopMove()` | L226 | `SetVelocity(0.f, 0.f, 0.f)`: a zero velocity command; it sets no FSM id |
| `Damp()` | L214 | `SetFsmId(1)` |
| `ZeroTorque()` | L224 | `SetFsmId(0)` |

And in `unitree_rl_gym` at
[3e70454](https://github.com/unitreerobotics/unitree_rl_gym/tree/3e70454dae9562febade2949c9ebc57440365dcb), the
real-robot deploy for the RL policies:

- `deploy/deploy_real/common/command_helper.py` L11-18: `create_damping_cmd` sets `kp = 0`, `kd = 8` on every motor;
  L21-28: `create_zero_cmd` sets `kp = 0`, `kd = 0`.
- `deploy/deploy_real/deploy_real.py` L93-99: while waiting for the start button it sends `create_zero_cmd`;
  L254-265: when the select key is pressed (or on Ctrl-C), the loop ends, `create_damping_cmd` is sent once and it
  prints `Exit`.
- `deploy/deploy_real/README.md` L69, Unitree's own instructions for that deploy: "In motion control mode, press the
  `select` button on the remote control, the robot will enter the damping mode and fall down, and the program will
  exit."

This script asks what each of those does to a G1 that is walking when the stop comes. `damp_kd8` and `zero_torque`
below reproduce `unitree_rl_gym`'s low-level `create_damping_cmd` and `create_zero_cmd`, held for the 4 s after the
stop. They are not the SDK's `Damp()` / `ZeroTorque()` FSM modes, whose behaviour on the robot is not in public code.
`deploy_real.py` sends the damping command once and exits; what the motors do after that last message is not in
public code either.

## How it works

`unitree_rl_gym` ships a trained G1 walking policy (`deploy/pre_train/g1/motion.pt`), a 12-joint MuJoCo model of the
G1 (`resources/robots/g1_description/scene.xml`, which loads `g1_12dof.xml`) and a MuJoCo runner
(`deploy/deploy_mujoco/deploy_mujoco.py` with `configs/g1.yaml`). [`sim/stop_walk.py`](sim/stop_walk.py) runs that
runner's loop step for step: PD torques from the config's gains every 2 ms physics step, the policy every 10 steps,
the same 47-number observation. It reads the TorchScript file with `zipfile` and a restricted `pickle.Unpickler`
(it never imports torch) and evaluates the network in numpy: an LSTM with 64 units, then 64 -> 32 -> 12 with ELU.

The G1 walks forward at the config's `cmd_init` (0.5 m/s) for 4.0 s. Then one stop is applied and the run goes on
for 4.0 s:

| Mode | What changes at the stop | Origin |
|---|---|---|
| `continue` | nothing (control run) | ours: no stop |
| `stopmove` | the velocity command goes to zero; the policy keeps running | the zero velocity command that the SDK's `StopMove()` sends; here this RL policy receives it, not the controller behind `StopMove()` on a real G1 |
| `hold_last` | the policy stops; every leg joint holds its last target with the config's PD gains | ours (the gains are the config's) |
| `hold_default` | the policy stops; the PD gains pull every leg joint to the config's default angles | ours (the gains and angles are the config's) |
| `damp_kd8` | kp 0, kd 8 on every joint (`create_damping_cmd`) | Unitree's `create_damping_cmd` in `unitree_rl_gym` |
| `zero_torque` | no torque at all (`create_zero_cmd`) | Unitree's `create_zero_cmd` in `unitree_rl_gym` |
| `stopmove_then_damp` | zero velocity first, then kp 0, kd 8 from 2.0 s after the stop | ours: two of the commands above, in an order and with a delay we chose |

## Results (one run, 2026-11-08)

From [results/2026-11-08-results.json](results/2026-11-08-results.json) (the printed table, with every shape that
touched the floor in order, is [results/2026-11-08-stop_walk.md](results/2026-11-08-stop_walk.md)). In every mode the
G1 had walked 1.756 m and was moving at 0.496 m/s when the stop came, with no floor contact by anything but its feet
while walking and at most 4.0 degrees of pelvis tilt.

- "First non-foot touch" is the first floor contact after the stop by any shape not on the four ankle bodies
  (`left/right_ankle_pitch_link`, `left/right_ankle_roll_link`), named by the shape (its mesh) and the body it
  belongs to.
- "First upper-body touch" is the first contact by a shape on the `pelvis` body. In this model that body carries the
  pelvis and also the torso, head, logo, shoulders, elbows and hands (see below).
- Speeds are the touching shape's linear speed at its centre at that moment (MuJoCo `mj_objectVelocity` on the geom).
- "Upright 4 s after the stop" means pelvis height above 0.6 m and pelvis tilt under 30 degrees at the last frame.

| Mode | First non-foot touch after the stop | First upper-body touch | Upright 4 s after the stop | Pelvis travel after the stop |
|---|---|---|---|---|
| `continue` | none | none | yes | 1.9 m (still walking) |
| `stopmove` | none | none | yes | 0.199 m |
| `hold_last` | right hand (`right_wrist_roll_rubber_hand`), 0.962 s, 3.0 m/s | the same | no | 1.243 m |
| `hold_default` | right hand (`right_wrist_roll_rubber_hand`), 0.928 s, 3.03 m/s | the same | no | 0.854 m |
| `damp_kd8` | `left_knee_link`, 0.722 s, 0.68 m/s | right hand, 0.894 s, 2.08 m/s | no | 1.005 m |
| `zero_torque` | `right_knee_link`, 0.292 s, 1.55 m/s | `left_shoulder_yaw_link`, 1.67 s, 0.95 m/s | no | 0.544 m |
| `stopmove_then_damp` | `right_knee_link`, 3.032 s (1.032 s after the damping began), 0.81 m/s | `right_elbow_link`, 3.124 s, 1.17 m/s | no | 0.605 m |

Read in this model:

- Only the zero-velocity command kept the G1 up. Its pelvis's horizontal speed never stayed under 0.05 m/s for 0.48 s
  (12 samples at 25 Hz) in the 4 s after the stop (`pelvis_still_after_s` is null), so "stopped" there means the policy is still controlling a body that
  moves a little.
- Every stop that takes control away from the policy (holding the last targets, pulling to the default pose,
  damping, zero torque) put a knee or a hand on the floor 0.29-0.96 s after the stop.
- The torso shape (`torso_link_23dof_rev_1_0`) reached the floor 1.394 s (`damp_kd8`), 1.496 s (`hold_last`) and
  1.732 s (`zero_torque`) after the stop. In `hold_default` it never did: the model ended propped on its two hands
  (pelvis 0.311 m high, tilted 68.6 degrees), which are rigid in this model.
- In both hold modes the first contact was the right hand at about 3 m/s. The hands sit at the end of a rigid arm
  fixed to the pelvis body here, so they swing with the whole upper body; on a G1 whose arm motors are running, the
  arms would move differently.
- Zero velocity first and damping 2.0 s later did not avoid the fall here; it only delayed it. Boston Dynamics'
  Spot names the difference between cutting power at once and stopping first: its E-Stop service's `CUT` level
  ("actuator power is cut immediately") versus `SETTLE_THEN_CUT` ("the robot attempts to come to a stop, sit down,
  and then cut actuator power"), from the
  [E-Stop service page](https://dev.bostondynamics.com/docs/concepts/estop_service) (read 2026-11-08). This script has
  no sit-down step.

## Sweep: the same stops at 20 moments of one gait cycle (2026-11-09)

The table above rests on one stop moment. [`sim/stop_sweep.py`](sim/stop_sweep.py) keeps `stop_walk.py`'s loader,
network, deploy loop and contact reporting, and applies five of the stops (`stopmove`, `hold_last`, `hold_default`,
`damp_kd8`, `zero_torque`) at 20 moments from 4.00 s to 4.76 s in 0.04 s steps: phase 0 to 0.95 of the policy's
0.8 s phase clock, so every part of a step is covered once. Each run goes on for 4 s after its stop. The printed table,
one row per moment and mode, is [results/2026-11-09-stop_sweep.md](results/2026-11-09-stop_sweep.md), and every
field is in [results/2026-11-09-sweep-results.json](results/2026-11-09-sweep-results.json).

- A control run walked on to 8.76 s with no stop: no floor contact by anything but the feet, upright, 2.3 degrees of
  tilt at the end.
- At the stop, one foot was on the floor at 18 of the 20 moments (the right at 9, the left at 9) and both feet at 2
  (4.04 s and 4.44 s).
- The 4.00 s rows repeat the single-run table above exactly (first touches at 0.962, 0.928, 0.722 and 0.292 s).

| Mode | Upright 4 s after the stop | First non-foot touch after the stop, s: min / median / max | Its speed, m/s: min / median / max | Which shape touched first | Torso shape touched the floor | Head shape touched the floor |
|---|---|---|---|---|---|---|
| `stopmove` | 20 of 20 | none | - | - | 0 of 20 | 0 of 20 |
| `hold_last` | 0 of 20 | 0.842 / 0.894 / 1.062 | 1.07 / 1.145 / 3.2 | `right_knee_link` 8, `left_wrist_roll_rubber_hand` 6, `left_knee_link` 5, `right_wrist_roll_rubber_hand` 1 | 19 of 20, 1.158-1.592 s, at most 1.33 m/s | 8 of 20, at most 0.59 m/s |
| `hold_default` | 0 of 20 | 0.878 / 0.909 / 0.96 | 3.01 / 3.03 / 3.13 | `left_wrist_roll_rubber_hand` 12, `right_wrist_roll_rubber_hand` 8 | 0 of 20 | 0 of 20 |
| `damp_kd8` | 0 of 20 | 0.662 / 0.684 / 0.798 | 0.5 / 0.665 / 0.72 | `left_knee_link` 10, `right_knee_link` 10 | 16 of 20, 1.318-3.334 s, at most 0.91 m/s | 2 of 20, at most 0.91 m/s |
| `zero_torque` | 0 of 20 | 0.268 / 0.301 / 0.318 | 1.45 / 1.545 / 1.6 | `left_knee_link` 11, `right_knee_link` 9 | 19 of 20, 1.234-2.638 s, at most 2.21 m/s | 4 of 20, at most 2.62 m/s |

Read in this model:

- The split did not depend on the moment. The zero-velocity command kept the G1 upright at all 20 moments (pelvis
  0.773-0.781 m high 4 s later, after 0.183-0.238 m of travel). Each of the other four stops left it not upright at
  all 20, with the pelvis 0.061-0.313 m above the floor 4 s after the stop, including the two moments with both feet
  down.
- The order of the first touch held at every moment: `zero_torque` first (0.268-0.318 s, a knee), then `damp_kd8`
  (0.662-0.798 s, a knee), then the two hold modes (0.842-1.062 s). The first touch moved by at most 0.22 s across the
  cycle in any mode.
- `hold_default` always landed on a hand first at about 3 m/s and ended propped on its two rigid hands (pelvis
  0.311-0.313 m); its torso shape never touched. `hold_last` hit a knee first at 13 moments and a hand at 7.
- `stopmove_then_damp` and `continue` are not in the sweep.

## Hand-off: zero velocity first, then the policy off (2026-11-09)

Only zero velocity kept the G1 up, but it leaves the learned policy in charge. Can a stop end with the policy off and
the robot still standing, if it slows down first? [`sim/stop_handoff.py`](sim/stop_handoff.py) keeps `stop_sweep.py`'s
loader, network, deploy loop and contact reporting. Every mode starts with zero velocity at the same 20 moments
(4.00-4.76 s); then, at a trigger, the policy is switched off and the legs are held by the config's PD gains alone
(kps 100/100/100/150/40/40, kds 2/2/2/4/2/2 per leg). Each run goes on for 6 s after the stop. The trigger and the
hold are our choices; the hold targets and gains are the config's:

| Mode | Hand-off trigger | After the hand-off |
|---|---|---|
| `stopmove` | none (reference) | the policy keeps running with a zero command |
| `zv1_hold_default` | 1.0 s after the stop | PD gains pull every leg joint to the config's default angles at once |
| `zv2_hold_default` | 2.0 s after the stop | the same |
| `zv_ds_hold_last` | first moment after 0.5 s with both feet on the floor | hold the last targets |
| `zv_ds_hold_default` | the same | default angles at once |
| `zv_ds_ramp_default` | the same | targets moved linearly from the last targets to the default angles over 1.0 s |
| `zv_ds_damp` | the same | kp 0, kd 8 on every joint (`create_damping_cmd`) |

The printed table, one row per moment and mode, is
[results/2026-11-09-stop_handoff.md](results/2026-11-09-stop_handoff.md).

| Mode | Upright 6 s after the stop | Hand-off, s after the stop: min / median / max | Pelvis speed at the hand-off, m/s, max | First non-foot touch, s after the stop: min / median / max | Which shape touched first | Torso shape touched the floor | End pelvis height, m |
|---|---|---|---|---|---|---|---|
| `stopmove` | 20 of 20 | - | - | none | - | 0 of 20 | 0.773-0.781 |
| `zv1_hold_default` | 0 of 20 | 1.0 / 1.0 / 1.0 | 0.129 | 2.248 / 2.314 / 2.362 | a hand (left 18, right 2) | 0 of 20 | 0.311-0.312 |
| `zv2_hold_default` | 0 of 20 | 2.0 / 2.0 / 2.0 | 0.129 | 3.338 / 3.385 / 3.468 | a hand (left 17, right 3) | 2 of 20 | 0.061-0.314 |
| `zv_ds_hold_last` | 0 of 20 | 0.5 / 0.657 / 0.836 | 0.196 | 1.768 / 2.249 / 2.344 | a hip pitch link 12, a hand 5, `right_knee_link` 3 | 10 of 20 | 0.061-0.323 |
| `zv_ds_hold_default` | 0 of 20 | 0.5 / 0.657 / 0.836 | 0.196 | 1.698 / 1.947 / 2.164 | a hand (left 13, right 7) | 8 of 20 | 0.061-0.311 |
| `zv_ds_ramp_default` | 0 of 20 | 0.5 / 0.657 / 0.836 | 0.196 | 1.912 / 2.206 / 2.664 | a hand 16, a hip link 4 | 17 of 20 | 0.061-0.311 |
| `zv_ds_damp` | 0 of 20 | 0.5 / 0.657 / 0.836 | 0.196 | 1.448 / 1.754 / 1.864 | a knee 11, a hip pitch link 5, `pelvis` cylinder 3, `left_shoulder_yaw_link` 1 | 10 of 20 | 0.061-0.072 |

Read in this model:

- No hand-off kept the G1 up: 0 of 20 in each of the six two-stage modes. That includes handing off after 2.0 s of
  zero velocity, when the pelvis was moving at 0.010-0.129 m/s.
- After a hand-off to the default angles at 1.0 s or 2.0 s, the first non-foot touch came 1.248-1.468 s after the
  hand-off, a hand first in all 40 runs. A 1.0 s ramp to the default angles did not prevent it either.
- Zero velocity alone again kept the G1 upright at all 20 moments (pelvis 0.773-0.781 m high 6 s after the stop).
- In Unitree's own real-robot procedure the hoist is still on during the default-position state; the rope is lowered
  further only once the policy is stepping. Its `deploy/deploy_real/README.md` (L54-60): in the zero torque state, "press the `start` button on the remote control,
  and the robot will move to the default joint position state. After the robot moves to the default joint position,
  you can slowly lower the hoisting mechanism to let the robot's feet touch the ground." Then: "press the `A` button
  on the remote control, and the robot will step on the spot. After the robot is stable, you can gradually lower the
  hoisting rope to give the robot a certain amount of space to move."
- Why the hold falls in this model is the next section.

## Hold stiffness: how stiff must a policy-off hold be? (2026-11-09; torque caps checked 2026-11-10)

A rough estimate first. Held only at the ankles, a standing G1 tips like a rigid body about its ankle-pitch axes,
and gravity pushes it over with a stiffness of about m*g*h (mass times gravity times the height of the centre of
mass above those axes). The hold pushes back with the summed ankle-pitch kp of both legs. The deploy config's ankle
kp is 40 per leg, 80 summed. [`sim/hold_stiffness.py`](sim/hold_stiffness.py) keeps `stop_handoff.py`'s loader,
network, deploy loop and contact reporting, prints those stance numbers, and tries eight gain sets with the policy
off: the config's (`unitree`), ankle pitch and roll kp 80, 120, 160, 240 and 320 with every other joint as the
config (`ankle80` to `ankle320`), and every leg gain times 2 or 4 (`legs_x2`, `legs_x4`). kd is scaled by the same
factor as kp. Each set is tried twice: from a clean start (the G1 placed with its feet on the floor at the default
angles, zero velocity, held for 6 s) and after a hand-off (`stop_handoff.py`'s `zv1_hold_default`: zero velocity at
the same 20 moments, then after 1.0 s the policy off and the gain set pulling to the default angles).

Torque caps. Unitree's `g1_12dof.xml` gives every leg joint a torque cap (`actuatorfrcrange`: hip pitch and yaw
88 N*m, hip roll and knee 139 N*m, ankle pitch and roll 50 N*m), and MuJoCo applies at most that torque to the joint.
The actuators' own control range is unlimited, so a hold can ask for more torque than the joint gets. The script
prints the caps, the largest requested and applied torques, and the share of steps on which any joint's requested
torque was over its cap. It runs every case twice: on the model as published (`caps`) and on a copy with the joint
caps switched off (`nocap`). The printed output, one row per model, gain set and moment, is
[results/2026-11-10-hold_stiffness.md](results/2026-11-10-hold_stiffness.md). The script's first run (2026-11-09)
printed a torque line that read the wrong limit, so its output was replaced
([results/2026-11-09-hold_stiffness.md](results/2026-11-09-hold_stiffness.md) says why). That run's first-touch times
and upright counts are written into the script as the expected values of its REPRO block (`EXPECT_CLEAN_TOUCH`,
`EXPECT_HANDOFF_TOUCH`, and 0 of 20 hand-offs upright), and the 2026-11-10 `caps` rows match them for every gain set
(the REPRO lines at the end of the results file). The `unitree` hand-off rows can also be checked against
`stop_handoff.py`'s published table (below).

Stance numbers printed by the run: mass 32.107 kg; centre of mass 0.6381 m above the ankle-pitch axes and 0.0463 m
ahead of them; m*g*h 201.0 N*m/rad. The config's summed ankle-pitch kp of 80 N*m/rad is 0.398 of that.

With the caps on (the model as published):

| Gain set | Summed ankle-pitch kp, N*m/rad (ratio to m*g*h) | Clean start: upright after 6 s | Clean start: first non-foot touch, s | Clean start: share of steps with a request over a cap | Hand-offs upright | Hand-off: first non-foot touch, s after the stop, min / median / max | Hand-offs: share of steps with a request over a cap, max | Hand-offs: largest ankle torque requested / applied, N*m |
|---|---|---|---|---|---|---|---|---|
| policy on, zero command (control) | - | yes (pelvis 0.776 m, max tilt 4.9 deg) | none | - | - | - | - | - |
| `unitree` | 80 (0.398) | no | 1.252 (left hand) | 0 | 0 of 20 | 2.248 / 2.314 / 2.362 | 0 | 20.8 / 20.8 |
| `ankle80` | 160 (0.796) | no | 1.426 (left hand) | 0 | 0 of 20 | 2.414 / 2.478 / 2.516 | 0 | 22.0 / 22.0 |
| `ankle120` | 240 (1.194) | no | 1.586 (left hand) | 0 | 0 of 20 | 2.558 / 2.618 / 2.648 | 0 | 28.8 / 28.8 |
| `ankle160` | 320 (1.592) | no | 1.754 (left hand) | 0 | 0 of 20 | 2.696 / 2.757 / 2.798 | 0 | 38.5 / 38.5 |
| `ankle240` | 480 (2.388) | no | 2.192 (left hand) | 0.5987 | 0 of 20 | 2.998 / 3.043 / 3.172 | 0.6032 | 72.8 / 50.0 |
| `ankle320` | 640 (3.184) | yes (pelvis 0.782 m, max tilt 8.5 deg) | none | 0 | 0 of 20 | 3.276 / 3.566 / 4.842 | 0.9132 | 128.1 / 50.0 |
| `legs_x2` | 160 (0.796) | no | 1.392 (left hand) | 0 | 0 of 20 | 2.388 / 2.417 / 2.436 | 0 | 23.2 / 23.2 |
| `legs_x4` | 320 (1.592) | no | 1.714 (left hand) | 0 | 0 of 20 | 2.574 / 2.62 / 2.686 | 0.0008 | 38.5 / 38.5 |

In the `ankle240` clean start the two ankle-roll joints asked for up to 68.6 and 68.7 N*m and got 50.0. While the
G1 walked under the policy before each stop, no step asked any joint for more than its cap (largest ankle request
31.2 N*m).

With the caps off, the five gain sets whose requests stayed under the caps (`unitree`, `ankle80` to `ankle160`,
`legs_x2`), and `legs_x4`, whose requests went over a cap on at most 0.08% of hand-off steps, gave the same
clean-start and hand-off results as with the caps on. For `ankle240` and `ankle320` the caps-off copy was numerically
unstable: MuJoCo printed its "Nan, Inf or huge value in QACC ... The simulation is unstable" warning 20 times over
each set's 20 hand-offs, and once in the `ankle240` clean start; requested ankle torques reached about 85 and 88
million N*m and pelvis tilt 150 and 164 degrees. When MuJoCo raises that warning it also resets the simulation state
to the model's initial pose and goes on from there (`mj_checkAcc` in `src/engine/engine_forward.c`, MuJoCo 3.3.7,
calls `mj_resetData` unless autoreset is disabled; this script does not disable it). So a run that raised it went on
from that pose, not from where the hold had left the robot, and those rows' "upright" counts (16 and 20 of 20 by this
script's measure) and contact speeds of thousands of m/s are not a hold result. Nothing below rests on them.

Read in this model:

- At the config's gains the hold cannot stand the G1 even from a clean start: the left hand touched the floor
  1.252 s after the start, at 3.1 m/s, and no joint asked for more than its cap. The policy, given a zero command
  from the same start, kept it up.
- The rough estimate was not enough: `ankle120` (1.19 times m*g*h) and `ankle160` (1.59) fell from the clean start
  with every request under its cap. `ankle240` (2.39) fell too, but with a request over a cap on 60% of its steps
  (ankle roll asked for up to 68.7 N*m and got 50.0), so the caps may be what decided it; its caps-off copy was
  unstable and settles nothing. Only `ankle320`, eight times the config's ankle kp (3.18 times m*g*h), kept the clean
  start upright, with no request over a cap (largest ankle torque 17.8 N*m). Doubling or quadrupling every leg gain
  fell at about the same time as raising the ankles alone to the same ankle kp (`legs_x2` 1.392 s vs `ankle80`
  1.426 s; `legs_x4` 1.714 s vs `ankle160` 1.754 s).
- After a hand-off from walking, no gain set kept the G1 up at any of the 20 moments. For `unitree`, `ankle80` to
  `ankle160` and `legs_x2` no request went over a cap, and for `legs_x4` at most 0.08% of steps did, so those falls
  are not the caps' doing. `ankle240` and `ankle320` asked for up to 72.8 and 128.1 N*m at the ankles and got 50.0,
  with a request over a cap on up to 60% and 91% of steps: under Unitree's caps those two holds did not get the
  torque their stiffness asked for, and whether they would hold without the caps is open (their caps-off copies were
  unstable). Stiffer ankles delayed the first touch (median 2.314 s at the config's gains, 3.566 s at `ankle320`).
- The `unitree` hand-off rows repeat `stop_handoff.py`'s `zv1_hold_default` rows (first touch 2.248 / 2.314 /
  2.362 s, pelvis 0.311-0.312 m at the end).

So in this model, with Unitree's torque caps, the only stop tried that left a walking G1 standing kept a balance
controller running (the policy with a zero command). After every hand-off a PD hold to the default pose fell: at the
config's gains and up to four times its ankle kp (`unitree`, `ankle80` to `ankle160`, `legs_x2`) without asking any
joint for more than its cap, at `legs_x4` with a request over a cap on at most 0.08% of steps, and at six and eight
times the config's ankle kp with ankle requests over the 50 N*m caps. This is one 12-joint model, one policy and one
start pose; it says nothing about a real G1's own controllers.

## What this does not show

- **The upper body is one rigid piece.** In Unitree's `g1_12dof.xml` the torso, head, logo, shoulders, elbows and
  hands are shapes of the pelvis body, fixed in one arm pose; several of them collide (the script prints them). So
  every contact is reported by the shape that touched, and the arms cannot move during a fall in this model.
- **The zero-velocity row is this RL policy with a zero command**, not the controller behind `StopMove()` on a real
  G1, whose code is in neither repo.
- **The port was not compared with torch's output.** The evidence that it runs the policy as trained is that the G1
  walks at the commanded speed (0.496 m/s at the stop, 1.756 m in 4 s) with no fall and no non-foot contact.
- One walking speed (the config's 0.5 m/s, straight ahead) and one gait cycle. The single-run table is one moment
  (4.0 s into the walk); the sweep and the hand-off section use 20 moments of one cycle, one run each. Turning, other
  speeds, standing still and pushes were not tried, and the hand-off triggers, the ramp and its length are our
  choices, not Unitree's.
- **Torque caps.** Every run uses the joint torque caps in Unitree's `g1_12dof.xml` (ankles 50 N*m). Only the
  hold-stiffness section also runs a caps-off copy, and for its two stiffest holds that copy was numerically unstable.
- Tested only in BotInfo's sealed sandbox, with numpy 2.0.2 and mujoco 3.3.7 (printed at the top of the run's
  table). Other versions were not tried.

## Prior art

Research on how a humanoid should stop is ahead of this script, which only measures the stop commands Unitree's
public code already has, on Unitree's public walking policy:

- Long, Abbeel, Sreenath, Horowitz, Shi and Liu, "Humanoid Safe Stop via Learned Stoppability Value"
  ([arXiv 2609.02358](https://arxiv.org/abs/2609.02358), 2026-09-02): "Humanoid robots responding to emergency stop
  commands typically execute a fixed maneuver, without reasoning about whether a safe stop is actually feasible from
  the current state." Their Safe-Stop "commits to the stop only when both estimators indicate that stopping remains
  feasible, otherwise it hands off to a fall policy, instantiated as a damping fallback."
- Sun, Pan, Li, Ding, Cui, Wang and Liu, "Learning Safe-Stoppability Monitors for Humanoid Robots"
  ([arXiv 2603.22703](https://arxiv.org/abs/2603.22703), 2026-03-24): "for humanoid robots, abruptly cutting power
  can itself cause catastrophic failures; instead, an emergency stop must execute a predefined fallback controller
  that preserves balance and drives the robot toward a minimum-risk condition."
- Meng et al., "SafeFall: Learning Protective Control for Humanoid Robots"
  ([arXiv 2511.18509](https://arxiv.org/abs/2511.18509), 2025-11-23): a fall predictor and a protective policy,
  "Validated on a full-scale Unitree G1 humanoid".

- A Japanese G1 guide on Classmethod's DevelopersIO ("Unitree G1 を動かす前の基本をまとめてみた", by atsu, 2026-06-30),
  [dev.classmethod.jp/articles/unitree-g1-basic](https://dev.classmethod.jp/articles/unitree-g1-basic/), says of
  the remote's damping mode: "減衰（L2+B）はその場でトルクを切るだけなので、安全クレーンとつながっていない場合、立位のまま放置すると倒れます。"
  (Damping only cuts the torque on the spot, so a G1 left standing without the safety crane falls over.) Its tests
  were all done with the G1 on the crane.
- Spot's `CUT` and `SETTLE_THEN_CUT` stop levels, quoted above.
- The machinery vocabulary for the same split is IEC 60204-1's stop categories, as IDEC's summary page
  ([idec.com](https://www.idec.com/en-us/solutions/safety/law/iso-iec/iec60204), read 2026-11-08) gives them:
  category 0, "Stopping by immediate removal of power to the machine actuators"; category 1, "A controlled stop with
  power available to the machine actuators to achieve the stop, and then removal of power when the stop is
  achieved"; category 2, "Controlled stop with power remaining available to the machine actuators even after the
  stop is completed". In this script's terms, `zero_torque` resembles category 0, `stopmove` resembles category 2,
  and none of the modes is a category-1 stop. That is an analogy for a simulated model, not a classification of any
  robot.
- Agility Robotics' FAQ ([agilityrobotics.com/faq](https://www.agilityrobotics.com/faq), read 2026-11-08) says Digit
  has "a CAT1 controlled stop function in response to an external safety system. This allows Digit to actively
  brake to a controlled stable position before cutting power completely—preventing it from collapsing."
- Unitree's own `deploy_mujoco.py` runs the same policy in MuJoCo, with torch and with no stop.
- Hyperspawn's humanoidrobots.training
  ([its G1 walk policy page](https://www.humanoidrobots.training/policies/unitree-g1-walk), read 2026-11-08) runs an
  ONNX export of the G1 walking checkpoint `deploy/pre_train/g1/motion.pt` from `unitree_rl_gym` (linked at commit
  276801e4) in the browser, with MuJoCo compiled to WebAssembly, and lists a walk test ("Falls" 0). The page has no
  stop or exit test.

## Run it

Python 3 with numpy and mujoco. The results here come from numpy 2.0.2 and mujoco 3.3.7, so pin those to compare
numbers (`pip install numpy==2.0.2 mujoco==3.3.7`); other versions were not tried.

```bash
mkdir -p in/unitreerobotics out out-sweep out-handoff out-stiffness
git clone https://github.com/unitreerobotics/unitree_rl_gym in/unitreerobotics/unitree_rl_gym
git -C in/unitreerobotics/unitree_rl_gym checkout 3e70454dae9562febade2949c9ebc57440365dcb
python3 sim/stop_walk.py in out
python3 sim/stop_sweep.py in out-sweep
python3 sim/stop_handoff.py in out-handoff
python3 sim/hold_stiffness.py in out-stiffness
```

Each script writes into the output folder you give it, which must exist (the `mkdir` line makes all four).
`hold_stiffness.py` prints its tables and writes them to `out-stiffness/hold_stiffness.md`, with every field in
`out-stiffness/results.json` and `qpos_*.npy` trajectories for the clean starts and the 4.00 s hand-offs, with the
caps on and off (`qpos_nocap_*`).
`stop_walk.py` prints the table and writes `out/results.json` and `out/stop_walk.md`. `stop_sweep.py` prints the
sweep table and writes `out-sweep/results.json` and `out-sweep/stop_sweep.md`. `stop_handoff.py` prints the hand-off
table and writes `out-handoff/results.json`, `out-handoff/stop_handoff.md` and seven `qpos_*.npy` trajectories for the
4.00 s stop. The 2026-11-08 run of `stop_walk.py` also printed a video description for BotInfo's own renderer; that
part of the script, and its lines at the end of `results/2026-11-08-stop_walk.md`, have since been removed. It ran
after the table was printed and changed none of it.

Each script reads `motion.pt` with a restricted unpickler that imports nothing the file names: plain containers map
to Python's own, tensor rebuilds to the script's reader, and every other name to an inert stand-in. A tensor whose
offset, size or stride would reach outside its stored data is refused.

Licence: Apache-2.0. The script reads Unitree's files (BSD-3-Clause) from your clone; none are included here.
