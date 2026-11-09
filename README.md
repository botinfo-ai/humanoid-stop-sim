# humanoid-stop-sim

What each kind of "stop" does to a **walking** Unitree G1, in simulation, under **Unitree's own published walking
policy**: seven modes stopped at one moment ([sim/stop_walk.py](sim/stop_walk.py)), and five of them stopped at 20
moments spread over one whole gait cycle ([sim/stop_sweep.py](sim/stop_sweep.py)). numpy and MuJoCo only (no torch).

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

| Mode | What changes at the stop |
|---|---|
| `continue` | nothing (control run) |
| `stopmove` | the velocity command goes to zero; the policy keeps running |
| `hold_last` | the policy stops; every leg joint holds its last target with the config's PD gains |
| `hold_default` | the policy stops; the PD gains pull every leg joint to the config's default angles |
| `damp_kd8` | kp 0, kd 8 on every joint (`create_damping_cmd`) |
| `zero_torque` | no torque at all (`create_zero_cmd`) |
| `stopmove_then_damp` | zero velocity first, then kp 0, kd 8 from 2.0 s after the stop |

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

- Only the zero-velocity command kept the G1 up. Its pelvis's horizontal speed never stayed under 0.05 m/s for 0.5 s in the 4 s after
  the stop (`pelvis_still_after_s` is null), so "stopped" there means the policy is still controlling a body that
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

## What this does not show

- **The upper body is one rigid piece.** In Unitree's `g1_12dof.xml` the torso, head, logo, shoulders, elbows and
  hands are shapes of the pelvis body, fixed in one arm pose; several of them collide (the script prints them). So
  every contact is reported by the shape that touched, and the arms cannot move during a fall in this model.
- **The zero-velocity row is this RL policy with a zero command**, not the controller behind `StopMove()` on a real
  G1, whose code is in neither repo.
- **The port was not compared with torch's output.** The evidence that it runs the policy as trained is that the G1
  walks at the commanded speed (0.496 m/s at the stop, 1.756 m in 4 s) with no fall and no non-foot contact.
- One walking speed (the config's 0.5 m/s, straight ahead) and one gait cycle. The single-run table is one moment
  (4.0 s into the walk); the sweep adds 20 moments of one cycle, one run each. Turning, other speeds, standing still
  and pushes were not tried.
- Tested only in BotInfo's sealed sandbox, with numpy 2.0.2 and mujoco 3.3.7 (printed at the top of the run's
  table). Other versions were not tried.

## Prior art

Research on how a humanoid should stop is ahead of this script, which only measures the stop commands Unitree's
public code already has, on Unitree's public walking policy:

- Long, Abbeel, Sreenath, Horowitz, Shi and Liu, "Humanoid Safe Stop via Learned Stoppability Value"
  ([arXiv 2609.02358](https://arxiv.org/abs/2609.02358), 2026-09-02): "Humanoid robots responding to emergency stop
  commands typically execute a fixed maneuver, without reasoning about whether a safe stop is actually feasible from
  the current state." Their Safe-Stop commits to a learned stop only when two estimators agree it is feasible;
  "otherwise it hands off to a fall policy, instantiated as a damping fallback."
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
  ([its G1 walk policy page](https://www.humanoidrobots.training/policies/unitree-g1-walk), read 2026-11-08) runs
  `deploy/pre_train/g1/motion.pt` from `unitree_rl_gym` (linked at commit 276801e4) in the browser, with MuJoCo
  compiled to WebAssembly, and lists a walk test ("Falls" 0). The page has no stop or exit test.

## Run it

Python 3 with numpy and mujoco (`pip install numpy mujoco`):

```bash
mkdir -p in/unitreerobotics out
git clone https://github.com/unitreerobotics/unitree_rl_gym in/unitreerobotics/unitree_rl_gym
git -C in/unitreerobotics/unitree_rl_gym checkout 3e70454dae9562febade2949c9ebc57440365dcb
python3 sim/stop_walk.py in out
python3 sim/stop_sweep.py in out-sweep
```

`stop_walk.py` prints the table and writes `out/results.json` and `out/stop_walk.md`. `stop_sweep.py` prints the
sweep table and writes `out-sweep/results.json` and `out-sweep/stop_sweep.md` (it needs no `in/menagerie` folder and
writes no video). The last part of the script is
BotInfo-only: it writes a video description for BotInfo's own renderer from `in/menagerie/unitree_g1/scene.xml`
(MuJoCo Menagerie's 29-joint G1, posed from the run's pelvis and leg angles, with its waist and arms at Menagerie's
zero rather than the 12-joint model's fixed arm pose). Without that folder it prints `video skipped` and the table is
unchanged.

Licence: Apache-2.0. The script reads Unitree's files (BSD-3-Clause) from your clone; none are included here.
