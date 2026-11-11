"""What each stop does to a walking G1 under Unitree's SECOND published walking policy: the 29-joint mjlab one.
Simulation only.

Unitree's unitree_rl_mjlab (commit 1425b15f) ships a G1 velocity policy trained with mjlab on MuJoCo physics
(deploy/robots/g1/config/policy/velocity/v0/exported/policy.onnx), the parameters its real-robot deploy loads
(.../params/deploy.yaml), the deploy's state machine (deploy/robots/g1/config/config.yaml) and the training model
(src/assets/robots/unitree_g1/xmls/g1.xml). This script rebuilds the training physics from those files and the
constants in g1_constants.py (armature from the motors' rotor inertias and gears, effort limits, foot contacts; sim
step 0.005 s, 4 sim steps per 0.02 s policy step), without onnxruntime or torch: the ONNX file is read by hand and
run in numpy. It asks:

  1. Does the rebuild hold up? The zero-observation output must equal the 2026-11-10 recon run's; the robot must
     stand 10 s at zero command and walk near the commanded speed.
  2. What does each stop do to the walking robot, at 30 moments spread over one 0.6 s gait cycle?

Stops (applied at the stop moment; 6 s simulated after it):
  continue                the command stays (control run)
  zero_cmd                the velocity command goes to zero, the policy keeps running (the stick let go)
  passive_kd3             Unitree's Passive state in this deploy: kp 0, kd 3 on all 29 motors (config.yaml), which
                          the walking state reaches on LT + B
  damp_kd8                kp 0, kd 8: unitree_rl_gym's create_damping_cmd, for comparison
  zero_torque             kp 0, kd 0
  hold_last               ours: the policy stops, every joint holds its last target with the policy's gains
  fixstand                ours as a transition: Unitree's FixStand state (its gains and 2 s ramp to the default
                          pose, config.yaml + State_FixStand.h) entered straight from walking. The deploy has no such
                          transition (walking exits only to Passive and to a dance); this asks what it would do.
  zero_cmd_then_passive   zero command for 2 s, then Passive (let go of the stick, then LT + B)
  zero_cmd_then_fixstand  zero command for 2 s, then FixStand (the same missing transition, from a standstill)

The same sweep also runs on MuJoCo Menagerie's unitree_g1 with the same actuators, as a second body.

usage: python3 mjlab_stop.py <inputs> <out>
A run is what one model did in simulation: data about the model, never about the machine.
"""
import hashlib
import json
import math
import os
import re
import struct
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

IN, OUT = sys.argv[1], sys.argv[2]
REPO = os.path.join(IN, "unitreerobotics", "unitree_rl_mjlab")


def P(rel):
    return os.path.join(REPO, *rel.split("/"))


ONNX = P("deploy/robots/g1/config/policy/velocity/v0/exported/policy.onnx")
DEPLOY = P("deploy/robots/g1/config/policy/velocity/v0/params/deploy.yaml")
FSMCFG = P("deploy/robots/g1/config/config.yaml")
G1XML = P("src/assets/robots/unitree_g1/xmls/g1.xml")
MENAGERIE = os.path.join(IN, "menagerie", "unitree_g1", "scene.xml")

SIM_DT = 0.005            # velocity_env_cfg.py L424
T_STAND, T_WALK, T_AFTER, HANDOFF_S = 2.0, 4.0, 6.0, 2.0
N_MOMENTS = 30            # one per policy step over the 0.6 s gait cycle
SPEEDS = {"mjlab": [0.5, 1.0], "menagerie": [0.5]}
UPRIGHT_Z, UPRIGHT_TILT = 0.6, 30.0
FPS = 30
# 2026-11-10 recon run (sim 20261110-0020), policy output for an all-zero observation, printed to 4 decimals
RECON_ZERO_OBS = [-0.1006, -0.0834, 0.0272, -0.132, 0.3561, 0.1868, -0.0476, -0.1745, 0.0837, -0.2542, -0.1654,
                  -0.0663, -0.0008, 0.026, 0.1977, 0.3081, 0.8527, 0.2442, 0.2298, -0.1907, -0.1309, 0.8322, 0.2828,
                  -0.5243, -0.4515, -0.1666, 0.1858, 0.1302, -0.243]
lines = []


def say(s=""):
    print(s, flush=True)
    lines.append(str(s))


say("# Unitree's mjlab G1 velocity policy: what each stop does to a walking G1 (simulation)")
say(f"numpy {np.__version__}, mujoco {mujoco.__version__}")
say("\n## inputs")
for root, _, files in os.walk(IN):
    if "menagerie" in root.split(os.sep):
        continue
    for f in sorted(files):
        p = os.path.join(root, f)
        say(f"{os.path.relpath(p, IN)}  {os.path.getsize(p)} bytes  sha256 "
            f"{hashlib.sha256(open(p, 'rb').read()).hexdigest()}")


# ------------------------------------------------------------------ deploy.yaml and config.yaml (Unitree's own)
def ylist(text, key, start=0):
    mt = re.compile(rf"^\s*{key}:\s*\[([^\]]*)\]", re.M).search(text, start)
    return [float(v) for v in mt.group(1).replace("\n", " ").split(",") if v.strip()]


def nums(s):
    return [float(v) for v in re.findall(r"-?[0-9]*\.?[0-9]+", s)]


dtext = open(DEPLOY).read()
KP_POL = np.array(ylist(dtext, "stiffness"))
KD_POL = np.array(ylist(dtext, "damping"))
DEFAULT = np.array(ylist(dtext, "default_joint_pos"))
ja = dtext.index("JointPositionAction")
SCALE = np.array(ylist(dtext, "scale", ja))
OFFSET = np.array(ylist(dtext, "offset", ja))
STEP_DT = float(re.search(r"^step_dt:\s*([0-9.]+)", dtext, re.M).group(1))
PERIOD = float(re.search(r"period:\s*([0-9.]+)", dtext).group(1))
OBS_TERMS = re.findall(r"^  (\w+):\s*$", dtext[dtext.index("\nobservations:"):], re.M)
DEC = int(round(STEP_DT / SIM_DT))
say("\n## deploy.yaml")
say(f"step_dt {STEP_DT}, sim steps per policy step {DEC}, gait period {PERIOD}")
say(f"stiffness {KP_POL.tolist()}")
say(f"damping {KD_POL.tolist()}")
say(f"default_joint_pos {DEFAULT.tolist()}")
say(f"action scale {SCALE.tolist()}")
say(f"action offset == default_joint_pos: {bool(np.allclose(OFFSET, DEFAULT))}")
say(f"observation terms, in order: {OBS_TERMS}")
assert OBS_TERMS == ["base_ang_vel", "projected_gravity", "velocity_commands", "gait_phase", "joint_pos_rel",
                     "joint_vel_rel", "last_action"], "unexpected observation layout"
assert len(KP_POL) == len(KD_POL) == len(DEFAULT) == len(SCALE) == 29

ftext = open(FSMCFG).read()
FIX_KP_HARD = [100, 100, 100, 150, 40, 40] * 2 + [200] * 3 + [40] * 14
FIX_KD_HARD = [2, 2, 2, 4, 2, 2] * 2 + [5] * 3 + [10] * 14
kd_lists = re.findall(r"\bkd:\s*\[([^\]]*)\]", ftext)
kp_lists = re.findall(r"\bkp:\s*\[([^\]]*)\]", ftext)
PASSIVE_KD = np.array(nums(kd_lists[0]))
FIX_KD = np.array(nums(kd_lists[1]))
FIX_KP = np.array(nums(kp_lists[0]))
FIX_T = nums(re.search(r"\bts:\s*\[([^\]]*)\]", ftext).group(1))[-1]
FIX_Q = np.array(nums(re.search(r"qs:\s*\[\s*\[\s*\]\s*,\s*\[([^\]]*)\]", ftext).group(1)))
vblock = ftext[ftext.index("  Velocity:\n    transitions:"):].split("\n\n")[0]
VEL_EXITS = re.findall(r"^      (\w+): (.+)$", vblock, re.M)
say("\n## config.yaml (the deploy's state machine)")
say(f"Passive kd {PASSIVE_KD.tolist()} (all 3: {bool(np.all(PASSIVE_KD == 3))})")
say(f"FixStand kp {FIX_KP.tolist()} (as expected: {FIX_KP.tolist() == FIX_KP_HARD})")
say(f"FixStand kd {FIX_KD.tolist()} (as expected: {FIX_KD.tolist() == FIX_KD_HARD})")
say(f"FixStand ramp {FIX_T} s to {FIX_Q.tolist()} (== default_joint_pos: {bool(np.allclose(FIX_Q, DEFAULT))})")
say(f"Velocity state's exits: {VEL_EXITS}")
assert len(PASSIVE_KD) == len(FIX_KD) == len(FIX_KP) == len(FIX_Q) == 29


# ------------------------------------------------------------------ the ONNX policy, read by hand
def varint(buf, i):
    out = shift = 0
    while True:
        b = buf[i]
        i += 1
        out |= (b & 0x7F) << shift
        if b < 0x80:
            return out, i
        shift += 7


def fields(buf):
    out, i = {}, 0
    while i < len(buf):
        key, i = varint(buf, i)
        fn, wt = key >> 3, key & 7
        if wt == 0:
            v, i = varint(buf, i)
        elif wt == 1:
            v, i = buf[i:i + 8], i + 8
        elif wt == 2:
            n, i = varint(buf, i)
            v, i = buf[i:i + n], i + n
        elif wt == 5:
            v, i = buf[i:i + 4], i + 4
        else:
            raise ValueError(f"wire type {wt} at {i}")
        out.setdefault(fn, []).append((wt, v))
    return out


def sfield(f, n):
    return f[n][0][1].decode("utf-8", "replace") if n in f else ""


def ints(f, n):
    vals = []
    for wt, v in f.get(n, []):
        if wt == 0:
            vals.append(v)
        else:
            i = 0
            while i < len(v):
                x, i = varint(v, i)
                vals.append(x)
    return vals


def tensor(buf):
    f = fields(buf)
    dims, dt = ints(f, 1), (f[2][0][1] if 2 in f else 1)
    if dt != 1:
        raise ValueError(f"initializer type {dt}, expected float32")
    if 9 in f:
        a = np.frombuffer(f[9][0][1], dtype="<f4").copy()
    else:
        a = np.array([x for wt, v in f.get(4, []) for x in
                      (struct.unpack(f"<{len(v) // 4}f", v) if wt == 2 else struct.unpack("<f", v))], dtype=np.float32)
    if a.size != int(np.prod(dims)):
        raise ValueError("initializer size does not match its shape")
    return sfield(f, 8), a.reshape(dims)


graph = fields(fields(open(ONNX, "rb").read())[7][0][1])
INITS = dict(tensor(b) for _, b in graph.get(5, []))
NODES = []
for _, b in graph.get(1, []):
    nf = fields(b)
    NODES.append((sfield(nf, 4), [v.decode() for _, v in nf.get(1, [])]))
OPS = [op for op, _ in NODES]
assert OPS == ["Sub", "Div", "Gemm", "Elu", "Gemm", "Elu", "Gemm", "Elu", "Gemm"], f"unexpected graph {OPS}"
MEAN = INITS[NODES[0][1][1]].reshape(-1)
DIV = INITS[NODES[1][1][1]].reshape(-1)
LAYERS = [(INITS[ins[1]], INITS[ins[2]]) for op, ins in NODES if op == "Gemm"]
say("\n## policy.onnx")
say(f"ops {OPS}; normalizer {MEAN.shape}; layers " + " -> ".join(str(w.shape[1]) for w, _ in LAYERS)
    + f" -> {LAYERS[-1][0].shape[0]} (Gemm transB, ELU alpha 1)")


def run_policy(obs):
    x = (obs.astype(np.float32) - MEAN) / DIV
    for j, (w, b) in enumerate(LAYERS):
        x = w @ x + b
        if j < len(LAYERS) - 1:
            x = np.where(x > 0, x, np.expm1(np.minimum(x, 0))).astype(np.float32)
    return x


z = run_policy(np.zeros(98, np.float32))
zdiff = float(np.max(np.abs(np.round(z.astype(np.float64), 4) - np.array(RECON_ZERO_OBS))))
REPRO_OK = zdiff <= 1.5e-4
say(f"REPRO zero observation vs the 2026-11-10 recon run: max |diff| {zdiff:.6f} -> match {REPRO_OK}")


# ------------------------------------------------------------------ actuators, from g1_constants.py
OMEGA = 10 * 2.0 * 3.1415926535                      # NATURAL_FREQ; DAMPING_RATIO 2.0


def reflected(rot, gears):                            # two-stage planetary: rotor through both stages, and so on
    return rot[0] * (gears[1] * gears[2]) ** 2 + rot[1] * gears[2] ** 2 + rot[2]


ARM = {"5020": reflected((0.139e-4, 0.017e-4, 0.169e-4), (1, 1 + 46 / 18, 1 + 56 / 16)),
       "7520_14": reflected((0.489e-4, 0.098e-4, 0.533e-4), (1, 4.5, 1 + 48 / 22)),
       "7520_22": reflected((0.489e-4, 0.109e-4, 0.738e-4), (1, 4.5, 5)),
       "4010": reflected((0.068e-4, 0.0, 0.0), (1, 5, 5))}
GROUPS = [(r".*_elbow_joint|.*_shoulder_pitch_joint|.*_shoulder_roll_joint|.*_shoulder_yaw_joint|.*_wrist_roll_joint",
           ARM["5020"], 25.0),
          (r".*_hip_pitch_joint|.*_hip_yaw_joint|waist_yaw_joint", ARM["7520_14"], 88.0),
          (r".*_hip_roll_joint|.*_knee_joint", ARM["7520_22"], 139.0),
          (r".*_wrist_pitch_joint|.*_wrist_yaw_joint", ARM["4010"], 5.0),
          (r"waist_pitch_joint|waist_roll_joint", 2 * ARM["5020"], 50.0),
          (r".*_ankle_pitch_joint|.*_ankle_roll_joint", 2 * ARM["5020"], 50.0)]


def group(name):
    for rx, arm, eff in GROUPS:
        if re.fullmatch(rx, name):
            return arm, eff
    raise KeyError(name)


# joint order: mjlab's g1.xml as text (the deploy's joint_ids_map is the identity)
JOINTS = [j.get("name") for j in ET.parse(G1XML).getroot().iter("joint")
          if j.get("type") != "free" and j.get("name")]
assert len(JOINTS) == 29, JOINTS
ARMV = np.array([group(n)[0] for n in JOINTS])
EFF = np.array([group(n)[1] for n in JOINTS])
kp_chk = np.round(ARMV * OMEGA ** 2, 1)
kd_chk = np.round(2.0 * 2.0 * ARMV * OMEGA, 1)
sc_chk = np.round(0.25 * EFF / (ARMV * OMEGA ** 2), 2)
say("\n## actuators (g1_constants.py, recomputed)")
say(f"joints {JOINTS}")
say(f"armature {np.round(ARMV, 6).tolist()}")
say(f"effort limit {EFF.tolist()}")
say(f"armature x (2 pi 10 Hz)^2, to 0.1, == deploy.yaml stiffness: {bool(np.allclose(kp_chk, KP_POL))}; "
    f"2 x 2 x armature x 2 pi 10, to 0.1, == damping: {bool(np.allclose(kd_chk, KD_POL))}; "
    f"0.25 x effort / stiffness, to 0.01, == action scale: {bool(np.allclose(sc_chk, SCALE))}")
say("the actuators below use deploy.yaml's gains (what the robot is sent), with these armatures and effort limits")
LEGS = slice(0, 12)


# ------------------------------------------------------------------ the two bodies
def build_mjlab():
    tree = ET.parse(G1XML)
    root = tree.getroot()
    pairs = [(p, c) for p in root.iter() for c in list(p)]
    removed, gone_tex = [], set()
    for p, c in pairs:
        if c.tag == "geom" and p.tag != "default" and ("mesh" in c.attrib or c.get("type") == "mesh"):
            removed.append(c.get("class") or "-")
            p.remove(c)
        elif p.tag == "asset" and (c.tag in ("mesh", "hfield", "skin") or "file" in c.attrib):
            if c.tag == "texture":
                gone_tex.add(c.get("name"))
            p.remove(c)
    for mat in root.iter("material"):
        if mat.get("texture") in gone_tex:
            del mat.attrib["texture"]
    say(f"mesh geoms removed (no mass: density 0 visuals): {len(removed)}, classes {sorted(set(removed))}")
    for k in root.findall("keyframe"):                     # keyframes sized for no actuators would not compile
        say(f"keyframes removed: {[e.get('name') for e in k]}")
        root.remove(k)
    opt = root.find("option")
    say(f"g1.xml <option> as written: {dict(opt.attrib) if opt is not None else None}")
    if opt is None:
        opt = ET.SubElement(root, "option")
    opt.set("timestep", str(SIM_DT))
    opt.set("iterations", "10")
    opt.set("ls_iterations", "20")
    opt.set("integrator", "implicitfast")
    ET.SubElement(root.find("worldbody"), "geom",
                  {"name": "floor_added", "type": "plane", "size": "0 0 0.05", "condim": "3"})
    for j in root.iter("joint"):
        if j.get("name") in JOINTS:
            j.set("armature", f"{ARMV[JOINTS.index(j.get('name'))]:.9g}")
    for a in root.findall("actuator"):
        root.remove(a)
    act = ET.SubElement(root, "actuator")
    for i, n in enumerate(JOINTS):
        ET.SubElement(act, "general", {
            "name": n, "joint": n, "gaintype": "fixed", "biastype": "affine", "gainprm": f"{KP_POL[i]:g}",
            "biasprm": f"0 {-KP_POL[i]:g} {-KD_POL[i]:g}", "forcelimited": "true",
            "forcerange": f"{-EFF[i]:g} {EFF[i]:g}"})
    feet = others = 0
    for g in root.iter("geom"):
        nm = g.get("name") or ""
        if re.fullmatch(r"(left|right)_foot[1-7]_collision", nm):     # FULL_COLLISION in g1_constants.py
            g.set("condim", "3")
            g.set("priority", "1")
            g.set("friction", "0.6")
            feet += 1
        elif nm.endswith("_collision"):
            g.set("condim", "1")
            others += 1
    say(f"collision geoms set as FULL_COLLISION: feet {feet} (condim 3, priority 1, friction 0.6), others {others} "
        f"(condim 1)")
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode"))


def build_menagerie():
    m = mujoco.MjModel.from_xml_path(MENAGERIE)
    names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in range(1, m.njnt)]
    assert names == JOINTS, "Menagerie joint order differs"
    for a in range(m.nu):
        j = int(m.actuator_trnid[a, 0])
        i = JOINTS.index(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j))
        assert i == a, "Menagerie actuator order differs"
        m.actuator_gainprm[a, :] = 0
        m.actuator_biasprm[a, :] = 0
        m.actuator_gainprm[a, 0] = KP_POL[i]
        m.actuator_biasprm[a, 1] = -KP_POL[i]
        m.actuator_biasprm[a, 2] = -KD_POL[i]
        m.actuator_forcelimited[a] = 1
        m.actuator_forcerange[a] = [-EFF[i], EFF[i]]
        m.actuator_ctrllimited[a] = 0
        m.dof_armature[m.jnt_dofadr[j]] = ARMV[i]
    m.opt.timestep = SIM_DT
    m.opt.iterations = 10
    m.opt.ls_iterations = 20
    m.opt.integrator = int(mujoco.mjtIntegrator.mjINT_IMPLICITFAST)
    return m


class Ctx:
    def __init__(self, name, m, floor):
        self.name, self.m = name, m
        assert m.nq == 36 and m.nv == 35 and m.nu == 29, (m.nq, m.nv, m.nu)
        assert int(m.jnt_type[0]) == int(mujoco.mjtJoint.mjJNT_FREE)
        assert [int(m.actuator_trnid[a, 0]) for a in range(29)] == list(range(1, 30))
        self.floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, floor)
        self.gbody = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g]) for g in range(m.ngeom)]
        self.gname = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or f"{self.gbody[g]} geom {g}"
                      for g in range(m.ngeom)]
        self.lfoot = {g for g in range(m.ngeom) if self.gbody[g] in ("left_ankle_roll_link", "left_ankle_pitch_link")}
        self.rfoot = {g for g in range(m.ngeom) if self.gbody[g] in ("right_ankle_roll_link", "right_ankle_pitch_link")}
        coll = [g for g in range(m.ngeom) if (m.geom_contype[g] or m.geom_conaffinity[g]) and g != self.floor]
        say(f"[{name}] nq {m.nq} nv {m.nv} nu {m.nu}, mass {m.body_subtreemass[0]:.3f} kg, timestep {m.opt.timestep}, "
            f"integrator {int(m.opt.integrator)}, iterations {m.opt.iterations}/{m.opt.ls_iterations}; colliding geoms "
            f"{len(coll)}, on the feet {len([g for g in coll if g in self.lfoot | self.rfoot])}; foot friction "
            f"{sorted({round(float(m.geom_friction[g][0]), 3) for g in coll if g in self.lfoot | self.rfoot})}")
        say(f"[{name}] armature {np.round(m.dof_armature[6:], 6).tolist() == np.round(ARMV, 6).tolist()}, "
            f"kp {np.allclose(m.actuator_gainprm[:, 0], KP_POL)}, kd {np.allclose(-m.actuator_biasprm[:, 2], KD_POL)}, "
            f"force range {np.allclose(m.actuator_forcerange[:, 1], EFF)}; joint actuatorfrcrange "
            f"{[round(float(v), 1) for v in m.jnt_actfrcrange[1:, 1]]}")


say("\n## bodies")
CTXS = []
try:
    CTXS.append(Ctx("mjlab", build_mjlab(), "floor_added"))
except Exception as e:                                    # keep going: the Menagerie body still pays for the run
    say(f"[mjlab] model build FAILED: {type(e).__name__}: {e}")
CTXS.append(Ctx("menagerie", build_menagerie(), "floor"))
if len(CTXS) == 2:
    a, b = CTXS
    names = lambda m: [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, i) for i in range(m.nbody)]
    common = sorted(set(names(a.m)) & set(names(b.m)) - {"world"})
    dm = [abs(float(a.m.body_mass[names(a.m).index(n)]) - float(b.m.body_mass[names(b.m).index(n)])) for n in common]
    say(f"body masses, mjlab vs Menagerie: {len(common)} bodies in common, max |diff| {max(dm):.4f} kg")


# ------------------------------------------------------------------ the deploy loop
def gravity_b(q):
    qw, qx, qy, qz = q
    return np.array([2 * (-qz * qx + qw * qy), -2 * (qz * qy + qw * qx), 1 - 2 * (qw * qw + qz * qz)])


def tilt_deg(q):
    c = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


def yaw_deg(q):
    qw, qx, qy, qz = q
    return math.degrees(math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz)))


class PS:
    def __init__(self):
        self.last = np.zeros(29, np.float32)
        self.phase = 0.0
        self.target = DEFAULT.copy()

    def copy(self):
        o = PS()
        o.last, o.phase, o.target = self.last.copy(), self.phase, self.target.copy()
        return o


def policy_step(d, ps, cmd):
    ps.phase = math.fmod(ps.phase + STEP_DT / PERIOD, 1.0)            # observations.h gait_phase
    obs = np.zeros(98, np.float32)
    obs[0:3] = d.qvel[3:6]
    obs[3:6] = gravity_b(d.qpos[3:7])
    obs[6:9] = cmd
    if math.sqrt(sum(c * c for c in cmd)) >= 0.1:                     # below 0.1 the phase reads (0, 0): standing
        obs[9], obs[10] = math.sin(ps.phase * 2 * math.pi), math.cos(ps.phase * 2 * math.pi)
    obs[11:40] = d.qpos[7:36] - DEFAULT
    obs[40:69] = d.qvel[6:35]
    obs[69:98] = ps.last
    a = run_policy(obs)
    ps.last = a
    ps.target = OFFSET + SCALE * a.astype(np.float64)
    d.ctrl[:] = ps.target


def set_gains(m, kp, kd):
    m.actuator_gainprm[:, 0] = kp
    m.actuator_biasprm[:, 1] = -np.asarray(kp, dtype=float)
    m.actuator_biasprm[:, 2] = -np.asarray(kd, dtype=float)


def clone(m, src):
    try:
        dst = mujoco.MjData(m)
        mujoco.mj_copyData(dst, m, src)
        return dst
    except AttributeError:
        import copy
        return copy.deepcopy(src)


def init_state(ctx):
    d = mujoco.MjData(ctx.m)
    d.qpos[:] = 0
    d.qpos[2], d.qpos[3] = 0.8, 1.0                    # HOME_KEYFRAME pos (0, 0, 0.8)
    d.qpos[7:36] = DEFAULT
    mujoco.mj_forward(ctx.m, d)
    return d


def POL(v):
    return {"kind": "policy", "cmd": [float(v), 0.0, 0.0]}


def DAMP(kd):
    return {"kind": "damp", "kd": kd}


def MODES(v):
    return {
        "continue": [(0.0, POL(v))],
        "zero_cmd": [(0.0, POL(0))],
        "passive_kd3": [(0.0, DAMP(PASSIVE_KD))],
        "damp_kd8": [(0.0, DAMP(8.0))],
        "zero_torque": [(0.0, DAMP(0.0))],
        "hold_last": [(0.0, {"kind": "hold"})],
        "fixstand": [(0.0, {"kind": "fixstand"})],
        "zero_cmd_then_passive": [(0.0, POL(0)), (HANDOFF_S, DAMP(PASSIVE_KD))],
        "zero_cmd_then_fixstand": [(0.0, POL(0)), (HANDOFF_S, {"kind": "fixstand"})],
    }


BADQACC = int(mujoco.mjtWarning.mjWARN_BADQACC)


def badqacc(d):
    try:
        return int(d.warning[BADQACC].number)
    except Exception:
        return -1


def simulate(ctx, d, ps, schedule, n, rec_q=False, snaps=()):
    """n policy steps under a schedule [(t_start, spec)]; records state after every policy step."""
    m = ctx.m
    cur, t_enter, q0 = None, 0.0, None
    R = {k: [] for k in ("t", "z", "tilt", "x", "y", "vxy", "lf", "rf", "non", "over", "reqmax", "f_leg", "qd_leg")}
    Q, S = [], []
    w0 = badqacc(d)
    for i in range(n):
        t = i * STEP_DT
        spec = None
        for start, sp in schedule:
            if t >= start - 1e-9:
                spec = sp
        if spec is not cur:
            cur, t_enter = spec, t
            kind = spec["kind"]
            if kind in ("policy", "hold"):
                set_gains(m, KP_POL, KD_POL)
            elif kind == "fixstand":
                set_gains(m, FIX_KP, FIX_KD)
                q0 = ps.target.copy()                    # State_FixStand starts from the last commanded q
            else:
                set_gains(m, 0.0, spec["kd"])
                d.ctrl[:] = d.qpos[7:36]
            if kind == "hold":
                d.ctrl[:] = ps.target
        kind = cur["kind"]
        if kind == "policy":
            policy_step(d, ps, cur["cmd"])
        elif kind == "fixstand":
            s = min(1.0, (t - t_enter) / FIX_T)
            d.ctrl[:] = q0 + s * (FIX_Q - q0)
        for _ in range(DEC):
            mujoco.mj_step(m, d)
        lf = rf = False
        non = -1
        for c in range(d.ncon):
            g1, g2 = d.contact[c].geom1, d.contact[c].geom2
            if ctx.floor not in (g1, g2):
                continue
            g = g2 if g1 == ctx.floor else g1
            if g in ctx.lfoot:
                lf = True
            elif g in ctx.rfoot:
                rf = True
            elif non < 0:
                non = g
        q = d.qpos
        kp, kd = m.actuator_gainprm[:, 0], -m.actuator_biasprm[:, 2]
        req = np.abs(kp * (d.ctrl - q[7:36]) - kd * d.qvel[6:35])[LEGS]
        tl = tilt_deg(q[3:7])
        for k, v in (("t", t + STEP_DT), ("z", float(q[2])), ("tilt", tl), ("x", float(q[0])), ("y", float(q[1])),
                     ("vxy", float(math.hypot(d.qvel[0], d.qvel[1]))), ("lf", lf), ("rf", rf), ("non", non),
                     ("over", bool(np.any(req > EFF[LEGS]))), ("reqmax", float(req.max())),
                     ("f_leg", float(np.mean(np.abs(d.actuator_force[LEGS])))),
                     ("qd_leg", float(np.mean(np.abs(d.qvel[6:18]))))):
            R[k].append(v)
        if rec_q:
            Q.append(d.qpos.copy())
        if i in snaps:
            S.append((clone(m, d), ps.copy(), i))
    R = {k: np.array(v) for k, v in R.items()}
    R["badqacc"] = badqacc(d) - w0
    return R, Q, S


def stats(vals, nd=2):
    v = [x for x in vals if x is not None]
    if not v:
        return "-"
    return f"{round(min(v), nd)}/{round(float(np.median(v)), nd)}/{round(max(v), nd)} (n {len(v)})"


def metrics(ctx, R, x0, y0, handoff=None):
    t = R["t"]
    up = (R["z"] > UPRIGHT_Z) & (R["tilt"] < UPRIGHT_TILT)
    out = {"upright_end": bool(up[-1]), "min_z": round(float(R["z"].min()), 3),
           "max_tilt": round(float(R["tilt"].max()), 1), "end_z": round(float(R["z"][-1]), 3),
           "end_tilt": round(float(R["tilt"][-1]), 1), "badqacc": R["badqacc"]}
    touch = np.nonzero(R["non"] >= 0)[0]
    travel = np.hypot(R["x"] - x0, R["y"] - y0)
    out["first_touch_s"] = round(float(t[touch[0]]), 2) if len(touch) else None
    out["first_touch"] = ctx.gname[int(R["non"][touch[0]])] if len(touch) else None
    out["travel_end_m"] = round(float(travel[-1]), 3)
    n = int(round(0.5 / STEP_DT))
    va = R["vxy"]
    still = next((i for i in range(len(va) - n + 1) if np.all(va[i:i + n] < 0.05)), None)
    out["still_after_s"] = None if still is None else round(float(t[still]), 2)
    out["travel_to_still_m"] = None if still is None else round(float(travel[still]), 3)
    lifts = []
    for key in ("lf", "rf"):
        c = R[key]
        for i in range(1, len(c) - 1):
            if c[i - 1] and not c[i] and not c[i + 1] and up[i]:
                lifts.append(float(t[i]))
    out["liftoffs"] = len(lifts)
    out["last_liftoff_s"] = round(max(lifts), 2) if lifts else None
    out["over_cap_share_legs"] = round(float(np.mean(R["over"])), 4)
    out["max_req_leg_nm"] = round(float(R["reqmax"].max()), 1)
    if handoff is not None:
        h = int(round(handoff / STEP_DT)) - 1
        out["upright_at_handoff"] = bool(up[h])
        out["vxy_at_handoff"] = round(float(R["vxy"][h]), 3)
        out["travel_at_handoff_m"] = round(float(travel[h]), 3)
        after = up[h + 1:]
        out["fell_after_handoff_s"] = (round(float(t[h + 1 + int(np.argmin(after))] - handoff), 2)
                                       if not after.all() else None)
    return out


def resample(rows):
    rows = np.array(rows)
    n = int(len(rows) * FPS / 50)
    return rows[np.minimum(np.round(np.arange(n) * 50 / FPS).astype(int), len(rows) - 1)]


ALL = {"gates": {}, "sweeps": [], "clean_start": {}, "summary": {}}
N_STAND10 = int(round(10.0 / STEP_DT))
N_PRE = int(round((T_STAND + T_WALK) / STEP_DT))
N_AFTER = int(round(T_AFTER / STEP_DT))

# ------------------------------------------------------------------ gate 1: stand 10 s at zero command
say("\n## gate: stand 10 s at zero command from the default pose (pelvis dropped from 0.8 m)")
for ctx in CTXS:
    d = init_state(ctx)
    R, _, _ = simulate(ctx, d, PS(), [(0.0, POL(0))], N_STAND10)
    g = metrics(ctx, R, 0.0, 0.0)
    g["drift_m"] = g.pop("travel_end_m")
    ALL["gates"][f"{ctx.name}_stand10"] = g
    say(f"[{ctx.name}] upright at 10 s {g['upright_end']}; end pelvis z {g['end_z']} m, tilt {g['end_tilt']} deg, "
        f"max tilt {g['max_tilt']}; drift {g['drift_m']} m; still after {g['still_after_s']} s; lift-offs "
        f"{g['liftoffs']} (last {g['last_liftoff_s']} s); non-foot touch {g['first_touch']}; bad-qacc {g['badqacc']}")

# ------------------------------------------------------------------ the walking sweeps
TRAJ = {}
for ctx in CTXS:
    for v in SPEEDS[ctx.name]:
        d = init_state(ctx)
        snaps = set(range(N_PRE - 1, N_PRE - 1 + N_MOMENTS))
        R, Q, S = simulate(ctx, d, PS(), [(0.0, POL(0)), (T_STAND, POL(v))], N_PRE + N_MOMENTS, rec_q=True,
                           snaps=snaps)
        Q = np.array(Q)
        i4, i6 = int(round((T_STAND + T_WALK - 2.0) / STEP_DT)) - 1, N_PRE - 1
        dx, dy = Q[i6, 0] - Q[i4, 0], Q[i6, 1] - Q[i4, 1]
        yaw = yaw_deg(Q[i6, 3:7])
        walk_ok = bool(R["z"][:N_PRE].min() > UPRIGHT_Z and np.all(R["non"][:N_PRE] < 0))
        say(f"\n## [{ctx.name}] walking at {v} m/s: {T_STAND} s standing, then {T_WALK} s walking, then the stop at "
            f"{N_MOMENTS} moments {T_STAND + T_WALK:.2f}-{T_STAND + T_WALK + (N_MOMENTS - 1) * STEP_DT:.2f} s")
        say(f"walking check: last 2 s moved {dx:.3f} m forward, {dy:.3f} m sideways ({math.hypot(dx, dy) / 2:.3f} m/s); "
            f"heading {yaw:.1f} deg; min pelvis z {R['z'][:N_PRE].min():.3f}; no fall and no non-foot touch: {walk_ok}")
        ALL["gates"][f"{ctx.name}_walk_{v}"] = {"dx_last2s": round(float(dx), 3), "dy_last2s": round(float(dy), 3),
                                               "speed_mps": round(math.hypot(dx, dy) / 2, 3),
                                               "heading_deg": round(yaw, 1), "ok": walk_ok}
        rows = {}
        for mode, sched in MODES(v).items():
            rows[mode] = []
            for k, (d0, ps0, i) in enumerate(S):
                want = ctx.name == "mjlab" and v == 0.5 and k == 0
                d1 = clone(ctx.m, d0)
                Rb, Qb, _ = simulate(ctx, d1, ps0.copy(), sched, N_AFTER, rec_q=want)
                row = metrics(ctx, Rb, float(d0.qpos[0]), float(d0.qpos[1]),
                              HANDOFF_S if mode.startswith("zero_cmd_then") else None)
                row.update({"model": ctx.name, "speed": v, "mode": mode, "moment": k,
                            "t_stop": round((i + 1) * STEP_DT, 2), "phase_at_stop": round(ps0.phase, 4),
                            "vxy_at_stop": round(float(math.hypot(d0.qvel[0], d0.qvel[1])), 3)})
                rows[mode].append(row)
                ALL["sweeps"].append(row)
                if want:
                    TRAJ[mode] = resample(list(Q[N_PRE - 1 - 100 + 1:N_PRE]) + Qb)
        say("mode | upright at end | first non-foot touch s after stop min/med/max | travel at end m (upright runs) "
            "| still after s | lift-offs after stop min/med/max | last lift-off s | legs over cap, max share | "
            "bad-qacc runs")
        summ = {}
        for mode, rs in rows.items():
            upr = [r for r in rs if r["upright_end"]]
            summ[mode] = {"upright": len(upr), "of": len(rs)}
            say(" | ".join(str(x) for x in [
                mode, f"{len(upr)}/{len(rs)}", stats([r["first_touch_s"] for r in rs]),
                stats([r["travel_end_m"] for r in upr], 3), stats([r["still_after_s"] for r in upr]),
                stats([r["liftoffs"] for r in rs], 0), stats([r["last_liftoff_s"] for r in rs]),
                max(r["over_cap_share_legs"] for r in rs), sum(1 for r in rs if r["badqacc"])]))
        for mode in ("zero_cmd_then_passive", "zero_cmd_then_fixstand"):
            rs = rows[mode]
            say(f"{mode}: upright at the hand-off {sum(r['upright_at_handoff'] for r in rs)}/{len(rs)}; speed then "
                f"{stats([r['vxy_at_handoff'] for r in rs], 3)} m/s; fell after the hand-off in "
                f"{stats([r['fell_after_handoff_s'] for r in rs])} s")
        ALL["summary"][f"{ctx.name}_{v}"] = summ
        if ctx.name == "mjlab" and v == 0.5:
            say("per moment, zero_cmd: moment, t_stop, speed at stop, upright, travel end m, still after s, lift-offs")
            for r in rows["zero_cmd"]:
                say(f"  {r['moment']} {r['t_stop']} {r['vxy_at_stop']} {r['upright_end']} {r['travel_end_m']} "
                    f"{r['still_after_s']} {r['liftoffs']}")

# ------------------------------------------------------------------ clean start: standing, then each policy-off stop
say("\n## clean start: 4 s standing at zero command, then the stop (6 s after it)")
for ctx in CTXS:
    d = init_state(ctx)
    _, _, S = simulate(ctx, d, PS(), [(0.0, POL(0))], 200, snaps={199})
    d0, ps0, _ = S[0]
    for mode in ("passive_kd3", "damp_kd8", "zero_torque", "hold_last", "fixstand", "zero_cmd"):
        Rb, _, _ = simulate(ctx, clone(ctx.m, d0), ps0.copy(), MODES(0)[mode], N_AFTER)
        r = metrics(ctx, Rb, float(d0.qpos[0]), float(d0.qpos[1]))
        ALL["clean_start"][f"{ctx.name}_{mode}"] = r
        say(f"[{ctx.name}] {mode}: upright at end {r['upright_end']}; first non-foot touch {r['first_touch_s']} s "
            f"({r['first_touch']}); end z {r['end_z']}, tilt {r['end_tilt']}; legs over cap {r['over_cap_share_legs']}, "
            f"max request {r['max_req_leg_nm']} N*m; bad-qacc {r['badqacc']}")

for mode, rows in TRAJ.items():
    np.save(os.path.join(OUT, f"qpos_{mode}.npy"), np.array(rows, dtype=np.float64))
say(f"\nsaved {FPS} fps qpos (2 s before the stop + 6 s after, mjlab body, 0.5 m/s, moment 0): {sorted(TRAJ)}")

# ------------------------------------------------------------------ verdicts
say("\n## verdicts")
st = ALL["gates"].get("mjlab_stand10")
say(f"seal 2c1d4b50 (mjlab policy keeps the G1 upright 10 s at zero command, numpy, sealed sim): "
    f"{None if st is None else st['upright_end']}")
say(f"REPRO zero-observation output matches the recon run: {REPRO_OK}")
with open(os.path.join(OUT, "results.json"), "w") as f:
    json.dump(ALL, f, indent=1, default=str)
with open(os.path.join(OUT, "mjlab_stop.md"), "w") as f:
    f.write("\n".join(lines) + "\n")
