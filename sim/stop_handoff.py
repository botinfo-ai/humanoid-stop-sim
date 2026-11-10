"""Can a walking G1 be stopped with its walking policy switched off, without a fall? (simulation)

stop_sweep.py (humanoid-stop-sim) found that, under Unitree's published G1 walking policy, only the zero-velocity
command kept the G1 upright at all 20 stop moments of a gait cycle; holding the last targets, holding the default
pose, damping and zero torque each left it on the floor at all 20. But zero velocity keeps the learned policy in
charge. Unitree's own real-robot deploy (unitree_rl_gym deploy_real) goes the other way on the way in: zero torque,
then the default pose held by the PD gains, with the hoist lowered only until the feet touch the ground, then the
policy (its README L54-60). On the way out it sends damping and the robot falls (its README L69).

This script tries two-stage stops that end with the policy off and the robot held by PD gains alone: zero velocity
first, then a hand-off. Every mode starts with zero velocity at the stop moment:

  stopmove             zero velocity, the policy keeps running (the reference)
  zv1_hold_default     after 1.0 s: policy off, PD gains pull every leg joint to the config's default angles at once
  zv2_hold_default     the same after 2.0 s
  zv_ds_hold_last      at the first moment after 0.5 s with both feet on the floor: policy off, hold the last targets
  zv_ds_hold_default   the same trigger: policy off, default angles at once
  zv_ds_ramp_default   the same trigger: policy off, targets moved linearly from the last targets to the default
                       angles over 1.0 s (our choice of ramp, not Unitree's)
  zv_ds_damp           the same trigger: kp 0, kd 8 on every joint (unitree_rl_gym's create_damping_cmd)

Each runs at the same 20 stop moments as stop_sweep.py (4.00-4.76 s, one 0.8 s gait cycle) and goes on for 6 s
after the stop. The loader, the network, the deploy loop and the contact reporting are stop_sweep.py's. The gains are
the config's (kps 100/100/100/150/40/40, kds 2/2/2/4/2/2 per leg). The 12-joint model's upper body is one rigid piece.

usage: python3 stop_handoff.py <inputs> <out>
A run is what one model did in simulation: data about the model, never about the machine.
"""
import io
import json
import math
import os
import pickle
import re
import sys
import zipfile
from collections import OrderedDict

import mujoco
import numpy as np

IN, OUT = sys.argv[1], sys.argv[2]
RL = os.path.join(IN, "unitreerobotics", "unitree_rl_gym")
CFG = os.path.join(RL, "deploy", "deploy_mujoco", "configs", "g1.yaml")
POLICY = os.path.join(RL, "deploy", "pre_train", "g1", "motion.pt")
SCENE = os.path.join(RL, "resources", "robots", "g1_description", "scene.xml")
HELPER = os.path.join(RL, "deploy", "deploy_real", "common", "command_helper.py")

GAIT_PERIOD = 0.8     # s: the policy's phase clock, (counter * dt) % 0.8 / 0.8 in deploy_mujoco.py
T_STOPS = [round(4.0 + 0.04 * i, 2) for i in range(20)]
T_AFTER = 6.0         # s simulated after the stop
ZV_MIN = 0.5          # s of zero velocity before a double-support hand-off may happen
RAMP_S = 1.0          # s for zv_ds_ramp_default
MODES = ["stopmove", "zv1_hold_default", "zv2_hold_default", "zv_ds_hold_last", "zv_ds_hold_default",
         "zv_ds_ramp_default", "zv_ds_damp"]
FOOT_BODIES = {"left_ankle_roll_link", "right_ankle_roll_link", "left_ankle_pitch_link", "right_ankle_pitch_link"}
TORSO, HEAD = "torso_link_23dof_rev_1_0", "head_link"
KEEP_QPOS_AT = 4.0    # the stop moment whose trajectories are saved (25 fps) for a later render

lines = []


def say(s=""):
    print(s)
    lines.append(str(s))


# ------------------------------------------------------------------ config (the deploy's own yaml, parsed by regex)
def cfg_list(text, key):
    m = re.search(rf"^{key}:\s*\[([^\]]*)\]", text, re.M | re.S)
    return [float(v) for v in m.group(1).replace("\n", " ").split(",") if v.strip()]


def cfg_num(text, key):
    return float(re.search(rf"^{key}:\s*([-0-9.eE]+)", text, re.M).group(1))


cfg_text = open(CFG).read()
C = {k: cfg_list(cfg_text, k) for k in ("kps", "kds", "default_angles", "cmd_scale", "cmd_init")}
for k in ("simulation_dt", "control_decimation", "ang_vel_scale", "dof_pos_scale", "dof_vel_scale", "action_scale",
          "num_actions", "num_obs"):
    C[k] = cfg_num(cfg_text, k)
helper = open(HELPER).read()
damp_block = helper[helper.index("def create_damping_cmd"):helper.index("def create_zero_cmd")]
KD_DAMP = float(re.search(r"\.kd\s*=\s*([0-9.]+)", damp_block).group(1))
NA, NOBS = int(C["num_actions"]), int(C["num_obs"])
say("# A walking G1 under Unitree's published policy: zero velocity, then a hand-off to PD gains alone (simulation)")
say(f"numpy {np.__version__}, mujoco {mujoco.__version__}")
say(f"config g1.yaml: kps {C['kps']} kds {C['kds']}")
say(f"default_angles {C['default_angles']} action_scale {C['action_scale']} cmd_init {C['cmd_init']} "
    f"cmd_scale {C['cmd_scale']} dt {C['simulation_dt']} decimation {int(C['control_decimation'])} obs {NOBS}")
say(f"create_damping_cmd kd (command_helper.py): {KD_DAMP:g}")


# ------------------------------------------------------------------ TorchScript weights without torch (as stop_walk.py)
class TObj:
    """Stands in for any class the pickle names (TorchScript modules, torch helpers)."""
    def __new__(cls, *a, **k):
        o = object.__new__(cls)
        o.args, o.state = a, None
        return o

    def __init__(self, *a, **k):
        pass

    def __setstate__(self, state):
        self.state = state


DTYPES = {"FloatStorage": np.float32, "DoubleStorage": np.float64, "HalfStorage": np.float16,
          "LongStorage": np.int64, "IntStorage": np.int32, "ShortStorage": np.int16, "CharStorage": np.int8,
          "ByteStorage": np.uint8, "BoolStorage": np.bool_}


class StorageType:
    def __init__(self, name):
        self.name = name


def rebuild_tensor(storage, offset, size, stride, *rest):
    raw, dt = storage
    item = np.dtype(dt).itemsize
    base = np.frombuffer(raw, dtype=dt)
    offset, size, stride = int(offset), tuple(int(n) for n in size), tuple(int(s) for s in stride)
    if offset < 0 or len(size) != len(stride) or any(v < 0 for v in size + stride):
        raise ValueError("policy file: a tensor has a malformed layout")
    if 0 not in size and offset + sum((n - 1) * s for n, s in zip(size, stride)) >= base.size:
        raise ValueError("policy file: a tensor reaches past the end of its storage")   # checked before as_strided
    if len(size) == 0:
        return np.array(base[offset])
    arr = np.lib.stride_tricks.as_strided(base[offset:], shape=size, strides=tuple(s * item for s in stride))
    return np.array(arr)


def load_torchscript(path):
    z = zipfile.ZipFile(path)
    names = z.namelist()
    pkl = [n for n in names if n.endswith("/data.pkl") and n.count("/") == 1]
    root = pkl[0].split("/")[0]
    say(f"torchscript archive: {len(names)} entries, root '{root}', data.pkl {pkl[0]}")

    safe = {("collections", "OrderedDict"): OrderedDict, ("builtins", "set"): set,
            ("builtins", "frozenset"): frozenset, ("_codecs", "encode"): lambda s, enc="latin1": s.encode(enc)}

    class U(pickle.Unpickler):
        def find_class(self, mod, name):                     # nothing in the file can call real code but these
            if (mod, name) in safe:
                return safe[(mod, name)]
            if mod == "torch._utils" and name in ("_rebuild_tensor_v2", "_rebuild_tensor"):
                return rebuild_tensor
            if mod == "torch._utils" and name == "_rebuild_parameter":
                return lambda data, *a: data
            if mod == "torch" and name.endswith("Storage"):
                return StorageType(name)
            if mod.startswith("torch"):
                return lambda *a, **k: a[0] if a else None
            return type(name, (TObj,), {"_mod": mod})

        def persistent_load(self, pid):
            kind, stype, key = pid[0], pid[1], pid[2]
            name = stype.name if isinstance(stype, StorageType) else str(stype)
            if name not in DTYPES:
                say(f"unknown storage type {name!r}: read as float32")
            return z.read(f"{root}/data/{key}"), DTYPES.get(name, np.float32)

    return U(io.BytesIO(z.read(pkl[0]))).load()


def walk(o, path, out, seen):
    if isinstance(o, np.ndarray):
        out[path] = o
        return
    if id(o) in seen:
        return
    seen.add(id(o))
    if isinstance(o, TObj):
        walk(o.state, path, out, seen)
    elif isinstance(o, dict):
        for k, v in o.items():
            walk(v, f"{path}.{k}" if path else str(k), out, seen)
    elif isinstance(o, (list, tuple)):
        for i, v in enumerate(o):
            walk(v, f"{path}[{i}]", out, seen)


tensors = {}
walk(load_torchscript(POLICY), "", tensors, set())


def pick(suffix):
    hits = [k for k in tensors if k.endswith(suffix) and "[" not in k]
    return tensors[hits[0]] if hits else None


W_IH, W_HH, B_IH, B_HH = (pick(s) for s in ("weight_ih_l0", "weight_hh_l0", "bias_ih_l0", "bias_hh_l0"))
lin = [(k, v) for k, v in tensors.items() if re.search(r"(^|\.)actor\.\d+\.weight$", k)] or \
      [(k, v) for k, v in tensors.items() if re.fullmatch(r"\d+\.weight", k)]
lin.sort(key=lambda kv: int(kv[0].split(".")[-2]))
ACTOR = [(w.astype(np.float64), tensors[k[:-len("weight")] + "bias"].astype(np.float64)) for k, w in lin]
RECURRENT = W_IH is not None
H0, C0 = pick("hidden_state"), pick("cell_state")
say(f"policy: {'LSTM ' + str(W_HH.shape[1]) + ' + ' if RECURRENT else ''}actor "
    + " -> ".join(str(w.shape[1]) for w, _ in ACTOR) + f" -> {ACTOR[-1][0].shape[0]} (ELU between layers)")
assert ACTOR and ACTOR[-1][0].shape[0] == NA, "actor output size does not match num_actions"
assert (W_IH.shape[1] if RECURRENT else ACTOR[0][0].shape[1]) == NOBS, "input size does not match num_obs"


def sig(x):
    return 1.0 / (1.0 + np.exp(-x))


class Policy:
    def __init__(self):
        n = W_HH.shape[1] if RECURRENT else 0
        self.h = H0.reshape(-1).astype(np.float64) if H0 is not None and H0.size == n else np.zeros(n)
        self.c = C0.reshape(-1).astype(np.float64) if C0 is not None and C0.size == n else np.zeros(n)

    def __call__(self, obs):
        x = obs.astype(np.float64)
        if RECURRENT:
            g = W_IH @ x + B_IH + W_HH @ self.h + B_HH
            i, f, gg, o = np.split(g, 4)
            self.c = sig(f) * self.c + sig(i) * np.tanh(gg)
            self.h = sig(o) * np.tanh(self.c)
            x = self.h
        for j, (w, b) in enumerate(ACTOR):
            x = w @ x + b
            if j < len(ACTOR) - 1:
                x = np.where(x > 0, x, np.expm1(np.minimum(x, 0)))
        return x.astype(np.float32)


# ------------------------------------------------------------------ the deploy loop (deploy_mujoco.py), plus a stop
def gravity_orientation(q):
    qw, qx, qy, qz = q
    return np.array([2 * (-qz * qx + qw * qy), -2 * (qz * qy + qw * qx), 1 - 2 * (qw * qw + qz * qz)])


m = mujoco.MjModel.from_xml_path(SCENE)
m.opt.timestep = C["simulation_dt"]
dt, dec = m.opt.timestep, int(C["control_decimation"])
floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
body = lambda g: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g])
pel = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
GEOM_TYPES = {int(getattr(mujoco.mjtGeom, n)): n[7:].lower() for n in
              ("mjGEOM_PLANE", "mjGEOM_SPHERE", "mjGEOM_CAPSULE", "mjGEOM_ELLIPSOID", "mjGEOM_CYLINDER", "mjGEOM_BOX",
               "mjGEOM_MESH")}


def part_name(g):
    if int(m.geom_type[g]) == int(mujoco.mjtGeom.mjGEOM_MESH):
        return mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MESH, m.geom_dataid[g])
    return (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g)
            or f"{body(g)} {GEOM_TYPES.get(int(m.geom_type[g]), 'geom')} #{g}")


PART = [part_name(g) for g in range(m.ngeom)]
collides = [g for g in range(m.ngeom) if m.geom_contype[g] or m.geom_conaffinity[g]]
say(f"model: nq {m.nq}, nu {m.nu}, mass {float(m.body_subtreemass[0]):.2f} kg, timestep {dt}")
say(f"colliding shapes on the pelvis body: {sorted(PART[g] for g in collides if body(g) == 'pelvis')}")

KP, KD = np.array(C["kps"]), np.array(C["kds"])
DEF = np.array(C["default_angles"], dtype=np.float32)


def phase_of(t):
    return round((t / GAIT_PERIOD) % 1.0, 3)


def feet_down(d):
    feet = {body(g) for c in range(d.ncon) for g in (d.contact[c].geom1, d.contact[c].geom2)
            if floor in (d.contact[c].geom1, d.contact[c].geom2) and body(g) in FOOT_BODIES}
    return sorted({"left" if f.startswith("left") else "right" for f in feet})


def run(mode, t_stop, keep_qpos=False):
    n_stop, n_all = int(round(t_stop / dt)), int(round((t_stop + T_AFTER) / dt))
    d = mujoco.MjData(m)
    pol = Policy()
    cmd = np.array(C["cmd_init"], dtype=np.float32)
    action = np.zeros(NA, dtype=np.float32)
    target = DEF.copy()
    obs = np.zeros(NOBS, dtype=np.float32)
    policy_on, damp = True, False
    walk_bad, first, first_part = 0, OrderedDict(), OrderedDict()
    res6 = np.zeros(6)
    at_stop, hand = None, None
    ramp_from, ramp_k0 = None, None
    after = {"pz": [], "tilt": [], "vxy": [], "t": [], "px": [], "py": []}
    qkeep = []
    for k in range(n_all):
        if k == n_stop:
            at_stop = {"x": float(d.qpos[0]), "y": float(d.qpos[1]), "v": float(np.hypot(d.qvel[0], d.qvel[1])),
                       "feet_on_floor": feet_down(d)}
            cmd = np.zeros(3, dtype=np.float32)              # every mode starts with zero velocity
        if k >= n_stop and hand is None and mode != "stopmove":
            since = (k - n_stop) * dt
            if mode.startswith("zv1_"):
                go = since >= 1.0 - 1e-9
            elif mode.startswith("zv2_"):
                go = since >= 2.0 - 1e-9
            else:
                go = since >= ZV_MIN - 1e-9 and len(feet_down(d)) == 2
            if go:
                hand = {"t_after_stop_s": round(since, 3), "feet_on_floor": feet_down(d),
                        "pelvis_speed_mps": round(float(np.hypot(d.qvel[0], d.qvel[1])), 3),
                        "pelvis_z_m": round(float(d.xpos[pel][2]), 3),
                        "tilt_deg": round(math.degrees(math.acos(max(-1.0, min(1.0, float(d.xmat[pel][8]))))), 1),
                        "x": float(d.qpos[0]), "y": float(d.qpos[1])}
                policy_on = False
                if mode.endswith("hold_default"):
                    target = DEF.copy()
                elif mode.endswith("ramp_default"):
                    ramp_from, ramp_k0 = target.copy(), k
                elif mode.endswith("damp"):
                    damp = True
        if ramp_from is not None:
            a = min(1.0, (k - ramp_k0) * dt / RAMP_S)
            target = ((1.0 - a) * ramp_from + a * DEF).astype(np.float32)
        q, dq = d.qpos[7:], d.qvel[6:]
        if damp:
            tau = -KD_DAMP * dq
        else:
            tau = (target - q) * KP + (0 - dq) * KD
        d.ctrl[:] = tau
        mujoco.mj_step(m, d)
        counter = k + 1
        if policy_on and counter % dec == 0:
            qj = (d.qpos[7:] - DEF) * C["dof_pos_scale"]
            dqj = d.qvel[6:] * C["dof_vel_scale"]
            phase = (counter * dt) % 0.8 / 0.8
            obs[:3] = d.qvel[3:6] * C["ang_vel_scale"]
            obs[3:6] = gravity_orientation(d.qpos[3:7])
            obs[6:9] = cmd * np.array(C["cmd_scale"], dtype=np.float32)
            obs[9:9 + NA] = qj
            obs[9 + NA:9 + 2 * NA] = dqj
            obs[9 + 2 * NA:9 + 3 * NA] = action
            obs[9 + 3 * NA:9 + 3 * NA + 2] = [math.sin(2 * math.pi * phase), math.cos(2 * math.pi * phase)]
            action = pol(obs)
            target = action * C["action_scale"] + DEF
        t = counter * dt
        for c in range(d.ncon):
            g1, g2 = d.contact[c].geom1, d.contact[c].geom2
            if floor not in (g1, g2):
                continue
            g = g2 if g1 == floor else g1
            b = body(g)
            if b in FOOT_BODIES:
                continue
            if k < n_stop:
                walk_bad += 1
            elif PART[g] not in first_part:
                mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_GEOM, g, res6, 0)
                hit = (round(t - t_stop, 3), round(float(np.linalg.norm(res6[3:])), 2))
                first_part[PART[g]] = (*hit, b)
                if b not in first:
                    first[b] = (*hit, PART[g])
        if k >= n_stop and counter % 20 == 0:
            after["t"].append(t - t_stop)
            after["pz"].append(float(d.xpos[pel][2]))
            after["tilt"].append(math.degrees(math.acos(max(-1.0, min(1.0, float(d.xmat[pel][8]))))))
            after["vxy"].append(float(np.hypot(d.qvel[0], d.qvel[1])))
            after["px"].append(float(d.qpos[0]))
            after["py"].append(float(d.qpos[1]))
            if keep_qpos:
                qkeep.append(d.qpos.copy())
    f0 = next(iter(first.items()), None)
    tor, head = first_part.get(TORSO), first_part.get(HEAD)
    va, n = np.array(after["vxy"]), 12                       # 12 rows at 25 Hz = 0.48 s under 0.05 m/s
    still = next((i for i in range(len(va) - n + 1) if np.all(va[i:i + n] < 0.05)), None)
    t_arr, tilt_arr = np.array(after["t"]), np.array(after["tilt"])
    post = t_arr >= (hand["t_after_stop_s"] if hand else 0.0) - 1e-9
    if keep_qpos:
        np.save(os.path.join(OUT, f"qpos_{mode}_{t_stop:g}.npy"), np.array(qkeep, dtype=np.float64))
    out = {
        "mode": mode, "t_stop_s": t_stop, "phase": phase_of(t_stop), "feet_on_floor_at_stop": at_stop["feet_on_floor"],
        "walked_m": round(at_stop["x"], 3), "speed_at_stop_mps": round(at_stop["v"], 3),
        "walk_nonfoot_contacts": walk_bad,
        "handoff": None if hand is None else {k: v for k, v in hand.items() if k not in ("x", "y")},
        "first_nonfoot_touch": None if f0 is None else {"shape": f0[1][2], "body": f0[0], "t_after_stop_s": f0[1][0],
                                                        "speed_mps": f0[1][1]},
        "torso_shape_touch": None if tor is None else {"t_after_stop_s": tor[0], "speed_mps": tor[1]},
        "head_shape_touch": None if head is None else {"t_after_stop_s": head[0], "speed_mps": head[1]},
        "end_pelvis_z_m": round(after["pz"][-1], 3), "end_tilt_deg": round(after["tilt"][-1], 1),
        "upright_at_end": bool(after["pz"][-1] > 0.6 and after["tilt"][-1] < 30),
        "max_tilt_after_handoff_deg": round(float(tilt_arr[post].max()), 1) if post.any() else None,
        "travel_after_stop_m": round(float(np.hypot(after["px"][-1] - at_stop["x"], after["py"][-1] - at_stop["y"])), 3),
        "travel_after_handoff_m": None if hand is None else
        round(float(np.hypot(after["px"][-1] - hand["x"], after["py"][-1] - hand["y"])), 3),
        "pelvis_still_after_s": None if still is None else round(after["t"][still], 3),
        "end_speed_mps": round(float(va[-1]), 3),
    }
    return out


def stats(xs):
    xs = sorted(xs)
    if not xs:
        return None
    return {"n": len(xs), "min": xs[0], "median": round(float(np.median(xs)), 3), "max": xs[-1]}


# ------------------------------------------------------------------ the sweep
rows = [run(mode, ts, keep_qpos=(ts == KEEP_QPOS_AT)) for ts in T_STOPS for mode in MODES]

say("")
say(f"## {len(MODES)} stops at {len(T_STOPS)} moments, {T_STOPS[0]}-{T_STOPS[-1]} s (phase 0-0.95 of the policy's "
    f"{GAIT_PERIOD} s clock); {T_AFTER:g} s after each stop; every mode starts with zero velocity")
say("upright at end = pelvis z > 0.6 m and tilt < 30 deg at the last frame; hand-off = when the policy was switched off")
say("t stop | mode | hand-off: s after stop, feet, pelvis m/s | first non-foot touch: shape, s, m/s | torso shape: s | "
    "upright at end | end pelvis z m | max tilt after hand-off deg | still after s | travel after stop m")
for r in rows:
    f, tor, h = r["first_nonfoot_touch"], r["torso_shape_touch"], r["handoff"]
    say(" | ".join(str(v) for v in [
        r["t_stop_s"], r["mode"],
        "-" if h is None else f"{h['t_after_stop_s']}, {'+'.join(h['feet_on_floor']) or 'none'}, {h['pelvis_speed_mps']}",
        "none" if f is None else f"{f['shape']}, {f['t_after_stop_s']}, {f['speed_mps']}",
        "-" if tor is None else tor["t_after_stop_s"],
        r["upright_at_end"], r["end_pelvis_z_m"], r["max_tilt_after_handoff_deg"], r["pelvis_still_after_s"],
        r["travel_after_stop_m"]]))

summary = {}
for mode in MODES:
    rs = [r for r in rows if r["mode"] == mode]
    f = [r["first_nonfoot_touch"] for r in rs if r["first_nonfoot_touch"]]
    hs = [r["handoff"] for r in rs if r["handoff"]]
    shapes = OrderedDict()
    for x in f:
        shapes[x["shape"]] = shapes.get(x["shape"], 0) + 1
    summary[mode] = {
        "stops": len(rs), "upright_at_end": sum(r["upright_at_end"] for r in rs),
        "handoffs": len(hs), "handoff_t_s": stats([h["t_after_stop_s"] for h in hs]),
        "handoff_pelvis_speed_mps": stats([h["pelvis_speed_mps"] for h in hs]),
        "with_nonfoot_touch": len(f), "first_touch_t_s": stats([x["t_after_stop_s"] for x in f]),
        "first_touch_shapes": shapes,
        "torso_touch": sum(r["torso_shape_touch"] is not None for r in rs),
        "end_pelvis_z_m": stats([r["end_pelvis_z_m"] for r in rs]),
        "max_tilt_after_handoff_deg": stats([r["max_tilt_after_handoff_deg"] for r in rs
                                             if r["max_tilt_after_handoff_deg"] is not None]),
        "still": sum(r["pelvis_still_after_s"] is not None for r in rs),
        "travel_after_stop_m": stats([r["travel_after_stop_m"] for r in rs]),
    }

say("")
say("## per mode, over all stop moments")
say("mode | upright at end | hand-offs: n, s after stop min/median/max, pelvis m/s at hand-off max | non-foot touch: "
    "n, s min/median/max | first shapes | torso shape touched | end pelvis z m min-max | max tilt after hand-off deg "
    "max | still (0.48 s under 0.05 m/s) | travel after stop m min-max")
for mode, s in summary.items():
    ht, hv, ft, ez, mt, tr = (s["handoff_t_s"], s["handoff_pelvis_speed_mps"], s["first_touch_t_s"],
                              s["end_pelvis_z_m"], s["max_tilt_after_handoff_deg"], s["travel_after_stop_m"])
    say(" | ".join(str(v) for v in [
        mode, f"{s['upright_at_end']}/{s['stops']}",
        "-" if ht is None else f"{s['handoffs']}, {ht['min']}/{ht['median']}/{ht['max']}, {hv['max']}",
        "0" if ft is None else f"{s['with_nonfoot_touch']}, {ft['min']}/{ft['median']}/{ft['max']}",
        ", ".join(f"{k} x{v}" for k, v in s["first_touch_shapes"].items()) or "-",
        s["torso_touch"], f"{ez['min']}-{ez['max']}", "-" if mt is None else mt["max"], s["still"],
        f"{tr['min']}-{tr['max']}"]))

with open(os.path.join(OUT, "results.json"), "w") as f:
    json.dump({"config": {k: v for k, v in C.items()}, "kd_damp": KD_DAMP, "t_stops_s": T_STOPS,
               "t_after_s": T_AFTER, "zv_min_s": ZV_MIN, "ramp_s": RAMP_S, "gait_period_s": GAIT_PERIOD,
               "summary": summary, "runs": rows}, f, indent=1)
with open(os.path.join(OUT, "stop_handoff.md"), "w") as f:
    f.write("\n".join(lines) + "\n")
