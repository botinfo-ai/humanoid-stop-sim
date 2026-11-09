"""The walking-G1 stop table, swept over one whole gait cycle (simulation).

stop_walk.py (humanoid-stop-sim) stops Unitree's published G1 walking policy at one moment, 4.0 s into the walk. This
script applies the same five stops at 20 moments spread over one gait cycle (the policy's phase clock is 0.8 s long),
from 4.00 s to 4.76 s in 0.04 s steps, so a result cannot hang on the one moment chosen:

  stopmove      the velocity command goes to zero and the policy keeps running
  hold_last     the policy stops; every leg joint holds its last target with the config's PD gains
  hold_default  the policy stops; the PD gains pull every leg joint to the config's default angles
  damp_kd8      kp 0, kd 8 on every joint (unitree_rl_gym's create_damping_cmd), held for T_AFTER s
  zero_torque   no torque at all (create_zero_cmd: kp 0, kd 0)

The loader, the network, the deploy loop and the contact reporting are stop_walk.py's, unchanged; only the stop moment
varies, plus a control run that walks to the last moment + T_AFTER with no stop. damp_kd8 and zero_torque reproduce
unitree_rl_gym's low-level commands, not the SDK's Damp() / ZeroTorque() FSM modes, whose behaviour on the robot is
not in public code. In g1_12dof.xml the torso, head, logo, shoulders, elbows and hands are shapes of the pelvis body,
fixed in one arm pose, so every floor contact is named by the shape that touched.

usage: python3 stop_sweep.py <inputs> <out>
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
T_AFTER = 4.0         # s simulated after the stop
MODES = ["stopmove", "hold_last", "hold_default", "damp_kd8", "zero_torque"]
FOOT_BODIES = {"left_ankle_roll_link", "right_ankle_roll_link", "left_ankle_pitch_link", "right_ankle_pitch_link"}
TORSO, HEAD = "torso_link_23dof_rev_1_0", "head_link"

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
say("# A walking G1 under Unitree's published policy: five stops at 20 moments over one gait cycle (simulation)")
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
    if len(size) == 0:
        return np.array(base[offset])
    arr = np.lib.stride_tricks.as_strided(base[offset:], shape=tuple(size), strides=tuple(s * item for s in stride))
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


def run(mode, t_stop):
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
    at_stop, after = None, {"pz": [], "tilt": [], "vxy": [], "t": [], "px": [], "py": []}
    for k in range(n_all):
        if k == n_stop:
            feet = sorted({body(g) for c in range(d.ncon) for g in (d.contact[c].geom1, d.contact[c].geom2)
                           if floor in (d.contact[c].geom1, d.contact[c].geom2) and body(g) in FOOT_BODIES})
            at_stop = {"x": float(d.qpos[0]), "y": float(d.qpos[1]), "v": float(np.hypot(d.qvel[0], d.qvel[1])),
                       "feet_on_floor": sorted({"left" if f.startswith("left") else "right" for f in feet})}
            if mode == "stopmove":
                cmd = np.zeros(3, dtype=np.float32)
            elif mode != "continue":
                policy_on = False
                if mode == "hold_default":
                    target = DEF.copy()
                damp = mode in ("damp_kd8", "zero_torque")
        q, dq = d.qpos[7:], d.qvel[6:]
        if damp:
            tau = -(KD_DAMP if mode != "zero_torque" else 0.0) * dq
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
    f0 = next(iter(first.items()), None)
    tor, head, p = first_part.get(TORSO), first_part.get(HEAD), first.get("pelvis")
    va, n = np.array(after["vxy"]), 12                       # 12 rows at 25 Hz = 0.48 s under 0.05 m/s
    still = next((i for i in range(len(va) - n + 1) if np.all(va[i:i + n] < 0.05)), None)
    return {
        "mode": mode, "t_stop_s": t_stop, "phase": phase_of(t_stop), "feet_on_floor_at_stop": at_stop["feet_on_floor"],
        "walked_m": round(at_stop["x"], 3), "speed_at_stop_mps": round(at_stop["v"], 3),
        "walk_nonfoot_contacts": walk_bad,
        "first_nonfoot_touch": None if f0 is None else {"shape": f0[1][2], "body": f0[0], "t_after_stop_s": f0[1][0],
                                                        "speed_mps": f0[1][1]},
        "pelvis_body_touch": None if p is None else {"shape": p[2], "t_after_stop_s": p[0], "speed_mps": p[1]},
        "torso_shape_touch": None if tor is None else {"t_after_stop_s": tor[0], "speed_mps": tor[1]},
        "head_shape_touch": None if head is None else {"t_after_stop_s": head[0], "speed_mps": head[1]},
        "end_pelvis_z_m": round(after["pz"][-1], 3), "end_tilt_deg": round(after["tilt"][-1], 1),
        "upright_at_end": bool(after["pz"][-1] > 0.6 and after["tilt"][-1] < 30),
        "min_pelvis_z_after_m": round(min(after["pz"]), 3),
        "travel_after_stop_m": round(float(np.hypot(after["px"][-1] - at_stop["x"], after["py"][-1] - at_stop["y"])), 3),
        "pelvis_still_after_s": None if still is None else round(after["t"][still], 3),
    }


def stats(xs):
    xs = sorted(xs)
    if not xs:
        return None
    return {"n": len(xs), "min": xs[0], "median": round(float(np.median(xs)), 3), "max": xs[-1]}


# ------------------------------------------------------------------ the sweep
control = run("continue", T_STOPS[-1])
say("")
say(f"control: walking with no stop to {T_STOPS[-1] + T_AFTER:g} s: non-foot floor contacts "
    f"{control['walk_nonfoot_contacts'] + (0 if control['first_nonfoot_touch'] is None else 1)}, upright at end "
    f"{control['upright_at_end']}, end tilt {control['end_tilt_deg']} deg")

rows = [run(mode, ts) for ts in T_STOPS for mode in MODES]

say("")
say(f"## stops at {len(T_STOPS)} moments, {T_STOPS[0]}-{T_STOPS[-1]} s (phase 0-0.95 of the policy's "
    f"{GAIT_PERIOD} s clock); {T_AFTER:g} s after each stop")
say("upright at end = pelvis z > 0.6 m and tilt < 30 deg at the last frame; speeds are the touching shape's centre")
say("t stop | phase | feet down at stop | mode | first non-foot touch: shape, s, m/s | torso shape: s, m/s | "
    "head shape: s, m/s | upright at end | end pelvis z m | travel m")
for r in rows:
    f, tor, hd = r["first_nonfoot_touch"], r["torso_shape_touch"], r["head_shape_touch"]
    say(" | ".join(str(v) for v in [
        r["t_stop_s"], r["phase"], "+".join(r["feet_on_floor_at_stop"]) or "none", r["mode"],
        "none" if f is None else f"{f['shape']}, {f['t_after_stop_s']}, {f['speed_mps']}",
        "-" if tor is None else f"{tor['t_after_stop_s']}, {tor['speed_mps']}",
        "-" if hd is None else f"{hd['t_after_stop_s']}, {hd['speed_mps']}",
        r["upright_at_end"], r["end_pelvis_z_m"], r["travel_after_stop_m"]]))

summary = {}
for mode in MODES:
    rs = [r for r in rows if r["mode"] == mode]
    f = [r["first_nonfoot_touch"] for r in rs if r["first_nonfoot_touch"]]
    tor = [r["torso_shape_touch"] for r in rs if r["torso_shape_touch"]]
    hd = [r["head_shape_touch"] for r in rs if r["head_shape_touch"]]
    shapes = OrderedDict()
    for x in f:
        shapes[x["shape"]] = shapes.get(x["shape"], 0) + 1
    summary[mode] = {
        "stops": len(rs), "upright_at_end": sum(r["upright_at_end"] for r in rs),
        "with_nonfoot_touch": len(f), "nonfoot_touch_within_1_5_s": sum(x["t_after_stop_s"] <= 1.5 for x in f),
        "first_touch_t_s": stats([x["t_after_stop_s"] for x in f]),
        "first_touch_speed_mps": stats([x["speed_mps"] for x in f]),
        "first_touch_shapes": shapes,
        "torso_touch": len(tor), "torso_touch_t_s": stats([x["t_after_stop_s"] for x in tor]),
        "torso_touch_speed_mps": stats([x["speed_mps"] for x in tor]),
        "head_touch": len(hd), "head_touch_speed_mps": stats([x["speed_mps"] for x in hd]),
        "travel_after_stop_m": stats([r["travel_after_stop_m"] for r in rs]),
    }

say("")
say("## per mode, over all stop moments")
say("mode | upright at end | non-foot touch (within 1.5 s) | first touch s: min / median / max | first touch m/s: "
    "min / median / max | first shapes | torso shape touched: n, s min-max, m/s max | head shape touched: n, m/s max")
for mode, s in summary.items():
    ft, fs, tt, ts, hs = (s["first_touch_t_s"], s["first_touch_speed_mps"], s["torso_touch_t_s"],
                          s["torso_touch_speed_mps"], s["head_touch_speed_mps"])
    say(" | ".join(str(v) for v in [
        mode, f"{s['upright_at_end']}/{s['stops']}", f"{s['with_nonfoot_touch']} ({s['nonfoot_touch_within_1_5_s']})",
        "-" if ft is None else f"{ft['min']} / {ft['median']} / {ft['max']}",
        "-" if fs is None else f"{fs['min']} / {fs['median']} / {fs['max']}",
        ", ".join(f"{k} x{v}" for k, v in s["first_touch_shapes"].items()) or "-",
        "0" if tt is None else f"{s['torso_touch']}, {tt['min']}-{tt['max']}, {ts['max']}",
        "0" if hs is None else f"{s['head_touch']}, {hs['max']}"]))

with open(os.path.join(OUT, "results.json"), "w") as f:
    json.dump({"config": {k: v for k, v in C.items()}, "kd_damp": KD_DAMP, "t_stops_s": T_STOPS,
               "t_after_s": T_AFTER, "gait_period_s": GAIT_PERIOD, "control": control, "summary": summary,
               "runs": rows}, f, indent=1)
with open(os.path.join(OUT, "stop_sweep.md"), "w") as f:
    f.write("\n".join(lines) + "\n")
