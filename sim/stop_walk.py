"""What each kind of stop does to a WALKING G1, under Unitree's own published walking policy (simulation).

Unitree's unitree_rl_gym ships a trained G1 walking policy (deploy/pre_train/g1/motion.pt), a 12-DoF MuJoCo model of
the G1 (resources/robots/g1_description/scene.xml -> g1_12dof.xml) and a MuJoCo runner (deploy/deploy_mujoco/
deploy_mujoco.py with configs/g1.yaml). This script runs that loop step for step, without torch: it reads the
TorchScript file's weights with zipfile + pickle and evaluates the network in numpy. The robot walks forward at the
config's cmd_init (0.5 m/s) for T_STOP seconds, then one stop is applied:

  continue          nothing changes (control run)
  stopmove          the velocity command goes to zero and the policy keeps running (what a zero-velocity stop asks of
                    a walking controller)
  hold_last         the policy stops; every leg joint holds its last target with the config's PD gains
  hold_default      the policy stops; the PD gains pull every leg joint to the config's default angles
  damp_kd8          kp 0, kd 8 on every joint: create_damping_cmd in Unitree's deploy_real/common/command_helper.py,
                    which deploy_real.py sends when the operator exits
  zero_torque       no torque at all (create_zero_cmd: kp 0, kd 0)
  stopmove_then_damp zero velocity first, damping SETTLE_S later

damp_kd8 and zero_torque reproduce unitree_rl_gym's low-level commands, held for T_AFTER seconds. They are not the
SDK's Damp() / ZeroTorque() FSM modes, whose behaviour on the robot is not in public code. deploy_real.py sends the
damping command once and exits; what the motors do after that last message is not in public code either.

In g1_12dof.xml the torso, head, logo, shoulders, elbows and hands are shapes of the pelvis body, and several of them
collide. So every floor contact is reported by the shape that touched (its mesh, or its body and geom type), with
that shape's speed at its centre.

usage: python3 stop_walk.py <inputs> <out>
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

T_STOP = 4.0          # s of walking before the stop
T_AFTER = 4.0         # s simulated after the stop
SETTLE_S = 2.0        # stopmove_then_damp: damping this long after the zero-velocity command
FPS = 25              # 1 / (25 * 0.002 s) = one recorded row every 20 physics steps, exactly
MODES = ["continue", "stopmove", "hold_last", "hold_default", "damp_kd8", "zero_torque", "stopmove_then_damp"]
FOOT_BODIES = {"left_ankle_roll_link", "right_ankle_roll_link", "left_ankle_pitch_link", "right_ankle_pitch_link"}

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
say("# A walking G1 under Unitree's published policy: what each stop does (simulation)")
say(f"numpy {np.__version__}, mujoco {mujoco.__version__}")
say(f"config g1.yaml: kps {C['kps']} kds {C['kds']}")
say(f"default_angles {C['default_angles']} action_scale {C['action_scale']} cmd_init {C['cmd_init']} "
    f"cmd_scale {C['cmd_scale']} dt {C['simulation_dt']} decimation {int(C['control_decimation'])} obs {NOBS}")
say(f"create_damping_cmd kd (command_helper.py): {KD_DAMP:g}")


# ------------------------------------------------------------------ TorchScript weights without torch
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
    if isinstance(o, np.ndarray):                           # a tensor can sit at two paths (_flat_weights): keep both
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
say("tensors: " + "; ".join(f"{k} {list(v.shape)}" for k, v in sorted(tensors.items())))


def pick(suffix):
    hits = [k for k in tensors if k.endswith(suffix) and "[" not in k]
    return tensors[hits[0]] if hits else None


W_IH, W_HH, B_IH, B_HH = (pick(s) for s in ("weight_ih_l0", "weight_hh_l0", "bias_ih_l0", "bias_hh_l0"))
lin = [(k, v) for k, v in tensors.items() if re.search(r"(^|\.)actor\.\d+\.weight$", k)] or \
      [(k, v) for k, v in tensors.items() if re.fullmatch(r"\d+\.weight", k)]      # a plain exported Sequential
lin.sort(key=lambda kv: int(kv[0].split(".")[-2]))
ACTOR = [(w.astype(np.float64), tensors[k[:-len("weight")] + "bias"].astype(np.float64)) for k, w in lin]
RECURRENT = W_IH is not None
H0, C0 = pick("hidden_state"), pick("cell_state")             # the exporter's buffers: the state torch.jit.load starts from
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
            i, f, gg, o = np.split(g, 4)                       # PyTorch gate order: input, forget, cell, output
            self.c = sig(f) * self.c + sig(i) * np.tanh(gg)
            self.h = sig(o) * np.tanh(self.c)
            x = self.h
        for j, (w, b) in enumerate(ACTOR):
            x = w @ x + b
            if j < len(ACTOR) - 1:
                x = np.where(x > 0, x, np.expm1(np.minimum(x, 0)))   # ELU, alpha 1
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
jnames = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in range(1, m.njnt)]
mass = float(m.body_subtreemass[0])
GEOM_TYPES = {int(getattr(mujoco.mjtGeom, n)): n[7:].lower() for n in
              ("mjGEOM_PLANE", "mjGEOM_SPHERE", "mjGEOM_CAPSULE", "mjGEOM_ELLIPSOID", "mjGEOM_CYLINDER", "mjGEOM_BOX",
               "mjGEOM_MESH")}


def part_name(g):
    """The shape that touched: its mesh name, else its geom name, else its body and geom type."""
    if int(m.geom_type[g]) == int(mujoco.mjtGeom.mjGEOM_MESH):
        return mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MESH, m.geom_dataid[g])
    return (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g)
            or f"{body(g)} {GEOM_TYPES.get(int(m.geom_type[g]), 'geom')} #{g}")


PART = [part_name(g) for g in range(m.ngeom)]
collides = [g for g in range(m.ngeom) if m.geom_contype[g] or m.geom_conaffinity[g]]
colliders = sorted({body(g) for g in collides} - {"world"})
say(f"model: nq {m.nq}, nu {m.nu}, mass {mass:.2f} kg, timestep {dt}, joints {jnames}")
say(f"bodies with colliding geoms: {colliders}")
say(f"colliding shapes on the pelvis body: {sorted(PART[g] for g in collides if body(g) == 'pelvis')}")
frc = [list(np.round(m.jnt_actfrcrange[j], 1)) for j in range(1, m.njnt)]
say(f"joint actuatorfrcrange: {frc}")

KP, KD = np.array(C["kps"]), np.array(C["kds"])
DEF = np.array(C["default_angles"], dtype=np.float32)
N_STOP, N_ALL = int(round(T_STOP / dt)), int(round((T_STOP + T_AFTER) / dt))
REC_EVERY = int(round(1 / (FPS * dt)))


def run(mode):
    d = mujoco.MjData(m)
    pol = Policy()
    cmd = np.array(C["cmd_init"], dtype=np.float32)
    action = np.zeros(NA, dtype=np.float32)
    target = DEF.copy()
    obs = np.zeros(NOBS, dtype=np.float32)
    policy_on, damp = True, False
    rec = {"t": [], "qpos": [], "pz": [], "tilt": [], "px": [], "py": [], "vxy": []}
    walk_bad = []                       # non-foot floor contacts while walking (should be none)
    first = OrderedDict()               # body -> (t after stop, speed m/s, shape) of its first floor contact after it
    first_part = OrderedDict()          # shape -> (t after stop, speed m/s, body), for every shape that touched
    res6 = np.zeros(6)
    at_stop = None
    for k in range(N_ALL):
        if k == N_STOP:
            at_stop = {"x": float(d.qpos[0]), "y": float(d.qpos[1]),
                       "v": float(np.hypot(d.qvel[0], d.qvel[1]))}
            if mode in ("stopmove", "stopmove_then_damp"):
                cmd = np.zeros(3, dtype=np.float32)
            elif mode in ("hold_last", "hold_default", "damp_kd8", "zero_torque"):
                policy_on = False
                if mode == "hold_default":
                    target = DEF.copy()
                damp = mode in ("damp_kd8", "zero_torque")
        if mode == "stopmove_then_damp" and k == N_STOP + int(round(SETTLE_S / dt)):
            policy_on, damp = False, True
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
            if k < N_STOP:
                walk_bad.append((round(t, 3), b, PART[g]))
            elif PART[g] not in first_part:
                mujoco.mj_objectVelocity(m, d, mujoco.mjtObj.mjOBJ_GEOM, g, res6, 0)   # the shape's centre
                hit = (round(t - T_STOP, 3), round(float(np.linalg.norm(res6[3:])), 2))
                first_part[PART[g]] = (*hit, b)
                if b not in first:
                    first[b] = (*hit, PART[g])
        if counter % REC_EVERY == 0:
            rec["t"].append(t)
            rec["qpos"].append(d.qpos.copy())
            rec["pz"].append(float(d.xpos[pel][2]))
            rec["tilt"].append(math.degrees(math.acos(max(-1.0, min(1.0, float(d.xmat[pel][8]))))))
            rec["px"].append(float(d.qpos[0]))
            rec["py"].append(float(d.qpos[1]))
            rec["vxy"].append(float(np.hypot(d.qvel[0], d.qvel[1])))
    for key in rec:
        rec[key] = np.array(rec[key])
    return rec, first, first_part, walk_bad, at_stop


def summarize(mode, rec, first, first_part, walk_bad, at_stop):
    t = rec["t"]
    before, after = t < T_STOP, t >= T_STOP
    out = {"mode": mode,
           "walk_x_at_stop_m": round(at_stop["x"], 3), "walk_speed_at_stop_mps": round(at_stop["v"], 3),
           "walk_min_pelvis_z_m": round(float(rec["pz"][before].min()), 3),
           "walk_max_tilt_deg": round(float(rec["tilt"][before].max()), 1),
           "walk_nonfoot_floor_contacts": len(walk_bad), "walk_nonfoot_first": walk_bad[:3]}
    first_any = next(iter(first.items()), None)
    out["first_nonfoot_touch"] = None if first_any is None else {
        "body": first_any[0], "shape": first_any[1][2], "t_after_stop_s": first_any[1][0],
        "speed_mps": first_any[1][1]}
    p = first.get("pelvis")                 # the pelvis body: pelvis, torso, head, logo, shoulders, elbows, hands
    out["pelvis_body_touch"] = None if p is None else {"shape": p[2], "t_after_stop_s": p[0], "speed_mps": p[1]}
    out["bodies_on_floor"] = list(first)
    out["shapes_on_floor"] = [{"shape": s, "body": v[2], "t_after_stop_s": v[0], "speed_mps": v[1]}
                              for s, v in first_part.items()]
    pz, tilt = rec["pz"][after], rec["tilt"][after]
    out["min_pelvis_z_after_m"] = round(float(pz.min()), 3)
    out["max_tilt_after_deg"] = round(float(tilt.max()), 1)
    out["end_pelvis_z_m"] = round(float(pz[-1]), 3)
    out["end_tilt_deg"] = round(float(tilt[-1]), 1)
    out["upright_at_end"] = bool(pz[-1] > 0.6 and tilt[-1] < 30)
    travel = np.hypot(rec["px"][after] - at_stop["x"], rec["py"][after] - at_stop["y"])
    out["travel_after_stop_m"] = round(float(travel[-1]), 3)
    va, n = rec["vxy"][after], int(0.5 * FPS)
    still = next((i for i in range(len(va) - n + 1) if np.all(va[i:i + n] < 0.05)), None)   # under 0.05 m/s for 0.5 s
    out["pelvis_still_after_s"] = None if still is None else round(float(t[after][still] - T_STOP), 3)
    out["travel_to_still_m"] = None if still is None else round(float(travel[still]), 3)
    return out


results = []
for mode in MODES:
    rec, first, first_part, walk_bad, at_stop = run(mode)
    results.append(summarize(mode, rec, first, first_part, walk_bad, at_stop))

say("")
say(f"## {T_STOP:g} s of walking at cmd {C['cmd_init']}, then the stop; {T_AFTER:g} s after it")
say("upright at end = pelvis z > 0.6 m and tilt < 30 deg at the last frame; speeds are the touching shape's centre")
say("mode | walked m | speed at stop m/s | walk: non-foot contacts, max tilt | first non-foot touch after stop: "
    "shape (body), s, m/s | first pelvis-body touch: shape, s, m/s | min pelvis z m | end pelvis z m | "
    "end tilt deg | upright at end | travel after stop m | still after s")
for s in results:
    f, p = s["first_nonfoot_touch"], s["pelvis_body_touch"]
    say(" | ".join(str(v) for v in [
        s["mode"], s["walk_x_at_stop_m"], s["walk_speed_at_stop_mps"],
        f"{s['walk_nonfoot_floor_contacts']}, {s['walk_max_tilt_deg']}",
        "none" if f is None else f"{f['shape']} ({f['body']}), {f['t_after_stop_s']}, {f['speed_mps']}",
        "-" if p is None else f"{p['shape']}, {p['t_after_stop_s']}, {p['speed_mps']}",
        s["min_pelvis_z_after_m"], s["end_pelvis_z_m"], s["end_tilt_deg"], s["upright_at_end"],
        s["travel_after_stop_m"], s["pelvis_still_after_s"]]))
say("")
say("every shape that touched the floor after the stop, in order: shape (body) s, m/s")
for s in results:
    say(f"{s['mode']}: " + ("none" if not s["shapes_on_floor"] else "; ".join(
        f"{x['shape']} ({x['body']}) {x['t_after_stop_s']}, {x['speed_mps']}" for x in s["shapes_on_floor"])))

with open(os.path.join(OUT, "results.json"), "w") as f:
    json.dump({"config": {k: v for k, v in C.items()}, "kd_damp": KD_DAMP, "t_stop_s": T_STOP,
               "t_after_s": T_AFTER, "settle_s": SETTLE_S, "results": results}, f, indent=1)

with open(os.path.join(OUT, "stop_walk.md"), "w") as f:
    f.write("\n".join(lines) + "\n")
