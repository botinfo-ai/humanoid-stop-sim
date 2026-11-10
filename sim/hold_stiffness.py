"""How stiff must a policy-off hold be to keep a G1 standing? (simulation)

stop_handoff.py found that, under Unitree's published G1 walking policy, every stop that switches the policy off and
holds the legs with the deploy config's PD gains (kps 100/100/100/150/40/40, kds 2/2/2/4/2/2 per leg) left the G1 on
the floor, even after 1 or 2 s of zero velocity. In Unitree's own real-robot procedure (deploy_real README L54-60) the
hoist stays on through the default-position state: it is lowered only until the feet touch the ground, and lowered
further only once the policy is stepping.

This script asks why, and what would hold:
  1. stance numbers: total mass, height of the centre of mass above the two ankle-pitch axes, the gravity stiffness
     m*g*h that tips a rigid body about its ankles, against the summed ankle-pitch kp of each gain set.
  2. clean start: the G1 placed with its feet on the floor at the config's default angles, zero velocity, then held
     for 6 s by each gain set with the policy off; plus one control run with the policy on and a zero command.
  3. hand-off: the walking G1 at the same 20 stop moments as stop_sweep.py / stop_handoff.py, zero velocity for
     1.0 s, then the policy off and each gain set pulling the legs to the default angles (stop_handoff.py's
     zv1_hold_default with other gains). Its 'unitree' rows must repeat stop_handoff.py's zv1_hold_default rows.

Gain sets (kd scaled by the same factor as kp, so kd/kp stays the config's):
  unitree            the config's gains
  ankle80 .. ankle320 ankle pitch and roll kp 80/120/160/240/320, other joints as the config
  legs_x2, legs_x4   every leg kp and kd times 2 or 4

Torque caps. g1_12dof.xml gives every leg joint an actuatorfrcrange (hip pitch and yaw 88, hip roll and knee 139,
ankle pitch and roll 50 N*m), and MuJoCo clamps the summed actuator torque on each joint to it (qfrc_actuator). The
actuators' own ctrl range is unlimited, so the torque a hold asks for (written to ctrl) can be larger than the torque
the joint gets. The script prints the caps, the largest requested and applied torque, and the share of steps with a
requested torque above its cap. Every run is done twice: on the model as published ('caps') and on a copy with the
joint caps switched off ('nocap', jnt_actfrclimited = 0), to show what the caps decide. The 'caps' rows must repeat
the 2026-11-09 run of this script (REPRO block at the end).

usage: python3 hold_stiffness.py <inputs> <out>
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

GAIT_PERIOD = 0.8
T_STOPS = [round(4.0 + 0.04 * i, 2) for i in range(20)]
T_AFTER = 6.0         # s simulated after the stop (hand-off runs) or after the start (clean starts)
ZV_S = 1.0            # s of zero velocity before the hand-off
FOOT_BODIES = {"left_ankle_roll_link", "right_ankle_roll_link", "left_ankle_pitch_link", "right_ankle_pitch_link"}
TORSO, HEAD = "torso_link_23dof_rev_1_0", "head_link"
KEEP_QPOS_AT = 4.0

lines = []


def say(s=""):
    print(s)
    lines.append(str(s))


def cfg_list(text, key):
    m_ = re.search(rf"^{key}:\s*\[([^\]]*)\]", text, re.M | re.S)
    return [float(v) for v in m_.group(1).replace("\n", " ").split(",") if v.strip()]


def cfg_num(text, key):
    return float(re.search(rf"^{key}:\s*([-0-9.eE]+)", text, re.M).group(1))


cfg_text = open(CFG).read()
C = {k: cfg_list(cfg_text, k) for k in ("kps", "kds", "default_angles", "cmd_scale", "cmd_init")}
for k in ("simulation_dt", "control_decimation", "ang_vel_scale", "dof_pos_scale", "dof_vel_scale", "action_scale",
          "num_actions", "num_obs"):
    C[k] = cfg_num(cfg_text, k)
NA, NOBS = int(C["num_actions"]), int(C["num_obs"])
say("# How stiff must a policy-off hold be to keep a G1 standing? (simulation)")
say(f"numpy {np.__version__}, mujoco {mujoco.__version__}")
say(f"config g1.yaml: kps {C['kps']} kds {C['kds']} default_angles {C['default_angles']}")


# ------------------------------------------------------------------ TorchScript weights without torch (as stop_walk.py)
class TObj:
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
    raw, dt_ = storage
    item = np.dtype(dt_).itemsize
    base = np.frombuffer(raw, dtype=dt_)
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
    safe = {("collections", "OrderedDict"): OrderedDict, ("builtins", "set"): set,
            ("builtins", "frozenset"): frozenset, ("_codecs", "encode"): lambda s, enc="latin1": s.encode(enc)}

    class U(pickle.Unpickler):
        def find_class(self, mod, name):
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
assert ACTOR and ACTOR[-1][0].shape[0] == NA and (W_IH.shape[1] if RECURRENT else ACTOR[0][0].shape[1]) == NOBS


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


def gravity_orientation(q):
    qw, qx, qy, qz = q
    return np.array([2 * (-qz * qx + qw * qy), -2 * (qz * qy + qw * qx), 1 - 2 * (qw * qw + qz * qz)])


# ------------------------------------------------------------------ model
m = mujoco.MjModel.from_xml_path(SCENE)
m.opt.timestep = C["simulation_dt"]
dt, dec = m.opt.timestep, int(C["control_decimation"])
m_nc = mujoco.MjModel.from_xml_path(SCENE)
m_nc.opt.timestep = C["simulation_dt"]
m_nc.jnt_actfrclimited[:] = 0
MODELS = OrderedDict([("caps", m), ("nocap", m_nc)])
G = abs(float(m.opt.gravity[2]))
floor = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "floor")
body = lambda g: mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, m.geom_bodyid[g])
pel = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "pelvis")


def part_name(g):
    if int(m.geom_type[g]) == int(mujoco.mjtGeom.mjGEOM_MESH):
        return mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MESH, m.geom_dataid[g])
    return mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) or f"{body(g)} geom #{g}"


PART = [part_name(g) for g in range(m.ngeom)]
JID = [int(m.actuator_trnid[a, 0]) for a in range(m.nu)]
JNAMES = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in JID]
DOF = np.array([int(m.jnt_dofadr[j]) for j in JID])
CTRL = np.array([m.actuator_ctrlrange[a, 1] if m.actuator_ctrllimited[a] else np.inf for a in range(m.nu)])
FRC = np.array([m.actuator_forcerange[a, 1] if m.actuator_forcelimited[a] else np.inf for a in range(m.nu)])
CAP = np.array([float(m.jnt_actfrcrange[j, 1]) if m.jnt_actfrclimited[j] else np.inf for j in JID])
ANK_PITCH = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in JNAMES if "ankle_pitch" in n]
ANKLE_IDX = [i for i, n in enumerate(JNAMES) if "ankle" in n]
say(f"model: nq {m.nq}, nu {m.nu}, mass {float(m.body_subtreemass[0]):.2f} kg, gravity {G:g}, timestep {dt}")
say("actuator ctrl range, N*m (unlimited = inf): " + ", ".join(f"{n}: {CTRL[i]:g}" for i, n in enumerate(JNAMES)))
say("actuator force range, N*m (unlimited = inf): " + ", ".join(f"{n}: {FRC[i]:g}" for i, n in enumerate(JNAMES)))
say("joint torque caps (actuatorfrcrange), N*m: " + ", ".join(f"{n}: {CAP[i]:g}" for i, n in enumerate(JNAMES)))
say(f"joint caps on: 'caps' model {[int(m.jnt_actfrclimited[j]) for j in JID]}, "
    f"'nocap' model {[int(m_nc.jnt_actfrclimited[j]) for j in JID]}")
assert len(ANK_PITCH) == 2 and len(ANKLE_IDX) == 4, "ankle joints not found by name"

KP0, KD0 = np.array(C["kps"]), np.array(C["kds"])
DEF = np.array(C["default_angles"], dtype=np.float32)
GAINS = OrderedDict()
GAINS["unitree"] = (KP0.copy(), KD0.copy())
for a in (80, 120, 160, 240, 320):
    kp, kd = KP0.copy(), KD0.copy()
    for i in ANKLE_IDX:
        kd[i] = KD0[i] * a / KP0[i]
        kp[i] = a
    GAINS[f"ankle{a}"] = (kp, kd)
for f_ in (2, 4):
    GAINS[f"legs_x{f_}"] = (KP0 * f_, KD0 * f_)
for name, (kp, kd) in GAINS.items():
    say(f"gains {name}: kp {[round(float(x), 2) for x in kp]} kd {[round(float(x), 2) for x in kd]}")


def lowest_z(d, g):
    t, p, R, s = int(m.geom_type[g]), d.geom_xpos[g], d.geom_xmat[g].reshape(3, 3), m.geom_size[g]
    if t == int(mujoco.mjtGeom.mjGEOM_SPHERE):
        return p[2] - s[0]
    if t in (int(mujoco.mjtGeom.mjGEOM_CAPSULE), int(mujoco.mjtGeom.mjGEOM_CYLINDER)):
        return min(p[2] + R[2, 2] * s[1], p[2] - R[2, 2] * s[1]) - s[0]
    if t == int(mujoco.mjtGeom.mjGEOM_BOX):
        c = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]) * s
        return float((p[2] + c @ R[2, :]).min())
    if t == int(mujoco.mjtGeom.mjGEOM_MESH):
        did = m.geom_dataid[g]
        v = m.mesh_vert[m.mesh_vertadr[did]:m.mesh_vertadr[did] + m.mesh_vertnum[did]]
        return float((p[2] + v @ R[2, :]).min())
    return p[2] - m.geom_rbound[g]


FOOT_GEOMS = [g for g in range(m.ngeom) if body(g) in FOOT_BODIES and (m.geom_contype[g] or m.geom_conaffinity[g])]
assert FOOT_GEOMS, "no colliding foot shapes found"
say(f"colliding foot shapes: {len(FOOT_GEOMS)} (types {sorted({int(m.geom_type[g]) for g in FOOT_GEOMS})})")


def stance(d):
    """Centre of mass against the two ankle-pitch axes, in the pelvis's heading frame."""
    com = d.subtree_com[pel].copy()
    anc = np.mean([d.xanchor[j] for j in ANK_PITCH], axis=0)
    yaw = math.atan2(d.xmat[pel][3], d.xmat[pel][0])
    rel = com - anc
    fwd = math.cos(yaw) * rel[0] + math.sin(yaw) * rel[1]
    side = -math.sin(yaw) * rel[0] + math.cos(yaw) * rel[1]
    mass = float(m.body_subtreemass[pel])
    return {"com_above_ankle_axes_m": round(float(rel[2]), 4), "com_ahead_of_ankle_axes_m": round(fwd, 4),
            "com_left_of_ankle_mid_m": round(side, 4), "mass_kg": round(mass, 3),
            "gravity_stiffness_Nm_per_rad": round(mass * G * float(rel[2]), 1)}


def feet_down(d):
    feet = {body(g) for c in range(d.ncon) for g in (d.contact[c].geom1, d.contact[c].geom2)
            if floor in (d.contact[c].geom1, d.contact[c].geom2) and body(g) in FOOT_BODIES}
    return sorted({"left" if f.startswith("left") else "right" for f in feet})


def run(gname, t_stop=None, clean=False, policy_hold=False, keep=False, model="caps"):
    mm = MODELS[model]
    kp_h, kd_h = GAINS[gname]
    d = mujoco.MjData(mm)
    start = None
    if clean:
        d.qpos[3:7] = [1, 0, 0, 0]
        d.qpos[7:] = DEF
        mujoco.mj_forward(mm, d)
        zmin = min(lowest_z(d, g) for g in FOOT_GEOMS)
        d.qpos[2] += 0.0005 - zmin
        mujoco.mj_forward(mm, d)
        start = {"pelvis_z_m": round(float(d.qpos[2]), 4), "foot_lowest_before_m": round(zmin, 4), **stance(d)}
        n_stop, n_hand = 0, (None if policy_hold else 0)
    else:
        n_stop = int(round(t_stop / dt))
        n_hand = n_stop + int(round(ZV_S / dt))
    n_all = n_stop + int(round(T_AFTER / dt))
    pol = Policy()
    cmd = np.zeros(3, dtype=np.float32) if clean else np.array(C["cmd_init"], dtype=np.float32)
    action = np.zeros(NA, dtype=np.float32)
    target = DEF.copy()
    obs = np.zeros(NOBS, dtype=np.float32)
    policy_on = not clean or policy_hold
    kp, kd = KP0, KD0
    walk_bad, first, first_part = 0, OrderedDict(), OrderedDict()
    res6 = np.zeros(6)
    at_stop, hand = None, None
    over, steps_after, req_max, app_max = 0, 0, np.zeros(NA), np.zeros(NA)
    walk_over, walk_steps, walk_req_max = 0, 0, np.zeros(NA)
    after = {"pz": [], "tilt": [], "vxy": [], "t": [], "px": [], "py": []}
    qkeep = []
    for k in range(n_all):
        if k == n_stop:
            at_stop = {"x": float(d.qpos[0]), "y": float(d.qpos[1]), "feet_on_floor": feet_down(d)}
            cmd = np.zeros(3, dtype=np.float32)
        if n_hand is not None and k == n_hand:
            hand = {"t_after_stop_s": round((k - n_stop) * dt, 3), "feet_on_floor": feet_down(d),
                    "pelvis_speed_mps": round(float(np.hypot(d.qvel[0], d.qvel[1])), 3),
                    "x": float(d.qpos[0]), "y": float(d.qpos[1]), **stance(d)}
            policy_on, target, kp, kd = False, DEF.copy(), kp_h, kd_h
        q, dq = d.qpos[7:], d.qvel[6:]
        tau = (target - q) * kp + (0 - dq) * kd
        a = np.abs(tau)
        if hand is not None:
            steps_after += 1
            over += int(np.any(a > CAP))
            req_max = np.maximum(req_max, a)
        elif k < n_stop:
            walk_steps += 1
            walk_over += int(np.any(a > CAP))
            walk_req_max = np.maximum(walk_req_max, a)
        d.ctrl[:] = tau
        mujoco.mj_step(mm, d)
        if hand is not None:
            app_max = np.maximum(app_max, np.abs(d.qfrc_actuator[DOF]))
        counter = k + 1
        if policy_on and counter % dec == 0:
            obs[:3] = d.qvel[3:6] * C["ang_vel_scale"]
            obs[3:6] = gravity_orientation(d.qpos[3:7])
            obs[6:9] = cmd * np.array(C["cmd_scale"], dtype=np.float32)
            obs[9:9 + NA] = (d.qpos[7:] - DEF) * C["dof_pos_scale"]
            obs[9 + NA:9 + 2 * NA] = d.qvel[6:] * C["dof_vel_scale"]
            obs[9 + 2 * NA:9 + 3 * NA] = action
            phase = (counter * dt) % 0.8 / 0.8
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
                mujoco.mj_objectVelocity(mm, d, mujoco.mjtObj.mjOBJ_GEOM, g, res6, 0)
                hit = (round(t - n_stop * dt, 3), round(float(np.linalg.norm(res6[3:])), 2))
                first_part[PART[g]] = (*hit, b)
                if b not in first:
                    first[b] = (*hit, PART[g])
        if k >= n_stop and counter % 20 == 0:
            after["t"].append(t - n_stop * dt)
            after["pz"].append(float(d.xpos[pel][2]))
            after["tilt"].append(math.degrees(math.acos(max(-1.0, min(1.0, float(d.xmat[pel][8]))))))
            after["vxy"].append(float(np.hypot(d.qvel[0], d.qvel[1])))
            after["px"].append(float(d.qpos[0]))
            after["py"].append(float(d.qpos[1]))
            if keep:
                qkeep.append(d.qpos.copy())
    f0 = next(iter(first.items()), None)
    tor = first_part.get(TORSO)
    va, n = np.array(after["vxy"]), 12
    still = next((i for i in range(len(va) - n + 1) if np.all(va[i:i + n] < 0.05)), None)
    t_arr, tilt_arr = np.array(after["t"]), np.array(after["tilt"])
    post = t_arr >= (hand["t_after_stop_s"] if hand else 0.0) - 1e-9
    if keep:
        tag = "clean" if clean else f"{t_stop:g}"
        pre = "" if model == "caps" else f"{model}_"
        np.save(os.path.join(OUT, f"qpos_{pre}{gname}{'_policy' if policy_hold else ''}_{tag}.npy"),
                np.array(qkeep, dtype=np.float64))
    return {
        "model": model, "gains": gname, "clean_start": clean, "policy_hold": policy_hold, "t_stop_s": t_stop,
        "start": start, "feet_on_floor_at_stop": at_stop["feet_on_floor"], "walk_nonfoot_contacts": walk_bad,
        "handoff": None if hand is None else {k: v for k, v in hand.items() if k not in ("x", "y")},
        "first_nonfoot_touch": None if f0 is None else {"shape": f0[1][2], "body": f0[0], "t_s": f0[1][0],
                                                        "speed_mps": f0[1][1]},
        "torso_shape_touch_s": None if tor is None else tor[0],
        "end_pelvis_z_m": round(after["pz"][-1], 3), "end_tilt_deg": round(after["tilt"][-1], 1),
        "upright_at_end": bool(after["pz"][-1] > 0.6 and after["tilt"][-1] < 30),
        "max_tilt_after_handoff_deg": round(float(tilt_arr[post].max()), 1) if post.any() else None,
        "travel_after_stop_m": round(float(np.hypot(after["px"][-1] - at_stop["x"], after["py"][-1] - at_stop["y"])), 3),
        "pelvis_still_after_s": None if still is None else round(after["t"][still], 3),
        "steps_after_handoff": steps_after,
        "share_steps_requested_over_cap": round(over / steps_after, 4) if steps_after else None,
        "max_abs_requested_torque_after_handoff_Nm": [round(float(x), 1) for x in req_max] if steps_after else None,
        "max_abs_applied_torque_after_handoff_Nm": [round(float(x), 1) for x in app_max] if steps_after else None,
        "share_walk_steps_requested_over_cap": round(walk_over / walk_steps, 4) if walk_steps else None,
        "max_abs_requested_torque_walking_Nm": [round(float(x), 1) for x in walk_req_max] if walk_steps else None,
        "bad_qacc_warnings": int(d.warning[int(mujoco.mjtWarning.mjWARN_BADQACC)].number),
    }


def fmt_touch(r):
    f = r["first_nonfoot_touch"]
    return "none" if f is None else f"{f['shape']}, {f['t_s']}, {f['speed_mps']}"


# ------------------------------------------------------------------ 1 + 2: clean starts
def ankles(v):
    return "-" if v is None else [v[i] for i in ANKLE_IDX]


say("")
say(f"## Clean start: feet on the floor at the default angles, zero velocity, held {T_AFTER:g} s")
clean_rows = []
for mname in MODELS:
    clean_rows.append(run("unitree", clean=True, policy_hold=True, keep=True, model=mname))
    clean_rows += [run(g, clean=True, keep=True, model=mname) for g in GAINS]
s0 = clean_rows[0]["start"]
say(f"start: pelvis z {s0['pelvis_z_m']} m (lowest foot point was {s0['foot_lowest_before_m']} m before placing), "
    f"mass {s0['mass_kg']} kg, centre of mass {s0['com_above_ankle_axes_m']} m above the ankle-pitch axes, "
    f"{s0['com_ahead_of_ankle_axes_m']} m ahead of them, {s0['com_left_of_ankle_mid_m']} m left of their midpoint")
say(f"gravity stiffness m*g*h about the ankle axes: {s0['gravity_stiffness_Nm_per_rad']} N*m/rad")
say("model | hold | summed ankle-pitch kp N*m/rad | ratio to m*g*h | upright at end | first non-foot touch: shape, s, "
    "m/s | torso shape s | end pelvis z m | max tilt deg | still after s | share of steps with a requested torque over "
    "its cap | max |requested torque| at the ankles N*m (L pitch, L roll, R pitch, R roll) | max |applied torque| at "
    "the ankles N*m (same order) | bad-acceleration warnings")
for r in clean_rows:
    kp = GAINS[r["gains"]][0]
    kps = float(sum(kp[i] for i, n in enumerate(JNAMES) if "ankle_pitch" in n))
    label = "policy on, zero command (control)" if r["policy_hold"] else r["gains"]
    say(" | ".join(str(v) for v in [
        r["model"], label, "-" if r["policy_hold"] else kps,
        "-" if r["policy_hold"] else round(kps / s0["gravity_stiffness_Nm_per_rad"], 3),
        r["upright_at_end"], fmt_touch(r), r["torso_shape_touch_s"] or "-", r["end_pelvis_z_m"],
        r["max_tilt_after_handoff_deg"], r["pelvis_still_after_s"], r["share_steps_requested_over_cap"],
        ankles(r["max_abs_requested_torque_after_handoff_Nm"]), ankles(r["max_abs_applied_torque_after_handoff_Nm"]),
        r["bad_qacc_warnings"]]))

# ------------------------------------------------------------------ 3: hand-offs from the walk
say("")
say(f"## Hand-off: walking at cmd_init, stop at {len(T_STOPS)} moments {T_STOPS[0]}-{T_STOPS[-1]} s, zero velocity "
    f"{ZV_S:g} s, then policy off and each gain set pulls to the default angles; {T_AFTER:g} s after the stop")
rows = [run(g, t_stop=ts, keep=(ts == KEEP_QPOS_AT), model=mname) for mname in MODELS for ts in T_STOPS
        for g in GAINS]
say("model | t stop | gains | hand-off: feet, pelvis m/s, com above ankles m, com ahead m | first non-foot touch: "
    "shape, s, m/s | torso shape s | upright at end | end pelvis z m | max tilt after hand-off deg | still after s | "
    "travel m | share of steps with a requested torque over its cap | max |requested| / |applied| ankle torque N*m | "
    "walk: share of steps with a requested torque over its cap")
for r in rows:
    h = r["handoff"]
    say(" | ".join(str(v) for v in [
        r["model"], r["t_stop_s"], r["gains"],
        f"{'+'.join(h['feet_on_floor']) or 'none'}, {h['pelvis_speed_mps']}, {h['com_above_ankle_axes_m']}, "
        f"{h['com_ahead_of_ankle_axes_m']}",
        fmt_touch(r), r["torso_shape_touch_s"] or "-", r["upright_at_end"], r["end_pelvis_z_m"],
        r["max_tilt_after_handoff_deg"], r["pelvis_still_after_s"], r["travel_after_stop_m"],
        r["share_steps_requested_over_cap"],
        f"{max(ankles(r['max_abs_requested_torque_after_handoff_Nm']))} / "
        f"{max(ankles(r['max_abs_applied_torque_after_handoff_Nm']))}",
        r["share_walk_steps_requested_over_cap"]]))


def stats(xs):
    xs = sorted(xs)
    if not xs:
        return None
    return {"n": len(xs), "min": xs[0], "median": round(float(np.median(xs)), 3), "max": xs[-1]}


summary = OrderedDict()
for mname in MODELS:
    for g in GAINS:
        rs = [r for r in rows if r["gains"] == g and r["model"] == mname]
        cr = next(r for r in clean_rows if r["gains"] == g and r["model"] == mname and not r["policy_hold"])
        f = [r["first_nonfoot_touch"] for r in rs if r["first_nonfoot_touch"]]
        summary[f"{mname}/{g}"] = {
            "model": mname, "gains": g, "clean_start_upright": cr["upright_at_end"],
            "clean_start_first_touch_t_s": None if cr["first_nonfoot_touch"] is None else cr["first_nonfoot_touch"]["t_s"],
            "handoffs": len(rs), "upright_at_end": sum(r["upright_at_end"] for r in rs),
            "with_nonfoot_touch": len(f), "first_touch_t_s": stats([x["t_s"] for x in f]),
            "torso_touch": sum(r["torso_shape_touch_s"] is not None for r in rs),
            "end_pelvis_z_m": stats([r["end_pelvis_z_m"] for r in rs]),
            "max_tilt_after_handoff_deg": stats([r["max_tilt_after_handoff_deg"] for r in rs]),
            "still": sum(r["pelvis_still_after_s"] is not None for r in rs),
            "share_requested_over_cap": stats([r["share_steps_requested_over_cap"] for r in rs]),
            "max_requested_ankle_Nm": max(max(ankles(r["max_abs_requested_torque_after_handoff_Nm"])) for r in rs),
            "max_applied_ankle_Nm": max(max(ankles(r["max_abs_applied_torque_after_handoff_Nm"])) for r in rs),
            "walk_share_requested_over_cap": stats([r["share_walk_steps_requested_over_cap"] for r in rs]),
            "walk_max_requested_ankle_Nm": max(max(ankles(r["max_abs_requested_torque_walking_Nm"])) for r in rs),
            "bad_qacc_warnings": sum(r["bad_qacc_warnings"] for r in rs),
            "handoff_com_above_ankles_m": stats([r["handoff"]["com_above_ankle_axes_m"] for r in rs]),
            "handoff_com_ahead_m": stats([r["handoff"]["com_ahead_of_ankle_axes_m"] for r in rs]),
        }
say("")
say("## per model and gain set")
say("model | gains | clean start upright | clean start first touch s | hand-offs upright | non-foot touch: n, s "
    "min/median/max | torso touched | end pelvis z m min-max | max tilt deg max | still | share of steps with a "
    "requested torque over its cap, max | ankle torque after hand-off, max requested / max applied N*m | walking: "
    "share of steps with a requested torque over its cap, max; max requested ankle torque N*m | bad-acceleration "
    "warnings | com above ankles at hand-off m min-max")
for key, s in summary.items():
    ft, ez, mt, so, ch, wo = (s["first_touch_t_s"], s["end_pelvis_z_m"], s["max_tilt_after_handoff_deg"],
                              s["share_requested_over_cap"], s["handoff_com_above_ankles_m"],
                              s["walk_share_requested_over_cap"])
    say(" | ".join(str(v) for v in [
        s["model"], s["gains"], s["clean_start_upright"], s["clean_start_first_touch_t_s"] or "none",
        f"{s['upright_at_end']}/{s['handoffs']}",
        "0" if ft is None else f"{s['with_nonfoot_touch']}, {ft['min']}/{ft['median']}/{ft['max']}",
        s["torso_touch"], f"{ez['min']}-{ez['max']}", mt["max"], s["still"], so["max"],
        f"{s['max_requested_ankle_Nm']} / {s['max_applied_ankle_Nm']}",
        f"{wo['max']}; {s['walk_max_requested_ankle_Nm']}", s["bad_qacc_warnings"], f"{ch['min']}-{ch['max']}"]))

# ------------------------------------------------------------------ REPRO: the 'caps' rows must repeat the 2026-11-09 run
EXPECT_CLEAN_TOUCH = {"unitree": 1.252, "ankle80": 1.426, "ankle120": 1.586, "ankle160": 1.754, "ankle240": 2.192,
                      "ankle320": None, "legs_x2": 1.392, "legs_x4": 1.714}
EXPECT_HANDOFF_TOUCH = {"unitree": (2.248, 2.314, 2.362), "ankle80": (2.414, 2.478, 2.516),
                        "ankle120": (2.558, 2.618, 2.648), "ankle160": (2.696, 2.757, 2.798),
                        "ankle240": (2.998, 3.043, 3.172), "ankle320": (3.276, 3.566, 4.842),
                        "legs_x2": (2.388, 2.417, 2.436), "legs_x4": (2.574, 2.62, 2.686)}
say("")
say("## REPRO: 'caps' rows against the 2026-11-09 run (clean-start first touch s; hand-off first touch min/median/max "
    "s; hand-offs upright)")
ok_all = True
for g in GAINS:
    s = summary[f"caps/{g}"]
    ft = s["first_touch_t_s"]
    got = (s["clean_start_first_touch_t_s"], None if ft is None else (ft["min"], ft["median"], ft["max"]),
           s["upright_at_end"])
    want = (EXPECT_CLEAN_TOUCH[g], EXPECT_HANDOFF_TOUCH[g], 0)
    ok = got == want
    ok_all = ok_all and ok
    say(f"{g}: got {got} want {want} match {ok}")
say(f"REPRO caps rows = 2026-11-09 run: {ok_all}")

with open(os.path.join(OUT, "results.json"), "w") as fh:
    json.dump({"config": C, "t_stops_s": T_STOPS, "t_after_s": T_AFTER, "zv_s": ZV_S,
               "gains": {k: {"kp": [float(x) for x in v[0]], "kd": [float(x) for x in v[1]]} for k, v in GAINS.items()},
               "actuators": {n: {"ctrl_range_Nm": float(CTRL[i]), "force_range_Nm": float(FRC[i]),
                                 "joint_cap_Nm": float(CAP[i])} for i, n in enumerate(JNAMES)},
               "repro_caps_rows_match_2026_11_09": ok_all,
               "clean_starts": clean_rows, "summary": summary, "runs": rows}, fh, indent=1)
with open(os.path.join(OUT, "hold_stiffness.md"), "w") as fh:
    fh.write("\n".join(lines) + "\n")
