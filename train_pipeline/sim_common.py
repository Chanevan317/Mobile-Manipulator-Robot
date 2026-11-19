"""Isaac-side helpers shared by record/record_sim.py, tools/replay_episode.py and eval/eval_sim.py (import after the scene
is loaded: they need the running Isaac app).

    import sim_common as SCM
    SCM.bind(task, items, PP)          # once, after scene_robot.load() and pick_place.init()
    SCM.state()                        # measured 7-vector (common.JOINTS order)
    SCM.sent()                         # the 7-vector command the drives got last step
    SCM.command(v)                     # 7-vector -> joint targets (gripper opening split over the two fingers)
    SCM.set_light(1.2); SCM.jitter_product(name, dx, dy, dyaw); SCM.teleport(name, poses)
"""
import math

import numpy as np
import omni.usd
from omni.physx import get_physx_simulation_interface
from pxr import PhysicsSchemaTools, UsdLux

import common as C

task = items = PP = None
IDX = []
LIGHTS = []


def bind(task_, items_, pp):
    global task, items, PP, IDX, LIGHTS
    task, items, PP = task_, items_, pp
    IDX = [[task.idx[j] for j in (u if isinstance(u, tuple) else (u,))] for _, u in C.JOINTS]
    LIGHTS = []
    for prim in omni.usd.get_context().get_stage().Traverse():
        if prim.HasAPI(UsdLux.LightAPI) and prim.IsActive():
            a = UsdLux.LightAPI(prim).GetIntensityAttr()
            if a and a.Get() is not None:
                LIGHTS.append((a, float(a.Get())))


def _vec(q):
    return np.array([float(sum(q[i] for i in ii)) for ii in IDX], np.float32)


def state():
    return _vec(task.robot.get_joint_positions())


def sent():
    return _vec(task.q_sent)


def command(v):
    for ii, x in zip(IDX, v):
        for i in ii:
            task.q_goal[i] = float(x) / len(ii)


def set_light(scale):
    for a, v0 in LIGHTS:
        a.Set(v0 * scale)


def body_poses(name):
    return [(np.array(p, float), np.array(q, float)) for p, q in (b.get_world_pose() for b in PP.MOVABLE[name])]


def _settle_body(b):
    sim = get_physx_simulation_interface(); sid_ = omni.usd.get_context().get_stage_id()
    bid = PhysicsSchemaTools.sdfPathToInt(b.prim_path)
    sim.put_to_sleep(sid_, bid); sim.wake_up(sid_, bid)              # sleeping zeroes the velocities


def teleport(name, poses):
    """poses: rows of (x, y, z, qw, qx, qy, qz), one per body (as stored in the episode files)."""
    for b, pq in zip(PP.MOVABLE[name], poses):
        b.set_world_pose(position=np.asarray(pq[:3], float), orientation=np.asarray(pq[3:], float))
        _settle_body(b)


def jitter_product(name, dx, dy, dyaw):
    """Move the product's bodies by (dx, dy) and turn them dyaw about the vertical through its resting centre."""
    _, cx, cy, _ = items[name]['rest']
    c, s = math.cos(dyaw), math.sin(dyaw)
    w1, x1, y1, z1 = math.cos(dyaw / 2), 0.0, 0.0, math.sin(dyaw / 2)
    for b, (p, q) in zip(PP.MOVABLE[name], body_poses(name)):
        rx, ry = p[0] - cx, p[1] - cy
        p2 = np.array([cx + c * rx - s * ry + dx, cy + s * rx + c * ry + dy, p[2]])
        w2, x2, y2, z2 = q
        q2 = np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                       w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])
        b.set_world_pose(position=p2, orientation=q2)
        _settle_body(b)


def in_tote(name):
    """(inside, base-frame position) of the product's first body, with the recorder's tote test."""
    import mono_poses as MP
    pb = task.to_base(np.array(PP.MOVABLE[name][0].get_world_pose()[0], float))
    x0, x1, y0, y1 = MP.TOTE_BOX
    return bool(x0 < pb[0] < x1 and y0 < pb[1] < y1 and pb[2] < 0.376), pb
