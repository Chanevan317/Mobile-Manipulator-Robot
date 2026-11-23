"""The mono robot in scene_v0, driven between the fixed stations (stations.json). Shared by drive_stations_mono.py and
random_drive_mono.py.

ARM TRAINING setup (2026-11-17, user: "we focus on arm training fidelity. navigation will be handled separately ... the
robot can move from a to b easily by script"): by default the robot is urdf_variants.planar_base (no wheels / rollers,
the base on driven base_x / base_y / base_yaw joints), with the settings in SIM below (picked from a timing benchmark).
wheels=True loads the full mecanum robot instead (roller simulation, the old drive_to with wheel speeds).

    app = start_app(headless)                   # before any omni / isaacsim import
    task, items, watch = load(app, headless)    # robot at 'home' (the middle of the room), arm folded in STOW, + the store scene
    leg(task, stations, 'home', 'rack_b0_box_cracker_a01')
"""
import math, os, re, subprocess, sys
here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(here, '..', '..', '..', 'common')); import repo_paths as RP   # every script folder + repo paths
import stations_mono as ST                    # no simulator needed

CAMERA = dict(eye=(-4.6, -0.4, 2.3), target=(-1.0, 0.0, 0.5))     # from the open side of the room
SIM = dict(physics_hz=240, self_collision=True, dynamic_fillers=True, gpu=False, render_every=1)   # set from the benchmark


def start_app(headless):
    """SimulationApp; in a window, full screen at the screen's resolution (scripts/common/sim_app.py)."""
    import sim_app
    return sim_app.start(headless)


def load(app, headless, fill=1.0, seed=0, urdf=None, wheels=False, layout='prototype_mono_arm', **sim):
    """Robot + scene. Returns (task, items, watch): proto_task.Task, the scene's items, {name: prim path} of the task
    products and the crate (to report anything knocked over). Also tracks the base's largest tilt in task.tilt_max.
    layout: 'prototype_mono_arm' (default) or 'prototype' (the dual-arm robot, both arms in proto_poses.STOW)."""
    import numpy as np
    import omni.usd
    from isaacsim.core.prims import SingleRigidPrim
    from proto_task import Task
    import scene as SC
    import mono_poses as MP
    import urdf_variants as UV
    cfg = dict(SIM, **sim)
    src = urdf or RP.urdf(layout)
    task = Task(app, src if wheels else UV.planar_base(src), headless, fix_base=not wheels, layout=layout,
                self_collision=cfg['self_collision'], physics_hz=cfg['physics_hz'], gpu=cfg['gpu'], render_every=cfg['render_every'])
    items = SC.build(omni.usd.get_context().get_stage(), fill=fill, seed=seed, dynamic_fillers=cfg['dynamic_fillers'])
    # prim paths only: registering them in world.scene (SingleRigidPrim + scene.add) made EVERY physics step ~7x slower
    # (1.67 -> 0.25 x real time, 2026-11-17); positions() reads them only when asked
    watch = {n: it['bodies'][0] for n, it in items.items() if it['role'] in ('product', 'basket')}
    task.start()
    # Isaac resets to the zero pose, where the arm's hand points straight ahead: fold it before any physics step
    task.set_pose({**stow_joints(layout), **_home_joints(task)})
    _place_home(task)
    if not headless:
        from isaacsim.core.utils.viewports import set_camera_view
        set_camera_view(eye=np.array(CAMERA['eye']), target=np.array(CAMERA['target']))
    task.tilt_max = 0.0
    step = task.step_once

    def step_and_watch():
        step()
        if task.planar:                       # base held by its joints: no tilt (the root is the floor anchor)
            return
        w, x, y, z = [float(v) for v in task.robot.get_world_pose()[1]]
        task.tilt_max = max(task.tilt_max, math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y))))))
    task.step_once = step_and_watch
    task.wait(1.0)
    return task, items, watch


def _home():
    h = ST.load()['home']
    return float(h['x']), float(h['y']), math.radians(float(h['yaw']))


def stow_joints(layout='prototype_mono_arm'):
    """Joint values of the travel fold: the mono STOW, or both dual arms in proto_poses.STOW."""
    if layout == 'prototype':
        import proto_poses as PPo
        return {**PPo.full('left', PPo.STOW['left']), **PPo.full('right', PPo.STOW['right'])}
    import mono_poses as MP
    return {'lift': MP.STOW['lift'], **{f'left_{k}': v for k, v in MP.STOW.items() if k != 'lift'}}


def _home_joints(task):
    """planar base: the start pose is just the base joints' values (the anchor stays at the world origin)"""
    if not task.planar:
        return {}
    x, y, yaw = _home()
    return {'base_x': x, 'base_y': y, 'base_yaw': yaw}


def _place_home(task):
    """wheeled base (--wheels): put the whole robot there instead"""
    if task.planar:
        return
    import numpy as np
    x, y, yaw = _home()
    p, _ = task.robot.get_world_pose()
    task.robot.set_world_pose(position=np.array([x, y, float(p[2])]), orientation=np.array([math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]))


def leg(task, st, a, b):
    """Drive from station a to station b along ST.route (back out to a's turning point, across, straight in); the
    wheels hold the base at b afterwards. Returns the drive time (s)."""
    import numpy as np
    t0 = task.t
    pts = ST.route(st, a, b)
    for k, (x, y, yaw) in enumerate(pts):
        lbl = b if k == len(pts) - 1 else f'  via {"hub" if (x, y) == tuple(ST.HUB) else "turning point"}'
        task.drive_to(np.array([x, y, math.radians(yaw)]), lbl)
    return task.t - t0


def positions(watch):
    """{name: world position} of the watched bodies, read now (unregistered prims: no per-step cost)."""
    import numpy as np
    from isaacsim.core.prims import SingleXFormPrim
    return {n: np.array(SingleXFormPrim(p).get_world_pose()[0], float) for n, p in watch.items()}


def knocked(watch, p0, limit=0.010):
    """{name: metres moved} for watched bodies that moved more than `limit` since p0 ({name: position})."""
    import numpy as np
    p1 = positions(watch)
    moved = {n: float(np.linalg.norm(p1[n] - p0[n])) for n in watch}
    return {n: d for n, d in moved.items() if d > limit}
