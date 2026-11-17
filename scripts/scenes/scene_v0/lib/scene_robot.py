"""The mono robot in scene_v0, driven between the fixed stations (stations.json). Shared by drive_stations.py and
random_drive.py.

ARM TRAINING setup (2026-11-17, user: "we focus on arm training fidelity. navigation will be handled separately ... the
robot can move from a to b easily by script"): by default the robot is urdf_variants.planar_base (no wheels / rollers,
the base on driven base_x / base_y / base_yaw joints), with the settings in SIM below (picked from a timing benchmark).
wheels=True loads the full mecanum robot instead (roller simulation, the old drive_to with wheel speeds).

    app = start_app(headless)                   # before any omni / isaacsim import
    task, items, watch = load(app, headless)    # robot (free base, arm folded in STOW) + the store scene
    leg(task, stations, 'home', 'rack_b0_box_cracker_a01')
"""
import math, os, re, subprocess, sys
here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(here, '..', '..', '..', 'common')); import repo_paths as RP   # every script folder + repo paths
import stations as ST                    # no simulator needed

CAMERA = dict(eye=(-4.6, -0.4, 2.3), target=(-1.0, 0.0, 0.5))     # from the open side of the room
SIM = dict(physics_hz=240, self_collision=True, dynamic_fillers=True, gpu=False, render_every=1)   # set from the benchmark


def start_app(headless):
    """SimulationApp; in a window, full screen at the screen's resolution (2560 x 1440 here; the GPU has plenty of room,
    the CPU-side physics sets the frame rate)."""
    from isaacsim import SimulationApp
    cfg = {'headless': headless}
    if not headless:
        try:
            m = re.search(r' connected primary (\d+)x(\d+)', subprocess.run(['xrandr'], capture_output=True, text=True, timeout=5).stdout)
            W, H = int(m.group(1)), int(m.group(2))
        except Exception:
            W, H = 1920, 1080
        cfg.update(width=W, height=H, window_width=W, window_height=H)
    return SimulationApp(cfg)


def load(app, headless, fill=1.0, seed=0, urdf=None, wheels=False, **sim):
    """Robot + scene. Returns (task, items, watch): proto_task.Task, the scene's items, {name: prim path} of the task
    products and the crate (to report anything knocked over). Also tracks the base's largest tilt in task.tilt_max."""
    import numpy as np
    import omni.usd
    from isaacsim.core.prims import SingleRigidPrim
    from proto_task import Task
    import scene as SC
    import mono_poses as MP
    import urdf_variants as UV
    cfg = dict(SIM, **sim)
    src = urdf or RP.urdf('prototype_mono_arm')
    task = Task(app, src if wheels else UV.planar_base(src), headless, fix_base=not wheels, layout='prototype_mono_arm',
                self_collision=cfg['self_collision'], physics_hz=cfg['physics_hz'], gpu=cfg['gpu'], render_every=cfg['render_every'])
    items = SC.build(omni.usd.get_context().get_stage(), fill=fill, seed=seed, dynamic_fillers=cfg['dynamic_fillers'])
    # prim paths only: registering them in world.scene (SingleRigidPrim + scene.add) made EVERY physics step ~7x slower
    # (1.67 -> 0.25 x real time, 2026-11-17); positions() reads them only when asked
    watch = {n: it['bodies'][0] for n, it in items.items() if it['role'] in ('product', 'basket')}
    task.start()
    # Isaac resets to the zero pose, where the arm's hand points straight ahead: fold it before any physics step
    task.set_pose({'lift': MP.STOW['lift'], **{f'left_{k}': v for k, v in MP.STOW.items() if k != 'lift'}})
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
