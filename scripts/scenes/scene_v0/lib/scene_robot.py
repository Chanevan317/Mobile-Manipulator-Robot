"""The mono robot in scene_v0, driven between the fixed stations (stations.json). Shared by drive_stations.py and
random_drive.py.

    app = start_app(headless)                   # before any omni / isaacsim import
    task, items, watch = load(app, headless)    # robot (free base, arm folded in STOW) + the store scene
    leg(task, stations, 'home', 'rack_b0_box_cracker_a01')
"""
import math, os, re, subprocess, sys
here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(here, '..', '..', '..', 'common')); import repo_paths as RP   # every script folder + repo paths
import stations as ST                    # no simulator needed

CAMERA = dict(eye=(-4.6, -0.4, 2.3), target=(-1.0, 0.0, 0.5))     # from the open side of the room


def start_app(headless):
    """SimulationApp; in a window, full screen at the screen's resolution (sharper, as check_scene.py)."""
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


def load(app, headless, fill=1.0, seed=0, urdf=None):
    """Robot + scene. Returns (task, items, watch): proto_task.Task, the scene's items, {name: rigid prim} of the task
    products and the crate (to report anything knocked over). Also tracks the base's largest tilt in task.tilt_max."""
    import numpy as np
    import omni.usd
    from isaacsim.core.prims import SingleRigidPrim
    from proto_task import Task
    import scene as SC
    import mono_poses as MP
    task = Task(app, urdf or RP.urdf('prototype_mono_arm'), headless, fix_base=False, layout='prototype_mono_arm')
    items = SC.build(omni.usd.get_context().get_stage(), fill=fill, seed=seed)
    watch = {n: task.world.scene.add(SingleRigidPrim(it['bodies'][0], name=f'w{k}'))
             for k, (n, it) in enumerate(items.items()) if it['role'] in ('product', 'basket')}
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


def knocked(watch, p0, limit=0.010):
    """{name: metres moved} for watched bodies that moved more than `limit` since p0 ({name: position})."""
    import numpy as np
    moved = {n: float(np.linalg.norm(np.array(w.get_world_pose()[0]) - p0[n])) for n, w in watch.items()}
    return {n: d for n, d in moved.items() if d > limit}
