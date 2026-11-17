"""Drive the mono robot between the fixed stations of scene_v0 (stations.json). Scripted base moves for arm training,
not navigation: each leg backs out to the station's turning point, crosses the aisle (via the hub when the straight
line isn't clear), and drives straight in. The arm stays folded (mono_poses.STOW) while driving; at a station the
wheels hold the base in place (proto_task.Task.hold_base).

    OMNI_KIT_ACCEPT_EULA=YES conda run --no-capture-output -n env_isaaclab python scenes/scene_v0/drive_stations.py --headless
    ... --route home,rack_b0_box_cracker_a01,shelf2_b1_ketchup_c01,home      any stations, in order (default: a short tour)
    ... --tour                                                              every station once, then home
    ... --list                                                              print the station names and stop

Reports, per station reached: stop error (mm, deg) and drive time; at the end: whether any task product or the crate
was knocked (moved > 10 mm), and the base's largest tilt (a bump into furniture shows up there).
"""
import argparse, json, math, os, sys
here = os.path.dirname(os.path.abspath(__file__))
ap = argparse.ArgumentParser()
ap.add_argument('--urdf', default=os.path.join(here, '..', '..', 'mobile_manipulator_description', 'urdf', 'mobile_manipulator_prototype_mono_arm.urdf'))
ap.add_argument('--route', default='home,rack_b0_boxedCereal_l01,rack2_b0_chocolate_e01,shelf2_b1_ketchup_c01,shelf_b1_milk_b01,table_yogurt_a01,home')
ap.add_argument('--tour', action='store_true')
ap.add_argument('--list', action='store_true')
ap.add_argument('--headless', action='store_true')
ap.add_argument('--fill', type=float, default=1.0)
ap.add_argument('--seed', type=int, default=0)
args = ap.parse_args()
sys.path.insert(0, here)
import stations as ST                  # no simulator needed for these
st = ST.load()
if args.list:
    for n, s_ in st.items():
        print(f"{n:28s} ({s_['x']:6.3f}, {s_['y']:6.3f}, {s_['yaw']:4.0f} deg)  {s_.get('section', '')}")
    sys.exit(0)
names = ['home'] + [n for n in st if n != 'home'] + ['home'] if args.tour else args.route.split(',')
unknown = [n for n in names if n not in st]
if unknown:
    sys.exit(f'unknown station(s): {unknown}  (--list shows them)')
if names[0] != 'home':
    names = ['home'] + names           # the robot starts at home

from isaacsim import SimulationApp
cfg = {'headless': args.headless}
if not args.headless:                  # full screen at the screen's resolution (as check_scene.py)
    import re, subprocess
    try:
        m = re.search(r' connected primary (\d+)x(\d+)', subprocess.run(['xrandr'], capture_output=True, text=True, timeout=5).stdout)
        W, H = int(m.group(1)), int(m.group(2))
    except Exception:
        W, H = 1920, 1080
    cfg.update(width=W, height=H, window_width=W, window_height=H)
app = SimulationApp(cfg)

import numpy as np
import omni.usd
from isaacsim.core.prims import SingleRigidPrim
sys.path.insert(1, os.path.normpath(os.path.join(here, '..', '..', 'mobile_manipulator_description', 'scripts')))
from proto_task import Task
import scene as SC
import mono_poses as MP

task = Task(app, args.urdf, args.headless, fix_base=False, layout='prototype_mono_arm')
stage = omni.usd.get_context().get_stage()
items = SC.build(stage, fill=args.fill, seed=args.seed)
watch = {n: task.world.scene.add(SingleRigidPrim(it['bodies'][0], name=f'w{k}'))
         for k, (n, it) in enumerate(items.items()) if it['role'] in ('product', 'basket')}
task.start()
task.set_pose({'lift': MP.STOW['lift'], **{f'left_{k}': v for k, v in MP.STOW.items() if k != 'lift'}})
if not args.headless:
    from isaacsim.core.utils.viewports import set_camera_view
    set_camera_view(eye=np.array([-4.6, -0.4, 2.3]), target=np.array([-1.0, 0.0, 0.5]))
task.wait(1.0)
p0 = {n: np.array(w.get_world_pose()[0]) for n, w in watch.items()}

tilt_max = [0.0]
_step = task.step_once


def step_and_watch():
    _step()
    w, x, y, z = [float(v) for v in task.robot.get_world_pose()[1]]
    tilt_max[0] = max(tilt_max[0], math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y))))))


task.step_once = step_and_watch
print(f'\nroute: {" -> ".join(names)}', flush=True)
for a, b in zip(names[:-1], names[1:]):
    t0 = task.t
    pts = ST.route(st, a, b)
    for k, (x, y, yaw) in enumerate(pts):
        task.drive_to(np.array([x, y, math.radians(yaw)]), f'{b}' if k == len(pts) - 1 else f'  via {"hub" if (x, y) == ST.HUB else "turning point"}')
    print(f'    {a} -> {b}: {task.t - t0:.1f} s, {len(pts)} waypoints', flush=True)

moved = {n: np.linalg.norm(np.array(w.get_world_pose()[0]) - p0[n]) for n, w in watch.items()}
knocked = {n: d for n, d in moved.items() if d > 0.010}
print(f'\nproducts / crate knocked (> 10 mm): {len(knocked)} {[(n, round(d * 1000)) for n, d in knocked.items()]}'
      f'  {"OK" if not knocked else "FAIL"}')
print(f'base tilt max {tilt_max[0]:.2f} deg  {"OK" if tilt_max[0] < 1.0 else "CHECK: bumped into something?"}')
arrivals = [r for r in task.results if not r[0].startswith('  via')]
print(f'stations reached: {sum(ok for _, ok, _ in arrivals)}/{len(arrivals)} within 15 mm / 1 deg; simulated {task.t:.0f} s')
if not args.headless:
    task.world.pause()
    print('Physics paused. Close the window to exit.', flush=True)
    while app.is_running():
        app.update()
app.close()
