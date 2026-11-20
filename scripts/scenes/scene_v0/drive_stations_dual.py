"""Drive the DUAL-arm robot between the fixed stations of scene_v0 (stations.json). Scripted base moves for arm training,
not navigation: each leg backs out to the station's turning point, crosses the aisle (via the hub when the straight
line isn't clear), and drives straight in. Both arms stay folded (proto_poses.STOW) while driving; at a station the
wheels hold the base in place (proto_task.Task.hold_base).

    OMNI_KIT_ACCEPT_EULA=YES conda run --no-capture-output -n env_isaaclab python scripts/scenes/scene_v0/drive_stations_dual.py --headless
    ... --route home,rack_b0_box_cracker_a01,shelf2_b1_ketchup_c01,home      any stations, in order (default: a short tour)
    ... --tour                                                              every station once, then home
    ... --list                                                              print the station names and stop

Reports, per station reached: stop error (mm, deg) and drive time; at the end: whether any task product or the crate
was knocked (moved > 10 mm), and the base's largest tilt (a bump into furniture shows up there).
"""
# NOTE: uses the mono robot's stations (stations_mono.py) until dual stations exist: driving only, the arms stay folded.
import argparse, json, math, os, sys
here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(here, '..', '..', 'common')); import repo_paths as RP   # every script folder + repo paths
ap = argparse.ArgumentParser()
ap.add_argument('--urdf', default=RP.urdf('prototype'))
ap.add_argument('--route', default='home,rack_b0_boxedCereal_l01,rack2_b0_chocolate_e01,shelf2_b1_ketchup_c01,shelf_b1_milk_b01,table_yogurt_a01,home')
ap.add_argument('--tour', action='store_true')
ap.add_argument('--list', action='store_true')
ap.add_argument('--headless', action='store_true')
ap.add_argument('--physics_hz', type=int, default=240, help='240 = grasp fidelity (default); 120 = about 2x faster, fine for driving')
ap.add_argument('--render_every', type=int, default=1, help='window: draw every Nth frame only (3 = 20 fps on screen, faster sim)')
ap.add_argument('--wheels', action='store_true', help='full mecanum robot (roller simulation) instead of the planar arm-training base')
ap.add_argument('--fill', type=float, default=1.0)
ap.add_argument('--seed', type=int, default=0)
args = ap.parse_args()
import stations_mono as ST                  # no simulator needed for these
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

import scene_robot as R
app = R.start_app(args.headless)
import numpy as np
task, items, watch = R.load(app, args.headless, layout='prototype', fill=args.fill, seed=args.seed, urdf=args.urdf, wheels=args.wheels, physics_hz=args.physics_hz, render_every=args.render_every)
p0 = R.positions(watch)

print(f'\nroute: {" -> ".join(names)}', flush=True)
for a, b in zip(names[:-1], names[1:]):
    dt = R.leg(task, st, a, b)
    print(f'    {a} -> {b}: {dt:.1f} s, {len(ST.route(st, a, b))} waypoints', flush=True)

k = R.knocked(watch, p0)
print(f'\nproducts / crate knocked (> 10 mm): {len(k)} {[(n, round(d * 1000)) for n, d in k.items()]}'
      f'  {"OK" if not k else "FAIL"}')
print(f'base tilt max {task.tilt_max:.2f} deg  {"OK" if task.tilt_max < 1.0 else "CHECK: bumped into something?"}')
arrivals = [r for r in task.results if not r[0].startswith('  via')]
print(f'stations reached: {sum(ok for _, ok, _ in arrivals)}/{len(arrivals)} within 15 mm / 1 deg; simulated {task.t:.0f} s')
if not args.headless:
    task.world.pause()
    print('Physics paused. Close the window to exit.', flush=True)
    while app.is_running():
        app.update()
app.close()
