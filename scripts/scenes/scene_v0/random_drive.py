"""Drive the mono robot to random stations of scene_v0 and stay at each one for a while (default 3 s).

Scripted base moves, not navigation: the next station is picked at random among the others (never the same twice in a
row), the robot drives there on the fixed route (stations.py: back out to the turning point, across, straight in),
the wheels hold it in place for --stay seconds, then the next one. The arm stays folded.

    OMNI_KIT_ACCEPT_EULA=YES conda run --no-capture-output -n env_isaaclab python scripts/scenes/scene_v0/random_drive.py
    ... --stay 3 --seed 1               # stay time (s), random sequence
    ... --count 10                      # stop after 10 stations (default: until the window is closed; headless: 10)
    ... --only task                     # only the task products' stations ('task'), or 'shelves' / 'table'
    ... --headless --count 5            # no window

Prints each move (drive time, how far from the station it stopped) and, at the end, whether anything was knocked
over and the base's largest tilt.
"""
import argparse, math, os, random, sys
here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(here, '..', '..', 'common')); import repo_paths as RP   # every script folder + repo paths
ap = argparse.ArgumentParser()
ap.add_argument('--stay', type=float, default=3.0, help='seconds at each station')
ap.add_argument('--count', type=int, default=None, help='stations to visit (default: endless in a window, 10 headless)')
ap.add_argument('--seed', type=int, default=None, help='random sequence (default: different every run)')
ap.add_argument('--only', choices=('all', 'task', 'shelves', 'table'), default='all')
ap.add_argument('--headless', action='store_true')
ap.add_argument('--physics_hz', type=int, default=240, help='240 = grasp fidelity (default); 120 = about 2x faster, fine for driving')
ap.add_argument('--render_every', type=int, default=1, help='window: draw every Nth frame only (3 = 20 fps on screen, faster sim)')
ap.add_argument('--wheels', action='store_true', help='full mecanum robot (roller simulation) instead of the planar arm-training base')
ap.add_argument('--fill', type=float, default=1.0)
ap.add_argument('--scene_seed', type=int, default=0, help="the scene's filler layout (as check_scene.py --seed)")
args = ap.parse_args()
count = args.count if args.count is not None else (10 if args.headless else 0)    # 0 = endless

import stations as ST
import scene_robot as R
st = ST.load()
pick = {'all': lambda n, s: n != 'home', 'task': lambda n, s: bool(s.get('task')),
        'shelves': lambda n, s: s.get('board', '').split('_')[0] in ('rack', 'rack2', 'shelf', 'shelf2'),
        'table': lambda n, s: s.get('board') == 'table'}[args.only]
pool = ['home'] + [n for n, s in st.items() if pick(n, s)]
rng = random.Random(args.seed)
print(f'random_drive: {len(pool)} stations ({args.only}, incl. home), stay {args.stay:g} s, '
      f'{"endless" if not count else f"{count} moves"}, seed {args.seed}', flush=True)

app = R.start_app(args.headless)
import numpy as np
task, items, watch = R.load(app, args.headless, fill=args.fill, seed=args.scene_seed, wheels=args.wheels, physics_hz=args.physics_hz, render_every=args.render_every)
p0 = R.positions(watch)

import time
here_at, n_moves, times = 'home', 0, []
wall0, sim0 = time.perf_counter(), task.t
try:
    while not count or n_moves < count:
        nxt = rng.choice([n for n in pool if n != here_at])
        s = st[nxt]
        print(f'\n[{task.t:7.1f} s] {here_at} -> {nxt}   ({s.get("section", "start")}'
              f'{", " + s["product"].replace("food_", "") if s.get("product") else ""})', flush=True)
        times.append(R.leg(task, st, here_at, nxt))
        print(f'    speed so far: {(task.t - sim0) / (time.perf_counter() - wall0):.2f} x real time', flush=True)
        here_at = nxt; n_moves += 1
        task.wait(args.stay)                          # parked: the wheels hold the base (proto_task.hold_base)
except SystemExit:                                    # window closed (proto_task.tick raises it)
    pass

arrivals = [r for r in task.results if not r[0].startswith('  via')]
k = R.knocked(watch, p0)
print(f'\n{n_moves} stations visited in {task.t:.0f} s simulated (drives {np.mean(times) if times else 0:.1f} s on average, '
      f'longest {max(times) if times else 0:.1f} s); stopped within 15 mm / 1 deg: {sum(ok for _, ok, _ in arrivals)}/{len(arrivals)}')
print(f'products / crate knocked (> 10 mm): {len(k)} {[(n, round(d * 1000)) for n, d in k.items()]}  {"OK" if not k else "FAIL"}')
print(f'base tilt max {task.tilt_max:.2f} deg  {"OK" if task.tilt_max < 1.0 else "CHECK: bumped into something?"}', flush=True)
if not args.headless and app.is_running():
    task.world.pause()
    print('Done. Physics paused. Close the window to exit.', flush=True)
    while app.is_running():
        app.update()
app.close()
