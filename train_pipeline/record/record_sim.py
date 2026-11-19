"""Record demonstration episodes in the Isaac store scene -> raw HDF5 files (one per episode). Step 1 of the pipeline.

    conda activate env_isaaclab           # Isaac Sim 5.1; run from the repo root
    OMNI_KIT_ACCEPT_EULA=YES python train_pipeline/record/record_sim.py --episodes 50 --name pick_tote_v1
    ... --place                  also record putting each product back as its own episode (a second task)
    ... --stations a,b           only these stations (default: every station that picks reliably, pick_place.PICKABLE)
    ... --no_jitter              no randomisation (same start every time)
    ... --window                 watch it in the Isaac window (slower); default headless
    ... --depth                  also store the scene camera's depth (uint16 mm; for point-cloud policies later)
    ... --noise mixed            human-like imperfections (until the leader arm exists): see NOISE below

Output: train_pipeline/data/raw/<name>/episode_XXXXXX.h5 (numbering continues in an existing folder).

Source of the demonstrations (--source):
    planner   the offline-planned pick / place of scripts/scenes/scene_v0/lib/pick_place.py (works now)
    leader    the leader arm (teleop) -- not available until the leader arm is built

One episode = from the robot parked at the station (arm folded) to the arm folded again: drive legs are not recorded
(navigation is scripted, not learned). Every episode starts with the moved products put back. Randomised per episode
(off with --no_jitter): where the robot stops (+-2 cm, +-3 deg), the product (+-1 cm, +-8 deg), the light (+-25 %).
Only successful episodes are kept (--keep_failed keeps the rest, marked success=False).

NOISE (--noise mixed; default none): each episode draws a level, clean 30 % / mild 40 % / noisy 30 %, stored in the file:
    tremor     a smooth random wobble (Ornstein-Uhlenbeck, 0.4 s) added to the commanded joints: wrist pitch / roll
               0.6 / 1.2 deg (mild / noisy), less up the arm (elbow 0.7x, shoulder 0.4x, ring 0.25x: longer levers),
               lift 0.5 / 1 mm; the gripper gets none
    speed      the whole episode at 80-100 % / 60-90 % of the normal joint speeds
    pauses     before a move or a jaw action: 15 % chance of 0.2-0.6 s / 30 % of 0.3-1.2 s
    aim        the hand aims +-3 / +-6 mm off the product's centre
The recorded action is the command actually sent (noise included), as a person's command would be. Demos the noise
spoils (product dropped or knocked) are dropped like any failed episode.
"""
import argparse, io, math, os, random, sys, time
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
import common as C                                                       # noqa: E402
sys.path.insert(0, str(C.REPO / 'scripts' / 'common')); import repo_paths as RP   # noqa: E402,F401  (all script folders)

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('--episodes', type=int, default=10, help='pick episodes to record (place episodes come on top)')
ap.add_argument('--name', default='pick_tote_planner', help='recording set: data/raw/<name>/')
ap.add_argument('--source', default='planner', choices=('planner', 'leader'))
ap.add_argument('--stations', default='', help='comma-separated stations (default: pick_place.PICKABLE)')
ap.add_argument('--place', action='store_true', help='also record the place-back of each picked product')
ap.add_argument('--no_jitter', action='store_true')
ap.add_argument('--base_xy', type=float, default=0.02, help='m, robot stop jitter')
ap.add_argument('--base_yaw', type=float, default=3.0, help='deg')
ap.add_argument('--product_xy', type=float, default=0.01, help='m, product jitter')
ap.add_argument('--product_yaw', type=float, default=8.0, help='deg')
ap.add_argument('--light', type=float, default=0.25, help='light intensity jitter (fraction)')
ap.add_argument('--tilt', type=float, default=40.0, help='scene camera pitch, deg')
ap.add_argument('--seed', type=int, default=0)
ap.add_argument('--depth', action='store_true')
ap.add_argument('--jpeg', type=int, default=95, help='JPEG quality of the stored frames')
ap.add_argument('--keep_failed', action='store_true')
ap.add_argument('--window', action='store_true')
ap.add_argument('--noise', choices=('none', 'mixed'), default='none')
args = ap.parse_args()
if args.source == 'leader':
    sys.exit('--source leader: the leader arm is not built yet; record with --source planner for now.')

import stations as ST                                                    # noqa: E402
import scene_robot as R                                                  # noqa: E402
st = ST.load()
app = R.start_app(not args.window)
import numpy as np                                                       # noqa: E402
import h5py                                                              # noqa: E402
from PIL import Image                                                    # noqa: E402
import cameras as CAM                                                    # noqa: E402
import pick_place as PP                                                  # noqa: E402
import sim_common as SCM                                                 # noqa: E402

task, items, watch = R.load(app, not args.window)
if not task.cams:
    sys.exit('no cameras (MM_CAMERAS=0?): the recorder needs them')
PP.init(task, items, watch, st, tilt=args.tilt, headless=not args.window, out=None)
SCM.bind(task, items, PP)
rng = random.Random(args.seed)
out_dir = C.RAW_DIR / args.name
out_dir.mkdir(parents=True, exist_ok=True)
next_index = 1 + max([int(p.stem.split('_')[1]) for p in out_dir.glob('episode_*.h5')] or [-1])
EVERY = round(60 / C.FPS)                                                # control steps per recorded frame

# ---------------------------------------------------------------- the recorder
class Recorder:
    def __init__(self):
        self.active = False; self.k = 0

    def start(self, kind, sid, name):
        self.meta = dict(kind=kind, station=sid, product=name)
        self.state, self.action, self.t = [], [], []
        self.img = {k: [] for k in C.CAMERAS}; self.depth = []
        self.q0 = task.robot.get_joint_positions().copy()
        self.pose0 = SCM.body_poses(name)
        self.base0 = task.base_pose()
        self.active = True; self.k = 0
        self.sample()

    def sample(self):
        self.state.append(SCM.state()); self.action.append(SCM.sent()); self.t.append(task.t)
        fr = CAM.capture(task, task.cams)
        for key, cam in C.CAMERAS.items():
            buf = io.BytesIO(); Image.fromarray(fr[f'{cam}_rgb']).save(buf, 'JPEG', quality=args.jpeg)
            self.img[key].append(np.frombuffer(buf.getvalue(), np.uint8))
        if args.depth:
            d = fr.get('scene_depth')
            self.depth.append(np.zeros(fr['scene_rgb'].shape[:2], np.uint16) if d is None else np.clip(d * 1000, 0, 65535).astype(np.uint16))

    def step(self):
        if self.active:
            self.k += 1
            if self.k % EVERY == 0:
                self.sample()

    def stop(self, success, why, extra=None):
        """Write the episode (if kept); returns its path or None."""
        global next_index
        if not self.active:
            return None
        self.active = False
        if not success and not args.keep_failed:
            print(f'      episode dropped ({why})', flush=True)
            return None
        m = self.meta
        prod = PP.NAMES.get(items[m['product']]['pkg'], m['product'])
        where = 'table' if m['station'].startswith('table') else 'shelf'
        path = out_dir / f'episode_{next_index:06d}.h5'
        with h5py.File(path, 'w') as f:
            f['observation.state'] = np.stack(self.state)
            f['action'] = np.stack(self.action)
            f['time'] = np.array(self.t) - self.t[0]
            vl = h5py.vlen_dtype(np.uint8)
            for key, frames in self.img.items():
                ds = f.create_dataset(key, (len(frames),), dtype=vl)
                for i, b in enumerate(frames):
                    ds[i] = b
            if args.depth:
                f.create_dataset('observation.depth.scene', data=np.stack(self.depth), compression='gzip', compression_opts=4)
            a = f.attrs
            a['fps'] = C.FPS; a['robot_type'] = C.ROBOT_TYPE; a['action_source'] = args.source
            a['joint_names'] = C.JOINT_NAMES; a['image_encoding'] = 'jpeg'
            a['task'] = C.task_text(m['kind'], prod, where); a['kind'] = m['kind']
            a['station'] = m['station']; a['product'] = m['product']; a['product_name'] = prod
            a['success'] = bool(success); a['result'] = why
            a['scene_camera'] = CAM.SCENE_MODEL; a['scene_camera_tilt_deg'] = args.tilt
            # what replay_episode.py needs to rebuild the start: every joint, the base, the product's bodies, the light
            a['q0_names'] = list(task.names); a['q0'] = self.q0
            a['base0'] = [float(self.base0[0][0]), float(self.base0[0][1]), float(self.base0[1])]
            a['product_pose0'] = np.array([np.concatenate([p, q]) for p, q in self.pose0])
            for k, v in (extra or {}).items():
                if v is not None:
                    a[k] = v
        next_index += 1
        n = len(self.state)
        print(f'      saved {path.name}: {n} frames ({n / C.FPS:.1f} s), "{C.task_text(m["kind"], prod, where)}"', flush=True)
        return path


rec = Recorder()
_step = task.step_once

# ---------------------------------------------------------------- human-like noise (--noise mixed)
LEVELS = {'clean': dict(tremor=0.0, lift=0.0, speed=(1.0, 1.0), pause=(0.0, 0, 0), aim=0.0),
          'mild': dict(tremor=0.6, lift=0.0005, speed=(0.8, 1.0), pause=(0.15, 0.2, 0.6), aim=0.003),
          'noisy': dict(tremor=1.2, lift=0.001, speed=(0.6, 0.9), pause=(0.30, 0.3, 1.2), aim=0.006)}
LEVER = {'left_shoulder_pan': 0.25, 'left_elbow_pan': 0.4, 'left_wrist_pan': 0.7, 'left_wrist_pitch': 1.0, 'left_wrist_roll': 1.0}
TAU, DT = 0.4, 1.0 / 60
NZ = {'level': None}
n_idx = [task.idx[j] for j in LEVER] + [task.idx['lift']]
ou = np.zeros(len(n_idx))


def step_and_record():
    global ou
    lv = NZ['level']
    if rec.active and lv and LEVELS[lv]['tremor'] > 0:
        L = LEVELS[lv]
        sig = np.array([math.radians(L['tremor']) * LEVER[j] for j in LEVER] + [L['lift']])
        ou = ou - ou / TAU * DT + sig * math.sqrt(2 * DT / TAU) * nrng.standard_normal(len(n_idx))
        off = np.zeros(len(task.names)); off[n_idx] = ou
        task.cmd_offset = off
    else:
        task.cmd_offset = None; ou[:] = 0
    _step(); rec.step()


task.step_once = step_and_record


def with_pause(fn):
    def w(*a, **k):
        lv = NZ['level']
        if rec.active and lv:
            p, t0, t1 = LEVELS[lv]['pause']
            if p and rng.random() < p:
                task.wait(rng.uniform(t0, t1))
        return fn(*a, **k)
    return w


task.joint_move = with_pause(task.joint_move); task.cart_move = with_pause(task.cart_move); task.jaw = with_pause(task.jaw)
nrng = np.random.default_rng(args.seed + 1)
EP = {}                                         # per-episode jitter, also stored in the file
PP.HOOKS.update(
    grasp_offset=lambda sid, name: EP.get('aim', (0.0, 0.0)),
    pick_chosen=lambda sid, name: (SCM.jitter_product(name, *EP['product']) if EP.get('product') else None),
    pick_start=lambda sid, name: rec.start('pick', sid, name),
    pick_end=lambda sid, name: None,            # stopped after run() returns: it knows whether the pick passed
    place_start=lambda sid, name: rec.start('place', sid, name),
    place_end=lambda sid, name: None)

stations = [s for s in (args.stations.split(',') if args.stations else sorted(PP.PICKABLE)) if s]
bad = [s for s in stations if s not in st]
if bad:
    sys.exit(f'unknown stations: {bad}')
print(f'\nrecording {args.episodes} pick episode(s){" + place-backs" if args.place else ""} from {len(stations)} station(s) '
      f'-> {out_dir}\n', flush=True)
t_wall = time.time(); kept = tried = 0
try:
    for e in range(args.episodes):
        sid = rng.choice(stations)
        jit = not args.no_jitter
        light = 1 + rng.uniform(-args.light, args.light) if jit else 1.0
        bx, by = (rng.uniform(-args.base_xy, args.base_xy) for _ in range(2)) if jit else (0.0, 0.0)
        byaw = rng.uniform(-args.base_yaw, args.base_yaw) if jit else 0.0
        px, py = (rng.uniform(-args.product_xy, args.product_xy) for _ in range(2)) if jit else (0.0, 0.0)
        pyaw = math.radians(rng.uniform(-args.product_yaw, args.product_yaw)) if jit else 0.0
        EP['product'] = (px, py, pyaw) if jit else None
        lv = None
        if args.noise == 'mixed':
            u = rng.random(); lv = 'clean' if u < 0.3 else 'mild' if u < 0.7 else 'noisy'
            L = LEVELS[lv]; spd = rng.uniform(*L['speed'])
            EP['aim'] = (rng.uniform(-L['aim'], L['aim']), rng.uniform(-L['aim'], L['aim']))
        else:
            spd = 1.0; EP['aim'] = (0.0, 0.0)
        NZ['level'] = lv
        vmax0 = task.vmax.copy(); task.vmax = vmax0 * spd
        extra = dict(light_scale=light, base_jitter=[bx, by, byaw], product_jitter=[px, py, math.degrees(pyaw)], seed=args.seed, episode_in_run=e,
                     noise_level=lv or 'none', speed_scale=spd, aim_offset=list(EP['aim']))
        SCM.set_light(light)
        S0 = st[sid]; st[sid] = {**S0, 'x': S0['x'] + bx, 'y': S0['y'] + by, 'yaw': S0['yaw'] + byaw}   # PP shares this dict
        print(f'\n=== episode {e + 1}/{args.episodes}: {sid}  [noise {lv or "none"}, speed x{spd:.2f}, aim {EP["aim"][0]*1000:+.0f}/{EP["aim"][1]*1000:+.0f} mm]  (stop {bx*100:+.1f}/{by*100:+.1f} cm {byaw:+.1f} deg, product '
              f'{px*100:+.1f}/{py*100:+.1f} cm {math.degrees(pyaw):+.1f} deg, light x{light:.2f})', flush=True)
        try:
            tried += 1
            ok, why = PP.run(sid)
            goal = np.array([np.concatenate([p, q]) for p, q in rec.pose0]) if rec.active else None   # where it stood
            kept += rec.stop(ok, why, extra) is not None
            if not ok:
                PP.reset_robot()
            elif args.place and sid not in PP.PICK_ONLY:
                tried += 1
                ok2, why2 = PP.run_place()
                kept += rec.stop(ok2, why2, {**extra, 'goal_pose': goal}) is not None
                if not ok2:
                    PP.reset_robot()
        finally:
            st[sid] = S0
            rec.active = False
            task.vmax = vmax0; NZ['level'] = None
except SystemExit:                                  # window closed
    pass
SCM.set_light(1.0)
PP.close()
dt = time.time() - t_wall
print(f'\nkept {kept} of {tried} episode(s) in {out_dir}  ({dt / 60:.1f} min wall time)', flush=True)
app.close()
