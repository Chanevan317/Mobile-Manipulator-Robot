"""Replay recorded episodes in Isaac from their saved start, sending only the recorded actions (open loop). Step 2 of
the pipeline: if the replay still puts the product in the tote (or back on its spot), the action stream alone is
enough to reproduce the demonstration, so it is worth training on. Also the skeleton of the policy evaluation.

    conda activate env_isaaclab           # run from the repo root
    OMNI_KIT_ACCEPT_EULA=YES python train_pipeline/tools/replay_episode.py pick_tote_v1            # every episode
    ... --episodes 0,3,7      only these
    ... --window              watch it

Rebuilds each start: products put back, the robot's base and every joint as recorded, the episode's product where it
stood (or lay in the tote), the light. Then per recorded frame: the 7 commanded joints, interpolated over the 2
control steps between frames. Reports the result like the recorder and how far the replayed joints drifted from the
recorded ones.
"""
import argparse, math, os, sys
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
import common as C                                                       # noqa: E402
sys.path.insert(0, str(C.REPO / 'scripts' / 'common')); import repo_paths as RP   # noqa: E402,F401

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('raw')
ap.add_argument('--episodes', default='', help='comma-separated episode numbers (default: all)')
ap.add_argument('--window', action='store_true')
args = ap.parse_args()
raw = C.RAW_DIR / args.raw if not os.path.isdir(args.raw) else __import__('pathlib').Path(args.raw)
files = sorted(raw.glob('episode_*.h5'))
if args.episodes:
    want = {int(x) for x in args.episodes.split(',')}
    files = [p for p in files if int(p.stem.split('_')[1]) in want]
if not files:
    sys.exit(f'no episodes in {raw}')

os.environ.setdefault('MM_CAMERAS', '0')       # not needed to replay (faster); MM_CAMERAS=1 to watch them
import stations as ST                                                    # noqa: E402
import scene_robot as R                                                  # noqa: E402
st = ST.load()
app = R.start_app(not args.window)
import numpy as np                                                       # noqa: E402
import h5py                                                              # noqa: E402
import omni.usd                                                          # noqa: E402
from pxr import UsdLux, PhysicsSchemaTools                               # noqa: E402
from omni.physx import get_physx_simulation_interface                    # noqa: E402
import mono_poses as MP                                                  # noqa: E402
import pick_place as PP                                                  # noqa: E402

task, items, watch = R.load(app, not args.window)
PP.init(task, items, watch, st, headless=not args.window, out=None)
EVERY = round(60 / C.FPS)
IDX = [[task.idx[j] for j in (u if isinstance(u, tuple) else (u,))] for _, u in C.JOINTS]
stage = omni.usd.get_context().get_stage()
LIGHTS = [(UsdLux.LightAPI(p).GetIntensityAttr(), float(UsdLux.LightAPI(p).GetIntensityAttr().Get()))
          for p in stage.Traverse() if p.HasAPI(UsdLux.LightAPI) and p.IsActive() and UsdLux.LightAPI(p).GetIntensityAttr().Get() is not None]


def set_cmd(v):
    """7-vector -> joint targets (the gripper opening split over the two fingers)."""
    for ii, x in zip(IDX, v):
        for i in ii:
            task.q_goal[i] = x / len(ii)


def teleport(name, poses):
    sim = get_physx_simulation_interface(); sid_ = omni.usd.get_context().get_stage_id()
    for b, pq in zip(PP.MOVABLE[name], poses):
        b.set_world_pose(position=pq[:3], orientation=pq[3:])
        bid = PhysicsSchemaTools.sdfPathToInt(b.prim_path)
        sim.put_to_sleep(sid_, bid); sim.wake_up(sid_, bid)


passed = 0
for p in files:
    with h5py.File(p, 'r') as f:
        a = dict(f.attrs); act = f['action'][:]; rec = f['observation.state'][:]
    name, kind = str(a['product']), str(a['kind'])
    PP.restore()
    for la, v0 in LIGHTS:
        la.Set(v0 * float(a.get('light_scale', 1.0)))
    task.set_pose({n: float(v) for n, v in zip(a['q0_names'], a['q0'])})
    task.q_cmd[:] = task.q_goal                                        # set_pose sets both; keep them equal
    teleport(name, a['product_pose0'])
    got = []
    for t in range(len(act)):
        got.append([float(sum(task.robot.get_joint_positions()[i] for i in ii)) for ii in IDX])
        nxt = act[min(t + 1, len(act) - 1)]
        for s in range(EVERY):
            set_cmd(act[t] + (nxt - act[t]) * (s + 1) / EVERY)
            task.step_once()
    task.wait(1.0)
    got = np.array(got, np.float32)
    drift = np.abs(got - rec[:len(got)])
    worst = int(np.argmax(drift.max(axis=0)))
    pb = task.to_base(PP.R.positions({name: PP.MOVABLE[name][0].prim_path})[name])
    if kind == 'pick':
        x0, x1, y0, y1 = MP.TOTE_BOX
        ok = x0 < pb[0] < x1 and y0 < pb[1] < y1 and pb[2] < 0.376
        res = f'product at ({pb[0]:+.3f}, {pb[1]:+.3f}, {pb[2]:.3f}) in the base frame: {"in the tote" if ok else "NOT in the tote"}'
    else:
        g = np.array(a['goal_pose'])[0, :3]
        d = np.linalg.norm(np.array(PP.MOVABLE[name][0].get_world_pose()[0])[:2] - g[:2]) * 1000
        ok = d < 20
        res = f'{d:.0f} mm from its spot'
    passed += ok
    print(f'{p.stem}: {"PASS" if ok else "FAIL"}  {kind:5s} {res}; joint drift max {drift.max():.4f} '
          f'({C.JOINT_NAMES[worst]}), mean {drift.mean():.4f}  [{a["task"]}]', flush=True)
print(f'\n{passed}/{len(files)} replays reproduce their episode')
PP.close()
app.close()
