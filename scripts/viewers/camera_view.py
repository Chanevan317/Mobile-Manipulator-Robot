"""What the robot's cameras see in scene_v0: drive the mono robot to a station, save each camera's picture.

    OMNI_KIT_ACCEPT_EULA=YES conda run --no-capture-output -n env_isaaclab python scripts/viewers/camera_view.py --headless
    ... --station table_cannedFood_h01     any station of stations.json (default: the milk on the grey shelf)
    ... --pitch 20                          scene camera pitch (deg, positive = tilted down; URDF limit -10 .. 80)
    ... --out DIR                           where the pictures go (default: camera_view/ in the current folder)

Saves <station>_scene_rgb.png, <station>_scene_depth.png (near = bright, no depth = black) and <station>_left_wrist_rgb.png,
and prints the depth range seen. Camera specs: scripts/common/cameras.py (the wrist lens is ASSUMED until calibrated).
"""
import argparse, math, os, sys
here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(here, '..', 'common')); import repo_paths as RP   # every script folder + repo paths
ap = argparse.ArgumentParser()
ap.add_argument('--station', default='shelf_b1_milk_b01')
ap.add_argument('--pitch', type=float, default=None, help='scene camera pitch in deg (default: as loaded)')
ap.add_argument('--out', default='camera_view')
ap.add_argument('--headless', action='store_true')
args = ap.parse_args()

import stations as ST
import scene_robot as R
st = ST.load()
if args.station not in st:
    sys.exit(f'unknown station {args.station}; see stations.json')
app = R.start_app(args.headless)
import numpy as np
from PIL import Image
import cameras as CAM
task, items, watch = R.load(app, args.headless)
cams = task.cams                                     # added by proto_task.Task.start()
if args.pitch is not None:
    task.q_goal[task.idx['scene_camera_pitch']] = math.radians(args.pitch)
R.leg(task, st, 'home', args.station)
task.wait(0.5)

os.makedirs(args.out, exist_ok=True)
fr = CAM.capture(task, cams)
for k, img in fr.items():
    if img is None:
        print(f'{k}: no image', flush=True); continue
    p = os.path.join(args.out, f'{args.station}_{k}.png')
    if k.endswith('_depth'):
        v = img > 0
        lo, hi = (float(img[v].min()), float(img[v].max())) if v.any() else (0.0, 0.0)
        print(f'{k}: {img.shape[1]} x {img.shape[0]}, {100 * v.mean():.0f} % with depth, '
              f'{lo:.2f} .. {hi:.2f} m (median {float(np.median(img[v])) if v.any() else 0:.2f})', flush=True)
        g = np.zeros(img.shape, np.uint8)
        if v.any():
            g[v] = (255 * (1 - (img[v] - CAM.SCENE['depth'][0]) / (hi - CAM.SCENE['depth'][0] + 1e-6)) * 0.85 + 38).astype(np.uint8)
        Image.fromarray(g).save(p)
    else:
        print(f'{k}: {img.shape[1]} x {img.shape[0]}, mean brightness {img.mean():.0f}', flush=True)
        Image.fromarray(img).save(p)
    print('   ->', p, flush=True)
if not args.headless:
    task.world.pause()
    print('Physics paused. Close the window to exit.', flush=True)
    while app.is_running():
        app.update()
app.close()
