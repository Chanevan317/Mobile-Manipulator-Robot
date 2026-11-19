"""Look at recorded raw episodes without Isaac (any env with h5py + Pillow: env_isaaclab or lerobot_env).

    python train_pipeline/tools/inspect_raw.py pick_tote_v1                 # one line per episode + totals
    python train_pipeline/tools/inspect_raw.py pick_tote_v1 --sheet 3       # contact sheet of episode 3 -> PNG
"""
import argparse, io, sys
from pathlib import Path

import h5py
import numpy as np
from PIL import Image, ImageDraw

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import common as C                                                       # noqa: E402

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('raw', help='folder name in data/raw/ (or a path)')
ap.add_argument('--sheet', type=int, default=None, help='episode number: save a contact sheet (8 moments, both cameras)')
args = ap.parse_args()
raw = Path(args.raw) if Path(args.raw).is_dir() else C.RAW_DIR / args.raw
files = sorted(raw.glob('episode_*.h5'))
if not files:
    sys.exit(f'no episode_*.h5 in {raw}')

tot = {'frames': 0, 'ok': 0}
print(f'{"episode":16s} {"kind":5s} {"frames":>6s} {"sec":>6s} {"ok":3s} {"station":26s} task')
for p in files:
    with h5py.File(p, 'r') as f:
        a = f.attrs; n = len(f['action'])
        tot['frames'] += n; tot['ok'] += bool(a['success'])
        print(f'{p.stem:16s} {a["kind"]:5s} {n:6d} {n / a["fps"]:6.1f} {"yes" if a["success"] else "NO ":3s} {a["station"]:26s} {a["task"]}')
print(f'\n{len(files)} episodes ({tot["ok"]} successful), {tot["frames"]} frames = {tot["frames"] / C.FPS / 60:.1f} min at {C.FPS} fps')

if args.sheet is not None:
    p = raw / f'episode_{args.sheet:06d}.h5'
    with h5py.File(p, 'r') as f:
        n = len(f['action']); ks = np.linspace(0, n - 1, 8).astype(int)
        rows = []
        for key in C.CAMERAS:
            ims = [Image.open(io.BytesIO(f[key][k].tobytes())).convert('RGB').resize((320, 200 if 'scene' in key else 180)) for k in ks]
            rows.append(ims)
        st = f['observation.state'][:]; task = f.attrs['task']
    W = 320 * 8; H = 30 + sum(r[0].height for r in rows) + 20
    sheet = Image.new('RGB', (W, H), (255, 255, 255)); d = ImageDraw.Draw(sheet)
    d.text((6, 8), f'{p.name}: {task}', fill=(0, 0, 0))
    y = 30
    for r in rows:
        for i, im in enumerate(r):
            sheet.paste(im, (320 * i, y))
        y += r[0].height
    for i, k in enumerate(ks):
        d.text((320 * i + 6, y + 4), f't {k / C.FPS:.1f} s  jaw {st[k, -1] * 1000:.0f} mm  lift {st[k, 0]:.2f} m', fill=(0, 0, 0))
    out = raw / f'{p.stem}_sheet.png'; sheet.save(out)
    print(f'contact sheet: {out}')
