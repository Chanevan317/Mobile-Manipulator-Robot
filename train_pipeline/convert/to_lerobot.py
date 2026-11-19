"""Raw HDF5 episodes -> a LeRobotDataset (v3, AV1 videos). Step 3 of the pipeline.

    conda activate lerobot_env            # LeRobot 0.6.1 (~/lerobot); NOT the Isaac env
    python train_pipeline/convert/to_lerobot.py --raw pick_tote_v1
    ... --out NAME          dataset folder name in data/lerobot/ (default: the raw folder's name)
    ... --kind pick|place   only that task (default: all episodes, each with its own instruction)
    ... --max_episodes N    quick test
    ... --push_to_hub --repo_id <hf user>/<name>

Reads train_pipeline/data/raw/<raw>/episode_*.h5 (record/record_sim.py), writes train_pipeline/data/lerobot/<out>/.
Features: observation.state / action (7, names in common.JOINT_NAMES), observation.images.scene / .wrist (video),
one language instruction per episode ('task'). Failed episodes (success=False, kept with --keep_failed) are skipped.
Refuses to mix action sources (planner vs leader) or frame rates in one dataset.
"""
import argparse, io, sys
from pathlib import Path

import h5py
import numpy as np
from PIL import Image

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import common as C                                                       # noqa: E402


def decode(b):
    return np.asarray(Image.open(io.BytesIO(b.tobytes())).convert('RGB'))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--raw', required=True, help='folder name in data/raw/ (or a path)')
    ap.add_argument('--out', default=None)
    ap.add_argument('--kind', choices=('pick', 'place'), default=None)
    ap.add_argument('--max_episodes', type=int, default=None)
    ap.add_argument('--repo_id', default=None, help='default local/<out>')
    ap.add_argument('--push_to_hub', action='store_true')
    args = ap.parse_args()

    raw = Path(args.raw) if Path(args.raw).is_dir() else C.RAW_DIR / args.raw
    files = sorted(raw.glob('episode_*.h5'))
    if not files:
        sys.exit(f'no episode_*.h5 in {raw}')
    out = C.LEROBOT_DIR / (args.out or raw.name)
    repo_id = args.repo_id or f'local/{out.name}'
    if args.push_to_hub and repo_id.startswith('local/'):
        sys.exit('--push_to_hub needs --repo_id <your Hugging Face user>/<name>')
    if out.exists() and any(out.iterdir()):
        sys.exit(f'{out} exists and is not empty: delete it or pass another --out (LeRobotDataset.create needs a new folder)')
    if out.exists():
        out.rmdir()

    use, sources, fpss, shapes = [], set(), set(), {}
    for p in files:
        with h5py.File(p, 'r') as f:
            a = f.attrs
            if not a.get('success', True):
                continue
            if args.kind and a.get('kind') != args.kind:
                continue
            if list(a['joint_names']) != C.JOINT_NAMES:
                sys.exit(f'{p.name}: joint names {list(a["joint_names"])} differ from common.JOINT_NAMES')
            sources.add(str(a['action_source'])); fpss.add(int(a['fps']))
            for key in C.CAMERAS:
                shapes.setdefault(key, decode(f[key][0]).shape)
            use.append(p)
    if args.max_episodes:
        use = use[:args.max_episodes]
    if not use:
        sys.exit('no episodes to convert (all failed, or none of that --kind)')
    if len(sources) > 1:
        sys.exit(f'{raw} mixes action sources {sorted(sources)}: convert them into separate datasets')
    if len(fpss) > 1:
        sys.exit(f'{raw} mixes frame rates {sorted(fpss)}')
    fps = fpss.pop()
    print(f'{len(use)} of {len(files)} episode(s), {fps} fps, action source {sources.pop()}, cameras '
          + ', '.join(f'{k.split(".")[-1]} {s[1]}x{s[0]}' for k, s in shapes.items()))
    print(f'writing {out}  (repo_id {repo_id})')

    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    features = {
        'observation.state': {'dtype': 'float32', 'shape': (len(C.JOINT_NAMES),), 'names': C.JOINT_NAMES},
        'action': {'dtype': 'float32', 'shape': (len(C.JOINT_NAMES),), 'names': C.JOINT_NAMES},
    }
    for key, shp in shapes.items():
        features[key] = {'dtype': 'video', 'shape': shp, 'names': ['height', 'width', 'channel']}
    ds = LeRobotDataset.create(repo_id=repo_id, root=out, fps=fps, features=features, robot_type=C.ROBOT_TYPE)
    frames = 0
    for i, p in enumerate(use):
        with h5py.File(p, 'r') as f:
            task = str(f.attrs['task'])
            st, ac = f['observation.state'][:], f['action'][:]
            n = len(st)
            for key in C.CAMERAS:
                if len(f[key]) != n:
                    sys.exit(f'{p.name}: {key} has {len(f[key])} frames, state {n}')
            print(f'[{i + 1}/{len(use)}] {p.name}: {n} frames, "{task}"', flush=True)
            for t in range(n):
                fr = {'observation.state': st[t].astype(np.float32), 'action': ac[t].astype(np.float32), 'task': task}
                for key in C.CAMERAS:
                    fr[key] = decode(f[key][t])
                ds.add_frame(fr)
            frames += n
        ds.save_episode()
    ds.finalize()                       # writes the parquet footers: without it the dataset is invalid (LeRobot >= 0.4)
    if args.push_to_hub:
        ds.push_to_hub()
    print(f'done: {len(use)} episodes, {frames} frames ({frames / fps / 60:.1f} min) in {out}')


if __name__ == '__main__':
    main()
