"""Download the NVIDIA SimReady assets listed in assets.py into scenes/assets/simready/ (git-ignored).

    python scripts/scenes/scene_v0/fetch_assets.py            # download / update (skips files already there)
    python scripts/scenes/scene_v0/fetch_assets.py --check    # only report what is present / missing

Needs `huggingface_hub` (in env_isaaclab). The dataset is public (no login needed; a login raises the rate limit).
Each package is downloaded whole except its thumbnails (about 525 MB for the current list).
License: CC-BY-4.0 (attribution in CREDITS.md).
"""
import argparse, os, sys
here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(here, '..', '..', 'common')); import repo_paths as RP   # every script folder + repo paths
sys.path.insert(0, here)
import assets as SA

ROOT = RP.SIMREADY


def check():
    missing = [p for p in SA.ALL if SA.usd_path(ROOT, p) is None]
    for p in SA.ALL:
        print(f"  {'ok     ' if p not in missing else 'MISSING'} {p}")
    return missing


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--check', action='store_true')
    a = ap.parse_args()
    if not a.check:
        from huggingface_hub import snapshot_download
        # whole packages: the layout differs between packages (older ones: sm_*.usd + Textures/; newer ones: sm_*.usda,
        # textures only in source_assets/, materials/ + runnables/), so nothing but the thumbnails is left out
        allow = [f'{p}/**' for p in SA.ALL]
        os.makedirs(ROOT, exist_ok=True)
        snapshot_download(SA.REPO, repo_type='dataset', local_dir=ROOT, allow_patterns=allow,
                          ignore_patterns=['*/.thumbs/*'])
    missing = check()
    print(f'{len(SA.ALL) - len(missing)} / {len(SA.ALL)} packages in {ROOT}')
    sys.exit(1 if missing else 0)


if __name__ == '__main__':
    main()
