"""Save scene_v0 as a USD file you can open straight in Isaac Sim (File > Open): scene_v0.usda next to this script.

    OMNI_KIT_ACCEPT_EULA=YES conda run --no-capture-output -n env_isaaclab python scripts/scenes/scene_v0/save_scene.py
    ... --fill 0.5 --seed 3 --out other.usda

The file is the scene only (no robot): room (3 walls + a floor), lights, furniture, the 15 task products and the filler
products for that fill / seed, with every physics fix of scene.py, plus a physics scene (gravity, 240 Hz). It REFERENCES
the downloaded assets in scenes/assets/simready by relative path, so download them first (fetch_assets.py); the file
itself holds no NVIDIA geometry or textures. The robot scripts build the same scene live (check_scene.py), not from it.
"""
import argparse, os, sys
here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(here, '..', '..', 'common')); import repo_paths as RP   # every script folder + repo paths
ap = argparse.ArgumentParser()
ap.add_argument('--fill', type=float, default=1.0)
ap.add_argument('--seed', type=int, default=0)
ap.add_argument('--out', default=os.path.join(RP.scene_dir('scene_v0'), 'scene_v0.usda'))
args = ap.parse_args()

from isaacsim import SimulationApp            # Isaac's pxr only loads inside a running Kit app
app = SimulationApp({'headless': True})
from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Gf
sys.path.insert(0, here)
import scene as SC

stage = Usd.Stage.CreateInMemory()
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z); UsdGeom.SetStageMetersPerUnit(stage, 1.0)
world = UsdGeom.Xform.Define(stage, '/World'); stage.SetDefaultPrim(world.GetPrim())
ps = UsdPhysics.Scene.Define(stage, '/World/physicsScene')
ps.CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1)); ps.CreateGravityMagnitudeAttr(9.81)
ps.GetPrim().CreateAttribute('physxScene:timeStepsPerSecond', Sdf.ValueTypeNames.UInt).Set(240)
items = SC.build(stage, fill=args.fill, seed=args.seed, floor=True)

out = os.path.abspath(args.out); d = os.path.dirname(out)
stage.GetRootLayer().Export(out)
layer = Sdf.Layer.FindOrOpen(out)
for dep in layer.GetCompositionAssetDependencies():           # absolute asset paths -> relative to the file
    if os.path.isabs(dep):
        layer.UpdateCompositionAssetDependency(dep, os.path.relpath(dep, d))
layer.Save()
n = {r: sum(1 for it in items.values() if it['role'] == r) for r in ('product', 'filler')}
print(f'saved {out}: {n["product"]} task products, {n["filler"]} fillers (fill {args.fill}, seed {args.seed}), '
      f'{os.path.getsize(out) / 1e6:.1f} MB', flush=True)
app.close()
