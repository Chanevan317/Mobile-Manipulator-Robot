"""Import the robot into Isaac Sim (5.x), set up drives, and play a looping motion demo.

Run with Isaac Sim's Python, from this package folder (or pass --urdf PATH):
    <isaac-sim>/python.sh scripts/isaac_import_and_test.py
Options:
    --headless
    --save robot.usd     write the converted USD (drives included) and exit
    --self_collision     robot parts collide with each other (see self_collision.py)
    --duration S         stop after S seconds (default: run until the window is closed)

The base is free on the floor. The demo moves everything at once, staying inside each joint's speed limit:
the lift goes up and down (one cycle about 52 s), each arm swings outward and folds back without crossing
the middle (so the arms never touch), the wrists pitch and roll, and the jaws open and close.
Every 5 s it prints where the base is, the lift height against its target, and the worst arm tracking error.
"""
import argparse, math, os, sys
ap = argparse.ArgumentParser()
here = os.path.dirname(os.path.abspath(__file__))
ap.add_argument('--layout', default='prototype', choices=('prototype',), help='robot layout (urdf/mobile_manipulator_<layout>.urdf)')
ap.add_argument('--urdf', default=None, help='URDF path (overrides --layout)')
ap.add_argument('--headless', action='store_true')
ap.add_argument('--save', default='')
ap.add_argument('--self_collision', action='store_true', help='robot parts collide with each other (see self_collision.py)')
ap.add_argument('--duration', type=float, default=0.0, help='stop after this many seconds (0 = run until the window is closed)')
args = ap.parse_args()
if args.urdf is None:
    args.urdf = os.path.join(here, '..', 'urdf', f'mobile_manipulator_{args.layout}.urdf')

from isaacsim import SimulationApp
app = SimulationApp({'headless': args.headless})

import numpy as np
import omni.kit.commands, omni.usd
from pxr import UsdLux, Sdf
from isaacsim.asset.importer.urdf import _urdf
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation as Articulation
from isaacsim.core.utils.types import ArticulationAction
sys.path.insert(0, here)
from drives import apply_drives, rate_limit

CTRL_HZ = 60
world = World(stage_units_in_meters=1.0, physics_dt=1 / 240.0, rendering_dt=1 / CTRL_HZ)
world.scene.add_ground_plane()
stage = omni.usd.get_context().get_stage()
UsdLux.DistantLight.Define(stage, Sdf.Path('/DistantLight')).CreateIntensityAttr(3000)

_, cfg = omni.kit.commands.execute('URDFCreateImportConfig')
cfg.merge_fixed_joints = False      # keep camera / tcp frames as prims
cfg.fix_base = False                # it's a mobile robot; set True to pin it while testing the arms
cfg.make_default_prim = True
cfg.self_collision = False          # set below with the right filtered pairs when --self_collision is given
cfg.import_inertia_tensor = True    # use the computed masses and inertias
cfg.convex_decomp = False           # collision meshes are already convex pieces
cfg.distance_scale = 1.0
cfg.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
_, robot_path = omni.kit.commands.execute('URDFParseAndImportFile', urdf_path=os.path.abspath(args.urdf),
                                          import_config=cfg, get_articulation_root=True)
print('imported at', robot_path)

# Drives: force drives with the gains from drives.py (the single place where gains live).
joints = apply_drives(stage, robot_path)
print(len(joints), 'driven/passive joints set up')
if args.self_collision:
    from self_collision import setup_self_collision
    print('self-collision on,', setup_self_collision(stage, os.path.abspath(args.urdf)), 'pairs ignored')

if args.save:
    stage.Export(args.save); print('saved', args.save); app.close(); raise SystemExit

robot = world.scene.add(Articulation(prim_path=robot_path, name='mobile_manipulator'))
world.reset()
robot.initialize()
if not args.headless:
    from isaacsim.core.utils.viewports import set_camera_view
    set_camera_view(eye=np.array([1.8, 1.2, 1.4]), target=np.array([0.3, 0.0, 0.6]))

names = list(robot.dof_names)
n = len(names)
idx = {nm: i for i, nm in enumerate(names)}
driven = [i for i, nm in enumerate(names) if 'roller' not in nm and not nm.startswith('wheel')]
arm = [i for i in driven if names[i] != 'lift' and 'finger' not in names[i]]
SUBSTEPS = max(1, round((1.0 / CTRL_HZ) / world.get_physics_dt()))
vmax = np.array(robot.dof_properties['maxVelocity'], dtype=float)


def tick():
    """One control period (1/60 s). world.step(render=False) is only one physics step."""
    if args.headless:
        for _ in range(SUBSTEPS):
            world.step(render=False)
    else:
        world.step(render=True)


def targets(t):
    q = np.zeros(n)
    q[idx['lift']] = 0.30 - 0.30 * math.cos(0.12 * t)             # 0 .. 0.60 m, peak 0.036 m/s (limit 0.05)
    u = 0.5 - 0.5 * math.cos(0.6 * t)                              # 0 .. 1, starts at 0 (the zero pose)
    for side, sg in (('left', 1), ('right', -1)):
        q[idx[f'{side}_shoulder_pan']] = sg * 0.8 * u                # swing outward ...
        q[idx[f'{side}_elbow_pan']] = -sg * 1.2 * u                  # ... fold the forearm back in ...
        q[idx[f'{side}_wrist_pan']] = sg * 0.4 * u                   # ... and keep the hand pointing forward
        q[idx[f'{side}_wrist_pitch']] = 1.2 * u
        q[idx[f'{side}_wrist_roll']] = 1.2 * math.sin(0.6 * t)
        g = 0.0225 - 0.0225 * math.cos(1.5 * t)                     # jaws 0 .. 45 mm
        q[idx[f'{side}_finger_right']] = q[idx[f'{side}_finger_left']] = g   # both jaws get the same command
    return q


p0 = None
worst = 0.0
k = 0
q_t = robot.get_joint_positions().copy()
while app.is_running() and (args.duration <= 0 or k < args.duration * CTRL_HZ):
    t = k / CTRL_HZ
    q_t = rate_limit(q_t, targets(t), vmax, 1.0 / CTRL_HZ)
    robot.apply_action(ArticulationAction(joint_positions=q_t[driven], joint_indices=np.array(driven)))
    tick(); k += 1
    q = robot.get_joint_positions()
    if not np.all(np.isfinite(q)):
        print('BLEW UP: NaN in the joint positions'); break
    worst = max(worst, float(np.max(np.abs(q[arm] - q_t[arm]))))
    p = np.array(robot.get_world_pose()[0], dtype=float)
    if p0 is None:
        p0 = p
    if k % (5 * CTRL_HZ) == 0:
        print(f't {k / CTRL_HZ:5.1f} s  base moved {np.linalg.norm(p - p0) * 1000:5.1f} mm (height {p[2]:+.3f} m)  '
              f'lift {q[idx["lift"]]:.3f} m (target {q_t[idx["lift"]]:.3f})  '
              f'worst arm tracking error so far {math.degrees(worst):4.1f} deg')
app.close()
