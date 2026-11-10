"""Grasp test: each arm picks a cube from a table, lifts it, swings it, and puts it back.

The base is fixed, the arms are at their zero pose (45 deg toward the rear,
reaching past the base sides), and the grippers are level. A 60 mm, 150 g cube sits on a small table under each TCP. Sequence (both arms at the same time):
  1. lift at 0.10 m, jaws open          gripper above the cube
  2. lower the lift to 0                jaws come down around the cube
  3. close the jaws                     (both jaws get the same target, 0)
  4. lift to 0.20 m                     cube leaves the table
  5. swing both shoulders 30 deg outward and back
  6. lower to 0, open the jaws, raise   cube is back on the table
It reports how far the cube slipped relative to the TCP, the jaw opening while it was held, the
grip force, and where the cube ended up. All materials use PhysX's default friction (0.5).

Run with Isaac Sim's Python, from the package folder:
    <isaac-sim>/python.sh scripts/isaac_grasp_test.py                    # window: watch it
    <isaac-sim>/python.sh scripts/isaac_grasp_test.py --headless
    <isaac-sim>/python.sh scripts/isaac_grasp_test.py --mass 0.5 --size 0.05 --no_self_collision
"""
import argparse, math, os, sys

ap = argparse.ArgumentParser()
here = os.path.dirname(os.path.abspath(__file__))
ap.add_argument('--layout', default='prototype', choices=('prototype',), help='robot layout (urdf/mobile_manipulator_<layout>.urdf)')
ap.add_argument('--urdf', default=None, help='URDF path (overrides --layout)')
ap.add_argument('--headless', action='store_true')
ap.add_argument('--size', type=float, default=0.06, help='cube edge (m); the jaws open 90 mm')
ap.add_argument('--mass', type=float, default=0.15, help='cube mass (kg)')
ap.add_argument('--no_self_collision', action='store_true', help='robot parts do not collide with each other')
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
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
from isaacsim.core.prims import SingleArticulation as Articulation, SingleXFormPrim
from isaacsim.core.utils.types import ArticulationAction
sys.path.insert(0, here)
from drives import apply_drives
import scara_ik as K
K.use(args.layout)          # with --urdf, pass the matching --layout too

CTRL_HZ = 60
world = World(stage_units_in_meters=1.0, physics_dt=1 / 240.0, rendering_dt=1 / CTRL_HZ)
world.scene.add_ground_plane()
stage = omni.usd.get_context().get_stage()
UsdLux.DistantLight.Define(stage, Sdf.Path('/DistantLight')).CreateIntensityAttr(3000)

_, cfg = omni.kit.commands.execute('URDFCreateImportConfig')
cfg.merge_fixed_joints = False
cfg.fix_base = True
cfg.make_default_prim = True
cfg.self_collision = False
cfg.import_inertia_tensor = True
cfg.convex_decomp = False
cfg.distance_scale = 1.0
cfg.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
_, art_path = omni.kit.commands.execute('URDFParseAndImportFile', urdf_path=os.path.abspath(args.urdf),
                                        import_config=cfg, get_articulation_root=True)
apply_drives(stage, art_path)
if not args.no_self_collision:
    from self_collision import setup_self_collision
    print('self-collision on,', setup_self_collision(stage, os.path.abspath(args.urdf)), 'pairs ignored')

# ---------------------------------------------------------------- scene
# At the zero pose with the lift at 0 each TCP sits 0.585 m out along its arm's zero heading, at z 0.411. The jaws
# reach 20 mm below the TCP (z 0.391) and the hand 22 mm below (z 0.389, up to 0.047 m back along the arm); the wrist
# goes lower further back (more than 0.106 m back). Each arm gets its own small table that starts in front of the wrist.
TCP_Z0 = 0.4109
TABLE_TOP = 0.380                          # 10 mm under the jaws, 9 mm under the hand
T_BACK, T_FWD, T_W = 0.07, 0.25, 0.20      # table extent behind / ahead of the TCP along the arm, and its width
CUBE_BACK = 0.01                           # cube centre 10 mm behind the TCP: the jaws overlap it by about 50 mm
h = args.size


def yaw_quat(a):
    return np.array([math.cos(a / 2), 0.0, 0.0, math.sin(a / 2)])


ZERO = {'shoulder_pan': 0.0, 'elbow_pan': 0.0, 'wrist_pan': 0.0, 'wrist_pitch': 0.0, 'lift': 0.0}
CUBE0 = {}
cubes = {}
for side in ('left', 'right'):
    tx, ty, _, psi = K.fk(side, ZERO); u = np.array([math.cos(psi), math.sin(psi)])
    c = np.array([tx, ty]) + u * (T_FWD - T_BACK) / 2
    world.scene.add(FixedCuboid(prim_path=f'/World/table_{side}', name=f'table_{side}',
                                position=np.array([c[0], c[1], TABLE_TOP / 2]), orientation=yaw_quat(psi),
                                scale=np.array([T_BACK + T_FWD, T_W, TABLE_TOP]), color=np.array([0.55, 0.42, 0.30])))
    CUBE0[side] = (np.array([tx, ty]) - u * CUBE_BACK, psi)
    p0 = CUBE0[side][0]
    cubes[side] = world.scene.add(DynamicCuboid(prim_path=f'/World/cube_{side}', name=f'cube_{side}',
                                                position=np.array([p0[0], p0[1], TABLE_TOP + h / 2]), orientation=yaw_quat(psi),
                                                scale=np.array([h, h, h]), mass=args.mass,
                                                color=np.array([0.2, 0.6, 0.9]) if side == 'left' else np.array([0.9, 0.4, 0.2])))
robot = world.scene.add(Articulation(prim_path=art_path, name='mobile_manipulator'))
tcp = {s: SingleXFormPrim(f'/mobile_manipulator/{s}_tcp') for s in ('left', 'right')}
world.reset()
robot.initialize()
if not args.headless:
    from isaacsim.core.utils.viewports import set_camera_view
    set_camera_view(eye=np.array([1.6, 0.9, 1.0]), target=np.array([0.65, 0.0, 0.45]))

names = list(robot.dof_names)
n = len(names)
idx = {nm: i for i, nm in enumerate(names)}
SUBSTEPS = max(1, round((1.0 / CTRL_HZ) / world.get_physics_dt()))


def tick():
    if args.headless:
        for _ in range(SUBSTEPS):
            world.step(render=False)
    else:
        world.step(render=True)


q_cmd = np.zeros(n)


def set_cmd(lift=None, jaw=None, pan=None):
    if lift is not None:
        q_cmd[idx['lift']] = lift
    if jaw is not None:
        for s in ('left', 'right'):
            q_cmd[idx[f'{s}_finger_right']] = jaw
            q_cmd[idx[f'{s}_finger_left']] = jaw
    if pan is not None:
        q_cmd[idx['left_shoulder_pan']] = pan
        q_cmd[idx['right_shoulder_pan']] = -pan


log = []


def phase(label, seconds, lift=None, jaw=None, pan=None):
    """Ease the commanded values to the new targets over `seconds`, logging cube vs TCP every tick."""
    start = q_cmd.copy()
    set_cmd(lift, jaw, pan); goal = q_cmd.copy()
    steps = max(1, int(seconds * CTRL_HZ))
    for k in range(steps):
        s = 0.5 - 0.5 * math.cos(math.pi * (k + 1) / steps)
        q_cmd[:] = start + (goal - start) * s
        robot.apply_action(ArticulationAction(joint_positions=q_cmd))
        tick()
        q = robot.get_joint_positions()
        row = {'phase': label, 'q': q.copy(), 'finite': bool(np.all(np.isfinite(q)))}
        for sd in ('left', 'right'):
            cp, _ = cubes[sd].get_world_pose(); tp, tq = tcp[sd].get_world_pose()
            row[sd] = (np.array(cp, float), np.array(tp, float), np.array(tq, float))
        log.append(row)
        if not app.is_running():
            raise SystemExit
    q_cmd[:] = goal


def to_tcp(cube_p, tcp_p, tcp_q):
    """Cube centre in the TCP frame (quaternion w, x, y, z)."""
    w, x, y, z = tcp_q
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    return R.T @ (cube_p - tcp_p)


# ---------------------------------------------------------------- run
start = np.zeros(n); start[idx['lift']] = 0.10
for s in ('left', 'right'):
    start[idx[f'{s}_finger_right']] = start[idx[f'{s}_finger_left']] = 0.045
robot.set_joint_positions(start); q_cmd[:] = start
# world.reset() starts the robot at the zero pose, where the closed jaws sit inside the cubes and kick them
# away. Put the cubes back now that the gripper is open and above them.
for sd in ('left', 'right'):
    p0, psi = CUBE0[sd]
    cubes[sd].set_world_pose(position=np.array([p0[0], p0[1], TABLE_TOP + h / 2]), orientation=yaw_quat(psi))
    cubes[sd].set_linear_velocity(np.zeros(3)); cubes[sd].set_angular_velocity(np.zeros(3))
phase('settle', 1.0)
phase('lower onto cube', 3.5, lift=0.0)
phase('close jaws', 1.5, jaw=0.0)
phase('grip', 1.0)
phase('lift', 7.0, lift=0.20)
phase('hold', 2.0)
phase('swing out', 2.5, pan=math.radians(30))
phase('swing back', 2.5, pan=0.0)
phase('lower', 7.0, lift=0.0)
phase('open jaws', 1.5, jaw=0.045)
phase('raise', 3.0, lift=0.10)
phase('settle', 1.0)

# ---------------------------------------------------------------- report
held_phases = ('lift', 'hold', 'swing out', 'swing back', 'lower')
print(f'\nGrasp test ({args.layout} layout): cube {h*1000:.0f} mm, {args.mass*1000:.0f} g, self-collision '
      f"{'off' if args.no_self_collision else 'on'}, default friction 0.5\n")
ok_all = all(r['finite'] for r in log)
for sd in ('left', 'right'):
    grip = [r for r in log if r['phase'] == 'grip'][-1]
    ref = to_tcp(*grip[sd])
    held = [r for r in log if r['phase'] in held_phases]
    slip = max(np.linalg.norm(to_tcp(*r[sd]) - ref) for r in held) * 1000
    top = [r for r in log if r['phase'] == 'hold'][-1]
    rise = (top[sd][0][2] - (TABLE_TOP + h / 2)) * 1000
    jaw = grip['q'][idx[f'{sd}_finger_right']] + grip['q'][idx[f'{sd}_finger_left']]
    force = 2000.0 * grip['q'][idx[f'{sd}_finger_right']]          # kp * squeeze, per jaw
    end = log[-1][sd][0]
    placed = np.linalg.norm(end[:2] - CUBE0[sd][0]) * 1000
    on_table = abs(end[2] - (TABLE_TOP + h / 2)) < 0.005
    ok = ok_all and slip < 5 and rise > 180 and abs(jaw * 1000 - h * 1000) < 3 and on_table and placed < 10
    print(f'{sd:5s} arm: jaw gap while gripping {jaw*1000:5.1f} mm (cube {h*1000:.0f}), grip force ~{force:4.1f} N per jaw, '
          f'cube raised {rise:5.1f} mm (lift 200; the rest is arm sag), slip in gripper {slip:4.1f} mm, '
          f"placed {placed:4.1f} mm from start {'on the table' if on_table else 'NOT on the table'}  {'OK' if ok else 'FAIL'}")
print('\nPass: slip < 5 mm (the grip), cube raised > 180 mm (it came up with the gripper; the shortfall from 200 mm is\n'
      'the arm sagging under the load, mostly wrist pitch), jaw gap = cube size within 3 mm, cube back on the table within 10 mm.')
if not ok_all:
    print('BLEW UP: NaN in the joint positions')
if not args.headless:
    print('Window stays open; close it to exit.')
    while app.is_running():
        world.step(render=True)
app.close()
