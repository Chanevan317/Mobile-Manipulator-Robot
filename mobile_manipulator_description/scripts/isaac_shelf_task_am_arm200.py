"""Shelf-to-shelf task: drive to shelf A, pick a product, put it in the tote, drive to shelf B, pick it from
the tote and place it on shelf B.

The robot (free base, mecanum wheels, self-collision on) starts at the origin facing +x.
  shelf A: front edge at x = 2.0, faces the robot (-x); board top 0.70 m; the product stands on it
  shelf B: front edge at y = -1.5, faces +y; board top 0.50 m
  product: 45 x 45 x 70 mm box, 150 g
The left arm does the work; the right arm stays tucked (hand hanging over the right half of the tote).

Steps:
   1. drive to shelf A          closed-loop on the base pose, mecanum wheel mixing
   2. pick from shelf A         gripper level, straight in, close, lift 30 mm, straight out
   3. place in the tote         through the ready pose (arm straight ahead), tilt down, lower, release
   4. drive to shelf B          turns 90 deg on the way
   5. pick from the tote        straight down onto the item, close, up
   6. place on shelf B          through the ready pose, gripper level, straight in, release, out
   7. park
Arm targets are worked out from where the base actually stopped and where the item actually is (read from
the simulation; a real robot would get them from its cameras). Every joint command is rate-limited.
The poses were checked collision-free with generator/urdf_selfcol.py (left arm, right arm parked).

Run with Isaac Sim's Python, from the package folder:
    <isaac-sim>/python.sh scripts/isaac_shelf_task.py               # window: watch it (about 2 minutes)
    <isaac-sim>/python.sh scripts/isaac_shelf_task.py --headless
    TASK_DEBUG=1 ... isaac_shelf_task.py      # also print the item position relative to the gripper
"""
import argparse, copy, math, os, sys, xml.etree.ElementTree as ET
from pathlib import Path

ap = argparse.ArgumentParser()
here = os.path.dirname(os.path.abspath(__file__))
pkg = os.path.abspath(os.path.join(here, '..'))
ap.add_argument('--urdf', default=os.path.join(pkg, 'urdf', 'mobile_manipulator_v0.urdf'))   # derived from the v0 layout
ap.add_argument('--am-urdf', default=os.path.join(pkg, 'third_party', 'AM-ARM', 'am-arm200', 'urdf', 'urdf', 'am-arm200.urdf'))
ap.add_argument('--headless', action='store_true')
ap.add_argument('--no_self_collision', action='store_true')
ap.add_argument('--stop-after-pick', action='store_true', help='stop immediately after the first shelf grasp attempt')
args = ap.parse_args()


def compose_dual_arm_urdf(base_urdf, arm_urdf):
    """Replace the project's two SCARA branches with two official AM-ARM200 branches."""
    base_tree = ET.parse(base_urdf)
    base_root = base_tree.getroot()
    arm_root = ET.parse(arm_urdf).getroot()
    meshes = os.path.abspath(os.path.join(os.path.dirname(arm_urdf), '..', 'am-arm200-meshes'))
    if not os.path.isdir(meshes):
        raise FileNotFoundError(f'AM-ARM200 mesh directory not found: {meshes}')

    # Remove each existing arm and every attached wrist-camera/TCP link from the chassis model.
    joints = list(base_root.findall('joint'))
    child_parent = {j.find('child').get('link'): j.find('parent').get('link') for j in joints}
    remove_links = set()
    for root_link in ('left_base', 'right_base'):
        remove_links.add(root_link)
        changed = True
        while changed:
            changed = False
            for child, parent in child_parent.items():
                if parent in remove_links and child not in remove_links:
                    remove_links.add(child)
                    changed = True
    for j in joints:
        if j.find('child').get('link') in remove_links or j.find('parent').get('link') in remove_links:
            base_root.remove(j)
    for link in list(base_root.findall('link')):
        if link.get('name') in remove_links:
            base_root.remove(link)

    for side, y in (('left', 0.20), ('right', -0.20)):
        rename = lambda s: s.replace('left_', side + '_')
        for src in arm_root.findall('link'):
            link = copy.deepcopy(src)
            link.set('name', rename(link.get('name')))
            if src.get('name') == 'left_camera':
                # The official CAD export assigns 1.27 kg to a 36 x 37 x 19 mm camera mesh.
                # Use a plausible 50 g camera + printed mount mass for this prototype simulation.
                mass = link.find('inertial/mass')
                inertia = link.find('inertial/inertia')
                if mass is not None and inertia is not None:
                    scale = 0.05 / float(mass.get('value'))
                    mass.set('value', '0.05')
                    for key in ('ixx', 'ixy', 'ixz', 'iyy', 'iyz', 'izz'):
                        inertia.set(key, str(float(inertia.get(key)) * scale))
            for mesh in link.findall('.//mesh'):
                mesh.set('filename', os.path.join(meshes, os.path.basename(mesh.get('filename'))))
            base_root.append(link)
        for src in arm_root.findall('joint'):
            joint = copy.deepcopy(src)
            joint.set('name', rename(joint.get('name')))
            joint.find('parent').set('link', rename(joint.find('parent').get('link')))
            joint.find('child').set('link', rename(joint.find('child').get('link')))
            joint_type = joint.get('type')
            if joint_type in ('revolute', 'continuous', 'prismatic'):
                limit = joint.find('limit')
                if joint.get('name').endswith('_gripper'):
                    # The official follower gripper closes by rotating its single moving jaw.
                    limit.set('lower', '0.0'); limit.set('upper', '1.2')
                    limit.set('effort', '1.9'); limit.set('velocity', '2.0')
                else:
                    limit.set('lower', '-3.14'); limit.set('upper', '3.14')
                    limit.set('effort', '4.9' if any(k in joint.get('name') for k in ('shoulder', 'elbow')) else '1.9')
                    limit.set('velocity', '2.5')
            base_root.append(joint)
        mount = ET.Element('joint', {'name': side + '_am_arm_mount', 'type': 'fixed'})
        ET.SubElement(mount, 'parent', {'link': 'carriage'})
        ET.SubElement(mount, 'child', {'link': side + '_Base'})
        ET.SubElement(mount, 'origin', {'xyz': f'0.055 {y} 0.0655', 'rpy': '0 0 1.57079632679'})
        base_root.append(mount)

    # Task TCP is the midpoint near the AM-ARM200 finger tips (the official model's jaw axis is local -Y).
    tcp = ET.Element('link', {'name': 'left_am_tcp'})
    inertial = ET.SubElement(tcp, 'inertial')
    ET.SubElement(inertial, 'mass', {'value': '0.001'})
    ET.SubElement(inertial, 'inertia', {'ixx': '1e-8', 'ixy': '0', 'ixz': '0', 'iyy': '1e-8', 'iyz': '0', 'izz': '1e-8'})
    base_root.append(tcp)
    fixed = ET.Element('joint', {'name': 'left_am_tcp_fixed', 'type': 'fixed'})
    ET.SubElement(fixed, 'parent', {'link': 'left_Fixed_Jaw'})
    ET.SubElement(fixed, 'child', {'link': 'left_am_tcp'})
    ET.SubElement(fixed, 'origin', {'xyz': '0 -0.08 0', 'rpy': '0 0 0'})
    base_root.append(fixed)

    out = os.path.join(pkg, 'urdf', 'mobile_manipulator_am_arm200.urdf')
    base_tree.write(out, encoding='utf-8', xml_declaration=True)
    return out


args.urdf = compose_dual_arm_urdf(os.path.abspath(args.urdf), os.path.abspath(args.am_urdf))

from isaacsim import SimulationApp
app = SimulationApp({'headless': args.headless})

import numpy as np
import omni.kit.commands, omni.usd
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from pxr import UsdLux, Sdf
from isaacsim.asset.importer.urdf import _urdf
from isaacsim.core.api import World
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
from isaacsim.core.prims import SingleArticulation as Articulation, SingleXFormPrim
from isaacsim.core.utils.types import ArticulationAction
sys.path.insert(0, here)
from drives import apply_drives, rate_limit


class AMArmIK:
    """Numerical FK/IK for the six official AM-ARM200 arm joints."""
    JOINTS = ('shoulder_pan', 'shoulder_lift', 'elbow_flex', 'wrist_flex', 'wrist_yaw_joint', 'wrist_roll')
    TCP_OFFSET = np.array([0.0, -0.08, 0.0])

    def __init__(self, urdf_path):
        root = ET.parse(urdf_path).getroot()
        self.parents = {j.find('child').get('link'): j for j in root.findall('joint')}
        self.chain = []
        link = 'left_Fixed_Jaw'
        while link != 'left_Base':
            joint = self.parents[link]
            self.chain.append(joint)
            link = joint.find('parent').get('link')
        self.chain.reverse()
        self.seed = {s: np.zeros(6) for s in ('left', 'right')}
        self.lift = 0.20
        self.tool_roll = math.pi

    @staticmethod
    def _origin(joint):
        o = joint.find('origin')
        xyz = np.fromstring(o.get('xyz', '0 0 0'), sep=' ')
        rpy = np.fromstring(o.get('rpy', '0 0 0'), sep=' ')
        T = np.eye(4)
        T[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
        T[:3, 3] = xyz
        return T

    def _fk(self, q):
        T = np.eye(4)
        for joint in self.chain:
            T = T @ self._origin(joint)
            name = joint.get('name')
            if joint.get('type') in ('revolute', 'continuous'):
                axis = np.fromstring(joint.find('axis').get('xyz', '0 0 1'), sep=' ')
                Trot = np.eye(4)
                Trot[:3, :3] = Rotation.from_rotvec(axis * q[self.JOINTS.index(name[5:])]).as_matrix()
                T = T @ Trot
        T[:3, 3] += T[:3, :3] @ self.TCP_OFFSET
        return T

    def ik(self, side, x, y, z, yaw, pitch=0.0, elbow=None):
        lift = self.lift
        mount_p = np.array([0.115, 0.20 if side == 'left' else -0.20, 0.4855 + lift])
        mount_R = Rotation.from_euler('z', math.pi / 2)
        target_p = mount_R.inv().apply(np.array([x, y, z]) - mount_p)
        target_R_base = (Rotation.from_euler('z', yaw + math.pi / 2) *
                         Rotation.from_euler('x', pitch) * Rotation.from_euler('y', self.tool_roll))
        target_R = mount_R.inv() * target_R_base
        seed = self.seed[side].copy()

        def residual(q):
            T = self._fk(q)
            pos_err = T[:3, 3] - target_p
            rot_err = (target_R.inv() * Rotation.from_matrix(T[:3, :3])).as_rotvec()
            return np.r_[pos_err, 0.12 * rot_err]

        starts = (seed, np.zeros(6), np.array([0, -1, 1, 1, 0, 0]), np.array([0, 1, -1, 1, 0, 0]))
        q = None
        for start in starts:
            sol = least_squares(residual, start, bounds=(-2.95, 2.95), max_nfev=70,
                                ftol=2e-4, xtol=2e-4, gtol=2e-4)
            T = self._fk(sol.x)
            pos_error = np.linalg.norm(T[:3, 3] - target_p)
            angle_error = np.linalg.norm((target_R.inv() * Rotation.from_matrix(T[:3, :3])).as_rotvec())
            if sol.success and pos_error <= 0.018 and angle_error <= 0.20:
                q = sol.x
                break
        if q is None:
            return None
        self.seed[side] = q.copy()
        result = {f'{side}_{name}': float(v) for name, v in zip(self.JOINTS, q)}
        result['lift'] = lift
        return result


AM = AMArmIK(os.path.abspath(args.am_urdf))

D = math.radians
P = math.pi / 2
CTRL_HZ = 60
DT = 1.0 / CTRL_HZ

# ---------------------------------------------------------------- task geometry
ITEM = np.array([0.045, 0.045, 0.070])         # product box (x, y, z size), stands upright on the shelf
ITEM_MASS = 0.15
SHELF_A = dict(edge_x=2.0, y=1.0, board=0.70)   # faces -x
SHELF_B = dict(edge_y=-1.5, x=0.8, board=0.50)  # faces +y
REACH = np.array([0.46, 0.20])                  # closer base stop for AM-ARM200's shorter reach
ITEM_A = np.array([SHELF_A['edge_x'] + 0.04, SHELF_A['y'], SHELF_A['board'] + ITEM[2] / 2])
ITEM_B = np.array([SHELF_B['x'], SHELF_B['edge_y'] - 0.04, SHELF_B['board'] + ITEM[2] / 2])   # target on shelf B


def goal_for(item_xy, yaw):
    """Base pose (x, y, yaw) that puts item_xy at REACH in the base frame."""
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([item_xy[0] - (c * REACH[0] - s * REACH[1]), item_xy[1] - (s * REACH[0] + c * REACH[1]), yaw])


GOAL_A = goal_for(ITEM_A[:2], 0.0)
GOAL_B = goal_for(ITEM_B[:2], -P)

# Arm poses in the base frame (TCP x, y, z, yaw, pitch, elbow), checked with urdf_selfcol.py.
ZT = 0.35                                        # higher tote drop point; the package settles to the tote floor
TOTE = dict(x=0.17, y=0.08, yaw=D(240), elbow=-1)   # AM-ARM200-compatible point inside the tote
RPARK = AM.ik('right', -0.05, -0.30, 0.96, 0.0, P)
LSTOW = AM.ik('left', -0.05, 0.36, 0.88, -P, P)
RSTOW = RPARK
# Ready pose: arm straight out to the left, used to retreat from shelf B before stowing.
READY = AM.ik('left', 0.37, 0.16, 0.90, 0.0, 0.0)
AM.seed['left'] = np.array([LSTOW[f'left_{name}'] for name in AM.JOINTS])
AM.seed['right'] = np.array([RSTOW[f'right_{name}'] for name in AM.JOINTS])
GRASP_DZ = 0.020                              # TCP 3 mm below the item centre (jaws 12 mm above the board)
APPROACH = 0.12                                # straight-line approach / retreat distance (m)
JAW_OPEN, JAW_CLOSED = 0.0, 1.05               # official AM-ARM200 single rotating jaw

# ---------------------------------------------------------------- scene
world = World(stage_units_in_meters=1.0, physics_dt=1 / 240.0, rendering_dt=DT)
world.scene.add_ground_plane()
stage = omni.usd.get_context().get_stage()
UsdLux.DistantLight.Define(stage, Sdf.Path('/DistantLight')).CreateIntensityAttr(3000)
UsdLux.DomeLight.Define(stage, Sdf.Path('/DomeLight')).CreateIntensityAttr(500)

_, cfg = omni.kit.commands.execute('URDFCreateImportConfig')
cfg.merge_fixed_joints = False
cfg.fix_base = False
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
    ignored = setup_self_collision(stage, os.path.abspath(args.urdf), ignore_internal_arm_collisions=True)
    print('self-collision on (including inter-arm and arm/environment),', ignored, 'pairs ignored')

WOOD = np.array([0.62, 0.48, 0.33])


def shelf(name, center, size):
    world.scene.add(FixedCuboid(prim_path=f'/World/{name}', name=name, position=np.array(center),
                                scale=np.array(size), color=WOOD))


# shelf A (x 2.0 .. 2.3, y 0.6 .. 1.4): boards at 0.30, 0.70, 1.10, sides, back
for i, zb in enumerate((0.30, SHELF_A['board'], 1.10)):
    shelf(f'shelfA_board{i}', (2.15, SHELF_A['y'], zb - 0.01), (0.30, 0.80, 0.02))
for i, yy in enumerate((0.59, 1.41)):
    shelf(f'shelfA_side{i}', (2.15, yy, 0.60), (0.30, 0.02, 1.20))
shelf('shelfA_back', (2.305, SHELF_A['y'], 0.60), (0.01, 0.84, 1.20))
# shelf B (x 0.4 .. 1.2, y -1.5 .. -1.8): boards at 0.10, 0.50, 0.90
for i, zb in enumerate((0.10, SHELF_B['board'], 0.90)):
    shelf(f'shelfB_board{i}', (SHELF_B['x'], -1.65, zb - 0.01), (0.80, 0.30, 0.02))
for i, xx in enumerate((0.39, 1.21)):
    shelf(f'shelfB_side{i}', (xx, -1.65, 0.50), (0.02, 0.30, 1.00))
shelf('shelfB_back', (SHELF_B['x'], -1.805, 0.50), (0.84, 0.01, 1.00))

item = world.scene.add(DynamicCuboid(prim_path='/World/product', name='product', position=ITEM_A,
                                     scale=ITEM, mass=ITEM_MASS, color=np.array([0.85, 0.15, 0.15])))
robot = world.scene.add(Articulation(prim_path=art_path, name='mobile_manipulator'))
world.reset()
robot.initialize()
if not args.headless:
    from isaacsim.core.utils.viewports import set_camera_view
    set_camera_view(eye=np.array([-1.2, -2.6, 2.6]), target=np.array([1.0, -0.2, 0.4]))

tcp_prim = SingleXFormPrim('/mobile_manipulator/left_am_tcp')
names = list(robot.dof_names)
n = len(names)
idx = {nm: i for i, nm in enumerate(names)}
vmax = np.array(robot.dof_properties['maxVelocity'], dtype=float)
wheel = {w: idx[f'wheel_{w}_joint'] for w in ('fl', 'fr', 'rl', 'rr')}
driven = [i for i, nm in enumerate(names) if 'roller' not in nm and not nm.startswith('wheel')]
SUBSTEPS = max(1, round(DT / world.get_physics_dt()))


def tick():
    if args.headless:
        for _ in range(SUBSTEPS):
            world.step(render=False)
    else:
        world.step(render=True)
    if not app.is_running():
        raise SystemExit


# ---------------------------------------------------------------- command state
q_goal = np.zeros(n)           # where the joints should be (driven joints)
q_cmd = np.zeros(n)            # rate-limited command actually sent
wheel_v = np.zeros(4)
log = []
t_sim = [0.0]


def set_left(q):
    for k, v in q.items():
        q_goal[idx[k]] = v
    AM.seed['left'] = np.array([q_goal[idx[f'left_{name}']] for name in AM.JOINTS])


def set_right(q):
    for k, v in q.items():
        if k != 'lift':
            q_goal[idx[k]] = v
    AM.seed['right'] = np.array([q_goal[idx[f'right_{name}']] for name in AM.JOINTS])


def deploy_right():
    """Move the right arm from its travel stow to the clear pose used during left-arm work."""
    set_right(RPARK)
    settle(timeout=30.0)


def stow_arms():
    """Tuck both arms alongside the base before driving or ending the task."""
    AM.lift = 0.20
    set_right(RSTOW)
    settle(timeout=30.0)
    joint_move(LSTOW)


def set_jaw(v):
    q_goal[idx['left_gripper']] = v


def step_once():
    q_cmd[driven] = rate_limit(q_cmd[driven], q_goal[driven], vmax[driven], DT)
    robot.apply_action(ArticulationAction(joint_positions=q_cmd[driven], joint_indices=np.array(driven)))
    robot.apply_action(ArticulationAction(joint_velocities=wheel_v, joint_indices=np.array(list(wheel.values()))))
    tick()
    t_sim[0] += DT


def wait(seconds):
    for _ in range(int(seconds * CTRL_HZ)):
        step_once()


def settle(tol=0.035, timeout=6.0):
    """Wait until the rate-limited command has reached the goal and the joints are within tol (2 deg / 35 mm
    is loose on purpose: wrist pitch sags about 1 deg under gravity), then 0.3 s more."""
    for _ in range(int(timeout * CTRL_HZ)):
        step_once()
        q = robot.get_joint_positions()
        if np.max(np.abs(q_cmd[driven] - q_goal[driven])) < 1e-6 and np.max(np.abs(q[driven] - q_goal[driven])) < tol:
            wait(0.3)
            return True
    return False


def base_pose():
    p, quat = robot.get_world_pose()
    w, x, y, z = [float(v) for v in quat]
    return np.array(p, dtype=float), math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def to_base(p_world):
    p, yaw = base_pose()
    d = np.asarray(p_world, dtype=float) - p
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], d[2]])


def joint_move(q_left):
    """Move the left arm (and lift) to q_left in joint space; rate limits set the pace."""
    set_left(q_left)
    settle(timeout=30.0)


def cart_move(a, b, elbow, seconds):
    """Straight-line TCP move from pose a to b (x, y, z, yaw, pitch in the base frame), IK every tick."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    steps = max(2, int(seconds * CTRL_HZ))
    for k in range(1, steps + 1):
        s = 0.5 - 0.5 * math.cos(math.pi * k / steps)
        q = AM.ik('left', *(a + (b - a) * s), elbow=elbow)
        if q is None:
            raise RuntimeError(f'no IK on the straight move at {np.round(a + (b - a) * s, 3)}')
        set_left(q)
        step_once()
    settle(timeout=3.0)


def cart_move_world(a_world, b_world, yaw_world, pitch, seconds):
    """Track a fixed shelf target in world coordinates while the unbraked mobile base reacts to arm motion."""
    a_world, b_world = np.asarray(a_world, float), np.asarray(b_world, float)
    steps = max(2, int(seconds * CTRL_HZ))
    for k in range(1, steps + 1):
        s = 0.5 - 0.5 * math.cos(math.pi * k / steps)
        target = a_world + (b_world - a_world) * s
        local = to_base(target)
        _, base_yaw = base_pose()
        yaw_local = math.atan2(math.sin(yaw_world - base_yaw), math.cos(yaw_world - base_yaw))
        q = AM.ik('left', *local, yaw_local, pitch)
        if q is None:
            raise RuntimeError(f'no AM-ARM200 IK on world-frame move at {np.round(target, 3)}')
        set_left(q)
        step_once()
    settle(timeout=3.0)


def jaw(v, seconds=1.0):
    set_jaw(v)
    wait(seconds)


def drive_to(goal, label, timeout=45.0):
    """Closed-loop base move to (x, y, yaw) in the world with mecanum wheel mixing."""
    R, LX, LY = 0.05, 0.17, 0.222
    for k in range(int(timeout * CTRL_HZ)):
        p, yaw = base_pose()
        ex, ey = goal[0] - p[0], goal[1] - p[1]
        eyaw = math.atan2(math.sin(goal[2] - yaw), math.cos(goal[2] - yaw))
        dist = math.hypot(ex, ey)
        if dist < 0.008 and abs(eyaw) < D(0.5):
            break
        vw = np.array([ex, ey]) * 1.2
        sp = np.linalg.norm(vw)
        if sp > 0.25:
            vw *= 0.25 / sp
        elif 0 < sp < 0.03:                    # creep: tiny wheel speeds don't beat roller friction
            vw *= 0.03 / sp
        wz = max(-0.5, min(0.5, 1.5 * eyaw))
        if 0 < abs(wz) < 0.03:
            wz = math.copysign(0.03, wz)
        c, s = math.cos(yaw), math.sin(yaw)
        vx, vy = c * vw[0] + s * vw[1], -s * vw[0] + c * vw[1]
        kk = LX + LY
        wheel_v[:] = [(vx - vy - kk * wz) / R, (vx + vy + kk * wz) / R, (vx + vy - kk * wz) / R, (vx - vy + kk * wz) / R]
        step_once()
    wheel_v[:] = 0
    wait(1.0)
    p, yaw = base_pose()
    err = (math.hypot(goal[0] - p[0], goal[1] - p[1]) * 1000, math.degrees(math.atan2(math.sin(goal[2] - yaw), math.cos(goal[2] - yaw))))
    report(label, f'stopped {err[0]:.1f} mm, {err[1]:+.2f} deg from the goal', ok=err[0] < 15 and abs(err[1]) < 1.0)
    return err


results = []


def report(step, text, ok=None):
    tag = '' if ok is None else ('  OK' if ok else '  FAIL')
    line = f'[{t_sim[0]:6.1f} s] {step:22s} {text}{tag}'
    print(line, flush=True)
    if ok is not None:
        results.append((step, ok, text))


def where(label):
    """Debug line: item centre relative to the left TCP, in the base frame (mm)."""
    if os.environ.get('TASK_DEBUG'):
        d = to_base(item_state()[0]) - to_base(tcp_prim.get_world_pose()[0])
        print(f'        {label:24s} item - tcp = ({d[0]*1000:+6.1f}, {d[1]*1000:+6.1f}, {d[2]*1000:+6.1f}) mm, item z {item_state()[0][2]:.3f}', flush=True)
        if label.startswith('at shelf') or label == 'after jaw close':
            q_actual = robot.get_joint_positions()
            keys = [f'left_{j}' for j in AM.JOINTS] + ['left_gripper', 'lift']
            print('        joints goal/actual       ' + ' '.join(
                f'{k.removeprefix("left_")}:{q_goal[idx[k]]:.2f}/{q_actual[idx[k]]:.2f}' for k in keys), flush=True)


def item_state():
    p, quat = item.get_world_pose()
    w, x, y, z = [float(v) for v in quat]
    up = np.array([2 * (x * z + w * y), 2 * (y * z - w * x), 1 - 2 * (x * x + y * y)])   # item's own z axis
    return np.array(p, dtype=float), up


def pick_from_shelf(label):
    """Level grasp of the upright item in front of the robot."""
    deploy_right()
    item_world = np.array(item_state()[0], dtype=float)
    item_start_z = float(item_world[2])
    _, yaw_world = base_pose()
    forward = np.array([math.cos(yaw_world), math.sin(yaw_world), 0.0])
    g_world = item_world + np.array([0, 0, GRASP_DZ])
    pre_world = g_world - APPROACH * forward
    pre = to_base(pre_world)
    jaw(JAW_OPEN, 0.5)
    joint_move(AM.ik('left', *pre, 0.0, 0.0, elbow=-1))
    where('at shelf pre-grasp')
    cart_move_world(tcp_prim.get_world_pose()[0], g_world, yaw_world, 0.0, 2.0)
    where('at shelf grasp')
    jaw(JAW_CLOSED, 1.5)
    where('after jaw close')
    up_world = g_world + np.array([0, 0, 0.03])
    cart_move_world(g_world, up_world, yaw_world, 0.0, 1.5)
    out_world = up_world - APPROACH * forward
    cart_move_world(up_world, out_world, yaw_world, 0.0, 2.0)
    q = robot.get_joint_positions()
    jaw_angle = math.degrees(q[idx['left_gripper']])
    rise = (item_state()[0][2] - item_start_z) * 1000
    report(label, f'gripper angle {jaw_angle:.0f} deg, item lifted {rise:.0f} mm and out of the shelf', ok=rise > 20)


def tote_pose(z):
    return np.array([TOTE['x'], TOTE['y'], z, TOTE['yaw'], P])


def place_in_tote():
    AM.lift = 0.0
    above = tote_pose(ZT + 0.17)
    joint_move(AM.ik('left', *above, elbow=TOTE['elbow']))
    cart_move(above, tote_pose(ZT), TOTE['elbow'], 2.5)
    jaw(JAW_OPEN, 1.0)
    cart_move(tote_pose(ZT), above, TOTE['elbow'], 2.0)
    jaw(JAW_CLOSED, 0.5)
    c, up = item_state()
    cb = to_base(c)
    inside = (-0.158 < cb[0] < 0.228) and (-0.233 < cb[1] < 0.233) and cb[2] < 0.376
    report('place in tote', f'item at ({cb[0]:+.3f}, {cb[1]:+.3f}, {cb[2]:.3f}) m in the base frame, lying '
           f'{"flat" if abs(up[2]) < 0.2 else "NOT flat"}', ok=inside)


def pick_from_tote():
    AM.lift = 0.0
    c, up = item_state()
    cb = to_base(c)
    # the item lies along its own z axis; pick the heading of that axis closest to the planned tote yaw
    p, byaw = base_pose()
    ax = math.atan2(up[1], up[0]) - byaw
    # The test item is square, so keep the planned tote heading; this avoids an unnecessary wrist roll.
    yaw = TOTE['yaw']
    g = np.array([cb[0], cb[1], cb[2], yaw, P])          # pitched down: TCP at the item centre
    above = g + np.array([0, 0, 0.17, 0, 0])
    jaw(JAW_OPEN, 0.5)
    q_above = AM.ik('left', *above, elbow=TOTE['elbow'])
    if q_above is None:
        report('pick from tote', f'no IK for the item at ({cb[0]:+.3f}, {cb[1]:+.3f}) heading {math.degrees(yaw):.0f} deg', ok=False)
        return False
    deploy_right()
    joint_move(q_above)
    cart_move(above, g, TOTE['elbow'], 2.5)
    jaw(JAW_CLOSED, 1.5)
    cart_move(g, above, TOTE['elbow'], 2.5)
    rise = (item_state()[0][2] - c[2]) * 1000
    report('pick from tote', f'item lifted {rise:.0f} mm (gripper 170 mm)', ok=rise > 140)
    where('above tote after pickup')
    return rise > 140


def place_on_shelf_b():
    _, yaw_world = base_pose()
    forward = np.array([math.cos(yaw_world), math.sin(yaw_world), 0.0])
    g_world = np.array(ITEM_B, dtype=float) + np.array([0, 0, GRASP_DZ])
    hi_world = g_world + np.array([0, 0, 0.02])
    pre_world = hi_world - APPROACH * forward
    pre = to_base(pre_world)
    joint_move(AM.ik('left', *pre, 0.0, 0.0, elbow=-1)); where('at pre-place')
    cart_move_world(tcp_prim.get_world_pose()[0], hi_world, yaw_world, 0.0, 2.0); where('above the place')
    cart_move_world(hi_world, g_world, yaw_world, 0.0, 1.5); where('at place')
    set_jaw(JAW_OPEN)
    for k in range(10):
        wait(0.1)
        if os.environ.get('TASK_DEBUG'):
            c = item_state()[0]; print(f'        t+{(k+1)*0.1:.1f} s item world ({c[0]:.3f}, {c[1]:.3f}, {c[2]:.3f})  jaw angle {math.degrees(robot.get_joint_positions()[idx["left_gripper"]]):.1f} deg', flush=True)
    where('released')
    out_world = g_world - APPROACH * forward
    cart_move_world(g_world, out_world, yaw_world, 0.0, 2.0)
    jaw(JAW_CLOSED, 0.5)
    joint_move(READY)


# ---------------------------------------------------------------- run
q0 = np.zeros(n)
for k, v in RSTOW.items():
    if k != 'lift': q0[idx[k]] = v
for k, v in LSTOW.items():
    q0[idx[k]] = v
q0[idx['lift']] = max(q0[idx['lift']], 0.0)
robot.set_joint_positions(q0)
q_goal[:] = q0; q_cmd[:] = q0
item.set_world_pose(position=ITEM_A, orientation=np.array([1.0, 0, 0, 0]))
wait(1.0)
print(f'\nShelf task: product {ITEM[0]*1000:.0f} x {ITEM[1]*1000:.0f} x {ITEM[2]*1000:.0f} mm, {ITEM_MASS*1000:.0f} g, '
      f"self-collision {'off' if args.no_self_collision else 'on'}\n")
try:
    drive_to(GOAL_A, 'drive to shelf A')
    pick_from_shelf('pick shelf A')
    if args.stop_after_pick:
        app.close()
        raise SystemExit
    place_in_tote()
    stow_arms()
    drive_to(GOAL_B, 'drive to shelf B')
    if pick_from_tote():
        place_on_shelf_b()
    stow_arms()
    wait(2.0)
except RuntimeError as e:
    report('aborted', str(e), ok=False)

c, up = item_state()
err = np.linalg.norm(c[:2] - ITEM_B[:2]) * 1000
on_board = abs(c[2] - ITEM_B[2]) < 0.01
upright = up[2] > math.cos(D(10))
report('result', f'item {err:.0f} mm from the target on shelf B, height {c[2]:.3f} m (board centre {ITEM_B[2]:.3f}), '
       f"{'upright' if upright else 'NOT upright'}", ok=err < 30 and on_board and upright)
print(f"\n{sum(ok for _, ok, _ in results)}/{len(results)} steps OK. Simulated time {t_sim[0]:.0f} s.")
if not args.headless:
    print('Window stays open; close it to exit.')
    while app.is_running():
        world.step(render=True)
app.close()
