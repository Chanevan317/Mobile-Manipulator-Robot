"""Check the leader calibration on the digital twin: the real leader drives the MONO arm alone in Isaac Sim.

    conda activate env_isaaclab           # run from the repo root
    OMNI_KIT_ACCEPT_EULA=YES PYTHONUNBUFFERED=1 python train_pipeline/leader/check_sim.py
    ... --port P          leader port (default LEADER_PORT, else the one in the calibration file)
    ... --calib NAME      calibration (train_pipeline/leader/calibration/<NAME>.json, default leader_mono)

The arm (the mono robot's ring, arm and gripper, no base / lift / scene) stands on a fixed post 0.55 m over the floor,
pointing along +x. It first moves SLOWLY to the leader's pose, then follows it 1:1 at the real servo speeds, with the
same mapping and clamps as teleop (leader_arm.Leader.command). The terminal shows a live table per joint: leader angle,
command (* = past the follower's limit, clamped), sim angle and lag. Move ONE leader joint at a time and check that
the same sim joint turns the same way; the zero pose must look straight in the sim. Close the window to quit.
"""
import argparse, math, os, sys
import xml.etree.ElementTree as ET
HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, '..', '..', 'scripts', 'common')); import repo_paths as RP   # noqa: E402,F401
import leader_arm as LA                                                  # noqa: E402

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('--port', default=None)
ap.add_argument('--calib', default=LA.DEFAULT_NAME)
args = ap.parse_args()

leader = LA.Leader(port=args.port, name=args.calib)      # connect first: no point opening Isaac without the leader
print('leader connected, calibration', leader.cal['name'], 'of', leader.cal['date'], flush=True)

from isaacsim import SimulationApp                                       # noqa: E402
import sim_app                                        # noqa: E402  (window at the screen's resolution)
app = sim_app.start(False)

import numpy as np                                                       # noqa: E402
import omni.kit.commands, omni.usd                                       # noqa: E402
from pxr import UsdLux, Sdf                                              # noqa: E402
from isaacsim.asset.importer.urdf import _urdf                           # noqa: E402
from isaacsim.core.api import World                                      # noqa: E402
from isaacsim.core.prims import SingleArticulation as Articulation      # noqa: E402
from isaacsim.core.utils.types import ArticulationAction                 # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view               # noqa: E402
from drives import apply_drives, rate_limit                              # noqa: E402
import urdf_variants                                                     # noqa: E402

LAYOUT = 'prototype_mono_arm'
CTRL_HZ = 60
SYNC_SPEED = 0.3                       # x the joint speeds until the arm has reached the leader's pose (as teleop_sim)
SYNC_TOL = math.radians(2)
# leader joint, follower command name (leader_arm.JOINTS / train_pipeline/common.py), URDF joint
MAP = [('J1', 'shoulder.pos', 'left_shoulder_pan'), ('J2', 'elbow.pos', 'left_elbow_pan'),
       ('J3', 'wrist_pan.pos', 'left_wrist_pan'), ('wrist_pitch', 'wrist_pitch.pos', 'left_wrist_pitch'),
       ('wrist_roll', 'wrist_roll.pos', 'left_wrist_roll')]
JAWS = ('left_finger_right', 'left_finger_left')          # gripper.pos = both jaws together: each gets half

world = World(stage_units_in_meters=1.0, physics_dt=1 / 240.0, rendering_dt=1 / CTRL_HZ)
world.scene.add_ground_plane()
stage = omni.usd.get_context().get_stage()
UsdLux.DistantLight.Define(stage, Sdf.Path('/DistantLight')).CreateIntensityAttr(3000)

urdf = urdf_variants.arm_only(urdf_variants.layout_urdf(LAYOUT), side='left', name='arm', stand_z=0.55)  # hand can point down
_, cfg = omni.kit.commands.execute('URDFCreateImportConfig')
cfg.merge_fixed_joints = False
cfg.fix_base = True
cfg.make_default_prim = True
cfg.self_collision = False
cfg.import_inertia_tensor = True
cfg.convex_decomp = False
cfg.distance_scale = 1.0
cfg.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
_, robot_path = omni.kit.commands.execute('URDFParseAndImportFile', urdf_path=urdf, import_config=cfg,
                                          get_articulation_root=True)
apply_drives(stage, robot_path, LAYOUT)
robot = world.scene.add(Articulation(prim_path=robot_path, name='arm'))
world.reset()
robot.initialize()
set_camera_view(eye=np.array([0.9, -0.9, 1.1]), target=np.array([0.15, 0.0, 0.45]))

names = list(robot.dof_names)
vel = {j.get('name'): float(j.find('limit').get('velocity')) for j in ET.parse(urdf).getroot().findall('joint')
       if j.find('limit') is not None}
idx = np.array([names.index(u) for _, _, u in MAP] + [names.index(u) for u in JAWS])
vmax = np.array([vel[u] for _, _, u in MAP] + [vel[u] for u in JAWS])
lim = {j: l for j, _, _, l in LA.JOINTS}

target = robot.get_joint_positions()[idx].copy()
speed, synced, k, shown = SYNC_SPEED, False, 0, 0
print('\nmoving SLOWLY to the leader\'s pose, then following 1:1 (close the window to quit)\n', flush=True)
try:
    while app.is_running():
        deg = leader.degrees(); cmd = leader.command(deg)
        goal = np.array([cmd[n] for _, n, _ in MAP] + [cmd['gripper.pos'] / 2] * 2)
        target = rate_limit(target, goal, vmax * speed, 1.0 / CTRL_HZ)
        if not synced and np.max(np.abs(target - goal)) < SYNC_TOL:
            synced, speed = True, 1.0
        robot.apply_action(ArticulationAction(joint_positions=target, joint_indices=idx))
        world.step(render=True)
        k += 1
        if k % (CTRL_HZ // 5) == 0:                                      # table 5 x per second, redrawn in place
            q = robot.get_joint_positions()[idx]
            rows = [f"state: {'FOLLOWING 1:1' if synced else 'moving slowly to the leader pose'}",
                    f"{'LEADER':<12} {'SIM JOINT':<18} | {'LEADER':>8} | {'COMMAND':>9} | {'SIM':>8} | {'LAG':>6}",
                    '-' * 74]
            for n, (j, cname, u) in enumerate(MAP):
                star = '*' if not lim[j][0] <= deg[j] <= lim[j][1] else ' '
                rows.append(f'{j:<12} {u:<18} | {deg[j]:+8.1f} | {math.degrees(cmd[cname]):+8.1f}{star} | '
                            f'{math.degrees(q[n]):+8.1f} | {math.degrees(target[n] - q[n]):+6.1f}')
            g, gt = (q[-2] + q[-1]) * 1000, (target[-2] + target[-1]) * 1000
            rows.append(f"{'gripper':<12} {'both fingers':<18} | {deg['gripper']:+7.1f}d | {cmd['gripper.pos'] * 1000:7.0f}mm "
                        f"| {g:6.0f}mm | {gt - g:+4.0f}mm")
            rows.append('(degrees; * = past the follower limit, clamped; LAG = sim target - sim angle)')
            if shown:
                sys.stdout.write(f'\033[{shown}F')
            sys.stdout.write(''.join('\033[2K' + r + '\n' for r in rows)); sys.stdout.flush()
            shown = len(rows)
finally:
    leader.close()
    app.close()
