"""Shelf-to-shelf task, ONE arm (mono-arm experiment, layout prototype_mono_arm): the prototype's isaac_shelf_task.py
scene and route, done by the single arm on the shoulder ring around the column.

The robot (free base, mecanum wheels, self-collision on) starts at the origin facing +x.
  shelf A: front edge at x = 2.0, faces the robot; boards 0.30 / 0.70 / 1.10 m; the product stands on the 0.70 board
  shelf B: front edge at y = -1.5, faces +y; boards 0.10 / 0.50 / 0.90 m; target 6 cm in on the 0.50 board
  product: 1 x (45 x 45 x 70 mm, 150 g)
Steps: drive to shelf A; level grasp straight in, lift 30 mm, out; lay it in the tote (hand down); stow; drive to
shelf B; pick it out of the tote across its axis; via the fold to in front of shelf B; straight in 20 mm high, down,
release wide, out; stow.
Plan: mono_poses.shelf_pick_steps / shelf_place_steps (checked offline: 224 samples clear, incl. the carried product
upright and lying), aimed at the product's measured position and axis.

    python scripts/isaac_mono_shelf_task.py [--headless] [--no_smooth]
"""
import argparse, math, os, sys

ap = argparse.ArgumentParser()
here = os.path.dirname(os.path.abspath(__file__))
ap.add_argument('--urdf', default=os.path.join(here, '..', 'urdf', 'mobile_manipulator_prototype_mono_arm.urdf'))
ap.add_argument('--headless', action='store_true')
ap.add_argument('--no_self_collision', action='store_true')
ap.add_argument('--no_smooth', action='store_true', help='step-by-step moves instead of the TOPP-RA blended ones (smooth_path.py)')
args = ap.parse_args()

from isaacsim import SimulationApp
app = SimulationApp({'headless': args.headless})

import numpy as np
from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid
sys.path.insert(0, here)
from proto_task import Task
import mono_poses as MP

D = math.radians
P = math.pi / 2
ITEM = np.array([0.045, 0.045, MP.ITEM_H]); ITEM_MASS = 0.15
SHELF_A = dict(edge_x=2.0, y=1.0, board=0.70)
SHELF_B = dict(edge_y=-1.5, x=0.8, board=0.50)
REACH = np.array(MP.SHELF['reach'])
ITEM_A = np.array([SHELF_A['edge_x'] + 0.04, SHELF_A['y'], SHELF_A['board'] + ITEM[2] / 2])
ITEM_B = np.array([SHELF_B['x'], SHELF_B['edge_y'] - 0.06, SHELF_B['board'] + ITEM[2] / 2])
WOOD = np.array([0.62, 0.48, 0.33])


def goal_for(item_xy, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([item_xy[0] - (c * REACH[0] - s * REACH[1]), item_xy[1] - (s * REACH[0] + c * REACH[1]), yaw])


task = Task(app, args.urdf, args.headless, fix_base=False, self_collision=not args.no_self_collision, layout='prototype_mono_arm')
w = task.world
task.profile = 'trap'     # trapezoid moves: full allowed speed after a 0.3 s ramp (same limits)
task.settle_mode = 'still'   # a move is done once the joints stop (the wrist pitch's gravity sag never gets within 2 deg)


def shelf(name, center, size):
    w.scene.add(FixedCuboid(prim_path=f'/World/{name}', name=name, position=np.array(center), scale=np.array(size), color=WOOD))
for i, zb in enumerate((0.30, SHELF_A['board'], 1.10)):
    shelf(f'shelfA_board{i}', (2.15, SHELF_A['y'], zb - 0.01), (0.30, 0.80, 0.02))
for i, yy in enumerate((0.59, 1.41)):
    shelf(f'shelfA_side{i}', (2.15, yy, 0.60), (0.30, 0.02, 1.20))
shelf('shelfA_back', (2.305, SHELF_A['y'], 0.60), (0.01, 0.84, 1.20))
for i, zb in enumerate((0.10, SHELF_B['board'], 0.90)):
    shelf(f'shelfB_board{i}', (SHELF_B['x'], -1.65, zb - 0.01), (0.80, 0.30, 0.02))
for i, xx in enumerate((0.39, 1.21)):
    shelf(f'shelfB_side{i}', (xx, -1.65, 0.50), (0.02, 0.30, 1.00))
shelf('shelfB_back', (SHELF_B['x'], -1.805, 0.50), (0.84, 0.01, 1.00))
item = w.scene.add(DynamicCuboid(prim_path='/World/product', name='product', position=ITEM_A, scale=ITEM, mass=ITEM_MASS,
                                 color=np.array([0.85, 0.15, 0.15])))
task.start()
if not args.headless:
    from isaacsim.core.utils.viewports import set_camera_view
    set_camera_view(eye=np.array([-1.2, -2.6, 2.6]), target=np.array([1.0, -0.2, 0.4]))

task.set_pose({'lift': MP.STOW['lift'], **{f'left_{k}': v for k, v in MP.STOW.items() if k != 'lift'}})
item.set_world_pose(position=ITEM_A, orientation=np.array([1.0, 0, 0, 0]))
task.wait(1.0)
print(f"\nShelf task, ONE arm (mono): product {ITEM[0]*1000:.0f} x {ITEM[1]*1000:.0f} x {ITEM[2]*1000:.0f} mm, {ITEM_MASS*1000:.0f} g, "
      f"self-collision {'off' if args.no_self_collision else 'on'}\n")
start_z = {}


def note(label):
    c, up = task.obj_state(item); cb = task.to_base(c)
    if label == 'grasp':
        start_z['z'] = c[2]
        tcp = task.to_base(task.tcp['left'].get_world_pose()[0])
        task.report('at the grasp', f'product ({cb[0]:+.3f}, {cb[1]:+.3f}, {cb[2]:.3f}), TCP ({tcp[0]:+.3f}, {tcp[1]:+.3f}, {tcp[2]:.3f})')
    elif label == 'release in the tote':
        task.wait(0.4); c, up = task.obj_state(item); cb = task.to_base(c)
        task.report('place in tote', f'item at ({cb[0]:+.3f}, {cb[1]:+.3f}, {cb[2]:.3f}) m, lying {"flat" if abs(up[2]) < 0.2 else "NOT flat"}',
                    ok=-0.238 < cb[0] < 0.098 and abs(cb[1]) < 0.233 and cb[2] < 0.376 and abs(up[2]) < 0.2)


try:
    task.drive_to(goal_for(ITEM_A[:2], 0.0), 'drive to shelf A')
    ia = task.to_base(task.obj_state(item)[0])
    task.report('product seen at', f'({ia[0]:+.3f}, {ia[1]:+.3f}, {ia[2]:.3f}) in the base frame (planned {REACH[0]:.3f}, {REACH[1]:.3f})')
    steps, yt, et = MP.shelf_pick_steps(tuple(ia))
    k_out = [i for i, s in enumerate(steps) if s[1] == 'straight out'][0]
    def dbg(label):
        pb = task.to_base(task.obj_state(item)[0]); t = task.to_base(task.tcp['left'].get_world_pose()[0])
        print(f'      [debug] {label:28s} product ({pb[0]:+.3f}, {pb[1]:+.3f})  TCP ({t[0]:+.3f}, {t[1]:+.3f}, {t[2]:.3f})', flush=True)
        bp, byaw = task.base_pose(); pw = task.obj_state(item)[0]
        print(f'      [debug]   base world ({bp[0]:+.4f}, {bp[1]:+.4f}) yaw {math.degrees(byaw):+.2f}  product world ({pw[0]:+.4f}, {pw[1]:+.4f})', flush=True)
    MP.run(task, steps[:k_out + 1], on_jaw=note, debug=dbg if os.environ.get('MONO_DEBUG') else None, smooth=not args.no_smooth)
    q = task.robot.get_joint_positions()
    gap = (q[task.idx['left_finger_right']] + q[task.idx['left_finger_left']]) * 1000
    rise = (task.obj_state(item)[0][2] - start_z['z']) * 1000
    task.report('pick shelf A', f'jaw gap {gap:.1f} mm (item 45), item lifted {rise:.0f} mm (gripper 30) and out of the shelf',
                ok=abs(gap - 45) < 5 and rise > 20)
    MP.run(task, steps[k_out + 1:], on_jaw=note, smooth=not args.no_smooth)
    task.drive_to(goal_for(ITEM_B[:2], -P), 'drive to shelf B')
    c, up = task.obj_state(item); cb = task.to_base(c)
    _, byaw = task.base_pose()
    axis = math.atan2(up[1], up[0]) - byaw                                   # the lying product's long axis (base frame)
    tb = task.to_base(ITEM_B)
    steps = MP.shelf_place_steps(tuple(tb), axis, item_in_tote=(cb[0], cb[1]), item_z=cb[2])
    k_lift = [i for i, s in enumerate(steps) if s[1] == 'lift it'][0]
    MP.run(task, steps[:k_lift + 1], smooth=not args.no_smooth)
    rise = (task.obj_state(item)[0][2] - c[2]) * 1000
    task.report('pick from tote', f'item lifted {rise:.0f} mm (gripper {MP.TOTE_UP*1000:.0f} mm)', ok=rise > MP.TOTE_UP * 1000 - 30)
    MP.run(task, steps[k_lift + 1:], smooth=not args.no_smooth)
    task.wait(2.0)
except RuntimeError as e:
    task.report('aborted', str(e), ok=False)
c, up = task.obj_state(item)
err = np.linalg.norm(c[:2] - ITEM_B[:2]) * 1000
task.report('result', f'item {err:.0f} mm from the target on shelf B, height {c[2]:.3f} m (board centre {ITEM_B[2]:.3f}), '
            f"{'upright' if up[2] > math.cos(D(10)) else 'NOT upright'}", ok=err < 30 and abs(c[2] - ITEM_B[2]) < 0.01 and up[2] > math.cos(D(10)))
print(f'task time {task.t:.0f} s')
task.summary()
if not args.headless:
    print('Window stays open; close it to exit.')
    while app.is_running():
        w.step(render=True)
app.close()
