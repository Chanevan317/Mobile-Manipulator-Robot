"""Table-to-table task, ONE arm (mono-arm experiment, layout prototype_mono_arm): the prototype's isaac_table_task.py
scene and route, done by the single arm on the shoulder ring around the column.

The robot (free base, mecanum wheels, self-collision on) starts at the origin facing +x.
  table A: front edge at x = 2.0, faces the robot;  table B: front edge at y = -1.5, faces +y (90 deg turn)
  the robot stops with the table edge 0.36 m in front of its centre (as the prototype task)
  products: 2 x (45 x 45 x 70 mm, 150 g), standing upright
Steps: drive to table A; product 1 and then product 2 into the tote (two spots behind the column); stow; drive to
table B; both products back out of the tote onto table B; stow.
Plan: mono_poses.table_steps (checked offline: 320 samples, robot / table / tote products / carried product all clear),
aimed at the products' measured positions.

    python scripts/isaac_mono_table_task.py [--headless] [--table_h 0.75] [--smooth]
"""
import argparse, math, os, sys

ap = argparse.ArgumentParser()
here = os.path.dirname(os.path.abspath(__file__))
ap.add_argument('--urdf', default=os.path.join(here, '..', 'urdf', 'mobile_manipulator_prototype_mono_arm.urdf'))
ap.add_argument('--headless', action='store_true')
ap.add_argument('--table_h', type=float, default=0.75)
ap.add_argument('--no_self_collision', action='store_true')
ap.add_argument('--smooth', action='store_true', help='TOPP-RA blended moves (smooth_path.py). Off by default since 2026-11-13: with the layout B arm the blended swing tipped product 1 at the tote (step moves 6/6, 124 s vs 122 s)')
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
H, EDGE = args.table_h, MP.TABLE['edge']
GOAL_A = np.array([2.0 - EDGE, 0.0, 0.0])
GOAL_B = np.array([1.0, -1.5 + EDGE, -P])


def world(goal, xy):
    c, s = math.cos(goal[2]), math.sin(goal[2])
    return np.array([goal[0] + c * xy[0] - s * xy[1], goal[1] + s * xy[0] + c * xy[1]])


task = Task(app, args.urdf, args.headless, fix_base=False, self_collision=not args.no_self_collision, layout='prototype_mono_arm')
w = task.world
task.profile = 'trap'     # trapezoid moves: full allowed speed after a 0.3 s ramp (same limits)
task.settle_mode = 'still'   # a move is done once the joints stop (the wrist pitch's gravity sag never gets within 2 deg)
WOOD = np.array([0.62, 0.48, 0.33])
w.scene.add(FixedCuboid(prim_path='/World/tableA', name='tableA', position=np.array([2.35, 0.0, H / 2]), scale=np.array([0.70, 1.20, H]), color=WOOD))
w.scene.add(FixedCuboid(prim_path='/World/tableB', name='tableB', position=np.array([1.0, -1.85, H / 2]), scale=np.array([1.20, 0.70, H]), color=WOOD))
START = [world(GOAL_A, s) for s in MP.TABLE['spots']]
TARGET = [world(GOAL_B, s) for s in MP.TABLE['spots']]
items = [w.scene.add(DynamicCuboid(prim_path=f'/World/product_{k}', name=f'product_{k}', position=np.array([*START[k], H + ITEM[2] / 2]),
                                   scale=ITEM, mass=ITEM_MASS, color=np.array(col))) for k, col in enumerate(((0.2, 0.6, 0.9), (0.9, 0.4, 0.2)))]
task.start()
if not args.headless:
    from isaacsim.core.utils.viewports import set_camera_view
    set_camera_view(eye=np.array([-0.6, 1.6, 2.4]), target=np.array([1.4, -0.6, 0.5]))

task.set_pose({'lift': MP.STOW['lift'], **{f'left_{k}': v for k, v in MP.STOW.items() if k != 'lift'}})
for k in range(2):
    items[k].set_world_pose(position=np.array([*START[k], H + ITEM[2] / 2]), orientation=np.array([1.0, 0, 0, 0]))
task.wait(1.0)
print(f'\nTable-to-table task, ONE arm (mono): tables {H:.2f} m, 2 products {ITEM[0]*1000:.0f} x {ITEM[1]*1000:.0f} x '
      f"{ITEM[2]*1000:.0f} mm, {ITEM_MASS*1000:.0f} g, self-collision {'off' if args.no_self_collision else 'on'}\n")
z_at = {}


def note(label):
    """Report after grasps and releases (product positions in the base frame)."""
    for k in range(2):
        if label.endswith(f'{k + 1}') or f' {k + 1} ' in label:
            c, up = task.obj_state(items[k]); cb = task.to_base(c)
            if label.startswith('grasp'):
                z_at[k] = c[2]
            elif label.startswith('release') and 'tote' in label:
                x0, x1, y0, y1 = -0.238, 0.098, -0.233, 0.233
                task.wait(0.4); c, up = task.obj_state(items[k]); cb = task.to_base(c)
                task.report(f'product {k + 1} into the tote', f'at ({cb[0]:+.3f}, {cb[1]:+.3f}, {cb[2]:.3f}), {"upright" if up[2] > 0.9 else "tipped"}',
                            ok=x0 < cb[0] < x1 and y0 < cb[1] < y1 and cb[2] < 0.376 and up[2] > 0.9)
            elif label.startswith('release') and 'table' in label:
                task.wait(0.4); c, up = task.obj_state(items[k])
                err = np.linalg.norm(c[:2] - TARGET[k]) * 1000
                task.report(f'product {k + 1} onto table B', f'{err:.0f} mm from the target, height {c[2]:.3f} m, {"upright" if up[2] > math.cos(D(10)) else "NOT upright"}',
                            ok=err < 20 and abs(c[2] - (H + ITEM[2] / 2)) < 0.01 and up[2] > math.cos(D(10)))


try:
    task.drive_to(GOAL_A, 'drive to table A')
    src = [tuple(task.to_base(task.obj_state(items[k])[0])[:2]) for k in range(2)]      # where the products are
    MP.run(task, MP.table_steps(H, order=(0, 1), to_tote=True, src=src), on_jaw=note, smooth=args.smooth)
    task.drive_to(GOAL_B, 'drive to table B')
    src = [tuple(task.to_base(task.obj_state(items[k])[0])[:2]) for k in range(2)]
    dst = [tuple(task.to_base(np.array([*TARGET[k], 0.0]))[:2]) for k in range(2)]
    MP.run(task, MP.table_steps(H, order=(1, 0), to_tote=False, src=src, dst=dst), on_jaw=note, smooth=args.smooth)
except RuntimeError as e:
    task.report('aborted', str(e), ok=False)
print(f'task time {task.t:.0f} s')
task.summary()
if not args.headless:
    print('Window stays open; close it to exit.')
    while app.is_running():
        w.step(render=True)
app.close()
