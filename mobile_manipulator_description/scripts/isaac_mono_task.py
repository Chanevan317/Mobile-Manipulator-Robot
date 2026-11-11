"""One-arm pick-and-place task for the mono-arm EXPERIMENT (layout prototype_mono_arm: the shoulder is a ring around the
lift column). The robot stands still (free base, wheels hold it); one arm:
   1. picks a product from the table in front (hand down)
   2. swings round the column (~160 deg) and drops it into the tote behind
   3. takes it out of the tote again
   4. swings back and places it at a second spot on the table
Plan: mono_poses.py (checked offline against the URDF meshes, the table and the carried product: 125 samples clear).
Moves: time-synchronised joint moves (proto_task.Task.joint_move) and straight TCP lines (cart_move), every command
rate-limited as on the real servos.

    python scripts/isaac_mono_task.py               # window
    python scripts/isaac_mono_task.py --headless
    add --no_smooth for step-by-step moves instead of the TOPP-RA blended ones
"""
import argparse, math, os, sys

ap = argparse.ArgumentParser()
here = os.path.dirname(os.path.abspath(__file__))
ap.add_argument('--urdf', default=os.path.join(here, '..', 'urdf', 'mobile_manipulator_prototype_mono_arm.urdf'))
ap.add_argument('--headless', action='store_true')
ap.add_argument('--table_h', type=float, default=0.75)
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

ITEM = np.array([0.045, 0.045, MP.ITEM_H]); ITEM_MASS = 0.15
H = args.table_h
JAW_OPEN, JAW_CLOSED = 0.030, 0.0

task = Task(app, args.urdf, args.headless, fix_base=False, self_collision=not args.no_self_collision, layout='prototype_mono_arm')
w = task.world
task.profile = 'trap'     # trapezoid moves: full allowed speed after a 0.3 s ramp (same limits)
task.settle_mode = 'still'   # a move is done once the joints stop (the wrist pitch's gravity sag never gets within 2 deg)
# table and product start far away: Isaac resets the robot to its zero pose, where the arm points forward low enough
# to be inside the table (the overlap threw the robot ~35 m on the first run); they move in after the arm is up
TABLE_POS = np.array([MP.TABLE_EDGE + 0.35, 0.0, H / 2]); AWAY = np.array([6.0, 0.0, 0.0])
table = w.scene.add(FixedCuboid(prim_path='/World/table', name='table', position=TABLE_POS + AWAY,
                        scale=np.array([0.70, 1.20, H]), color=np.array([0.62, 0.48, 0.33])))
item = w.scene.add(DynamicCuboid(prim_path='/World/product', name='product', position=np.array([MP.PICK[0], MP.PICK[1], H + ITEM[2] / 2]) + AWAY,
                                 scale=ITEM, mass=ITEM_MASS, color=np.array([0.2, 0.6, 0.9])))
task.start()
if not args.headless:
    from isaacsim.core.utils.viewports import set_camera_view
    set_camera_view(eye=[1.4, 1.2, 1.5], target=[0.1, 0.0, 0.6])

steps, info = MP.plan(H)
q0 = steps[0][2]
# start: arm over the product (teleported, then settled), base held at the origin
task.set_pose({'lift': q0['lift'], **{f'left_{k}': v for k, v in q0.items() if k != 'lift'}})
task.hold = np.array([0.0, 0.0, 0.0]); task.wait(1.0)
table.set_world_pose(position=TABLE_POS)
item.set_world_pose(position=np.array([MP.PICK[0], MP.PICK[1], H + ITEM[2] / 2])); item.set_linear_velocity(np.zeros(3))
task.wait(1.0)
bp, yaw = task.base_pose(); task.report('start', f'base at ({bp[0]*1000:+.1f}, {bp[1]*1000:+.1f}) mm, {math.degrees(yaw):+.2f} deg; arm over the product')
t0 = task.t
tote_ok = None
run_steps = steps[1:]
if not args.no_smooth:                       # TOPP-RA: blended runs between the jaw actions (smooth_path.py)
    import smooth_path as SP
    run_steps, _ = SP.expand(run_steps, task.arm_now('left'), SP.vmax_of(task))
for kind, label, tgt in run_steps:
    if kind == 'jaw':
        task.jaw('left', JAW_OPEN if tgt == 1 else JAW_CLOSED, 0.8)
        p, up = Task.obj_state(item); pb = task.to_base(p)
        if label == 'release in the tote':
            inside = (-0.238 < pb[0] < 0.098) and abs(pb[1]) < 0.233 and pb[2] < 0.263 + 0.07
            task.wait(0.5); p, up = Task.obj_state(item); pb = task.to_base(p)
            tote_ok = inside and up[2] > 0.9
            task.report('in the tote', f'product at ({pb[0]:+.3f}, {pb[1]:+.3f}, {pb[2]:.3f}), upright {up[2]:.2f}', ok=tote_ok)
        elif label.startswith('grasp'):
            task.report(label, f'product at ({pb[0]:+.3f}, {pb[1]:+.3f}, {pb[2]:.3f})')
        continue
    if kind == 'traj':
        SP.play(task, tgt)
    elif kind == 'joint':
        task.joint_move('left', tgt)
    else:
        task.cart_move('left', np.asarray(MP.K.fk('left', task.arm_now('left'))[:3] + (tgt[3], tgt[4])), tgt[:5], tgt[5])
    p, up = Task.obj_state(item); pb = task.to_base(p)
    if label in ('lift it', 'lift it out'):
        tcp = task.to_base(task.tcp['left'].get_world_pose()[0])
        slip = np.linalg.norm(pb - tcp) * 1000
        task.report(label, f'product {slip:.1f} mm from the TCP (held)', ok=slip < 15 and pb[2] > info['zb'] + 0.05)
    else:
        task.report(label, f'done, product at ({pb[0]:+.3f}, {pb[1]:+.3f}, {pb[2]:.3f})')
task.wait(1.0)
p, up = Task.obj_state(item); pb = task.to_base(p)
err = math.hypot(pb[0] - MP.PLACE[0], pb[1] - MP.PLACE[1]) * 1000
task.report('placed on the table', f'{err:.1f} mm from the target, height {pb[2]:.3f} (table {H}), upright {up[2]:.2f}',
            ok=err < 20 and abs(pb[2] - (H + ITEM[2] / 2)) < 0.01 and up[2] > 0.95)
bp, yaw = task.base_pose()
task.report('base held still', f'moved {np.linalg.norm(bp[:2]) * 1000:.1f} mm, {math.degrees(yaw):+.2f} deg', ok=np.linalg.norm(bp[:2]) < 0.015)
print(f'task time {task.t - t0:.1f} s (one arm, table -> tote -> table)')
task.summary()
app.close()
