"""Shelf-to-shelf task: drive to shelf A, pick a product, put it in the tote, drive to shelf B, pick it from
the tote and place it on shelf B.

The robot (free base, mecanum wheels, self-collision on) starts at the origin facing +x.
  shelf A: front edge at x = 2.0, faces the robot (-x); board top 0.70 m; the product stands on it
  shelf B: front edge at y = -1.5, faces +y; board top 0.50 m
  product: 45 x 45 x 70 mm box, 150 g
The left arm does the work; the right arm parks straight out to its side.

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
    <isaac-sim>/python.sh scripts/isaac_shelf_task.py               # window (the task code is in shelf_task_prototype.py)
    <isaac-sim>/python.sh scripts/isaac_shelf_task.py --headless
    TASK_DEBUG=1 ... isaac_shelf_task.py      # also print the item position relative to the gripper
"""
import argparse, math, os, sys

ap = argparse.ArgumentParser()
here = os.path.dirname(os.path.abspath(__file__))
ap.add_argument('--layout', default='prototype', choices=('prototype',), help='robot layout (only prototype since 2026-11-09)')
ap.add_argument('--urdf', default=None, help='URDF path (overrides --layout)')
ap.add_argument('--headless', action='store_true')
ap.add_argument('--no_self_collision', action='store_true')
args = ap.parse_args()
if args.urdf is None:
    args.urdf = os.path.join(here, '..', 'urdf', f'mobile_manipulator_{args.layout}.urdf')

from isaacsim import SimulationApp
app = SimulationApp({'headless': args.headless})

sys.path.insert(0, here)
import shelf_task_prototype
shelf_task_prototype.run(app, args)
