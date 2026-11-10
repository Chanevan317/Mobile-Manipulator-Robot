"""Poses and routes for the prototype-layout tasks (isaac_table_task.py, isaac_shelf_task.py --layout prototype).

Planned offline on 2026-11-06 with the URDF collision meshes (every arm link against the robot, the other arm and
the arm's own non-adjacent links, 11 samples per joint-space segment, jaws open 60 mm). Joint-space segments are
straight lines; proto_task.Task.joint_move drives them time-synchronised, so the robot sweeps exactly what was checked.
Right-arm poses are the left ones with the three pan angles negated (checked separately: the arms share parts but
are not mirror images).

Lessons from planning:
  - Re-planned 2026-11-09 for the 130 / 130 links and the C-Beam XLarge lift (0.885 m): the stow pose is hand level
    and clear at any lift. The only way in or out is still the elbow alone: STOW <-> ELBOW_OPEN.
  - The arm that waits during work parks straight out to its side with the hand level (PARK).
  - On a 0.75 m table the vertical approach is 0.12 m (lift travel 0.885 m).
  - Don't flip the elbow while carrying a lying item: the fast swing slides it down the jaws (TOTE_SHELF).
  - Both arms at once: unfold one at a time (left, then right) and fold the other way round (right, then left);
    unfolding together crosses the forearms. At the tote use TOTE2 (spots 0.26 m apart).
  - Near a table, raise the lift before swinging over it and lower it only after swinging away; a joint-space line
    that lowers while it swings drags the held product into the table (seen in Isaac).
"""
import math

D = math.radians
P = math.pi / 2
PAN = ('shoulder_pan', 'elbow_pan', 'wrist_pan')


def _q(a, b, c, p, lift=None):
    q = dict(shoulder_pan=D(a), elbow_pan=D(b), wrist_pan=D(c), wrist_pitch=D(p), wrist_roll=0.0)
    if lift is not None: q['lift'] = lift
    return q


def mirror(q):
    return {k: (-v if k in PAN else v) for k, v in q.items()}


def both(q_left):
    return {'left': q_left, 'right': mirror(q_left)}


def full(side, q):
    """Joint-name dict for Task.set_pose()."""
    return {(k if k == 'lift' else f'{side}_{k}'): v for k, v in q.items()}


STOW = both(_q(30, 105, -120, 0, 0.20))         # travel / idle, hand level: x -0.18..0.22, |y| <= 0.23, clear at any lift (130 / 130 links)
ELBOW_OPEN = both(_q(30, 0, -120, 0, 0.20))      # the way in and out of STOW
PARK = both(_q(45, 0, 0, 0))                     # waiting arm: straight out to its side, hand level (lift follows the work)
READY = both(_q(-45, 0, 0, 0, 0.45))             # working arm straight out to its side, hand level (shelf B retreat)

# tote spot per arm (hand down), base frame; flat item (shelf task) and upright 70 mm product (table task)
TOTE = {'left': dict(x=-0.07, y=0.13, yaw=D(180), elbow=1), 'right': dict(x=-0.07, y=-0.13, yaw=-D(180), elbow=-1)}   # = TOTE2 since 130 / 130 links
# both arms at the tote at the same time: the spots above make the arms collide, these don't (two-arm check 2026-11-06)
TOTE2 = {'left': dict(x=-0.07, y=0.13, yaw=D(180), elbow=1), 'right': dict(x=-0.07, y=-0.13, yaw=-D(180), elbow=-1)}
# shelf task (left arm, one item): same spot as TOTE but with the shelf's elbow bend, so shelf A -> tote -> shelf B
# needs no elbow flip. With the flip (TOTE) the fast swing slid the lying item 52 mm down the jaws (Isaac, 2026-11-09).
TOTE_SHELF = {'left': dict(x=-0.07, y=0.13, yaw=D(180), elbow=-1), 'right': dict(x=-0.07, y=-0.13, yaw=-D(180), elbow=1)}
ZT_FLAT = 0.263 + 0.045 / 2 + 0.008              # TCP height that sets a lying 45 mm item 8 mm above the tote floor
ZT_UPRIGHT = 0.263 + 0.035 + 0.008               # same for an upright 70 mm product
TOTE_UP = 0.12                                   # vertical approach above the tote
TOTE_INNER = (-0.238, 0.098, -0.233, 0.233)      # inside of the tote walls (x0, x1, y0, y1), base frame

# table task (left arm; the right arm uses y -> -y and the opposite elbow)
TABLE_EDGE = 0.36                                # table front edge, m in front of the base centre (0.10 m past the bumper)
TABLE_PICK, TABLE_PLACE = (0.42, 0.15), (0.42, 0.28)   # 6 cm in from the edge; reach ends ~0.17 m past the edge
TABLE_YAW, TABLE_ELBOW = 0.0, -1

# shelf task (left arm): item straight ahead of the left arm, hand level, 0.08 m straight approach
SHELF_REACH = (0.66, 0.22)                      # 130 / 130 links: whole straight-in move > 20 deg inside the joint limits
SHELF_APPROACH = 0.08
SHELF_ELBOW = -1


def _via_open(side):
    return [ELBOW_OPEN[side]]


ROUTE = {s: {
    # waiting arm (the working arm is stowed, lift 0.20)
    'stow_to_park': _via_open(s), 'park_to_stow': _via_open(s),
    # (the table task sequences its own lift-first / lower-last moves, see isaac_table_task.py)
    # shelf task (planned for the left arm)
    'stow_to_shelf': _via_open(s), 'shelf_to_tote': [], 'tote_to_stow': _via_open(s), 'stow_to_tote': _via_open(s),
    'tote_to_shelf': [], 'shelf_to_ready': [], 'ready_to_stow': _via_open(s),
} for s in ('left', 'right')}
