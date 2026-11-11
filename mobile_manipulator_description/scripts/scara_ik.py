"""Closed-form inverse kinematics for one SCARA arm plus the lift, in the base_link frame.

The arm: shoulder, elbow and wrist pan about vertical axes (link lengths 0.13 and 0.13 m), then wrist pitch
(+ = tip down) and wrist roll. The lift moves the whole carriage up. Two robot layouts (generator/urdf_build.py):
    prototype  shoulder axis base_link (0.18, ±0.15), arm zero pose at ±135 deg (45 deg toward the rear tote)
The only layout is 'prototype' (v0 was deleted on 2026-11-09); use('prototype') is kept for the existing callers.
Geometry (metres):
    shoulder z         0.4855 + lift (both layouts)
    wrist pitch axis   0.04 m along the hand heading from the wrist pan axis, 0.0746 m below it
    TCP                0.215 m from the pitch axis along the gripper (0.085 to the roll axis + 0.130)
So with yaw psi = mount yaw + shoulder + elbow + wrist_pan, pitch theta and roll 0:
    TCP xy = wrist_pan_axis + (0.04 + 0.215 cos theta) (cos psi, sin psi)
    TCP z  = lift + 0.4109 - 0.215 sin theta
No ROS / Isaac imports: usable anywhere.
"""
import math

L1, L2 = 0.13, 0.13       # was 0.18 / 0.15 until 2026-11-09
PITCH_OFF = 0.04          # wrist pan axis -> pitch axis, along the hand heading
TOOL = 0.215              # pitch axis -> TCP
Z0 = 0.4109               # TCP height at lift 0, pitch 0
D = math.radians
# per layout: side -> (shoulder x, shoulder y, arm zero-pose yaw); must match generator/urdf_build.py
LAYOUTS = {'prototype': {'left': (0.18, 0.15, D(135)), 'right': (0.18, -0.15, -D(135))},
           # experiment (2026-11-10): one arm, shoulder = a ring around the column (shoulder axis = column axis), zero pose forward
           'prototype_mono_arm': {'left': (0.18, 0.0, 0.0)}}
SHOULDER_LIM = {'prototype': D(115), 'prototype_mono_arm': D(170)}   # mono: URDF +-180 (cable loop +-195, servo multi-turn: bench test pending); plans keep 10 deg margin
ELBOW_LIM = {'prototype': D(125), 'prototype_mono_arm': D(117)}   # mono: forearm meets the ring at 122-123 deg
LAYOUT = 'prototype'
ARM_Y = 0.15              # arm mount y (ARM_Y / SHOULDER / MOUNT_YAW are set by use(); see the end of this file)
SHOULDER = {'left': (0.18, ARM_Y), 'right': (0.18, -ARM_Y)}
MOUNT_YAW = {'left': D(135), 'right': -D(135)}


def use(layout):
    """Select the robot layout ('prototype' or the 'prototype_mono_arm' experiment) for fk / ik and ARM_Y, SHOULDER, MOUNT_YAW."""
    global LAYOUT, ARM_Y, SHOULDER, MOUNT_YAW
    L = LAYOUTS[layout]; LAYOUT = layout
    SHOULDER = {s: (v[0], v[1]) for s, v in L.items()}; MOUNT_YAW = {s: v[2] for s, v in L.items()}
    ARM_Y = L['left'][1]
    LIMITS['lift'] = (0.0, LIFT_MAX[layout])
    LIMITS['shoulder_pan'] = (-SHOULDER_LIM[layout], SHOULDER_LIM[layout])
    LIMITS['elbow_pan'] = (-ELBOW_LIM[layout], ELBOW_LIM[layout])


def urdf_name(layout=None, ros=False):
    """File name of a layout's URDF in ../urdf."""
    return f"mobile_manipulator_{layout or LAYOUT}{'_ros' if ros else ''}.urdf"
LIMITS = {'shoulder_pan': (-D(115), D(115)), 'elbow_pan': (-D(125), D(125)), 'wrist_pan': (-D(122), D(122)),
          'wrist_pitch': (-D(15), D(115)), 'wrist_roll': (-D(120), D(120)), 'lift': (0.0, 0.885)}
LIFT_MAX = {'prototype': 0.885, 'prototype_mono_arm': 0.77}   # mono: the shoulder ring meets the column's top cap above ~0.78     # prototype: C-Beam XLarge, 1000 mm beam, 885 mm travel


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def fk(side, q):
    """TCP (x, y, z) and yaw in base_link for joint dict q (keys without the side prefix, plus 'lift')."""
    sx, sy = SHOULDER[side]
    a1 = MOUNT_YAW[side] + q['shoulder_pan']; a2 = a1 + q['elbow_pan']; psi = a2 + q['wrist_pan']; th = q.get('wrist_pitch', 0.0)
    r = PITCH_OFF + TOOL * math.cos(th)
    x = sx + L1 * math.cos(a1) + L2 * math.cos(a2) + r * math.cos(psi)
    y = sy + L1 * math.sin(a1) + L2 * math.sin(a2) + r * math.sin(psi)
    return x, y, q['lift'] + Z0 - TOOL * math.sin(th), wrap(psi)


def ik(side, x, y, z, yaw, pitch=0.0, elbow=None, margin=D(3)):
    """Joint dict reaching TCP (x, y, z) with heading `yaw` and pitch `pitch` (roll 0), or None.

    elbow: +1 or -1 picks the elbow bend; None tries both and returns the one with the most margin to
    the joint limits. margin: how far inside each limit a solution must stay (default 3 deg / 3 mm).
    """
    sx, sy = SHOULDER[side]
    lift = z - Z0 + TOOL * math.sin(pitch)
    r = PITCH_OFF + TOOL * math.cos(pitch)
    wx, wy = x - r * math.cos(yaw) - sx, y - r * math.sin(yaw) - sy
    d2 = wx * wx + wy * wy
    c2 = (d2 - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    if abs(c2) > 1:
        return None
    best, best_m = None, -1e9
    for e in ((elbow,) if elbow else (1, -1)):
        q2 = e * math.acos(c2)
        a1 = math.atan2(wy, wx) - math.atan2(L2 * math.sin(q2), L1 + L2 * math.cos(q2))   # world heading of the upper arm
        q1 = a1 - MOUNT_YAW[side]
        q = {'shoulder_pan': wrap(q1), 'elbow_pan': q2, 'wrist_pan': wrap(yaw - a1 - q2),
             'wrist_pitch': pitch, 'wrist_roll': 0.0, 'lift': lift}
        m = min(min(v - LIMITS[k][0], LIMITS[k][1] - v) / (1.0 if k == 'lift' else 1.0) for k, v in q.items())
        lin_ok = LIMITS['lift'][0] + 0.003 <= lift <= LIMITS['lift'][1] - 0.003
        ang_ok = all(LIMITS[k][0] + margin <= v <= LIMITS[k][1] - margin for k, v in q.items() if k != 'lift')
        if lin_ok and ang_ok and m > best_m:
            best, best_m = q, m
    return best


def joint_names(side, q):
    """Prefix the arm joint names with the side ('lift' stays as it is)."""
    return {(k if k == 'lift' else f'{side}_{k}'): v for k, v in q.items()}


use('prototype')
