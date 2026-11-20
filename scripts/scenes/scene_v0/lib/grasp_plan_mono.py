"""Grasp plans for the SimReady scene products with the MONO arm (no simulator: scara_ik only; kinematics + joint
limits, NOT collisions).

  short products (<= TOP_H): from above, hand pointing down, TCP 30 mm under the top (fingertips >= 1 mm over the
                             surface)
  taller ones:               from the side, hand level, straight in along the hand heading, gripped 60 % up
                             (>= 20 mm under the top, >= 50 mm over the surface: hand clearance). Gripped at 45 %,
                             on the cone, a cup and a yogurt tub pivoted out of the jaws (Isaac 2026-11-16)
Each plan = approach start a, grasp b, lifted c (TCP x, y, z, yaw, pitch in the base frame) + elbow.
"""
import math, os, sys
import numpy as np
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'common')); import repo_paths as RP
import scara_ik as K
import assets as SA

K.use('prototype_mono_arm')
SIDE = 'left'                 # the mono arm's joint prefix
TOP_H = 0.070
APPROACH = 0.10               # side: straight-in length; top: height above the grasp
FRONT_MIN = 0.36              # furniture front at least this far ahead of the base centre (camera at x 0.341, bumper 0.259)


def dims(pkg):
    sx, sy, sz = (v / 1000 for v in SA.size(pkg))
    return min(sx, sy), max(sx, sy), sz            # narrow (across the jaws), deep (along the approach), height


def _ok(poses):
    for e in (1, -1):
        if all(K.ik(SIDE, *p, elbow=e) for p in poses):
            return e
    return None


def side_poses(c, yaw, deep, gz, dz):
    u = np.array([math.cos(yaw), math.sin(yaw)])
    tcp = np.asarray(c) - u * (deep / 2 - min(deep / 2, 0.03))       # TCP 30 mm past the near face (centre if shallow)
    a = tcp - u * APPROACH
    return (a[0], a[1], gz, yaw, 0.0), (tcp[0], tcp[1], gz, yaw, 0.0), (tcp[0], tcp[1], gz + dz, yaw, 0.0)


def top_poses(c, yaw, fz, dz):
    p = lambda z: (c[0], c[1], z, yaw, math.pi / 2)
    return p(fz + APPROACH * 0.8), p(fz), p(fz + dz)


def grip_heights(surface, h):
    return surface + max(min(0.6 * h, h - 0.02), 0.05), surface + max(h - 0.03, 0.013)


def table_plan(pkg, table_top, edge, y_max=0.25, dz=0.10):
    """A free spot on the table in front of the robot (|y| <= y_max) where the arm can pick the product.
    Returns dict(top, centre, yaw (hand heading), a, b, c, elbow, narrow) or None."""
    narrow, deep, h = dims(pkg); gz, fz = grip_heights(table_top, h)
    if h <= TOP_H:
        for x in np.arange(edge + 0.06, edge + 0.20, 0.01):
            for y in (0.0, 0.05, -0.05, 0.10, -0.10, 0.15, -0.15):
                for yaw in np.radians((0, -30, 30, 60, -60, 90)):
                    P = top_poses((x, y), yaw, fz, dz); e = _ok(P)
                    if e and _fits((x, y), yaw, narrow, deep, edge, y_max):
                        return dict(top=True, centre=(x, y), yaw=yaw, a=P[0], b=P[1], c=P[2], elbow=e, narrow=narrow)
        return None
    for hd in np.radians((0, 15, -15, 30, -30, 45, -45)):
        u = np.array([math.cos(hd), math.sin(hd)])
        for x in np.arange(edge + 0.04, edge + 0.45, 0.01):
            for y in np.arange(-0.20, 0.201, 0.02):
                c = np.array([x, y]) + u * deep / 2                      # (x, y) = middle of the near face
                P = side_poses(c, hd, deep, gz, dz); e = _ok(P)
                if e and _fits(c, hd, narrow, deep, edge, y_max):
                    return dict(top=False, centre=tuple(c), yaw=hd, a=P[0], b=P[1], c=P[2], elbow=e, narrow=narrow)
    return None


def _fits(c, yaw, narrow, deep, edge, y_max):
    """Product footprint (deep along yaw, narrow across) on the table, >= 10 mm in from the edge, inside |y| <= y_max."""
    u = np.array([math.cos(yaw), math.sin(yaw)]); v = np.array([-u[1], u[0]])
    pts = [np.asarray(c) + su * u * deep / 2 + sv * v * narrow / 2 for su in (-1, 1) for sv in (-1, 1)]
    return all(p[0] >= edge + 0.01 and abs(p[1]) <= y_max for p in pts)


def rest_reach(pkg, surface, front, dz):
    """Can the arm pick the product where it rests, from SOME base stop facing the furniture? `front` = distance from
    the furniture's front edge to the product centre along the approach. Product deep side along the approach.
    Returns dict(top, d, b, elbow) (product centre at (d, b) in the base frame) or None."""
    narrow, deep, h = dims(pkg); gz, fz = grip_heights(surface, h)
    for d in np.arange(0.40, 0.95, 0.01):
        if d - front < FRONT_MIN:
            continue
        for b in (0.0, 0.02, -0.02, 0.05, -0.05, 0.10, -0.10, 0.15, -0.15):
            if h <= TOP_H:
                for yaw in (0.0, math.pi):                  # the jaws must close across the approach (narrow side)
                    e = _ok(top_poses((d, b), yaw, fz, dz))
                    if e:
                        return dict(top=True, d=d, b=b, elbow=e)
            else:
                e = _ok(side_poses((d, b), 0.0, deep, gz, dz))
                if e:
                    return dict(top=False, d=d, b=b, elbow=e)
    return None
