"""Joint drive settings for the picking robot in Isaac Sim. The single place where gains live.

Isaac Sim's URDF importer creates *acceleration* drives with its own default stiffness and
ignores the gains we want. `apply_drives()` overwrites every revolute/prismatic joint of the
imported robot with a *force* drive (torque = kp * error + kd * velocity error), clamped at
max_force. Call it right after the import, before the simulation starts.

Gains below are in SI units: N m/rad and N m s/rad for revolute joints, N/m and N s/m for
prismatic ones. USD stores angular gains per DEGREE; `apply_drives()` converts.
"""
import math

DEG = math.pi / 180.0
LAYOUT = 'prototype'      # set to 'prototype_mono_arm' (set_layout) for the one-arm experiment


def set_layout(layout):
    global LAYOUT
    LAYOUT = layout


def gains(name):
    """Return (kind, kp, kd, max_force) in SI units for a joint name."""
    if 'roller' in name:                       return ('angular', 0.0, 0.0, 0.0)       # passive mecanum rollers
    if name.startswith('wheel'):               return ('angular', 0.0, 2.0, 6.0)       # velocity-driven wheels
    if name == 'lift':                         return ('linear', 100000.0, 2000.0, 300.0)  # lead-screw lift: stiff
    if 'finger' in name:                       return ('linear', 2000.0, 100.0, 80.0)
    if 'shoulder' in name and LAYOUT == 'prototype_mono_arm':
        # mono-arm experiment: the shoulder is a ring around the column, an STS3215 through a 4:1 gear (7.6 N m); the
        # arm swings about the column (~0.15 kg m2): kp 160 / kd 6 gives a damping ratio of ~0.6 (kp 640 / kd 12 changed
        # nothing in the shelf task: the sideways lag there came from the elbow / wrist servos, see mono_poses SLOW_IN)
        return ('angular', 160.0, 6.0, 4.56)   # 2026-11-13: drive servo at a 60 % torque limit (gear teeth; load test peak 3.5 N m)
    if 'shoulder' in name:                     return ('angular', 40.0, 1.5, 1.9)      # STS3215; stiffer: it swings the whole arm
    if 'elbow' in name and LAYOUT == 'prototype_mono_arm':
        return ('angular', 40.0, 1.5, 1.9)     # mono layout B: the 'elbow' is the arm base's shoulder servo (swings the whole arm)
    return ('angular', 20.0, 0.8, 1.9)                                                 # STS3215 7.4 V, SO-100 version (every other arm joint)


def rate_limit(previous, target, max_speed, dt, margin=0.9):
    """Move each joint target from `previous` toward `target` by at most margin * max_speed * dt.

    Use this on every command you send: a real servo or lead-screw lift ramps too. It also avoids a PhysX
    problem: when a joint is asked to move faster than its speed limit (physxJoint:maxJointVelocity, from
    the URDF velocity) while the base is free, the lift crawls at about 5 mm/s instead of 50 mm/s.
    Works on numpy arrays (or floats). max_speed comes from Articulation.dof_properties['maxVelocity'].
    """
    import numpy as np
    step = margin * np.asarray(max_speed) * dt
    return np.asarray(previous) + np.clip(np.asarray(target) - np.asarray(previous), -step, step)


def apply_drives(stage, root_path, layout=None):
    """Set a force drive with our gains on every revolute/prismatic joint under root_path.

    Returns {joint name: UsdPhysics.DriveAPI}, so callers can set targets on it.
    """
    from pxr import UsdPhysics
    if layout: set_layout(layout)
    root = '/' + root_path.strip('/').split('/')[0]
    drives = {}
    for prim in stage.Traverse():
        if not str(prim.GetPath()).startswith(root + '/'):
            continue
        if not (prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint)):
            continue
        name = prim.GetName()
        kind, kp, kd, fmax = gains(name)
        if kind == 'angular':                  # USD: per degree
            kp, kd = kp * DEG, kd * DEG
        drv = UsdPhysics.DriveAPI.Apply(prim, kind)
        drv.CreateTypeAttr().Set('force')
        drv.CreateStiffnessAttr().Set(kp)
        drv.CreateDampingAttr().Set(kd)
        drv.CreateMaxForceAttr().Set(fmax)
        drives[name] = drv
    return drives
