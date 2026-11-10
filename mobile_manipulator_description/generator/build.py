import sys, time, json, math
import numpy as np
sys.path.insert(0, '.')
from lib import *
from hw import *
import arm
from arm import *


def frames(q=(0, 0, 0), pitch=0.0, roll=0.0, opening=60.0):
    F = {}
    F['arm'] = np.eye(4)
    F['upper'] = Rz(q[0])
    F['fore'] = F['upper'] @ T(L1, 0, 0) @ Rz(q[1])
    F['wrist'] = F['fore'] @ T(L2, 0, 0) @ Rz(q[2])
    F['tilt'] = F['wrist'] @ T(40.0, 0, PZ) @ Ry(pitch)
    F['roll'] = F['tilt'] @ T(ROLL_X, 0, 0) @ Rx(roll)
    F['opening'] = opening
    return F


def build_parts(F):
    arm.PARTS.clear()
    base_parts()
    link_parts('upper_arm', L1, F['upper'])
    link_parts('forearm', L2, F['fore'])
    wrist_parts(F['wrist'])
    tilt_parts(F['tilt'])
    camera_parts(F['tilt'])
    yoke_part(F['roll'])
    front_plate_part(F['roll'])
    pinion_part(F['roll'])
    arm.PARTS['pinion_24T_m1']['frame'] = arm.PARTS['pinion_24T_m1']['frame'] @ Rz(math.degrees(F['opening'] / 2 / PIN_R))
    v = F['opening']
    for side, nm in ((1, 'finger_carrier_right'), (-1, 'finger_carrier_left')):
        s, r = carrier_part(side, F['roll'])
        add(nm, s, F['roll'] @ T(0, side * v / 2, 0), (0.20, 0.20, 0.22), r,
            note='bolts to one MGN9H carriage; its rack meshes with the pinion')
    for side, nm in ((1, 'finger_right'), (-1, 'finger_left')):
        s, r = finger_part(None)
        if side < 0:
            mir = np.diag([1.0, -1.0, 1.0, 1.0]); s = xf(s, mir); r = arm.reg_xf(r, mir)
        add(nm, s, F['roll'] @ T(0, side * v / 2, 0), (0.12, 0.12, 0.13), r, note='wedge finger, 1.5 mm rubber pad pocket on the grip face')


def bought(F):
    """Purchased parts placed in ARM frame: list of dict(name, solid, color, metal)."""
    out = []
    srv = [('servo_shoulder_STS3215', F['arm']), ('servo_elbow_STS3215', F['upper'] @ T(L1, 0, 0)),
           ('servo_wrist_pan_STS3215', F['fore'] @ T(L2, 0, 0)), ('servo_pitch_STS3215', F['wrist'] @ T(40, 0, PZ) @ PITCH_J),
           ('servo_roll_STS3215', F['tilt'] @ ROLL_J), ('servo_gripper_STS3215', F['roll'] @ GRIP_J)]
    for nm, fr in srv:
        out.append(dict(name=nm, solid=xf(SERVO, fr), color=(0.13, 0.13, 0.15), metal=0.2, frame=fr))
    b = bearing_6808()
    for nm, fr in (('J1', F['arm']), ('J2', F['upper'] @ T(L1, 0, 0)), ('J3', F['fore'] @ T(L2, 0, 0)),
                   ('pitch', F['wrist'] @ T(40, 0, PZ) @ PITCH_J)):
        out.append(dict(name=f'bearing_6808_{nm}_top', solid=xf(b, fr @ T(0, 0, P['brgT'][0])), color=(0.72, 0.74, 0.78), metal=0.8))
        out.append(dict(name=f'bearing_6808_{nm}_bottom', solid=xf(b, fr @ T(0, 0, P['brgB'][0])), color=(0.72, 0.74, 0.78), metal=0.8))
    rb = basis((0, 0, -1), (0, 1, 0), (1, 0, 0), (74.0, 0, RAIL_Z))
    rail, _ = mgn9_rail(170.0)
    out.append(dict(name='MGN9_rail_170', solid=xf(rail, F['roll'] @ rb), color=(0.78, 0.80, 0.83), metal=0.8))
    v = F['opening']
    for side in (1, -1):
        yc = side * (v / 2 + 7.0 + 13.5)
        out.append(dict(name=f'MGN9H_block_{"right" if side > 0 else "left"}',
                        solid=xf(mgn9h_block(), F['roll'] @ T(0, yc, 0) @ rb), color=(0.70, 0.72, 0.76), metal=0.8))
    from arm import cam_frame, camera_board
    out.append(dict(name='wrist_cam_OV_32x32 (approx. envelope)', solid=xf(camera_board(), F['tilt'] @ cam_frame()),
                    color=(0.12, 0.25, 0.20), metal=0.1))
    return out


def fasteners():
    out = []
    for nm, p in arm.PARTS.items():
        for it in p['reg'].items:
            fr = p['frame']
            q = fr @ np.r_[it['p'], 1.0]; d = fr[:3, :3] @ np.asarray(it['d'])
            out.append(dict(part=nm, **{k: it[k] for k in ('kind', 'size', 'length', 'note')}, p=q[:3], d=d,
                            p_local=it['p'], d_local=it['d']))
    return out


def fastener_solid(f):
    k = f['kind']
    if k in ('SHCS', 'selftap'):
        s = screw_model(f['size'], f['length'], 'SHCS' if k == 'SHCS' else 'pan')
    elif k == 'insert':
        s = insert_model(f['size'], f['length'])
    elif k == 'nut':
        s = xf(M.cylinder(2.4, 5.5 / 2 / math.cos(math.pi / 6), 5.5 / 2 / math.cos(math.pi / 6), 6), Rz(30)) - cyl(1.5, -1, 3)
    return place(s, f['p'], f['d'])


if __name__ == '__main__':
    t = time.time()
    F = frames()
    build_parts(F)
    print('parts', len(arm.PARTS), round(time.time() - t, 1))
    for nm, p in arm.PARTS.items():
        s = p['solid']
        print(f"{nm:32s} vol {s.volume()/1000:8.1f} cm3  genus {s.genus():3d}  tris {s.num_tri()}  status {s.status()}")
