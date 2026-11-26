"""Full robot URDF: mobile base (mecanum with passive rollers), lift, carriage, two SCARA arms built from the CAD parts.
Writes a package:  <out>/meshes/visual/*.stl, <out>/meshes/collision/*.stl (metres) and
  urdf/mobile_manipulator_prototype.urdf (+ _ros)  column 0.18 m toward the front (working face), arms right beside it at
                                              y = +-0.15 with their zero pose 45 deg toward the rear tote (layout study 2026-11-06)
The old v0 layout (column at the rear, arms forward on carriage beams) was deleted on 2026-11-09 (user).
  urdf/mobile_manipulator_prototype_mono_arm.urdf (+ _ros)  EXPERIMENT (user, 2026-11-10; does not replace prototype):
                                              one arm whose shoulder is a ring bearing AROUND the lift column
                                              (shoulder axis = column axis), +-170 deg, ring driven by an STS3215 + 4:1 gear"""
import sys, os, math, collections, json
import numpy as np
sys.path.insert(0, '.')
from build import *
from stlio import write_stl, read_stl

OUT = sys.argv[1] if len(sys.argv) > 1 else 'out/mobile_manipulator_description'
VIS = os.path.join(OUT, 'meshes', 'visual'); COL = os.path.join(OUT, 'meshes', 'collision')
for d in (VIS, COL, os.path.join(OUT, 'urdf')):
    os.makedirs(d, exist_ok=True)

MM = 0.001
RHO_PRINT = 0.80e-3 * (1.24 / 1.27)  # PLA 1.24 vs PETG 1.27 g/cm^3, same effective fill
# prototype layout (see the docstring): column x, arm mount y, arm zero-pose yaw (left; right is mirrored)
PROTO_CX, PROTO_ARM_Y, PROTO_YAW = 0.18, 0.15, math.radians(90.0)   # mount yaw +-90 since 2026-11-25 (was 135: each arm's
# blind sector pointed forward-inward, nothing in the centre strip was reachable; dual layout study)
# SIDE layout test (user, 2026-11-18: "side tote, side lift"): the one-arm robot with the column in the middle of the
# right side edge and the tote turned lengthwise along the left side. The arm works over the front edge with a ~90 deg
# swing to the tote (front layout: ~180), and the scene camera sits 0.18 m further back from what it looks at.
SIDE_CX, SIDE_CY = 0.0, -0.18               # column axis in base_link (front layout: (PROTO_CX, 0))
SIDE_TOTE = (0.0, 0.075, math.pi / 2)       # tote centre x, y and yaw (0.48 along x, y -0.10 .. 0.25)
# ONE-ARM PROTOTYPE since 2026-11-23 (user: the ring replaced by the mono HOLDER; front only; "the lift slightly on the
# side to keep the carriage at center", the carriage facing the side it was shifted from): mono_holder.py. The column
# stands MH.ARM_DX right of the centre, turned -90 deg (its gantry plate faces the centre); the arm's own shoulder on the
# centreline, zero pose to the LEFT; the tote on the BASE behind the arm, as in the original design (MH.BASE_TOTE). Layout name 'prototype_mono_arm' (the ring layouts are parked as
# 'prototype_mono_ring' / 'prototype_mono_ring_side').
import mono_holder as MH
import dual_holder as DH
SINGLE_CX, SINGLE_CY = MH.ROBOT_X, -MH.ARM_DX * 0.001   # column axis in base_link: ROBOT_X forward, 0.122 right (shoulder at (ROBOT_X + ARM_DY, 0))
SINGLE_YAW = -math.pi / 2                   # column / carriage yaw: carriage +y = robot front, carriage -x = robot left
# lift speed limit (m/s): 0.10 since 2026-11-06 (user decision; 8 mm-lead C-Beam + closed-loop NEMA 23 at ~750 rpm).
# Was 0.05 until then. Re-time on the real lift and keep sim and real equal (it shapes the motion policies learn).
LIFT_SPEED = 0.10
RHO_STEEL = 7.85e-3
RHO_BRASS = 8.5e-3


# ---------------------------------------------------------------- mass properties
def mass_props(V, F, mass=None, rho=None):
    """Return (mass_kg, com_m, inertia_kg_m2 about com) for a closed mesh in mm."""
    a, b, c = V[F[:, 0]], V[F[:, 1]], V[F[:, 2]]
    v = np.einsum('ij,ij->i', a, np.cross(b, c)) / 6.0
    vol = v.sum()
    com = (v[:, None] * (a + b + c) / 4.0).sum(0) / vol
    s = a + b + c
    C = np.zeros((3, 3))
    for x in (a, b, c, s):
        C += np.einsum('i,ij,ik->jk', v, x, x)
    C /= 20.0
    m = mass if mass is not None else vol * rho / 1000.0          # kg (rho in g/mm^3)
    k = m / vol
    C = C * k                                                      # kg mm^2
    I0 = np.trace(C) * np.eye(3) - C
    Icom = I0 - m * (np.dot(com, com) * np.eye(3) - np.outer(com, com))
    return m, com * MM, Icom * MM * MM


def combine(props):
    m = sum(p[0] for p in props)
    com = sum(p[0] * p[1] for p in props) / m
    I = np.zeros((3, 3))
    for mi, ci, Ii in props:
        d = ci - com
        I += Ii + mi * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return m, com, I


def box_props(m, size, xyz=(0, 0, 0)):
    x, y, z = size
    return m, np.asarray(xyz, float), np.diag([m * (y * y + z * z) / 12, m * (x * x + z * z) / 12, m * (x * x + y * y) / 12])


def cyl_props(m, r, h, xyz=(0, 0, 0), axis=2):
    Ia = m * r * r / 2; It = m * (3 * r * r + h * h) / 12
    d = [It, It, It]; d[axis] = Ia
    return m, np.asarray(xyz, float), np.diag(d)


# ---------------------------------------------------------------- convex pieces for collision
def convex_pieces(solid, cell=32.0, min_vol=30.0):
    bb = solid.bounding_box()
    out = []
    xs = np.arange(bb[0], bb[3] + cell, cell); ys = np.arange(bb[1], bb[4] + cell, cell); zs = np.arange(bb[2], bb[5] + cell, cell)
    for x0 in xs:
        for y0 in ys:
            for z0 in zs:
                ch = solid ^ box(x0, x0 + cell, y0, y0 + cell, z0, z0 + cell)
                if ch.is_empty():
                    continue
                for comp in ch.decompose():
                    if comp.volume() > min_vol:
                        out.append(comp.hull())
    return out


def mesh_m(solid, mat=np.eye(4)):
    V, F = mesh_arrays(xf(solid, mat))
    return V * MM, F


# ---------------------------------------------------------------- build the arm at zero pose, fingers closed
F0 = frames((0, 0, 0), 0.0, 0.0, 0.0)
build_parts(F0)
BOUGHT = bought(F0)
FAST = fasteners()

LINKF = {
    'base': np.eye(4), 'upper_arm': np.eye(4), 'forearm': T(L1, 0, 0), 'wrist': T(L1 + L2, 0, 0),
    'tilt': T(L1 + L2 + 40.0, 0, PZ), 'hand': T(L1 + L2 + 40.0 + ROLL_X, 0, PZ),
    'finger_right': T(L1 + L2 + 40.0 + ROLL_X, 0, PZ), 'finger_left': T(L1 + L2 + 40.0 + ROLL_X, 0, PZ),
}


def part_link(nm):
    if nm.startswith('base'): return 'base'
    if nm.startswith('upper_arm'): return 'upper_arm'
    if nm.startswith('forearm'): return 'forearm'
    if nm.startswith(('wrist', 'pitch')): return 'wrist'
    if nm.startswith(('tilt', 'camera')): return 'tilt'
    if nm.endswith('_right') and nm.startswith('finger'): return 'finger_right'
    if nm.endswith('_left') and nm.startswith('finger'): return 'finger_left'
    return 'hand'


def bought_link(nm):
    if 'shoulder' in nm or '_J1_' in nm: return 'base'
    if 'elbow' in nm or '_J2_' in nm: return 'upper_arm'
    if 'wrist_pan' in nm or '_J3_' in nm: return 'forearm'
    if 'pitch' in nm: return 'wrist'
    if 'roll_' in nm or 'wrist_cam' in nm: return 'tilt'
    if 'MGN9H_block_right' in nm: return 'finger_right'
    if 'MGN9H_block_left' in nm: return 'finger_left'
    return 'hand'


BOUGHT_MASS = {'STS3215': 0.055, 'bearing': 0.025, 'MGN9_rail': 0.065, 'MGN9H': 0.026, 'wrist_cam': 0.015}

LINKS = collections.OrderedDict((k, dict(vis=[], col=[], props=[])) for k in LINKF)


def add_vis(link, fname, solid_world, color):
    inv = np.linalg.inv(LINKF[link])
    V, F = mesh_m(solid_world, inv)
    write_stl(os.path.join(VIS, fname + '.stl'), V, F)
    LINKS[link]['vis'].append((fname, color))
    return V, F


for nm, p in arm.PARTS.items():
    lk = part_link(nm)
    sw = xf(p['solid'], p['frame'])
    V, F = add_vis(lk, 'arm_' + nm, sw, p['color'])
    LINKS[lk]['props'].append(mass_props(V / MM, F, rho=RHO_PRINT))
    loc = xf(sw, np.linalg.inv(LINKF[lk]))
    if nm.startswith('finger_carrier'):          # the racks sit behind the other finger's plate: collide with the plate only
        loc = loc ^ box(83.9, 93.0, -100, 100, -100, 100)
    for k, h in enumerate(convex_pieces(loc)):
        Vc, Fc = mesh_arrays(h)
        fn = f'arm_{nm}_c{k:02d}'
        write_stl(os.path.join(COL, fn + '.stl'), Vc * MM, Fc)
        LINKS[lk]['col'].append(fn)

bear_groups = collections.defaultdict(list)
for b in BOUGHT:
    nm = b['name']; lk = bought_link(nm)
    key = next(k for k in BOUGHT_MASS if k in nm)
    if key == 'bearing':
        bear_groups[lk].append(b['solid']); continue
    fname = 'arm_' + nm.split(' ')[0]
    V, F = add_vis(lk, fname, b['solid'], b['color'])
    LINKS[lk]['props'].append(mass_props(V / MM, F, mass=BOUGHT_MASS[key]))
    if key in ('STS3215', 'MGN9_rail', 'MGN9H', 'wrist_cam'):
        Vc, Fc = mesh_arrays(xf(b['solid'], np.linalg.inv(LINKF[lk])).hull())
        write_stl(os.path.join(COL, fname + '_c.stl'), Vc * MM, Fc)
        LINKS[lk]['col'].append(fname + '_c')
for lk, sols in bear_groups.items():
    V, F = add_vis(lk, f'arm_{lk}_bearings', union(sols), (0.72, 0.74, 0.78))
    for s in sols:
        Vb, Fb = mesh_arrays(xf(s, np.linalg.inv(LINKF[lk])))
        LINKS[lk]['props'].append(mass_props(Vb, Fb, mass=BOUGHT_MASS['bearing']))

fast_groups = collections.defaultdict(list)
for f in FAST:
    fast_groups[part_link(f['part'])].append(f)
for lk, fs in fast_groups.items():
    sols = [fastener_solid(f) for f in fs]
    u = union(sols)
    V, F = add_vis(lk, f'arm_{lk}_fasteners', u, (0.08, 0.08, 0.09))
    for f, s in zip(fs, sols):
        Vs, Fs = mesh_arrays(xf(s, np.linalg.inv(LINKF[lk])))
        if len(Fs):
            LINKS[lk]['props'].append(mass_props(Vs, Fs, rho=RHO_BRASS if f['kind'] == 'insert' else RHO_STEEL))

INERTIA = {k: combine(v['props']) for k, v in LINKS.items()}

# ---------------------------------------------------------------- mono arm (user 2026-11-10; layout B since 2026-11-13)
# One arm whose shoulder is a ring around the C-Beam (ring axis = column axis = carriage origin), mono_ring.py: 2 x 6820-2RS
# bearings, PETG rotor + 72 / 18 T gear, drive STS3215 under the disc. The prototype's own arm base (01 + 02 + its
# shoulder servo) bolts onto the rotor's shelf, 161 mm out; then one 130 mm link (the forearm part) and the wrist.
# Joint names stay those of the 3-pan arm: left_shoulder_pan = the RING, left_elbow_pan = the base's SHOULDER servo
# (0.161 m out), left_wrist_pan 0.13 m further. Links: left_base = fixed ring parts (carriage), left_upper_arm = rotor +
# tray, left_arm_base = the prototype base (fixed to the rotor), left_forearm = the 130 mm link + wrist-pan servo.
import mono_ring as MRB
MONO = dict(ratio=MRB.ZR / MRB.ZP, az=MRB.AZ, l1=MRB.L1)   # gear ratio 4, arm plane (carriage mm), shoulder offset (mm)
RING_TORQUE = round(0.6 * 1.9 * MRB.ZR / MRB.ZP, 2)   # ring drive servo at a 60 % torque limit (4.56 N m): the PETG gear teeth (mono_ring_strength.py)
MLINKS = {k: dict(vis=list(v['vis']), col=list(v['col']), props=list(v['props'])) for k, v in LINKS.items()}
MLINKS['base'] = dict(vis=[], col=[], props=[]); MLINKS['upper_arm'] = dict(vis=[], col=[], props=[])
RHO_PETG = 0.80e-3                          # same fill assumption as RHO_PRINT


def m_add(lk, fname, loc, color, mass=None, rho=None, col=True):
    V, F = mesh_m(loc)
    write_stl(os.path.join(VIS, fname + '.stl'), V, F)
    MLINKS[lk]['vis'].append((fname, color)); MLINKS[lk]['props'].append(mass_props(V / MM, F, mass=mass, rho=rho))
    if col:
        for k, h in enumerate(convex_pieces(loc)):
            Vc, Fc = mesh_arrays(h); fn = f'{fname}_c{k:02d}'
            write_stl(os.path.join(COL, fn + '.stl'), Vc * MM, Fc); MLINKS[lk]['col'].append(fn)


def _mono_links():
    """mono_ring.py: fixed parts -> left_base, turning parts (rotor, tray) -> left_upper_arm, bearings split."""
    for f in os.listdir(VIS):                          # old ring meshes (earlier designs) go
        if f.startswith('mono_'): os.remove(os.path.join(VIS, f))
    for f in os.listdir(COL):
        if f.startswith('mono_'): os.remove(os.path.join(COL, f))
    MRB.build_all()
    toarm = T(0, 0, -MONO['az'])                       # carriage frame (mm) -> arm frame (z 0 = arm plane)
    for nm, p in MRB.PARTS.items():
        lk = 'upper_arm' if p['moving'] else 'base'
        m_add(lk, 'mono_' + nm, xf(p['solid'], toarm), p['color'], rho=RHO_PETG if p['material'] == 'PETG' else RHO_PRINT)
    for k in ('bearing_1', 'bearing_2'):               # 6820-2RS ~0.25 kg: inner ring fixed, outer ring + balls turning
        b = MRB.CONTEXT[k]
        m_add('base', f'mono_{k}_inner', xf(b['inner'], toarm), (0.72, 0.74, 0.78), mass=0.10, col=False)
        m_add('upper_arm', f'mono_{k}_outer', xf(union([b['outer'], b['band']]), toarm), (0.72, 0.74, 0.78), mass=0.15, col=False)
    m_add('base', 'mono_ring_drive_STS3215', xf(MRB.CONTEXT['drive_servo'], toarm), (0.12, 0.12, 0.14), mass=0.055)
    m_add('base', 'mono_servo_board', xf(MRB.CONTEXT['servo_board'], toarm), (0.10, 0.35, 0.20), mass=0.045)   # plate + Waveshare board


def mono_mast_xml(z0=0.066):
    """Scene camera on the CARRIAGE, always facing forward (user, 2026-11-10: it moves with the lift and sees the front by
    default). The arm sweeps all the way round the column at the carriage, so the post stands in the socket on the
    (fixed) clamp ring, inside the cable channel, in front of the beam: x 0.031..0.045 (outside the column's top cap,
    |x| <= 0.030). Above the arm (nothing of it rises past
    carriage z ~0.11) a bar carries the camera forward to the prototype's spot: (0.14, 0, 0.19), pitched 0.95 rad down."""
    # z0: bottom of the post socket (mono_ring.py clamp: 0.066; dual_holder.py front bridge: 0.0275)
    parts = [((0.014, 0.016, 0.20 - z0), (0.038, 0.0, (z0 + 0.20) / 2)), ((0.119, 0.02, 0.02), (0.0905, 0.0, 0.19))]
    v = ''.join(vprim(f3(c), bx(*sz), ALU) for sz, c in parts); c_ = ''.join(cprim(f3(c), bx(*sz)) for sz, c in parts)
    o = '  <link name="camera_mast">\n' + inertial(*combine([box_props(0.06, sz, c) for sz, c in parts])) + v + c_ + '  </link>\n'
    o += joint('camera_mast_fixed', 'fixed', 'carriage', 'camera_mast', '0 0 0')
    o += '  <link name="scene_camera">\n' + inertial(*box_props(0.07, (0.03, 0.097, 0.029))) + vprim('0 0 0', bx(0.03, 0.097, 0.029), ORANGE) + \
         cprim('0 0 0', bx(0.03, 0.097, 0.029)) + '  </link>\n'
    o += joint('scene_camera_fixed', 'fixed', 'camera_mast', 'scene_camera', '0.14 0 0.19', '0 0.95 0')
    o += frame_link('scene_camera_optical')
    o += joint('scene_camera_optical_fixed', 'fixed', 'scene_camera', 'scene_camera_optical', '0.016 0 0', '-1.5708 0 -1.5708')
    return o


# ---------------------------------------------------------------- URDF text helpers
def f3(v): return ' '.join(f'{x:.6g}' for x in v)


def rpy_from_R(R):
    p = math.asin(-max(-1, min(1, R[2, 0])))
    if abs(math.cos(p)) > 1e-6:
        r = math.atan2(R[2, 1], R[2, 2]); y = math.atan2(R[1, 0], R[0, 0])
    else:
        r = 0.0; y = math.atan2(-R[0, 1], R[1, 1])
    return r, p, y


def inertial(m, com, I, rpy='0 0 0'):
    return (f'    <inertial><origin xyz="{f3(com)}" rpy="{rpy}"/><mass value="{m:.6g}"/>'
            f'<inertia ixx="{I[0,0]:.6g}" ixy="{I[0,1]:.6g}" ixz="{I[0,2]:.6g}" iyy="{I[1,1]:.6g}" iyz="{I[1,2]:.6g}" izz="{I[2,2]:.6g}"/></inertial>\n')


def frame_link(name):
    """A massless-in-practice frame link (camera / optical / tcp / root frames).

    It needs an explicit tiny <inertial>: without one, Isaac Sim's importer gives the link a
    default mass of 1 kg, which put 3 kg of phantom mass on every hand.
    """
    return f'  <link name="{name}">\n' + inertial(1e-3, (0, 0, 0), np.eye(3) * 1e-8) + '  </link>\n'


MATS = {}


def mat(rgba):
    key = tuple(round(x, 3) for x in rgba)
    if key not in MATS:
        MATS[key] = f'mat_{len(MATS):02d}'
    return MATS[key]


def vis_mesh(fname, color):
    rgba = list(color) + [1.0]
    return (f'    <visual><origin xyz="0 0 0" rpy="0 0 0"/><geometry><mesh filename="../meshes/visual/{fname}.stl"/></geometry>'
            f'<material name="{mat(rgba)}"/></visual>\n')


def col_mesh(fname):
    return f'    <collision><origin xyz="0 0 0" rpy="0 0 0"/><geometry><mesh filename="../meshes/collision/{fname}.stl"/></geometry></collision>\n'


def vprim(xyz, geom, rgba, rpy='0 0 0'):
    return f'    <visual><origin xyz="{xyz}" rpy="{rpy}"/><geometry>{geom}</geometry><material name="{mat(rgba)}"/></visual>\n'


def cprim(xyz, geom, rpy='0 0 0'):
    return f'    <collision><origin xyz="{xyz}" rpy="{rpy}"/><geometry>{geom}</geometry></collision>\n'


def bx(x, y, z): return f'<box size="{x:.6g} {y:.6g} {z:.6g}"/>'
def cy(r, l): return f'<cylinder radius="{r:.6g}" length="{l:.6g}"/>'
def sp(r): return f'<sphere radius="{r:.6g}"/>'


def joint(name, typ, parent, child, xyz, rpy='0 0 0', axis=None, lim=None, extra=''):
    s = f'  <joint name="{name}" type="{typ}"><parent link="{parent}"/><child link="{child}"/><origin xyz="{xyz}" rpy="{rpy}"/>'
    if axis: s += f'<axis xyz="{axis}"/>'
    if lim: s += f'<limit lower="{lim[0]:.6g}" upper="{lim[1]:.6g}" effort="{lim[2]:.6g}" velocity="{lim[3]:.6g}"/>'
    if len(lim or ()) > 4: s += f'<dynamics damping="{lim[4]:.6g}" friction="{lim[5]:.6g}"/>'
    return s + extra + '</joint>\n'


# ---------------------------------------------------------------- arm links and joints
D = math.radians
STS15 = (1.9, 4.7, 0.05, 0.02)          # effort N m, velocity rad/s, damping, friction (STS3215 7.4 V, the SO-100 version)


def arm_xml(side, links=None, inertia=None, shoulder=None, elbow=D(125), l1=None):
    """One arm. links / inertia / shoulder (limit tuple) override the default (prototype) ones: the mono-arm experiment."""
    links = links or LINKS; inertia = inertia or INERTIA; shoulder = shoulder or ((-D(115), D(115)) + STS15)
    p = side
    o = ''
    for lk in LINKF:
        m, com, I = inertia[lk]
        body = ''.join(vis_mesh(fn, c) for fn, c in links[lk]['vis']) + ''.join(col_mesh(fn) for fn in links[lk]['col'])
        o += f'  <link name="{p}_{lk}">\n' + inertial(m, com, I) + body + '  </link>\n'
    o += joint(f'{p}_shoulder_pan', 'revolute', f'{p}_base', f'{p}_upper_arm', '0 0 0', axis='0 0 1', lim=shoulder)
    o += joint(f'{p}_elbow_pan', 'revolute', f'{p}_upper_arm', f'{p}_forearm', f'{(l1 or L1)*MM:.6g} 0 0', axis='0 0 1', lim=(-elbow, elbow) + STS15)
    o += joint(f'{p}_wrist_pan', 'revolute', f'{p}_forearm', f'{p}_wrist', f'{L2*MM:.6g} 0 0', axis='0 0 1', lim=(-D(122), D(122)) + STS15)
    o += joint(f'{p}_wrist_pitch', 'revolute', f'{p}_wrist', f'{p}_tilt', f'{40*MM:.6g} 0 {PZ*MM:.6g}', axis='0 1 0', lim=(-D(15), D(115)) + STS15)
    o += joint(f'{p}_wrist_roll', 'revolute', f'{p}_tilt', f'{p}_hand', f'{ROLL_X*MM:.6g} 0 0', axis='1 0 0', lim=(-D(120), D(120)) + STS15)   # +-120 (user, 2026-11-09): cable loop
    o += joint(f'{p}_finger_right', 'prismatic', f'{p}_hand', f'{p}_finger_right', '0 0 0', axis='0 1 0',
               lim=(0.0, 0.045, 80.0, 0.056, 2.0, 0.5))
    o += joint(f'{p}_finger_left', 'prismatic', f'{p}_hand', f'{p}_finger_left', '0 0 0', axis='0 -1 0',
               lim=(0.0, 0.045, 80.0, 0.056, 2.0, 0.5))   # driven on its own: command it with the same value as finger_right
    # wrist camera optical frame (REP-103 optical: z forward, x right, y down)
    from arm import CAM_C, CAM_TILT
    camf = T(*CAM_C) @ Ry(CAM_TILT)
    r, pp, y = rpy_from_R(camf[:3, :3])
    o += frame_link(f'{p}_wrist_camera')
    o += joint(f'{p}_wrist_camera_fixed', 'fixed', f'{p}_tilt', f'{p}_wrist_camera', f3(camf[:3, 3] * MM), f'{r:.6g} {pp:.6g} {y:.6g}')
    o += frame_link(f'{p}_wrist_camera_optical')
    o += joint(f'{p}_wrist_camera_optical_fixed', 'fixed', f'{p}_wrist_camera', f'{p}_wrist_camera_optical', '0 0 0', '-1.5708 0 -1.5708')
    # tool centre point between the fingertips (12 mm in from the tip)
    o += frame_link(f'{p}_tcp')
    o += joint(f'{p}_tcp_fixed', 'fixed', f'{p}_hand', f'{p}_tcp', f'{(142.0 - 12.0)*MM:.6g} 0 0')
    return o


# ---------------------------------------------------------------- mobile base, lift, carriage (catalogue / placeholder parts)
SHELL = (0.94, 0.95, 0.96, 1); TRIM = (0.18, 0.21, 0.25, 1); ANOD = (0.30, 0.33, 0.37, 1); ALU = (0.78, 0.80, 0.83, 1)
ORANGE = (0.90, 0.45, 0.12, 1); BLUE = (0.29, 0.55, 0.80, 1); RED = (0.85, 0.16, 0.14, 1); RUBBER = (0.10, 0.11, 0.12, 1)


# ---------------------------------------------------------------- real C-Beam XLarge actuator (cbeam_import.py)
CB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'cbeam')
CB = json.load(open(os.path.join(CB_DIR, 'cbeam.json')))
CB_L = CB['_frame']['beam_len'] * MM                  # 1.000
CB_PH = CB['_frame']['plate_h'] * MM                  # 0.125
LIFT_STOP = 0.002                                     # lift ends: the plate stops 2 mm before each end mount
CB_TRAVEL = round(CB_L - CB_PH - 2 * LIFT_STOP, 4)    # 0.871
CB_PZ = 0.02 - CB_PH / 2                              # plate centre, carriage frame
CB_Z0 = 0.42 + CB_PZ - CB_PH / 2 - LIFT_STOP          # beam bottom (bottom end mount's inner face), base frame: 0.313
CB_PX = (CB['_frame']['plate_back_x'] + CB['_frame']['plate_front_x']) / 2 * MM
DECK_TOP = 0.246
CB_VIS, CB_PROPS = {'column': [], 'top_beam': [], 'carriage': []}, {'column': [], 'top_beam': [], 'carriage': []}
for g, link, dz, color in (('beam', 'column', 0.0, ANOD),
                           ('column_hw', 'column', 0.0, (0.78, 0.80, 0.83, 1)), ('motor', 'column', 0.0, (0.10, 0.11, 0.12, 1)),
                           ('top', 'top_beam', -CB_L / MM, (0.78, 0.80, 0.83, 1)), ('carriage', 'carriage', CB_PZ / MM, (0.30, 0.33, 0.37, 1))):
    T3 = read_stl(os.path.join(CB_DIR, g + '.stl')).astype(float) + np.array([0, 0, dz])   # (n, 3, 3) mm in the link frame
    V, F = T3.reshape(-1, 3), np.arange(len(T3) * 3).reshape(-1, 3)
    write_stl(os.path.join(VIS, 'cbeam_' + g + '.stl'), V * MM, F)
    CB_VIS[link].append(('cbeam_' + g, color[:3]))
    CB_PROPS[link].append(mass_props(V, F, mass=CB[g]['mass']))
    if g == 'carriage':                               # V-wheel envelope (beside the beam's 80 mm sides)
        w = V[(np.abs(V[:, 1]) > 37.0) & (V[:, 0] > -20.0)]
        CB_WX, CB_WY = (w[:, 0].min() + w[:, 0].max()) / 2 * MM, (np.abs(w[:, 1]).min() + np.abs(w[:, 1]).max()) / 2 * MM
        CB_WDX, CB_WDY = (w[:, 0].max() - w[:, 0].min()) * MM, (np.abs(w[:, 1]).max() - np.abs(w[:, 1]).min()) * MM
_hw = read_stl(os.path.join(CB_DIR, 'column_hw.stl')).reshape(-1, 3); _mo = read_stl(os.path.join(CB_DIR, 'motor.stl')).reshape(-1, 3)
_lo = np.vstack([_hw[_hw[:, 2] < 0], _mo]) * MM       # bottom end mount + spacers + coupler + motor
CB_BOTTOM_BOX = (f3((_lo.min(0) + _lo.max(0)) / 2), bx(*(_lo.max(0) - _lo.min(0))))
# ESTIMATE (ours, not the kit): a 6 mm aluminium angle on the beam's closed front face, its foot bolted to the deck
CB_FOOT = dict(x0=0.020, t=0.006, w=0.080, h=0.12, foot=0.08)
CB_FOOT_BOX = (f'{CB_FOOT["x0"] + CB_FOOT["t"]/2:.6g} 0 {DECK_TOP - CB_Z0 + CB_FOOT["h"]/2:.6g}', bx(CB_FOOT['t'], CB_FOOT['w'], CB_FOOT['h']))
CB_FOOT_VIS = (vprim(CB_FOOT_BOX[0], CB_FOOT_BOX[1], ALU) +
               vprim(f'{CB_FOOT["x0"] + CB_FOOT["t"] + 0.02:.6g} 0 {DECK_TOP - CB_Z0 + CB_FOOT["t"]/2:.6g}', bx(0.04, CB_FOOT['w'], CB_FOOT['t']), ALU))
CB_PROPS['column'].append(box_props(0.20, (CB_FOOT['t'], CB_FOOT['w'], CB_FOOT['h']), (CB_FOOT['x0'] + CB_FOOT['t'] / 2, 0, DECK_TOP - CB_Z0 + CB_FOOT['h'] / 2)))
MONO_LIFT_MAX = 0.81                                  # mono ring (layout B): its clamp meets the top end mount between 0.815 and 0.820


def base_xml(mono=False, side=False, single=False):
    o = frame_link('base_link')
    B = MH.BASE if single else (DH.BASE if not (mono or side) else 0.50)   # one-arm robot 0.45 since 2026-11-24, the dual
    # robot DH.BASE (0.45 since 2026-11-25); the parked ring robots keep 0.50
    r4 = lambda v_: round(v_, 4)
    h, sb, pc, pr, bm = r4(B / 2), r4(B / 2 - 0.0275), r4(B / 2 - 0.015), r4(B / 2 - 0.025), r4(B / 2 + 0.003)
    v = vprim('0 0 0.22', bx(B, B, 0.04), SHELL) + vprim('0 0 0.243', bx(B - 0.01, B - 0.01, 0.006), TRIM)
    v += vprim('0 0 0.08', bx(B, B - 0.11, 0.10), SHELL)
    for sy in (1, -1):
        v += vprim(f'0 {sy*sb} 0.08', bx(B - 0.26, 0.055, 0.10), SHELL)
    for x, y in [(pc, pc), (pc, -pc), (-pc, pc), (-pc, -pc), (-pr, 0.035), (-pr, -0.035)]:
        v += vprim(f'{x} {y} 0.165', bx(0.015, 0.015, 0.07), ALU)
    v += vprim(f'{bm} 0 0.07', bx(0.012, B - 0.02, 0.035), RUBBER) + vprim(f'-{bm} 0 0.07', bx(0.012, B - 0.02, 0.035), RUBBER)
    v += vprim(f'{r4(B / 2 - 0.002)} 0 0.205', bx(0.006, 0.30, 0.008), BLUE)
    c = cprim('0 0 0.22', bx(B, B, 0.04)) + cprim('0 0 0.08', bx(B, B - 0.11, 0.10))
    for sy in (1, -1):
        c += cprim(f'0 {sy*sb} 0.08', bx(B - 0.26, 0.055, 0.10))
    m, com, I = combine([box_props(14.0, (B, B - 0.11, 0.10), (0, 0, 0.08)), box_props(4.0, (B, B, 0.04), (0, 0, 0.22)),
                         box_props(1.0, (B - 0.03, B - 0.03, 0.07), (0, 0, 0.165))])
    o += '  <link name="base_body">\n' + inertial(m, com, I) + v + c + '  </link>\n'
    o += joint('base_body_fixed', 'fixed', 'base_link', 'base_body', '0 0 0')
    # mecanum wheels with passive rollers, standard layout: the ground-contact roller axes form an X
    # around the base centre, so a pure turn has lever arm LX + LY. With +wheel speed = roll forward:
    #   fl = (vx - vy - (LX+LY) wz) / R    fr = (vx + vy + (LX+LY) wz) / R
    #   rl = (vx + vy - (LX+LY) wz) / R    rr = (vx - vy + (LX+LY) wz) / R
    # (hand signs were the other way round before 2026-11-03: sideways moves came out reversed and a
    #  pure turn only had a lever arm of LY - LX = 5 cm.)
    R0, W, RR, NR = 0.05, 0.045, 0.011, 9
    WX, WY = r4(B / 2 - 0.08), r4(B / 2 - 0.028)          # wheel centres (0.17, 0.222 on the 0.50 base)
    for nm, x, y, hand in (('wheel_fl', WX, WY, -1), ('wheel_fr', WX, -WY, 1), ('wheel_rl', -WX, WY, 1), ('wheel_rr', -WX, -WY, -1)):
        hub = vprim('0 0 0', cy(0.030, W + 0.006), (0.62, 0.66, 0.70, 1), '1.5708 0 0')
        m, com, I = cyl_props(0.30, 0.03, W, axis=1)
        o += f'  <link name="{nm}">\n' + inertial(m, com, I) + hub + cprim('0 0 0', cy(0.028, W), '1.5708 0 0') + '  </link>\n'
        o += joint(f'{nm}_joint', 'continuous', 'base_link', nm, f'{x} {y} {R0}', axis='0 1 0', lim=(-1e9, 1e9, 6.0, 25.0, 0.1, 0.05))
        for k in range(NR):
            t = 2 * math.pi * k / NR
            rad = np.array([math.sin(t), 0, math.cos(t)]); ax = np.array([0, 1.0, 0])
            c_, s_ = math.cos(math.pi / 4 * hand), math.sin(math.pi / 4 * hand)
            K = np.array([[0, -rad[2], rad[1]], [rad[2], 0, -rad[0]], [-rad[1], rad[0], 0]])
            a = (np.eye(3) + s_ * K + (1 - c_) * K @ K) @ ax
            z = np.array([0, 0, 1.0]); vv = np.cross(z, a); sn = np.linalg.norm(vv); cc = z @ a
            Kv = np.array([[0, -vv[2], vv[1]], [vv[2], 0, -vv[0]], [-vv[1], vv[0], 0]])
            Rm = np.eye(3) + Kv + Kv @ Kv * ((1 - cc) / sn ** 2) if sn > 1e-9 else np.eye(3)
            r_, p_, y_ = rpy_from_R(Rm); pos = (R0 - RR) * rad
            rn = f'{nm}_roller_{k}'
            vis = vprim('0 0 0', cy(RR, 0.040), (0.13, 0.16, 0.19, 1))
            col = cprim('0 0 0', sp(RR)) + cprim('0 0 0.016', sp(0.0094)) + cprim('0 0 -0.016', sp(0.0094))
            m, com, I = cyl_props(0.012, RR, 0.04)
            o += f'  <link name="{rn}">\n' + inertial(m, com, I) + vis + col + '  </link>\n'
            o += joint(f'{rn}_joint', 'continuous', nm, rn, f3(pos), f'{r_:.6g} {p_:.6g} {y_:.6g}', axis='0 0 1',
                       lim=(-1e9, 1e9, 0.0, 200.0, 0.0005, 0.0))
    # lidar
    o += '  <link name="lidar_link">\n' + inertial(*cyl_props(0.17, 0.035, 0.055)) + vprim('0 0 0', cy(0.035, 0.055), TRIM) + '  </link>\n'
    o += joint('lidar_fixed', 'fixed', 'base_link', 'lidar_link', '0.04 0 0.165')
    o += lift_single() if single else lift_prototype(mono, side)
    return o


def tote_link(L, Wt, H, t):
    """The open tote on the base deck (simple boxes, estimated mass), link origin at its floor's underside centre."""
    tv = vprim(f'0 0 {t/2}', bx(L, Wt, t), BLUE)
    for sx in (1, -1): tv += vprim(f'{sx*(L/2-t/2)} 0 {H/2}', bx(t, Wt, H), BLUE)
    for sy in (1, -1): tv += vprim(f'0 {sy*(Wt/2-t/2)} {H/2}', bx(L - 2 * t, t, H), BLUE)
    tc = cprim(f'0 0 {t/2}', bx(L, Wt, t)) + ''.join(cprim(f'{sx*(L/2-t/2)} 0 {H/2}', bx(t, Wt, H)) for sx in (1, -1)) + \
         ''.join(cprim(f'0 {sy*(Wt/2-t/2)} {H/2}', bx(L - 2 * t, t, H)) for sy in (1, -1))
    return '  <link name="tote">\n' + inertial(*box_props(1.05 * L / 0.35, (L, Wt, H), (0, 0, 0.04))) + tv + tc + '  </link>\n'


def lift_prototype(mono=False, side=False):
    """Prototype: column 0.18 m toward the front (working face), arms right beside it, tote behind it,
    scene camera on a carriage bracket in front of the column. Compact carriage: gantry plate on the column's rear face + two 8 mm arm brackets."""
    o = ''
    CX, CY = (SIDE_CX, SIDE_CY) if side else ((PROTO_CX, 0) if mono else (DH.ROBOT_X, 0))   # dual: column with the base
    # tote behind the column (x -0.245 .. 0.105), full width
    L, Wt, H, t = 0.35, 0.48, 0.12, 0.007; TX = -0.07
    if not mono and not side:     # the dual robot: the box sized to its base (dual_holder.dual_tote, 0.45 since 2026-11-25)
        T_ = DH.dual_tote(); L, Wt = T_['x'][1] - T_['x'][0] + 2 * t, 2 * T_['y'][1] + 2 * t; TX = (T_['x'][0] + T_['x'][1]) / 2
        o += tote_link(L, Wt, H, t)
        o += joint('tote_fixed', 'fixed', 'base_link', 'tote', f'{TX:.6g} 0 0.256')
    else:
        o += tote_link(L, Wt, H, t)
    if side:                      # turned lengthwise along the left side, beside the column
        o += joint('tote_fixed', 'fixed', 'base_link', 'tote', f'{SIDE_TOTE[0]} {SIDE_TOTE[1]} 0.256', f'0 0 {SIDE_TOTE[2]:.6g}')
    elif mono:
        o += joint('tote_fixed', 'fixed', 'base_link', 'tote', f'{TX} 0 0.256')   # the ring robots: the old flat box
    # Lift (user's hardware): OpenBuilds C-Beam XLarge actuator, 1000 mm, from the kit's STEP (cbeam_import.py ->
    # cbeam/*.stl, every part as in the kit; the 500 mm beam and lead screw stretched to 1000). NEMA 23 at the BOTTOM,
    # on 40 mm spacers under the bottom end mount: it goes through a hole in the deck (motor bottom at base z 0.205).
    # The beam's open 80 mm face (gantry plate, V-wheels) faces the rear (tote): 40 mm along x, 80 along y.
    # Heights are set by the carriage: carriage origin at base z 0.42 at lift 0 (arm / IK heights unchanged), plate top at
    # carriage z 0.02, lift 0 = plate LIFT_STOP above the bottom end mount -> beam bottom at base z CB_Z0.
    o += '  <link name="column">\n' + inertial(*combine(CB_PROPS['column'])) + ''.join(vis_mesh(f, c) for f, c in CB_VIS['column']) + CB_FOOT_VIS + \
         cprim(f'0 0 {CB_L/2:.6g}', bx(0.04, 0.08, CB_L)) + cprim(*CB_BOTTOM_BOX) + cprim(*CB_FOOT_BOX) + '  </link>\n'
    o += joint('column_fixed', 'fixed', 'base_link', 'column', f'{CX} {CY} {CB_Z0:.6g}')
    o += '  <link name="top_beam">\n' + inertial(*combine(CB_PROPS['top_beam'])) + ''.join(vis_mesh(f, c) for f, c in CB_VIS['top_beam']) + \
         cprim('-0.005 0 0.006', bx(0.05, 0.08, 0.012)) + '  </link>\n'
    o += joint('top_beam_fixed', 'fixed', 'base_link', 'top_beam', f'{CX} {CY} {CB_Z0 + CB_L:.6g}')
    o += '  <link name="estop">\n' + inertial(*cyl_props(0.03, 0.022, 0.02)) + vprim('0 0 0', cy(0.022, 0.02), RED) + '  </link>\n'
    o += joint('estop_fixed', 'fixed', 'top_beam', 'estop', '-0.005 0 0.022')
    o += SCENE_XML['prototype_mono_ring' if mono else 'prototype']
    # carriage (origin on the column axis): the kit's gantry plate (125 x 125 x 6, 1.5 mm behind the beam's open face) with
    # its 4 V-wheels in the beam's V-grooves and the lead-screw nut block, + (two-arm robot) the printed dual-arm holder
    # (dual_holder.py, since 2026-11-13; was two estimated 8 mm aluminium brackets). The plate's top is at carriage
    # z 0.02: the arms sweep 0.032 m above it, and an upper arm pointing back over the column would hit a taller one.
    cv = ''.join(vis_mesh(f, c) for f, c in CB_VIS['carriage'])
    cc_ = cprim(f'{CB_PX:.6g} 0 {CB_PZ:.6g}', bx(0.006, 0.125, 0.125))
    for sy in (1, -1):                                                               # V-wheels (beside the beam's sides)
        cc_ += cprim(f'{CB_WX:.6g} {sy*CB_WY:.6g} {CB_PZ:.6g}', bx(CB_WDX, CB_WDY, 0.125))
    props = list(CB_PROPS['carriage'])
    if not mono:                  # the printed dual-arm holder (dual_holder.py): both halves on the carriage
        cv += ''.join(vis_mesh(f, c) for f, c in HOLDER['vis']); cc_ += ''.join(col_mesh(f) for f in HOLDER['col'])
        props += HOLDER['props']
    o += '  <link name="carriage">\n' + inertial(*combine(props)) + cv + cc_ + '  </link>\n'
    o += joint('lift', 'prismatic', 'base_link', 'carriage', f'{CX} {CY} 0.42', axis='0 0 1',
               lim=(0.0, MONO_LIFT_MAX if mono else CB_TRAVEL, 300.0, LIFT_SPEED, 50.0, 5.0))
    if mono:                      # mono: mono_mast_xml() from the ring's clamp socket (added with the arm)
        return o
    # scene camera: OAK-D Pro on a pitch servo, on a mast in the dual holder's front bridge (scene_cam.py, SCENE_XML)
    return o


def lift_single():
    """One-arm prototype (mono_holder.py): column at (0, SINGLE_CY) turned SINGLE_YAW, carriage = kit plate + the mono
    holder, the scene camera mast on the holder's camera arm; the tote on the base behind the arm (MH.BASE_TOTE)."""
    T_ = MH.BASE_TOTE                                  # the whole-deck L tote with sloped edges (mono_holder.tote_solid)
    o = '  <link name="tote">\n' + inertial(*TOTE1['inertial']) + ''.join(vis_mesh(f, c) for f, c in TOTE1['vis']) + \
        ''.join(col_mesh(f) for f in TOTE1['col']) + '  </link>\n'
    o += joint('tote_fixed', 'fixed', 'base_link', 'tote', '0 0 0')
    CX, CY, yaw = SINGLE_CX, SINGLE_CY, f'0 0 {SINGLE_YAW:.6g}'
    o += '  <link name="column">\n' + inertial(*combine(CB_PROPS['column'])) + ''.join(vis_mesh(f, c) for f, c in CB_VIS['column']) + CB_FOOT_VIS + \
         cprim(f'0 0 {CB_L/2:.6g}', bx(0.04, 0.08, CB_L)) + cprim(*CB_BOTTOM_BOX) + cprim(*CB_FOOT_BOX) + '  </link>\n'
    o += joint('column_fixed', 'fixed', 'base_link', 'column', f'{CX} {CY} {CB_Z0:.6g}', yaw)
    o += '  <link name="top_beam">\n' + inertial(*combine(CB_PROPS['top_beam'])) + ''.join(vis_mesh(f, c) for f, c in CB_VIS['top_beam']) + \
         cprim('-0.005 0 0.006', bx(0.05, 0.08, 0.012)) + '  </link>\n'
    o += joint('top_beam_fixed', 'fixed', 'base_link', 'top_beam', f'{CX} {CY} {CB_Z0 + CB_L:.6g}', yaw)
    o += '  <link name="estop">\n' + inertial(*cyl_props(0.03, 0.022, 0.02)) + vprim('0 0 0', cy(0.022, 0.02), RED) + '  </link>\n'
    o += joint('estop_fixed', 'fixed', 'top_beam', 'estop', '-0.005 0 0.022')
    cv = ''.join(vis_mesh(f, c) for f, c in CB_VIS['carriage']) + ''.join(vis_mesh(f, c) for f, c in SHOLDER['holder']['vis'])
    cc_ = cprim(f'{CB_PX:.6g} 0 {CB_PZ:.6g}', bx(0.006, 0.125, 0.125)) + ''.join(col_mesh(f) for f in SHOLDER['holder']['col'])
    for sy in (1, -1):
        cc_ += cprim(f'{CB_WX:.6g} {sy*CB_WY:.6g} {CB_PZ:.6g}', bx(CB_WDX, CB_WDY, 0.125))
    o += '  <link name="carriage">\n' + inertial(*combine(list(CB_PROPS['carriage']) + SHOLDER['holder']['props'])) + cv + cc_ + '  </link>\n'
    o += joint('lift', 'prismatic', 'base_link', 'carriage', f'{CX} {CY} 0.42', yaw, axis='0 0 1',
               lim=(MH.LIFT_MIN, CB_TRAVEL, 300.0, LIFT_SPEED, 50.0, 5.0))
    o += SCENE_XML['prototype_mono_arm']
    return o


SHOLDER = {k: dict(vis=[], col=[], props=[]) for k in ('holder',)}
TOTE1 = dict(vis=[], col=[])


def _tote_links(prefix='mtote_'):
    """The one-arm robot's tote (mono_holder.tote_solid, robot frame): visual mesh; collision = floor + slopes and the
    walls as separate convex pieces (a hull over a floor + wall corner would fill the corner)."""
    t = MH.BASE_TOTE; store = TOTE1
    for f in os.listdir(VIS) + os.listdir(COL):
        if f.startswith(prefix):
            for d in (VIS, COL):
                if os.path.exists(os.path.join(d, f)): os.remove(os.path.join(d, f))
    ts = MH.tote_solid(t); z0, w = t['z'][0] * 1000, t['wall'] * 1000
    TOTE1_ = store
    inner = union([box(r[0] * 1000, r[1] * 1000, r[2] * 1000, r[3] * 1000, z0 - 1, z0 + 1000) for r in t['rects']])
    floor = ts ^ inner; wall = ts - inner
    V, F = mesh_m(ts); write_stl(os.path.join(VIS, prefix + 'tote.stl'), V, F); TOTE1_['vis'].append((prefix + 'tote', BLUE[:3]))
    k = 0
    for part, cell in ((floor, 45.0), (wall, 45.0)):
        for hh in convex_pieces(part, cell=cell):
            Vc, Fc = mesh_arrays(hh); fn = f'{prefix}c{k:02d}'; k += 1
            write_stl(os.path.join(COL, fn + '.stl'), Vc * MM, Fc); TOTE1_['col'].append(fn)
    area = sum((r[1] - r[0]) * (r[3] - r[2]) for r in t['rects'])
    TOTE1_['inertial'] = mass_props(V / MM, F, mass=round(1.05 * area / (0.35 * 0.48) * 1.3, 3))   # sheet + the slope inserts


def _single_holder_links():
    """mono_holder.py's holder (-> carriage), PETG, carriage frame."""
    for f in os.listdir(VIS) + os.listdir(COL):
        if f.startswith('mholder_'):
            for d in (VIS, COL):
                if os.path.exists(os.path.join(d, f)): os.remove(os.path.join(d, f))
    MH.build_all()
    for nm, p in MH.PARTS.items():
        fname = 'mholder_' + nm; L_ = SHOLDER[nm]
        V, F = mesh_m(p['solid']); write_stl(os.path.join(VIS, fname + '.stl'), V, F)
        L_['vis'].append((fname, p['color'])); L_['props'].append(mass_props(V / MM, F, rho=RHO_PETG))
        for k, h in enumerate(convex_pieces(p['solid'])):
            Vc, Fc = mesh_arrays(h); fn = f'{fname}_c{k:02d}'
            write_stl(os.path.join(COL, fn + '.stl'), Vc * MM, Fc); L_['col'].append(fn)


import dual_holder as DH
HOLDER = dict(vis=[], col=[], props=[])


def _holder_links():
    """dual_holder.py's two printed halves (PETG) as carriage meshes (carriage frame mm = carriage link frame)."""
    DH.build_all()
    for nm, p in DH.PARTS.items():
        fname = 'dual_' + nm
        V, F = mesh_m(p['solid']); write_stl(os.path.join(VIS, fname + '.stl'), V, F)
        HOLDER['vis'].append((fname, p['color'])); HOLDER['props'].append(mass_props(V / MM, F, rho=RHO_PETG))
        for k, h in enumerate(convex_pieces(p['solid'])):
            Vc, Fc = mesh_arrays(h); fn = f'{fname}_c{k:02d}'
            write_stl(os.path.join(COL, fn + '.stl'), Vc * MM, Fc); HOLDER['col'].append(fn)


_holder_links()
_single_holder_links()
_tote_links()


def _scene_cam_xml(layout, mast_T=None):
    """Scene camera (scene_cam.py: SC.CAMERA, the Orbbec Gemini 2 since 2026-11-24) on an STS3215 pitch servo, on a mast on the carriage (two-arm: the dual
    holder's front bridge; one-arm: the ring clamp's pedestal); it moves with the lift. scene_mast (fixed to the
    carriage) -> scene_camera_pitch (revolute about y, + = tilt down, -10 .. +80 deg) -> scene_camera (origin on the
    tilt axis, x forward) -> scene_camera_optical (the RGB lens)."""
    import scene_cam as SC
    SC.build_all(layout)
    mast = dict(vis=[], col=[], props=[]); cam = dict(vis=[], col=[], props=[])
    def put(L, fname, solid, color, mass=None, rho=None, col=True):
        V, F = mesh_m(solid); write_stl(os.path.join(VIS, fname + '.stl'), V, F)
        L['vis'].append((fname, color)); L['props'].append(mass_props(V / MM, F, mass=mass, rho=rho))
        if col:
            for k, h in enumerate(convex_pieces(solid)):
                Vc, Fc = mesh_arrays(h); fn = f'{fname}_c{k:02d}'
                write_stl(os.path.join(COL, fn + '.stl'), Vc * MM, Fc); L['col'].append(fn)
    for nm in [k for k in SC.PARTS if k != 'camera_cradle']:
        put(mast, 'scene_' + nm, SC.PARTS[nm]['solid'], SC.PARTS[nm]['color'], rho=RHO_PETG)
    put(mast, 'scene_pitch_STS3215', SC.CONTEXT['servo'], (0.12, 0.12, 0.14), mass=0.055, col=False)
    toax = T(-SC.CAM['axis_x'], 0, 0)                       # camera frame C -> the scene_camera link (origin on the axis)
    put(cam, 'scene_camera_cradle', xf(SC.CONTEXT['cradle_C'], toax), SC.PARTS['camera_cradle']['color'], rho=RHO_PETG)
    E = SC.camera_mesh_C() + np.array([-SC.CAM['axis_x'], 0, 0])
    V = E.reshape(-1, 3); F = np.arange(len(V)).reshape(-1, 3)
    write_stl(os.path.join(VIS, 'scene_camera_body.stl'), V * MM, F); cam['vis'].append(('scene_camera_body', (0.15, 0.15, 0.17)))
    body = xf(SC.CONTEXT['cam_body_C'], toax)
    Vb, Fb = mesh_arrays(body); cam['props'].append(mass_props(Vb, Fb, mass=SC.CAM['mass']))
    for k, h in enumerate(convex_pieces(body)):
        Vc, Fc = mesh_arrays(h); fn = f'scene_camera_body_c{k:02d}'
        write_stl(os.path.join(COL, fn + '.stl'), Vc * MM, Fc); cam['col'].append(fn)
    def link(name, L):
        return (f'  <link name="{name}">\n' + inertial(*combine(L['props'])) + ''.join(vis_mesh(f, c) for f, c in L['vis']) +
                ''.join(col_mesh(f) for f in L['col']) + '  </link>\n')
    if mast_T is None:
        o = link('scene_mast', mast) + joint('scene_mast_fixed', 'fixed', 'carriage', 'scene_mast', '0 0 0')
    else:                                                    # mono holder: the mast turned onto its camera arm
        r_, p_, y_ = rpy_from_R(mast_T[:3, :3])
        o = link('scene_mast', mast) + joint('scene_mast_fixed', 'fixed', 'carriage', 'scene_mast', f3(mast_T[:3, 3] * MM),
                                             f'{r_:.6g} {p_:.6g} {y_:.6g}')
    o += link('scene_camera', cam)
    ax = (SC.AX * MM, 0.0, (SC.CONTEXT['zh'] + SC.HEAD_AXIS) * MM)
    o += joint('scene_camera_pitch', 'revolute', 'scene_mast', 'scene_camera', f3(ax), axis='0 1 0',
               lim=(math.radians(SC.PITCH[0]), math.radians(SC.PITCH[1]), 1.9, 4.7, 0.05, 0.02))
    lx, ly, lz = SC.CAM['lens']
    o += frame_link('scene_camera_optical')
    o += joint('scene_camera_optical_fixed', 'fixed', 'scene_camera', 'scene_camera_optical', f3(((lx - SC.CAM['axis_x']) * MM, ly * MM, lz * MM)), '-1.5708 0 -1.5708')
    return o


for _f in os.listdir(VIS) + os.listdir(COL):
    if _f.startswith('scene_'):
        for _d in (VIS, COL):
            if os.path.exists(os.path.join(_d, _f)): os.remove(os.path.join(_d, _f))
SCENE_XML = {lay: _scene_cam_xml(lay) for lay in ('prototype', 'prototype_mono_ring')}
SCENE_XML['prototype_mono_arm'] = _scene_cam_xml('prototype_mono_arm', MH.MAST_T)
_mono_links()
MINERTIA = {k: combine(v['props']) for k, v in MLINKS.items()}

ARM_Z = 0.0255 + 0.040        # arm frame origin: 40 mm above the 8 mm mount plate whose top is at carriage z 0.0255
ARMS = ''.join(arm_xml(side) for side in ('left', 'right'))
BODY = {}
for layout in ('prototype',):
    body = base_xml() + ARMS
    for side, sy in (('left', 1), ('right', -1)):
        body += joint(f'{side}_arm_mount', 'fixed', 'carriage', f'{side}_base', f'0 {sy*PROTO_ARM_Y} {ARM_Z}', f'0 0 {sy*PROTO_YAW:.6g}')
    BODY[layout] = body
# mono-arm experiment: one arm on the shoulder ring around the column (ring drive: STS3215 x 4:1 gear)
_ab = ''.join(vis_mesh(fn, c) for fn, c in LINKS['base']['vis']) + ''.join(col_mesh(fn) for fn in LINKS['base']['col'])
def _mono_body(side=False):
    return (base_xml(mono=True, side=side) + arm_xml('left', MLINKS, MINERTIA, (-D(180), D(180), RING_TORQUE,
                               4.7 / MONO['ratio'], 0.05, 0.02), elbow=D(115), l1=MONO['l1']) +   # elbow = the base's shoulder servo: +-115 as on the prototype
                               '  <link name="left_arm_base">\n' + inertial(*INERTIA['base']) + _ab + '  </link>\n' +
                               joint('left_arm_base_fixed', 'fixed', 'left_upper_arm', 'left_arm_base', f"{MONO['l1']*MM:.6g} 0 0") +
                               joint('left_arm_mount', 'fixed', 'carriage', 'left_base', f"0 0 {MONO['az']*MM:.6g}"))


# PARKED since 2026-11-23 (the one-arm prototype is the mono holder now): the ring layouts, URDF only
BODY['prototype_mono_ring'] = _mono_body()
BODY['prototype_mono_ring_side'] = _mono_body(side=True)  # TEST layout (2026-11-18): column on the right side, tote along the left
# THE one-arm prototype: mono holder (mono_holder.py); the prototype's own arm, shoulder on the centreline
BODY['prototype_mono_arm'] = (base_xml(single=True) + arm_xml('left') +
                              joint('left_arm_mount', 'fixed', 'carriage', 'left_base', f'{-MH.ARM_DX * MM:.6g} {MH.ARM_DY * MM:.6g} {ARM_Z}', f'0 0 {math.radians(MH.ARM_YAW):.6g}'))

mats = ''.join(f'  <material name="{n}"><color rgba="{f3(k)}"/></material>\n' for k, n in MATS.items())
xml = ('<?xml version="1.0" encoding="utf-8"?>\n'
       '<!-- Wheeled dual-arm picking robot (prototype). Units: metres, kg, radians.\n'
       '     Arms: meshes, masses and inertias computed from printed-part CAD (PLA at 0.781 g/cm3 effective),\n'
       '     STS3215 55 g (every joint, same servo as the SO-100), 6808 25 g, MGN9 rail 65 g, MGN9H 26 g, plus every screw and insert.\n'
       '     Base, lift and carriage: primitive shapes with estimated masses.\n'
       '     Joint limits are the collision-free ranges of the CAD minus a 5 deg margin. -->\n'
       '<robot name="mobile_manipulator">\n' + mats + '%BODY%' + '</robot>\n')
for layout, body in BODY.items():
    used = ''.join(l for l in mats.splitlines(True) if f'name="{l.split(chr(34))[1]}"' in body)   # only this layout's materials
    x = xml.replace(mats, used).replace('%BODY%', body).replace('<!-- Wheeled dual-arm picking robot (prototype).',
                                            f'<!-- Wheeled dual-arm picking robot, layout {layout}.')
    open(os.path.join(OUT, 'urdf', f'mobile_manipulator_{layout}.urdf'), 'w').write(x)
    open(os.path.join(OUT, 'urdf', f'mobile_manipulator_{layout}_ros.urdf'), 'w').write(
        x.replace('filename="../meshes/', 'filename="package://mobile_manipulator_description/meshes/'))
summary = {k: dict(mass_kg=round(v[0], 4), com_m=[round(x, 4) for x in v[1]]) for k, v in INERTIA.items()}
json.dump(summary, open(os.path.join(OUT, 'arm_link_masses.json'), 'w'), indent=1)
print(json.dumps(summary, indent=1))
print('collision pieces', {k: len(v['col']) for k, v in LINKS.items()})
