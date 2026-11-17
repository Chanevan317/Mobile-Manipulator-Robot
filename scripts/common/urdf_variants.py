"""Temporary URDF copies for the Isaac view scripts (never written into urdf/; the generated URDFs stay untouched).

- renamed(): the whole robot under another name (two robots in one stage need different prim names).
- arm_only(): one arm cut out of a robot URDF (left_base and everything below it), on a fixed stand link.
Mesh paths become absolute, so the copies work from a temp folder.
"""
import os, tempfile
import xml.etree.ElementTree as ET

HERE = os.path.dirname(os.path.abspath(__file__))
import repo_paths as RP
URDF_DIR = RP.URDF_DIR


def layout_urdf(layout):
    return os.path.join(URDF_DIR, f'mobile_manipulator_{layout}.urdf')


def _absolute_meshes(root, src_dir):
    for m in root.iter('mesh'):
        fn = m.get('filename')
        if fn and not os.path.isabs(fn) and '://' not in fn:
            m.set('filename', os.path.normpath(os.path.join(src_dir, fn)))


def _write(root, name):
    out = os.path.join(tempfile.mkdtemp(prefix='mm_urdf_'), name + '.urdf')
    ET.ElementTree(root).write(out, xml_declaration=True, encoding='utf-8')
    return out


def renamed(src, name):
    root = ET.parse(src).getroot()
    _absolute_meshes(root, os.path.dirname(os.path.abspath(src)))
    root.set('name', name)
    return _write(root, name)


def arm_only(src, side='left', name='arm', stand_z=0.30):
    """Keep <side>_base and every link below it; a 'stand' link holds it at stand_z, arm pointing along +x."""
    root = ET.parse(src).getroot()
    _absolute_meshes(root, os.path.dirname(os.path.abspath(src)))
    joints = root.findall('joint')
    keep = {f'{side}_base'}
    grew = True
    while grew:
        grew = False
        for j in joints:
            p, c = j.find('parent').get('link'), j.find('child').get('link')
            if p in keep and c not in keep:
                keep.add(c); grew = True
    for el in list(root):
        if el.tag == 'link' and el.get('name') not in keep:
            root.remove(el)
        elif el.tag == 'joint' and not (el.find('child').get('link') in keep and el.find('parent').get('link') in keep):
            root.remove(el)
    stand = ET.Element('link', name='stand')
    inert = ET.SubElement(stand, 'inertial')
    ET.SubElement(inert, 'origin', xyz='0 0 0', rpy='0 0 0')
    ET.SubElement(inert, 'mass', value='0.001')
    ET.SubElement(inert, 'inertia', ixx='1e-6', ixy='0', ixz='0', iyy='1e-6', iyz='0', izz='1e-6')
    j = ET.Element('joint', name='stand_mount', type='fixed')
    ET.SubElement(j, 'parent', link='stand'); ET.SubElement(j, 'child', link=f'{side}_base')
    ET.SubElement(j, 'origin', xyz=f'0 0 {stand_z}', rpy='0 0 0')
    root.insert(0, j); root.insert(0, stand)
    root.set('name', name)
    return _write(root, name)


PLANAR = dict(v_lin=1.2, v_yaw=2.0, f_lin=1000.0, t_yaw=500.0)     # base joint speed / effort limits (URDF; the moves
                                                                    # use 1 m/s, 1.5 rad/s: headroom to catch up the lag)


def _origin(o):
    """4x4 from a URDF <origin> (None = identity)"""
    import numpy as np
    xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]
    r, p, y = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    T = np.eye(4)
    T[:3, :3] = [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                 [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                 [-sp, cp * sr, cp * cr]]
    T[:3, 3] = xyz
    return T


def _rpy(R):
    import math
    p = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    if abs(R[2, 0]) < 0.999999:
        return math.atan2(R[2, 1], R[2, 2]), p, math.atan2(R[1, 0], R[0, 0])
    return math.atan2(-R[1, 2], R[1, 1]), p, 0.0


def planar_base(src, name='mobile_manipulator'):
    """The robot for ARM training (2026-11-17, user: "arm training fidelity ... the robot can move from a to b easily by
    script"): the mecanum wheels and their 36 rollers stay as LOOKS only (user: "keep the wheels but no need to spin them"):
    each roller's visual is folded into its wheel link (no extra links: the articulation stays small), the wheels are
    fixed to the base and have no collisions,
    and base_link rides on three driven joints from a floor anchor: base_x, base_y (prismatic) and base_yaw (revolute).
    Import it with fix_base=True; the base then moves by joint targets (stiff drives, drives.gains) instead of wheel
    speeds, and holds still while the arm works. Navigation is not simulated here (handled separately)."""
    root = ET.parse(src).getroot()
    _absolute_meshes(root, os.path.dirname(os.path.abspath(src)))
    links = {l.get('name'): l for l in root.findall('link')}
    for j in [j for j in root.findall('joint') if 'roller' in (j.get('name') or '')]:
        parent, child = links[j.find('parent').get('link')], links[j.find('child').get('link')]
        Tj = _origin(j.find('origin'))
        for v in child.findall('visual'):                       # roller visual -> the wheel link, at the roller's pose
            o = v.find('origin'); To = Tj @ _origin(o)
            if o is None: o = ET.SubElement(v, 'origin')
            o.set('xyz', ' '.join(f'{x:.6g}' for x in To[:3, 3])); o.set('rpy', ' '.join(f'{x:.6g}' for x in _rpy(To[:3, :3])))
            parent.append(v)
        root.remove(j); root.remove(child)
    for j in root.findall('joint'):
        if j.get('name', '').startswith('wheel_') and j.get('type') == 'continuous':
            j.set('type', 'fixed')
            for tag in ('axis', 'limit', 'dynamics'):
                for e in j.findall(tag): j.remove(e)
    for l in root.findall('link'):
        if l.get('name', '').startswith('wheel_'):
            for c in l.findall('collision'): l.remove(c)

    def link(nm, mass):
        l = ET.Element('link', name=nm); i = ET.SubElement(l, 'inertial')
        ET.SubElement(i, 'origin', xyz='0 0 0', rpy='0 0 0'); ET.SubElement(i, 'mass', value=str(mass))
        I = str(mass * 1e-2)
        ET.SubElement(i, 'inertia', ixx=I, ixy='0', ixz='0', iyy=I, iyz='0', izz=I)
        return l

    def joint(nm, kind, parent, child, axis, vel, eff):
        j = ET.Element('joint', name=nm, type=kind)
        ET.SubElement(j, 'parent', link=parent); ET.SubElement(j, 'child', link=child)
        ET.SubElement(j, 'origin', xyz='0 0 0', rpy='0 0 0'); ET.SubElement(j, 'axis', xyz=axis)
        lim = dict(effort=str(eff), velocity=str(vel))
        if kind == 'prismatic': lim.update(lower='-20', upper='20')
        ET.SubElement(j, 'limit', **lim)
        return j
    P = PLANAR
    new = [link('base_anchor', 1.0), link('base_x_link', 0.5), link('base_y_link', 0.5),
           joint('base_x', 'prismatic', 'base_anchor', 'base_x_link', '1 0 0', P['v_lin'], P['f_lin']),
           joint('base_y', 'prismatic', 'base_x_link', 'base_y_link', '0 1 0', P['v_lin'], P['f_lin']),
           joint('base_yaw', 'continuous', 'base_y_link', 'base_link', '0 0 1', P['v_yaw'], P['t_yaw'])]
    for k, el in enumerate(new):
        root.insert(k, el)
    root.set('name', name)
    return _write(root, name + '_planar')
