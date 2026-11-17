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
