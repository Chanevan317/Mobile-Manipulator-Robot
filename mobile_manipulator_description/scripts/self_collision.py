"""Turn on self-collision for the imported robot, ignoring the same pairs as generator/urdf_selfcol.py.

PhysX only ignores contacts between two links joined *directly* by a joint. With merge_fixed_joints off,
parts welded together by fixed joints are separate links, so the column, carriage, tote, base body etc.
(all children of base_link) would collide with each other although they are built to touch. Here, as in
urdf_selfcol.py:
  - links joined by fixed joints count as one body;
  - a body never collides with itself or with the body it is jointed to (parent / child);
  - mecanum rollers never collide with each other.
Every other pair of links collides, for example arm vs arm, arm vs column, or one jaw vs the other jaw.

Call setup_self_collision() after the URDF import (or before saving a USD), before the simulation starts.
"""
import itertools
import xml.etree.ElementTree as ET


def ignored_pairs(urdf_path):
    """Return {(link_a, link_b), ...} for link pairs that must not collide."""
    root = ET.parse(urdf_path).getroot()
    links = [l.get('name') for l in root.findall('link')]
    parent, fixed_child = {}, set()
    for j in root.findall('joint'):
        c, p = j.find('child').get('link'), j.find('parent').get('link')
        parent[c] = p
        if j.get('type') == 'fixed':
            fixed_child.add(c)

    def body(l):
        while l in fixed_child:
            l = parent[l]
        return l

    body_parent = {body(c): body(p) for c, p in parent.items() if body(c) != body(p)}
    out = set()
    for a, b in itertools.combinations(links, 2):
        ba, bb = body(a), body(b)
        if ba == bb or body_parent.get(ba) == bb or body_parent.get(bb) == ba or ('roller' in a and 'roller' in b):
            out.add((a, b))
    return out


def setup_self_collision(stage, urdf_path, robot_root='/mobile_manipulator', ignore_internal_arm_collisions=False):
    """Enable self-collision on the articulation and add the filtered pairs. Returns the number of pairs."""
    from pxr import UsdPhysics, PhysxSchema, Sdf
    prims = {p.GetName(): p for p in stage.Traverse()
             if str(p.GetPath()).startswith(robot_root + '/') and p.HasAPI(UsdPhysics.RigidBodyAPI)}
    pairs = ignored_pairs(urdf_path)
    if ignore_internal_arm_collisions:
        links = [link.get('name') for link in ET.parse(urdf_path).getroot().findall('link')]
        pairs.update((a, b) for a, b in itertools.combinations(links, 2)
                     if (a.startswith('left_') and b.startswith('left_')) or
                        (a.startswith('right_') and b.startswith('right_')))
    n = 0
    for a, b in pairs:
        if a in prims and b in prims:
            UsdPhysics.FilteredPairsAPI.Apply(prims[a]).CreateFilteredPairsRel().AddTarget(prims[b].GetPath())
            n += 1
    roots = [p for p in stage.Traverse() if str(p.GetPath()).startswith(robot_root) and p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    for r in roots:
        PhysxSchema.PhysxArticulationAPI.Apply(r).CreateEnabledSelfCollisionsAttr().Set(True)
    if not roots:
        raise RuntimeError('no articulation root found under ' + robot_root)
    return n
