"""scene_v0: shelf / table store scene from the NVIDIA SimReady assets (download them first with fetch_assets.py).

    from scene import build
    items = build(stage)          # after the World exists, before world.reset()

World frame = the robot's start pose (origin, facing +x). Robot: the MONO prototype (official, user 2026-11-16). Its arm
reaches (offline, scara_ik): on the 0.85 table hand-down only x 0.36..0.48 |y| <= 0.25 (0-12 cm in from the edge), level
from the side further in with the hand turned; shelves straight in with the shelf front ~0.64 m ahead of the base centre,
boards up to ~1.1 m (lift 0.81: a level TCP tops out at 1.24). The base drives to each product (mecanum, scripted).
Layout (metres):
  table  table_metal_stainless_a03, long side along y, front edge x 0.36 (the table tasks' stop distance), top 0.85.
         Kept free in front of the robot (|y| <= 0.25: the grasp test's spot); crate (storage_basket_plastic_a01) at
         y +0.45, 4 short products in a row at x 0.42, y -0.45..-0.81 (hand-down reach after a sidestep)
  rack   rack_f04 on the robot's left: front at y +1.10, centred on x -0.6; products on the 0.761 board only
         (1.212 / 1.663 are out of the mono arm's reach). Collides through invisible boxes too (boards, 3 back posts,
         feet): its mesh left boxes leaning up to 3 deg
  shelf  storage_shelf_basic_b01 on the robot's right: front at y -1.165, centred on x -0.6; products on 0.667 / 1.070.
         Its own collision mesh tipped standing bottles over (Isaac 2026-11-16), so it collides through invisible boxes
         instead: a flat plate on each board (top 0.264 + k 0.403, just over the board mesh) + its 4 corner posts
  Store sections (products.json, store_db.py): blue rack 1 breakfast and dry goods, blue rack 2 snacks and canned, grey shelf 1 dairy,
  grey shelf 2 sauces; each board holds runs of one product side by side (facings), each run with its own random stock
  level; the 11 shelf task products sit mid-run on boards the arm reaches (TASK_BOARD). The table is the work /
  staging area: 4 short task products at its edge, the crate, a mixed back row.
  Products face the robot's approach with their narrow side across it (the jaws close across the approach).

Physics changes made in OUR stage only (the downloaded files are untouched):
  - PhysX variant 'Enabled' on every asset (food_box_cracker_a01 ships 'Disabled');
  - instancing off (instance proxies can't be edited);
  - furniture static (rigid body disabled) with triangle-mesh colliders (the packages: SDF, rigid bodies of 80 kg..3 t);
  - products: convex-decomposition colliders (packages: SDF; one convex hull each left 2 products leaning 5 deg), mass = assets.PRODUCTS[...]['mass'] spread over the
    package's bodies in the package's own ratio (caps / lids are a 2nd body on a fixed joint), inertia scaled with it;
  - caps / lids: their fixed joints re-pointed at the two rigid bodies (as shipped they target mesh prims and did
    not hold in Isaac: lids came off, bottles fell);
  - crate: dynamic (SDF kept: it is concave), its 4 folding walls locked upright (revolute limits 0..0);
  - empty xform ops on bodies (declared without values in the packages) filled with identity; lay=True products get
    their 90 deg turn on the body itself (on an extra parent Xform the bar started upright in Isaac and fell off the
    shelf onto the floor).
Measured offline (2026-11-16): table top at 0.055 in its own frame (origin at the top), rack boards 0.76 / 1.21 / 1.66,
shelf boards ~0.265 + k 0.405 (ridged: the products settle ~0.668 / 1.071; 0.675 / 1.08 made tall bottles tip off the drop), crate floor 0.010, product sizes as in the manifest.
"""
import math, os, sys
import random
try:                                  # the layout (REST, LOCATIONS, PRODUCTS, FURNITURE, ...) also loads without Isaac (stations.py)
    from pxr import Usd, UsdGeom, UsdPhysics, UsdLux, UsdShade, Sdf, Gf
except ImportError:
    Usd = None

here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(here, '..', '..', '..', 'common')); import repo_paths as RP   # every script folder + repo paths
sys.path.insert(0, here)
import assets as SA

ROOT = RP.SIMREADY   # scenes/assets/simready (git-ignored)
TABLE_TOP = 0.85
TABLE_EDGE = 0.36
RACK_FRONT, RACK_X = 1.10, -0.6
RACK_PLATES = (0.761, 1.212, 1.663)   # tops of the rack's box colliders (board mesh tops 0.7606 / 1.2117 / 1.6629)
RACK_BOARDS = RACK_PLATES[:2]
SHELF_FRONT, SHELF_X, SHELF_BOARDS = -1.165, -0.6, (0.667, 1.070)
SHELF_PLATES = [0.264 + 0.403 * k for k in range(5)]   # tops of the shelf's box colliders (board mesh tops ~0.257 + k 0.405)
DROP = 0.003                       # products start this far above their surface
TASK_GAP = 0.05                    # free space each side of a task product: the open jaws' outer faces reach ~29 mm out
                                   # (finger 14 mm + 15 mm opening margin); 30 mm left ~1 mm to a filler neighbour

# furniture: package, (x, y) of the bounding-box centre, yaw (deg), z of the bounding-box bottom
FURNITURE = {
    'table': ('table_metal_stainless_a03', (TABLE_EDGE + 0.349, 0.0), 90.0, TABLE_TOP - 0.850),
    'rack':  ('rack_f04', (RACK_X, RACK_FRONT + 0.230), 0.0, 0.0),
    'shelf': ('storage_shelf_basic_b01', (SHELF_X, SHELF_FRONT - 0.235), 90.0, 0.0),
    'rack2': ('rack_f04', (RACK_X - 2.15, RACK_FRONT + 0.230), 0.0, 0.0),                 # beside the rack (user 2026-11-16)
    'shelf2': ('storage_shelf_basic_b01', (SHELF_X - 1.55, SHELF_FRONT - 0.235), 90.0, 0.0),  # beside the shelf
    'crate': ('storage_basket_plastic_a01', (0.53, 0.45), 0.0, TABLE_TOP),
}
# room: 3 walls (user 2026-11-16) behind the table (+x), the racks (+y) and the shelves (-y); open toward -x
ROOM = dict(x0=-4.0, x1=1.25, y0=-1.85, y1=1.75, h=2.6, t=0.10, color=(0.86, 0.85, 0.82))
# lights: soft fill (dome) + overhead strips like a store ceiling at LIGHT_Z (Isaac's default sun / dome are turned off)
DOME_INTENSITY = 600.0
LIGHT_Z, STRIP = 2.75, dict(w=1.2, h=0.2, intensity=30000.0)
STRIPS = [(x, y) for x in (-3.0, -1.6, -0.2) for y in (-1.0, 1.0)] + [(0.7, -0.6), (0.7, 0.6)]
# product resting places: package -> (surface, x, y, approach heading of the hand there (deg))
#   table: the hand comes along +x;  rack: along +y;  shelf: along -y
_T = [('food_processed_yogurt_a01', 0.42, -0.45), ('food_processed_cannedFood_h01', 0.42, -0.57),
      ('food_beverage_coffeeCup_b01', 0.42, -0.69), ('food_processed_yogurt_c01', 0.42, -0.81)]
# store sections: which product covers which location is in the PRODUCT DATABASE (scenes/scene_v0/products.json,
# store_db.py; user 2026-11-17); the unit names stay here
SECTION_NAMES = {'rack': 'breakfast and dry goods', 'rack2': 'snacks and canned', 'shelf': 'dairy', 'shelf2': 'sauces'}
_P, _F = 'food_processed_', 'food_'
CEREAL, CRACKER, BOX_B, BOX_C, BOX_D = _P + 'boxedCereal_l01', _F + 'box_cracker_a01', _P + 'boxedFood_b01', _P + 'boxedFood_c01', _P + 'boxedFood_d01'
CAN, CHOC, CUP = _P + 'cannedFood_h01', _P + 'chocolate_e01', _F + 'beverage_coffeeCup_b01'
MILK_B, MILK_C, YOG_A, YOG_C = _P + 'milk_b01', _P + 'milk_c01', _P + 'yogurt_a01', _P + 'yogurt_c01'
KET_C, KET_I, KET_A = _P + 'ketchup_c01', _P + 'ketchup_i01', _F + 'condiment_ketchup_a02'
# the 11 shelf task products: in the middle of their own run on a board the mono arm reaches. Short products (<= 70 mm,
# picked from above) only on boards <= ~0.8 m: from above at 1.07 m needs lift 0.87 > 0.81 (stations.py found it)
TASK_BOARD = {CEREAL: 'rack_b0', CRACKER: 'rack_b0', BOX_B: 'rack_b0', BOX_C: 'rack_b0', BOX_D: 'rack_b0',
              CHOC: 'rack2_b0', MILK_B: 'shelf_b1', MILK_C: 'shelf_b2', KET_C: 'shelf2_b1', KET_I: 'shelf2_b1',
              KET_A: 'shelf2_b2'}

def product_yaw(pkg, heading_deg):
    """Yaw (deg) that puts the product's narrow horizontal side across the hand heading (along the jaw axis)."""
    sx, sy, _ = SA.size(pkg)
    return heading_deg + (90.0 if sx < sy else 0.0)


def footprint(pkg, yaw_deg, heading_deg):
    """Product depth along the heading (m) at that yaw."""
    sx, sy, _ = SA.size(pkg)
    a = math.radians(yaw_deg - heading_deg)
    return (abs(math.cos(a)) * sx + abs(math.sin(a)) * sy) / 1000


def _slot(stage, path, pkg, xy, yaw_deg, z_bottom, lay=False):
    """Xform at `path` placing the package so its bounding-box bottom centre lands at (xy, z_bottom), turned yaw.
    lay: the product's (single) body is turned 90 deg about its y axis first."""
    slot = UsdGeom.Xform.Define(stage, Sdf.Path(path))
    asset = stage.DefinePrim(Sdf.Path(path + '/asset'), 'Xform')
    asset.GetReferences().AddReference(os.path.abspath(SA.usd_path(ROOT, pkg)))
    vs = asset.GetVariantSets()
    if vs.HasVariantSet('PhysX'):
        vs.GetVariantSet('PhysX').SetVariantSelection('Enabled')
    _deinstance(asset)
    _fill_ops(asset)
    if lay:
        bs = _bodies(asset)
        assert len(bs) == 1, f'{pkg}: lay=True needs a single-body product'
        _set_op(bs[0], 'xformOp:rotateXYZ', (0.0, 90.0, 0.0))
    r = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ['default', 'render']).ComputeRelativeBound(asset, slot.GetPrim()).ComputeAlignedRange()
    rng = (tuple(r.GetMin()), tuple(r.GetMax()))       # in the slot frame (incl. the asset's own transform)
    m = slot_matrix(xy, yaw_deg, z_bottom, rng)
    slot.AddTranslateOp().Set(m.ExtractTranslation())
    slot.AddRotateZOp().Set(yaw_deg)
    return slot, asset, rng


def _set_op(prim, name, value):
    a = prim.GetAttribute(name)
    t = str(a.GetTypeName())
    a.Set((Gf.Vec3d if t.startswith('double') else Gf.Vec3h if t.startswith('half') else Gf.Vec3f)(*value))


def _fill_ops(asset):
    """Give xform ops declared without a value an identity value (the packages leave them empty on the bodies; Isaac
    warned 'expected GfVec3f, got void' when a body was moved)."""
    for p in Usd.PrimRange(asset):
        if not p.IsA(UsdGeom.Xformable):
            continue
        for op in UsdGeom.Xformable(p).GetOrderedXformOps():
            if op.Get() is None and op.GetOpType() in (UsdGeom.XformOp.TypeTranslate, UsdGeom.XformOp.TypeScale,
                                                       UsdGeom.XformOp.TypeRotateXYZ):
                _set_op(p, op.GetName(), (1.0, 1.0, 1.0) if op.GetOpType() == UsdGeom.XformOp.TypeScale else (0.0, 0.0, 0.0))


def _deinstance(prim):
    for _ in range(6):
        inst = [p for p in Usd.PrimRange(prim) if p.IsInstance()]
        if not inst:
            return
        for p in inst:
            p.SetInstanceable(False)


def _bodies(asset):
    return [p for p in Usd.PrimRange(asset) if p.HasAPI(UsdPhysics.RigidBodyAPI)]


def _colliders(asset):
    return [p for p in Usd.PrimRange(asset) if p.HasAPI(UsdPhysics.CollisionAPI)]


def _make_static(asset):
    for b in _bodies(asset):
        UsdPhysics.RigidBodyAPI(b).CreateRigidBodyEnabledAttr(False)
    for c in _colliders(asset):
        UsdPhysics.MeshCollisionAPI.Apply(c).CreateApproximationAttr('none')


def _set_product_mass(asset, mass):
    """Spread `mass` over the package's bodies in their own ratio (inertia scaled with it). A body's own mass wins over
    its colliders' (USD physics rule; some packages author the same value on both, so summing both counted it twice)."""
    entries = []                                         # the MassAPI prims PhysX actually uses
    for b in _bodies(asset):
        if b.HasAPI(UsdPhysics.MassAPI) and (UsdPhysics.MassAPI(b).GetMassAttr().Get() or 0) > 0:
            entries.append(b); continue
        entries += [p for p in Usd.PrimRange(b) if p != b and p.HasAPI(UsdPhysics.MassAPI)
                    and (UsdPhysics.MassAPI(p).GetMassAttr().Get() or 0) > 0]
    if not entries:                                      # no masses at all: put it on the first body
        UsdPhysics.MassAPI.Apply(_bodies(asset)[0]).CreateMassAttr(mass); return
    k = mass / sum(UsdPhysics.MassAPI(p).GetMassAttr().Get() for p in entries)
    for p in entries:
        api = UsdPhysics.MassAPI(p)
        api.GetMassAttr().Set(api.GetMassAttr().Get() * k)
        I = api.GetDiagonalInertiaAttr().Get()
        if I is not None and any(v > 0 for v in I):
            api.GetDiagonalInertiaAttr().Set(Gf.Vec3f(*(v * k for v in I)))


def _box_colliders(stage, root, asset, boxes):
    """Turn the asset's own collision off and collide through invisible static boxes (name, centre, size) instead."""
    for c in _colliders(asset):
        UsdPhysics.CollisionAPI(c).CreateCollisionEnabledAttr().Set(False)
    for nm, ctr, sz in boxes:
        c = UsdGeom.Cube.Define(stage, Sdf.Path(f'{root}/{nm}')); c.CreateSizeAttr(1.0)
        c.AddTranslateOp().Set(Gf.Vec3d(*ctr)); c.AddScaleOp().Set(Gf.Vec3f(*sz))
        UsdPhysics.CollisionAPI.Apply(c.GetPrim()); c.MakeInvisible()


def _rigid_ancestor(prim):
    while prim and not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        prim = prim.GetParent()
    return prim


def _fix_joints(stage, asset):
    """Re-point each fixed joint (caps / lids) at the two rigid bodies themselves, with frames from where the parts sit.
    As shipped they target mesh prims inside the bodies, and in Isaac they did not hold: ketchup lids came off by
    24-26 cm and the bottles fell over; milk / cup lids sagged 6-8 mm and tilted them (2026-11-16)."""
    xc = UsdGeom.XformCache()
    for j in Usd.PrimRange(asset):
        if not j.IsA(UsdPhysics.FixedJoint):
            continue
        J = UsdPhysics.FixedJoint(j)
        b0 = _rigid_ancestor(stage.GetPrimAtPath(J.GetBody0Rel().GetTargets()[0]))
        b1 = _rigid_ancestor(stage.GetPrimAtPath(J.GetBody1Rel().GetTargets()[0]))
        m = xc.GetLocalToWorldTransform(b1) * xc.GetLocalToWorldTransform(b0).GetInverse()     # body1 in body0's frame
        J.GetBody0Rel().SetTargets([b0.GetPath()]); J.GetBody1Rel().SetTargets([b1.GetPath()])
        J.CreateLocalPos0Attr().Set(Gf.Vec3f(m.ExtractTranslation()))
        J.CreateLocalRot0Attr().Set(Gf.Quatf(m.ExtractRotationQuat()))
        J.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0)); J.CreateLocalRot1Attr().Set(Gf.Quatf(1, 0, 0, 0))
        J.CreateBreakForceAttr().Set(float('inf')); J.CreateBreakTorqueAttr().Set(float('inf'))
        J.CreateJointEnabledAttr().Set(True)


def build(stage, root='/World/store', products=True, fill=1.0, seed=0, room=True, floor=True, markers=True, dynamic_fillers=True):
    """Add the scene. Returns {name: dict(pkg, role, slot, bodies, rest=(surface z, x, y, heading), yaw, range)}.
    fill: filler-product density scale (0 = none; each board gets a random fill 10..100 % times this); seed: their layout."""
    global Usd, UsdGeom, UsdPhysics, UsdLux, UsdShade, Sdf, Gf
    if Usd is None:                    # imported before Isaac started (e.g. through stations.py): load pxr now
        from pxr import Usd, UsdGeom, UsdPhysics, UsdLux, UsdShade, Sdf, Gf
    UsdGeom.Xform.Define(stage, Sdf.Path(root))
    out = {}
    if room:
        _walls(stage, root); _lights(stage, root)
    if floor:                                      # grey floor (the robot scripts and the saved file both use it)
        _floor(stage, root)
    if markers:                                    # the robot stations (stations.json) as floor marks
        _markers(stage, root)
    for name, (pkg, xy, yaw, zb) in FURNITURE.items():
        slot, asset, rng = _slot(stage, f'{root}/{name}', pkg, xy, yaw, zb)
        if name == 'crate':
            for j in Usd.PrimRange(asset):
                if j.IsA(UsdPhysics.RevoluteJoint):
                    rj = UsdPhysics.RevoluteJoint(j); rj.GetLowerLimitAttr().Set(0.0); rj.GetUpperLimitAttr().Set(0.0)
        else:
            _make_static(asset)
        if pkg == 'storage_shelf_basic_b01':
            # its own collision mesh tipped standing bottles over (with or without plates; plates alone: 0.0 deg, Isaac
            # 2026-11-16) -> mesh collision off; invisible boxes instead: one plate per board + the 4 corner posts
            boxes = [(f'{name}_plate_{k}', (xy[0], xy[1], zt - 0.002), (1.462, 0.47, 0.004)) for k, zt in enumerate(SHELF_PLATES)]
            boxes += [(f'{name}_post_{i}', (xy[0] + sx * 0.7193, xy[1] + sy * 0.2184, 1.015), (0.0415, 0.0432, 2.03))
                      for i, (sx, sy) in enumerate(((-1, -1), (-1, 1), (1, -1), (1, 1)))]
            _box_colliders(stage, root, asset, boxes)
        if pkg == 'rack_f04':
            # same as the shelf: its collision mesh left boxes leaning 0.4-3.1 deg on a flat board (Isaac 2026-11-16).
            # Cantilever rack: 3 posts at the back (local y 0.17..0.22), boards y -0.23..0.217, feet on the floor
            boxes = [(f'{name}_board_{k}', (xy[0], xy[1] - 0.0065, zt - 0.002), (2.068, 0.447, 0.004)) for k, zt in enumerate(RACK_PLATES)]
            for i, px in enumerate((-1.01, 0.0, 1.01)):
                boxes += [(f'{name}_post_{i}', (xy[0] + px, xy[1] + 0.195, 0.975), (0.045, 0.05, 1.95)),
                          (f'{name}_foot_{i}', (xy[0] + px, xy[1] - 0.002, 0.0365), (0.05, 0.438, 0.073))]
            _box_colliders(stage, root, asset, boxes)
        out[name] = dict(pkg=pkg, role=SA.FURNITURE[pkg]['role'], slot=str(slot.GetPath()),
                         bodies=[str(b.GetPath()) for b in _bodies(asset)], yaw=yaw, range=rng)
    if not products:
        return out
    for pkg, (z, x, y, hd) in REST.items():
        yaw = product_yaw(pkg, hd); d = footprint(pkg, yaw, hd)
        if hd == 90.0: y = RACK_FRONT + 0.04 + d / 2
        elif hd == -90.0: y = SHELF_FRONT - 0.04 - d / 2
        name = pkg.replace('food_', '')
        slot, asset, rng = _slot(stage, f'{root}/{name}', pkg, (x, y), yaw, z + DROP, lay=SA.PRODUCTS[pkg].get('lay', False))
        for c in _colliders(asset):     # convex decomposition: a single hull left ketchup_c01 / the cup leaning 5 deg (Isaac 2026-11-16)
            UsdPhysics.MeshCollisionAPI.Apply(c).CreateApproximationAttr('convexDecomposition')
        _set_product_mass(asset, SA.PRODUCTS[pkg]['mass'])
        _fix_joints(stage, asset)
        out[name] = dict(pkg=pkg, role='product', slot=str(slot.GetPath()), bodies=[str(b.GetPath()) for b in _bodies(asset)],
                         rest=(z, x, y, hd), yaw=yaw, range=rng)
    if fill > 0:
        _fillers(stage, root, out, fill, seed, dynamic_fillers)
    return out


def _material(stage, path, rgb, rough=0.8):
    m = UsdShade.Material.Define(stage, Sdf.Path(path))
    sh = UsdShade.Shader.Define(stage, Sdf.Path(path + '/shader')); sh.CreateIdAttr('UsdPreviewSurface')
    sh.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
    sh.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(rough)
    m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), 'surface')
    return m


def _walls(stage, root):
    R = ROOM; mat = _material(stage, f'{root}/looks/wall', R['color'])
    cx, cy, t, h = (R['x0'] + R['x1']) / 2, (R['y0'] + R['y1']) / 2, R['t'], R['h']
    for nm, ctr, sz in (('wall_table', (R['x1'] + t / 2, cy, h / 2), (t, R['y1'] - R['y0'] + 2 * t, h)),
                        ('wall_racks', (cx, R['y1'] + t / 2, h / 2), (R['x1'] - R['x0'], t, h)),
                        ('wall_shelves', (cx, R['y0'] - t / 2, h / 2), (R['x1'] - R['x0'], t, h))):
        c = UsdGeom.Cube.Define(stage, Sdf.Path(f'{root}/{nm}')); c.CreateSizeAttr(1.0)
        c.AddTranslateOp().Set(Gf.Vec3d(*ctr)); c.AddScaleOp().Set(Gf.Vec3f(*sz))
        UsdPhysics.CollisionAPI.Apply(c.GetPrim()); UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(mat)


def _floor(stage, root):
    R = ROOM; mat = _material(stage, f'{root}/looks/floor', (0.55, 0.55, 0.53), rough=0.6)
    c = UsdGeom.Cube.Define(stage, Sdf.Path(f'{root}/floor')); c.CreateSizeAttr(1.0)
    c.AddTranslateOp().Set(Gf.Vec3d((R['x0'] + R['x1']) / 2, (R['y0'] + R['y1']) / 2, -0.05))
    c.AddScaleOp().Set(Gf.Vec3f(R['x1'] - R['x0'] + 2 * R['t'] + 2.0, R['y1'] - R['y0'] + 2 * R['t'], 0.10))
    UsdPhysics.CollisionAPI.Apply(c.GetPrim()); UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(mat)
    # Isaac's own ground plane (robot scripts) stays for collision; its grid visual is hidden so it doesn't show through
    g = stage.GetPrimAtPath('/World/defaultGroundPlane')
    if g and g.IsValid():
        for p in Usd.PrimRange(g):
            if p.IsA(UsdGeom.Gprim):
                UsdGeom.Imageable(p).MakeInvisible()


def _markers(stage, root):
    """A flat square + a heading bar on the floor at every station in stations.json (visual only, no collision):
    blue = home, green = a task product's station, yellow = another product run, orange dot = the turning (approach) point."""
    import json
    f = os.path.join(RP.scene_dir('scene_v0'), 'stations.json')
    if not os.path.exists(f):
        return
    st = json.load(open(f))['stations']
    looks = {k: _material(stage, f'{root}/looks/mark_{k}', rgb, rough=0.5) for k, rgb in
             (('home', (0.10, 0.35, 0.95)), ('task', (0.10, 0.75, 0.25)), ('run', (0.95, 0.80, 0.10)), ('turn', (0.95, 0.45, 0.10)))}

    def mark(nm, x, y, yaw, sx, sy, look, dx=0.0):
        c = UsdGeom.Cube.Define(stage, Sdf.Path(f'{root}/marks/{nm}')); c.CreateSizeAttr(1.0)
        a = math.radians(yaw)
        c.AddTranslateOp().Set(Gf.Vec3d(x + dx * math.cos(a), y + dx * math.sin(a), 0.0015)); c.AddRotateZOp().Set(yaw)
        c.AddScaleOp().Set(Gf.Vec3f(sx, sy, 0.003)); UsdShade.MaterialBindingAPI.Apply(c.GetPrim()).Bind(looks[look])
    for name, s_ in st.items():
        if 'x' not in s_:
            continue
        look = 'home' if name == 'home' else ('task' if s_.get('task') else 'run')
        mark(name, s_['x'], s_['y'], s_['yaw'], 0.10, 0.10, look)
        mark(name + '_heading', s_['x'], s_['y'], s_['yaw'], 0.16, 0.025, look, dx=0.13)
        if s_.get('approach'):
            mark(name + '_turn', s_['approach'][0], s_['approach'][1], 0.0, 0.04, 0.04, 'turn')


def _lights(stage, root):
    for p in ('/DistantLight', '/DomeLight', '/World/defaultDistantLight', '/World/DistantLight'):
        q = stage.GetPrimAtPath(p)
        if q and q.IsValid():
            q.SetActive(False)
    UsdLux.DomeLight.Define(stage, Sdf.Path(f'{root}/lights/dome')).CreateIntensityAttr(DOME_INTENSITY)
    for i, (x, y) in enumerate(STRIPS):
        L = UsdLux.RectLight.Define(stage, Sdf.Path(f'{root}/lights/strip_{i}'))     # emits along its -Z: down
        L.CreateWidthAttr(STRIP['w']); L.CreateHeightAttr(STRIP['h']); L.CreateIntensityAttr(STRIP['intensity'])
        L.AddTranslateOp().Set(Gf.Vec3d(x, y, LIGHT_Z))


def _surfaces():
    """Filler rows: (key, board top z, along-board range (u0, u1) in world x or y, front coordinate, heading deg,
    clearance above, dynamic). One row per board, at the front. Dynamic only where the mono arm reaches (rack 0.761,
    shelf 0.667 / 1.070, table); the rest are fixed in place (solid, no physics cost: 11 fps with physics on already)."""
    out = []
    for name, (pkg, xy, yaw, zb) in FURNITURE.items():
        if pkg == 'rack_f04':
            tops = list(RACK_PLATES) + [1.95]
            for k, z in enumerate(RACK_PLATES):
                out.append((f'{name}_b{k}', z, (xy[0] - 1.03, xy[0] + 1.03), RACK_FRONT, 90.0, tops[k + 1] - z - 0.01, k == 0))
        if pkg == 'storage_shelf_basic_b01':
            tops = list(SHELF_PLATES) + [2.03]
            for k, z in enumerate(SHELF_PLATES):
                out.append((f'{name}_b{k}', z, (xy[0] - 0.69, xy[0] + 0.69), SHELF_FRONT, -90.0, tops[k + 1] - z - 0.01, k in (1, 2)))
    # along the back edge, x 0.95..1.04: out of the mono arm's reach (~0.5-0.7 m into the table) -> fixed in place
    out.append(('table_back', TABLE_TOP, (-0.85, 0.85), TABLE_EDGE + 0.698, 180.0, 1.0, False))
    return out


def _segments(key, u0, u1):
    """The board's product runs: (u start, u end, package) from the product database: each product's locations on
    this board, neighbouring slots merged into one run (a product may have several runs, on several boards)."""
    runs = []
    for p in PRODUCTS.values():
        for a, b in sorted(LOCATIONS[l]['u'] for l in p['locations'] if LOCATIONS[l]['board'] == key):
            if runs and runs[-1][2] == p['package'] and abs(runs[-1][1] - a) < 1e-6:
                runs[-1] = (runs[-1][0], b, p['package'])
            else:
                runs.append((a, b, p['package']))
    return sorted(runs)


def _task_rest():
    """Resting place (surface z, x, y, approach heading) of every task product: the table ones from _T, the shelf ones
    in the middle of their run on TASK_BOARD (y is set in build() from the product's depth: 40 mm in from the front)."""
    rest = {p['package']: (TABLE_TOP, *LOCATIONS[l]['centre'], 0.0)              # table spot from the database
            for p in PRODUCTS.values() for l in p['locations'] if l.startswith('T1-')}
    surf = {sf[0]: sf for sf in _surfaces()}
    for pkg, key in TASK_BOARD.items():
        _, z, (u0, u1), front, hd, clr, dyn = surf[key]
        assert dyn, f'{pkg}: task board {key} must be one the arm reaches'
        a, b, _ = [sg for sg in _segments(key, u0, u1) if sg[2] == pkg][0]
        rest[pkg] = (z, (a + b) / 2, None, hd)
    return rest


def _fillers(stage, root, out, fill, seed, dynamic_fillers=True):
    pk = list(SA.PRODUCTS)
    taken = {}                                     # board key -> reserved (u0, u1) intervals of the task products
    for n, it in out.items():
        if it['role'] != 'product':
            continue
        z, x, y, hd = it['rest']; w = min(SA.size(it['pkg'])[:2]) / 1000
        for key, zt, (u0, u1), front, h, clr, dyn in _surfaces():
            along_x = h in (90.0, -90.0)
            if abs(zt - z) < 0.01 and (u0 <= (x if along_x else y) <= u1) and abs((y if along_x else x) - front) < 0.3:
                c = x if along_x else y
                taken.setdefault(key, []).append((c - w / 2 - TASK_GAP, c + w / 2 + TASK_GAP))
    n_dyn = n_fix = 0
    for key, zt, (u0, u1), front, hd, clr, dyn in _surfaces():
        rnd = random.Random(f'{seed}/{key}')     # one stream per board: a change on one board leaves the others as they were
        # shelves: runs of one product (store sections); the table's back row: a mixed staging pile ("to put away")
        runs = [(u0, u1, None)] if key == 'table_back' else _segments(key, u0, u1)
        i = 0
        dyn = dyn and dynamic_fillers                # False: every filler fixed in place (only the task products move)
        for a, b, run in runs:
            if run and SA.size(run)[2] / 1000 > clr:
                print(f'scene_v0: {key}: {run} too tall for the board, run left empty', flush=True); continue
            f = min(1.0, rnd.uniform(0.25 if run else 0.10, 1.0) * fill)      # this run's stock level
            u = a + 0.01
            while True:
                pkg = run or rnd.choice(pk); nar, dep, hh = (v / 1000 for v in sorted(SA.size(pkg)[:2]) + [SA.size(pkg)[2]])
                if hh > clr:
                    continue
                w = nar + (0.004 if run else 0.008)          # yaw jitter margin
                if u + w > b - 0.01:
                    break
                hit = [e for (s_, e) in taken.get(key, []) if s_ < u + w and u < e]
                if hit:
                    u = max(hit) + 0.005; continue
                if rnd.random() < f:
                    c = u + w / 2
                    inset = (0.03 if run else rnd.uniform(0.02, 0.05)) + dep / 2   # facings line up at the front
                    if hd == 90.0: xy = (c, front + inset)
                    elif hd == -90.0: xy = (c, front - inset)
                    else: xy = (front - inset, c)      # table back row: the products face the robot (-x side)
                    yaw = product_yaw(pkg, 0.0 if hd == 180.0 else hd) + rnd.uniform(-1.5, 1.5) * (1 if run else 3)
                    nm = f'{key}_{i:02d}_{pkg.replace("food_", "").replace("processed_", "")}'
                    slot, asset, rng = _slot(stage, f'{root}/fill/{nm}', pkg, xy, yaw, zt + DROP, lay=SA.PRODUCTS[pkg].get('lay', False))
                    if dyn:
                        for c_ in _colliders(asset):
                            UsdPhysics.MeshCollisionAPI.Apply(c_).CreateApproximationAttr('convexDecomposition')
                        _set_product_mass(asset, SA.PRODUCTS[pkg]['mass']); _fix_joints(stage, asset); n_dyn += 1
                    else:
                        for bd in _bodies(asset):
                            UsdPhysics.RigidBodyAPI(bd).CreateRigidBodyEnabledAttr(False)
                        for j in Usd.PrimRange(asset):
                            if j.IsA(UsdPhysics.Joint):
                                UsdPhysics.Joint(j).CreateJointEnabledAttr().Set(False)
                        for c_ in _colliders(asset):
                            UsdPhysics.MeshCollisionAPI.Apply(c_).CreateApproximationAttr('convexHull')
                        n_fix += 1
                    out[f'fill/{nm}'] = dict(pkg=pkg, role='filler', dynamic=dyn, slot=str(slot.GetPath()), section=key,
                                             bodies=[str(bd.GetPath()) for bd in _bodies(asset)], rest=(zt, xy[0], xy[1], hd), yaw=yaw, range=rng)
                    i += 1
                u += w + (0.006 if run else rnd.uniform(0.004, 0.012))
    print(f'scene_v0: {n_dyn} filler products with physics, {n_fix} fixed in place (fill {fill}, seed {seed})', flush=True)

def slot_matrix(xy, yaw_deg, z_bottom, rng):
    """World matrix (row-vector convention) of a slot that puts the bounding box `rng` (min, max in the slot frame,
    as build() stores it) with its bottom centre at (xy, z_bottom), turned yaw."""
    c = [(rng[0][i] + rng[1][i]) / 2 for i in range(2)]
    a = math.radians(yaw_deg); ca, sa = math.cos(a), math.sin(a)
    t = Gf.Vec3d(xy[0] - (ca * c[0] - sa * c[1]), xy[1] - (sa * c[0] + ca * c[1]), z_bottom - rng[0][2])
    m = Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), yaw_deg)); m.SetTranslateOnly(t)
    return m


# ------------------------------------------------------------ location IDs + product database
# (user 2026-11-17: "each location an ID and ... a product database where one product can have 1 or more ID locations")
# Every board is cut into fixed-width SLOTS; ID = <unit>-B<board, 1 = lowest>-<slot, 01 = leftmost seen from the
# aisle>, e.g. R1-B1-03. Table pick spots: T1-01..04. The slot counts keep each product run of the first layout on
# whole slots (racks: 10 x 0.206 m, so 5 / 3 / 2 products = 2 / 3-4 / 5 slots; shelves: 6 x 0.23 m, 2 products = 3 each).
UNITS = {'rack': ('R1', 10), 'rack2': ('R2', 10), 'shelf': ('S1', 6), 'shelf2': ('S2', 6)}   # unit code, slots per board
UNIT_NAMES = {'R1': 'blue rack 1', 'R2': 'blue rack 2', 'S1': 'grey shelf 1', 'S2': 'grey shelf 2', 'T1': 'table'}
TABLE_SPOTS = [(x, y) for _, x, y in _T]                 # T1-01.. (the table task products' pick spots)
PRODUCTS_FILE = os.path.join(RP.scene_dir('scene_v0'), 'products.json')


def locations():
    """{ID: dict(unit, section, board, level, slot, z, u (start, end along the board), centre (x, y at the board front),
    width, facing (approach heading, deg), reachable (the mono arm reaches the board))} for every slot + table spot."""
    out = {}
    for key, zt, (u0, u1), front, hd, clr, dyn in _surfaces():
        unit = key.split('_')[0]
        if unit not in UNITS:
            continue
        code, n = UNITS[unit]; lvl = int(key.rsplit('_b', 1)[1]) + 1; w = (u1 - u0) / n
        for i in range(n):
            k = i if hd == 90.0 else n - 1 - i                 # racks face +y (left = -x), shelves face -y (left = +x)
            a, b = u0 + k * w, u0 + (k + 1) * w
            out[f'{code}-B{lvl}-{i + 1:02d}'] = dict(
                unit=code, section=SECTION_NAMES[unit], board=key, level=lvl, slot=i + 1, z=round(zt, 3),
                u=[round(a, 4), round(b, 4)], centre=[round((a + b) / 2, 3), round(front, 3)], width=round(w, 3),
                facing=hd, reachable=dyn)
    for i, (x, y) in enumerate(TABLE_SPOTS):
        out[f'T1-{i + 1:02d}'] = dict(unit='T1', section='table (work area)', board='table', level=1, slot=i + 1,
                                      z=TABLE_TOP, u=None, centre=[x, y], width=0.12, facing=0.0, reachable=True)
    return out


def load_products(path=None):
    """The product database: {product id: dict(name, package, mass_kg, size_mm, locations [IDs])}."""
    import json
    return json.load(open(path or PRODUCTS_FILE))['products']


LOCATIONS = locations()
try:
    PRODUCTS = load_products()
except FileNotFoundError:                                # first run of store_db.py --init (it writes the file)
    PRODUCTS = {}


def check_products(products=None, locs=None):
    """Problems in the product database: unknown IDs, a location claimed twice, task products off their board."""
    products, locs = PRODUCTS if products is None else products, LOCATIONS if locs is None else locs
    bad, owner = [], {}
    for pid, p in products.items():
        if not p['locations']:
            bad.append(f'{pid}: no locations')
        for l in p['locations']:
            if l not in locs:
                bad.append(f'{pid}: unknown location {l}')
            elif l in owner:
                bad.append(f'{l}: claimed by {owner[l]} and {pid}')
            else:
                owner[l] = pid
    pk = {p['package']: pid for pid, p in products.items()}
    for pkg, key in TASK_BOARD.items():
        if pkg not in pk or not any(locs[l]['board'] == key for l in products[pk[pkg]]['locations'] if l in locs):
            bad.append(f'task product {pkg}: no location on its task board {key}')
    for i in range(len(TABLE_SPOTS)):
        if f'T1-{i + 1:02d}' not in owner:
            bad.append(f'T1-{i + 1:02d}: no product (the table task needs one per spot)')
    return bad


REST = _task_rest()          # after _surfaces(): the shelf task products' spots come from the sections
