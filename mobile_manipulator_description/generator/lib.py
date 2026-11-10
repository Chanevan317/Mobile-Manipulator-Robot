"""Small CSG helper layer over manifold3d. Units: mm. All solids are manifold3d.Manifold."""
import math
import numpy as np
import manifold3d as m

M = m.Manifold
CS = m.CrossSection


def segs(r):
    return int(max(24, min(160, round(2 * math.pi * r / 0.6))))


def box(x0, x1, y0, y1, z0, z1):
    return M.cube((x1 - x0, y1 - y0, z1 - z0)).translate((x0, y0, z0))


def cyl(r, z0, z1, x=0.0, y=0.0, hole=False, n=None):
    """Cylinder on the Z axis. hole=True makes the polygon circumscribe r, so the printed hole is never undersize."""
    n = n or segs(r)
    rr = r / math.cos(math.pi / n) if hole else r
    return M.cylinder(z1 - z0, rr, rr, n).translate((x, y, z0))


def cone(r0, r1, z0, z1, x=0.0, y=0.0, n=None):
    n = n or segs(max(r0, r1))
    return M.cylinder(z1 - z0, r0, r1, n).translate((x, y, z0))


def ring(r_in, r_out, z0, z1, x=0.0, y=0.0):
    return cyl(r_out, z0, z1, x, y) - cyl(r_in, z0 - 0.01, z1 + 0.01, x, y, hole=True)


def union(parts):
    parts = [p for p in parts if p is not None]
    if not parts:
        return M()
    return M.batch_boolean(parts, m.OpType.Add) if len(parts) > 1 else parts[0]


# ---------- rigid transforms as 4x4 numpy ----------
def T(x=0.0, y=0.0, z=0.0):
    a = np.eye(4); a[:3, 3] = (x, y, z); return a


def Rx(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    a = np.eye(4); a[1:3, 1:3] = [[c, -s], [s, c]]; return a


def Ry(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    a = np.eye(4); a[0, 0], a[0, 2], a[2, 0], a[2, 2] = c, s, -s, c; return a


def Rz(deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    a = np.eye(4); a[:2, :2] = [[c, -s], [s, c]]; return a


def basis(xc, yc, zc, origin=(0, 0, 0)):
    """Frame whose X/Y/Z axes map to the given vectors (columns), at origin."""
    a = np.eye(4); a[:3, 0], a[:3, 1], a[:3, 2] = xc, yc, zc; a[:3, 3] = origin; return a


def xf(solid, mat):
    return solid.transform(np.asarray(mat, dtype=float)[:3, :])


def along(solid_z, axis):
    """Re-orient a solid built along +Z so its +Z points along `axis` ('x','-x','y','-y','z','-z')."""
    R = {'z': np.eye(4), '-z': Rx(180), 'x': Ry(90), '-x': Ry(-90), 'y': Rx(-90), '-y': Rx(90)}[axis]
    return xf(solid_z, R)


def mesh_arrays(solid):
    mm = solid.to_mesh()
    return np.asarray(mm.vert_properties)[:, :3].astype(np.float64), np.asarray(mm.tri_verts).astype(np.int64)
