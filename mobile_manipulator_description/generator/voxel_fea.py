"""Small linear-elastic voxel FEA (numpy + scipy only): a solid is cut into h-mm cubes, each an 8-node hexahedron with
Wilson's incompatible bending modes (statically condensed; exact for pure bending of a cube, so thin walls only 3-5
voxels thick are not over-stiff the way plain 8-node bricks are). Used by mono_holder_fea.py.

    vox = voxelize(solid, h, origin)       -> integer voxel indices (i, j, k), cube [o + i h, o + (i+1) h]
    m = Model(vox, h, origin, E, nu, scale)  scale: per-voxel stiffness factor (e.g. a stiff block standing in for a bought part)
    u = m.solve(F, fixed)                  F (n_nodes, 3) N; fixed: node ids held in all 3 directions
    s = m.stress(u)                        (n_vox, 6) xx yy zz yz xz xy at each voxel centre, MPa (mm, N)
Units: mm, N, MPa.
"""
import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spla
from matplotlib.path import Path

CORN = np.array([[0, 0, 0], [1, 0, 0], [1, 1, 0], [0, 1, 0], [0, 0, 1], [1, 0, 1], [1, 1, 1], [0, 1, 1]])
SGN = 2 * CORN - 1                                                    # natural coordinates of the corners


def D_iso(E, nu):
    l = E * nu / ((1 + nu) * (1 - 2 * nu)); mu = E / (2 * (1 + nu))
    D = np.zeros((6, 6)); D[:3, :3] = l; D[range(3), range(3)] += 2 * mu; D[range(3, 6), range(3, 6)] = mu
    return D


def _B(dN):
    """dN (n, 3) shape-function derivatives in x -> strain-displacement matrix (6, 3n), engineering shear."""
    n = len(dN); B = np.zeros((6, 3 * n))
    B[0, 0::3] = dN[:, 0]; B[1, 1::3] = dN[:, 1]; B[2, 2::3] = dN[:, 2]
    B[3, 1::3] = dN[:, 2]; B[3, 2::3] = dN[:, 1]
    B[4, 0::3] = dN[:, 2]; B[4, 2::3] = dN[:, 0]
    B[5, 0::3] = dN[:, 1]; B[5, 1::3] = dN[:, 0]
    return B


def element(h, D):
    """Condensed 24 x 24 stiffness of an h-cube, and the centre B matrix (stress recovery)."""
    g = 1 / np.sqrt(3); J = h / 2.0
    Kuu = np.zeros((24, 24)); Kua = np.zeros((24, 9)); Kaa = np.zeros((9, 9))
    for xi in (-g, g):
        for et in (-g, g):
            for ze in (-g, g):
                p = np.array([xi, et, ze])
                dN = np.array([[s[0] * (1 + s[1] * et) * (1 + s[2] * ze), s[1] * (1 + s[0] * xi) * (1 + s[2] * ze),
                                s[2] * (1 + s[0] * xi) * (1 + s[1] * et)] for s in SGN]) / 8.0 / J
                B = _B(dN)
                dM = np.diag(-2 * p) / J                               # incompatible modes 1 - xi^2, 1 - eta^2, 1 - zeta^2
                G = _B(dM)                                             # (6, 9)
                w = J ** 3
                Kuu += B.T @ D @ B * w; Kua += B.T @ D @ G * w; Kaa += G.T @ D @ G * w
    K = Kuu - Kua @ np.linalg.solve(Kaa, Kua.T)
    dN0 = np.array([[s[0], s[1], s[2]] for s in SGN]) / 8.0 / J
    return K, _B(dN0)


def voxelize(solid, h, origin, bounds=None):
    """Voxels whose centre is inside `solid` (manifold, mm): slices at each voxel-centre z, even-odd over the contours."""
    bb = solid.bounding_box() if bounds is None else bounds
    o = np.asarray(origin, float)
    i0, j0, k0 = [int(np.floor((bb[a] - o[a]) / h)) for a in range(3)]
    i1, j1, k1 = [int(np.ceil((bb[a + 3] - o[a]) / h)) for a in range(3)]
    I, Jj = np.meshgrid(np.arange(i0, i1), np.arange(j0, j1), indexing='ij')
    XY = np.c_[(o[0] + (I.ravel() + 0.5) * h), (o[1] + (Jj.ravel() + 0.5) * h)]
    out = []
    for k in range(k0, k1):
        z = o[2] + (k + 0.5) * h
        polys = solid.slice(z).to_polygons()
        if not len(polys): continue
        cnt = np.zeros(len(XY), int)
        for p in polys:
            cnt += Path(np.asarray(p)).contains_points(XY)
        sel = cnt % 2 == 1
        if sel.any(): out.append(np.c_[I.ravel()[sel], Jj.ravel()[sel], np.full(sel.sum(), k)])
    return np.vstack(out)


class Model:
    def __init__(s, vox, h, origin, E=1800.0, nu=0.35, scale=None):
        s.vox, s.h, s.o = np.asarray(vox), h, np.asarray(origin, float)
        s.D = D_iso(E, nu); s.Ke, s.B0 = element(h, s.D)
        corners = (s.vox[:, None, :] + CORN[None]).reshape(-1, 3)
        s.nodes_ijk, inv = np.unique(corners, axis=0, return_inverse=True)
        s.conn = inv.reshape(-1, 8)
        s.xyz = s.o + s.nodes_ijk * h                                  # node coordinates
        s.cen = s.o + (s.vox + 0.5) * h                                # voxel centres
        s.scale = np.ones(len(vox)) if scale is None else np.asarray(scale, float)
        dof = (3 * s.conn[:, :, None] + np.arange(3)).reshape(-1, 24)
        s.dof = dof
        rows = np.repeat(dof, 24, axis=1).ravel(); cols = np.tile(dof, (1, 24)).ravel()
        vals = (s.scale[:, None, None] * s.Ke[None]).reshape(len(vox), -1).ravel()
        n = 3 * len(s.xyz)
        s.K = sp.csc_matrix((vals, (rows, cols)), shape=(n, n))
        s.K.sum_duplicates()

    def factor(s, fixed, method='auto'):
        """Prepare the solver with `fixed` nodes held. 'direct' (SuperLU: fine to ~150k DOF), 'amg' (pyamg smoothed
        aggregation with the 6 rigid-body modes + CG: large models; pip install pyamg), 'auto' picks by size."""
        n = 3 * len(s.xyz)
        fixd = np.zeros(n, bool); fixd[(3 * np.asarray(fixed)[:, None] + np.arange(3)).ravel()] = True
        s.free = np.flatnonzero(~fixd)
        Kf = s.K[s.free][:, s.free].tocsr()
        if method == 'auto':
            method = 'direct' if len(s.free) < 150000 else 'amg'
        s.method = method
        if method == 'direct':
            s.lu = spla.splu(Kf.tocsc(), permc_spec='MMD_AT_PLUS_A', options=dict(SymmetricMode=True))
        else:
            import pyamg
            c = s.xyz - s.xyz.mean(0); B = np.zeros((n, 6))
            B[0::3, 0] = 1; B[1::3, 1] = 1; B[2::3, 2] = 1
            B[1::3, 3] = -c[:, 2]; B[2::3, 3] = c[:, 1]
            B[0::3, 4] = c[:, 2]; B[2::3, 4] = -c[:, 0]
            B[0::3, 5] = -c[:, 1]; B[1::3, 5] = c[:, 0]
            s.Kf = Kf
            s.ml = pyamg.smoothed_aggregation_solver(Kf, B=B[s.free], symmetry='symmetric', strength=('symmetric', dict(theta=0.0)),
                                                     smooth=('energy', dict(degree=2)), max_coarse=2000)

    def solve(s, F, tol=1e-9):
        u = np.zeros(3 * len(s.xyz))
        b = np.asarray(F, float).ravel()[s.free]
        if s.method == 'direct':
            u[s.free] = s.lu.solve(b)
        else:
            res = []
            u[s.free] = s.ml.solve(b, tol=tol, accel='cg', maxiter=2000, residuals=res)
            s.last_iters, s.last_res = len(res), res[-1] / res[0] if res else 0
        return u.reshape(-1, 3)

    def reactions(s, u, F):
        return (s.K @ u.ravel() - np.asarray(F, float).ravel()).reshape(-1, 3)

    def stress(s, u):
        ue = u.ravel()[s.dof]                                          # (n_vox, 24)
        return s.scale[:, None] * (ue @ (s.D @ s.B0).T)


def von_mises(S):
    xx, yy, zz, yz, xz, xy = S.T
    return np.sqrt(0.5 * ((xx - yy) ** 2 + (yy - zz) ** 2 + (zz - xx) ** 2) + 3 * (yz ** 2 + xz ** 2 + xy ** 2))


def rbe3(xyz, ids, F, M, about):
    """Nodal forces on nodes `ids` equivalent to force F + moment M about the point `about` (rigid distribution)."""
    P = xyz[ids]; c = P.mean(0); r = P - c
    Mc = np.asarray(M, float) + np.cross(np.asarray(about) - c, F)
    Jt = (r ** 2).sum() * np.eye(3) - r.T @ r
    w = np.linalg.solve(Jt, Mc)
    f = np.asarray(F, float) / len(ids) + np.cross(w, r)
    return f
