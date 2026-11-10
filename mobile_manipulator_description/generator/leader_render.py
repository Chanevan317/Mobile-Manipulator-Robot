import sys; sys.path.insert(0, '.')
import numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import leader as LD
LD.build()
def draw(ax, q, el, az, trig=0.0, zoom=1.0, center=None):
    W = LD.world(q, trig)
    tris, cols = [], []
    for nm, s, c, b in W:
        m = s.to_mesh(); t = m.vert_properties[:, :3][m.tri_verts]
        if len(t): tris.append(t); cols.append(np.tile(np.r_[c[:3]], (len(t), 1)))
    T_ = np.vstack(tris); C = np.vstack(cols)
    N = np.cross(T_[:, 1] - T_[:, 0], T_[:, 2] - T_[:, 0]); N /= np.linalg.norm(N, axis=1, keepdims=True) + 1e-12
    L = np.array([np.cos(np.radians(el + 25)) * np.cos(np.radians(az + 30)), np.cos(np.radians(el + 25)) * np.sin(np.radians(az + 30)), np.sin(np.radians(el + 25))])
    sh = 0.35 + 0.65 * np.abs(N @ L)
    ax.add_collection3d(Poly3DCollection(T_, facecolors=np.c_[C * sh[:, None], np.ones(len(sh))], edgecolor='none'))
    lo, hi = T_.reshape(-1, 3).min(0), T_.reshape(-1, 3).max(0); c = (lo + hi) / 2 if center is None else np.array(center); r = (hi - lo).max() / 2 / zoom
    ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
    ax.view_init(el, az); ax.set_proj_type('ortho'); ax.set_axis_off()
fig = plt.figure(figsize=(22, 13), facecolor='white')
ax = fig.add_subplot(2, 2, 1, projection='3d'); draw(ax, (0, 0, 0, 0, 0), 30, -60, zoom=1.7); ax.set_title('zero pose', fontsize=14)
ax = fig.add_subplot(2, 2, 2, projection='3d'); draw(ax, (40, -70, 50, 45, 30), 25, -120, trig=20, zoom=1.7); ax.set_title('working pose: J1 40, J2 -70, J3 50, pitch 45, roll 30, trigger 20', fontsize=14)
ax = fig.add_subplot(2, 2, 3, projection='3d'); draw(ax, (0, 0, 0, 0, 0), 20, -35, zoom=2.2, center=(LD.L1 + LD.L2 + LD.PITCH_X + 45, 0, 0)); ax.set_title('wrist: pitch fork (encoder lid on +Y), roll tube, grip + trigger', fontsize=14)
ax = fig.add_subplot(2, 2, 4, projection='3d'); draw(ax, (0, 0, 0, 0, 0), 10, -90, zoom=3.2, center=(0, 0, 5)); ax.set_title('pan joint: hub on the parent, fork with bearings + encoder housing on the child', fontsize=14)
plt.tight_layout(); plt.savefig(sys.argv[1], dpi=55)
