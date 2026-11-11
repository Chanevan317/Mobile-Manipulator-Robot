import sys; sys.path.insert(0, '.')
import numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import leader as LD
from lib import box
LD.build()


def draw(ax, q, el, az, trig=0.0, zoom=1.0, center=None, variant='mono', q2=None, title=''):
    W = LD.world(q, trig, variant=variant, q2=q2)
    tris, cols = [], []
    for nm, s, c, b in W:
        m = s.to_mesh(); t = np.asarray(m.vert_properties)[:, :3][np.asarray(m.tri_verts)]
        if len(t): tris.append(t); cols.append(np.tile(np.r_[c[:3]], (len(t), 1)))
    T_ = np.vstack(tris); C = np.vstack(cols)
    N = np.cross(T_[:, 1] - T_[:, 0], T_[:, 2] - T_[:, 0]); N /= np.linalg.norm(N, axis=1, keepdims=True) + 1e-12
    L = np.array([np.cos(np.radians(el + 25)) * np.cos(np.radians(az + 30)), np.cos(np.radians(el + 25)) * np.sin(np.radians(az + 30)), np.sin(np.radians(el + 25))])
    sh = 0.35 + 0.65 * np.abs(N @ L)
    ax.add_collection3d(Poly3DCollection(T_, facecolors=np.c_[C * sh[:, None], np.ones(len(sh))], edgecolor='none'))
    lo, hi = T_.reshape(-1, 3).min(0), T_.reshape(-1, 3).max(0); c = (lo + hi) / 2 if center is None else np.array(center); r = (hi - lo).max() / 2 / zoom
    ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
    ax.view_init(el, az); ax.set_proj_type('ortho'); ax.set_axis_off(); ax.set_title(title, fontsize=14)


fig = plt.figure(figsize=(22, 13), facecolor='white')
draw(fig.add_subplot(2, 2, 1, projection='3d'), (0, 0, 0, 0, 0), 20, -60, zoom=1.1,
     title='leader v3 on the mono base (0.7 x the follower, zero pose)')
draw(fig.add_subplot(2, 2, 2, projection='3d'), (40, -70, 30, 90, 0), 20, -125, trig=20, zoom=1.1,
     title='hand down: J1 40, J2 -70, J3 30, pitch 90, gripper open 20 deg')
draw(fig.add_subplot(2, 2, 3, projection='3d'), (0, 0, 0, 0, 0), 25, -60, variant='proto', zoom=1.05,
     title='same arm modules on the prototype base (yaw +-135, column stand-in between)')
draw(fig.add_subplot(2, 2, 4, projection='3d'), (0, 0, 0, 0, 0), 35, -50, trig=25, zoom=2.2, center=(LD.L1 + LD.L2 + LD.XP + LD.XR + 50, 0, -LD.ZPD - 20),
     title='pinch gripper: fixed jaw + jaw on the gripper servo, a finger loop on each (open 25 deg)')
plt.tight_layout(); plt.savefig(sys.argv[1], dpi=55)
