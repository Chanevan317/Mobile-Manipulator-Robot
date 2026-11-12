"""Preview image of a robot layout from its URDF (visual meshes and colours), e.g. the mono-arm experiment.

    python3 render_robot.py .. prototype_mono_arm ../preview_prototype_mono.png ['{"lift": 0.3, "left_shoulder_pan": 0.5}']
"""
import sys, json, math
import numpy as np
import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from urdf_fk import Robot

BG = (0.906, 0.922, 0.941)


def subdivide(t, edge):
    """Split triangles (n x 3 x 3) into 4 until no edge is longer than `edge` (m)."""
    out = []
    while len(t):
        L = np.max(np.linalg.norm(t - np.roll(t, 1, axis=1), axis=2), axis=1)
        ok = L <= edge; out.append(t[ok]); t = t[~ok]
        if not len(t): break
        a, b_, c = t[:, 0], t[:, 1], t[:, 2]; ab, bc, ca = (a + b_) / 2, (b_ + c) / 2, (c + a) / 2
        t = np.concatenate([np.stack([a, ab, ca], 1), np.stack([ab, b_, bc], 1), np.stack([ca, bc, c], 1), np.stack([ab, bc, ca], 1)])
    return np.concatenate(out) if out else t


def render(pkg, layout, out, q=None, el=22, az=-55, title=None, zoom=1.7):
    R = Robot(pkg, layout)
    render_items([(s, c) for nm, s, c in R.solids(q or {})], out, el, az, title, zoom)


def render_items(items, out, el=22, az=-55, title=None, zoom=1.7):
    """Draw any list of (solid in metres, rgb) in the same style."""
    tris, cols = [], []
    for s, c in items:
        m = s.to_mesh(); t = np.asarray(m.vert_properties)[:, :3][np.asarray(m.tri_verts)]
        if not len(t): continue
        t = subdivide(t, 0.02)                                       # big flat faces -> small pieces, so the depth sort works
        tris.append(t); cols.append(np.tile(np.r_[c[:3]], (len(t), 1)))
    T_ = np.vstack(tris); C = np.vstack(cols)
    N = np.cross(T_[:, 1] - T_[:, 0], T_[:, 2] - T_[:, 0]); N /= np.linalg.norm(N, axis=1, keepdims=True) + 1e-12
    L = np.array([math.cos(math.radians(el + 25)) * math.cos(math.radians(az + 30)),
                  math.cos(math.radians(el + 25)) * math.sin(math.radians(az + 30)), math.sin(math.radians(el + 25))])
    sh = 0.35 + 0.65 * np.abs(N @ L)
    fig = plt.figure(figsize=(12, 8.5), facecolor=BG)
    ax = fig.add_subplot(111, projection='3d'); ax.set_facecolor(BG)
    ax.add_collection3d(Poly3DCollection(T_, facecolors=np.c_[C * sh[:, None], np.ones(len(sh))], edgecolor='none'))
    lo, hi = T_.reshape(-1, 3).min(0), T_.reshape(-1, 3).max(0); c = (lo + hi) / 2; r = (hi - lo).max() / 2 / zoom
    ax.set_xlim(c[0] - r, c[0] + r); ax.set_ylim(c[1] - r, c[1] + r); ax.set_zlim(c[2] - r, c[2] + r)
    ax.view_init(el, az); ax.set_proj_type('ortho'); ax.set_axis_off()
    if title: ax.set_title(title, fontsize=14)
    plt.tight_layout(); plt.savefig(out, dpi=100, facecolor=BG); plt.close(fig)


if __name__ == '__main__':
    pkg, layout, out = sys.argv[1], sys.argv[2], sys.argv[3]
    q = json.loads(sys.argv[4]) if len(sys.argv) > 4 else None
    render(pkg, layout, out, q)


def render_pair(pkg, layout, out, q1, q2, titles, tmp_prefix, az=(-55, -55)):
    """Two views side by side (e.g. the mono arm facing the front and turned toward the tote); az per view."""
    render(pkg, layout, tmp_prefix + '_a.png', q1, az=az[0], zoom=1.5)
    render(pkg, layout, tmp_prefix + '_b.png', q2, az=az[1], zoom=1.5)
    fig, ax = plt.subplots(1, 2, figsize=(20, 8), facecolor=BG)
    for a, f, t in zip(ax, ('_a.png', '_b.png'), titles):
        a.imshow(plt.imread(tmp_prefix + f)); a.set_axis_off(); a.set_title(t, fontsize=15)
    plt.tight_layout(); plt.savefig(out, dpi=90, facecolor=BG); plt.close(fig)
