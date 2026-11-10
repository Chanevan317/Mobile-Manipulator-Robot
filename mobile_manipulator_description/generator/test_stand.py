"""Calibration plate for one arm (user, 2026-11-09: "for calibration and stuff only"). Not part of the robot or URDF.

A flat square plate, C-clamped on a table with its front edge (x = PLATE_FRONT in the arm frame) at the table edge.
The arm base sits on its rear part (the pillar); the round joint end and the shoulder axis overhang the edge, so the
arm swings out beyond the table: the wrist hangs ~70 mm below the base and must not come back over the table.
4 x M5 x 12 from below into the base's M5 inserts (same pattern as the carriage bracket), heads flush in counterbores."""
from lib import box, cyl, union, xf, T

SIZE, THICK = 150.0, 10.0
PLATE_FRONT = -15.0            # front edge of the plate in the arm frame (shoulder axis 15 mm past it)
M5 = [(-80.0, 13.5), (-80.0, -13.5), (-54.0, 13.5), (-54.0, -13.5)]
CLAMP_X, CLAMP_Y, CLAMP_DEPTH = (-90.0, -64.0), (30.0, 55.0), 2.0   # clamp pockets (arm frame, mm): 49..75 mm from the table edge


def stand():
    zt = -40.0; zb = zt - THICK
    s = box(PLATE_FRONT - SIZE, PLATE_FRONT, -SIZE / 2, SIZE / 2, zb, zt)
    cut = []
    for x, y in M5:
        cut.append(xf(cyl(2.75, zb - 1, zt + 1, hole=True), T(x, y, 0)))         # M5 clearance
        cut.append(xf(cyl(4.75, zb - 1, zb + 5.5, hole=True), T(x, y, 0)))       # head counterbore, flush underneath
    # C-clamp pockets (2 mm deep): beside the pillar, toward the back (~150 deg from the arm's zero heading), the only
    # spots near the table edge the upper arm never sweeps over (clear up to +-120 deg). Needs C-clamps with >= 80 mm throat.
    for sy in (-1, 1):
        y0, y1 = sorted((sy * CLAMP_Y[0], sy * CLAMP_Y[1]))
        cut.append(box(CLAMP_X[0], CLAMP_X[1], y0, y1, zt - CLAMP_DEPTH, zt + 1))
    return s - union(cut)
