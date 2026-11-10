"""Heat-set insert test block (user, 2026-11-10): print it before the arm parts to pick the insert hole size.

Top row: five M3 holes, Ø4.1 / 4.2 / 4.3 / 4.4 / 4.5, 8.5 mm deep (fits the kit's M3 x 4 / 6 / 8, 5.0 mm outside).
Bottom row: three M5 holes, Ø6.2 / 6.4 / 6.6, 10.5 mm deep (kit M5 x 8, 7.0 mm outside).
Hole size grows from left to right; a notch in the left edge marks the smallest hole. Each hole has the same
0.4 mm entry chamfer as the parts (hw.insert_cut). Print it flat, holes up, with the same settings as the arm parts.
The chosen sizes go into hw.SPEC ('ins'). Not part of the robot or the URDF."""
from lib import box, cyl, union

M3_HOLES = (4.1, 4.2, 4.3, 4.4, 4.5)
M5_HOLES = (6.2, 6.4, 6.6)
H = 12.0            # block height: 10.5 mm M5 hole + 1.5 mm floor


def block():
    W, D = 5 * 11.0 + 6.0, 34.0                       # 61 x 34 x 12 mm
    s = box(0, W, 0, D, 0, H)
    cut = []
    for i, d in enumerate(M3_HOLES):                  # top row, 11 mm apart (>= 1.5 mm wall for a 5 mm insert)
        x, y = 8.5 + i * 11.0, D - 9.0
        cut += [cyl(d / 2, H - 8.5, H + 1, x, y, hole=True), cyl(d / 2 + 0.4, H - 0.4, H + 1, x, y, hole=True)]
    for i, d in enumerate(M5_HOLES):                  # bottom row, 17 mm apart
        x, y = 13.5 + i * 17.0, 10.0
        cut += [cyl(d / 2, H - 10.5, H + 1, x, y, hole=True), cyl(d / 2 + 0.4, H - 0.4, H + 1, x, y, hole=True)]
    cut.append(box(-1, 2.0, D / 2 - 2.0, D / 2 + 2.0, -1, H + 1))    # notch: smallest holes on this side
    return s - union(cut)


if __name__ == '__main__':
    b = block(); bb = b.bounding_box()
    print('insert test block', [round(v, 1) for v in bb], 'volume cm3', round(b.volume() / 1000, 1))
