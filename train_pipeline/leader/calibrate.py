"""Calibrate the leader arm: its zero POSE, which way each joint turns, its ranges and the gripper's opening.

    python train_pipeline/leader/calibrate.py --port /dev/serial/by-id/usb-1a86_..._-if00     # or LEADER_PORT=...
    ... --name leader_mono        calibration file name (default; prototype later: leader_left / leader_right)

Writes train_pipeline/leader/calibration/<name>.json (commit it: a calibration belongs to the arm, not to the PC).
Runs in env_isaaclab or lerobot_env (needs only feetech-servo-sdk). Every servo's torque is off: move the arm by hand.

Why not lerobot-calibrate: it puts each joint's zero in the middle of the range you happen to sweep, and gives the
wrist roll a full turn (the Store-Picker-Bot needed calibrate_wrist_roll.py for exactly that). Here the zero is the
follower's own zero pose, so the angles copy 1:1 and the encoder seam ends up half a turn from it: behind J1's end
stop (+-174), out of reach of the other joints, and the roll (no stop) is unwrapped in software (leader_arm.py).
Do NOT run lerobot-calibrate on this leader afterwards: it rewrites the servos' homing offsets (leader_arm.py notices
and asks for a new calibration).
"""
import argparse, json, math, os, select, sys, time
from datetime import date
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import leader_arm as LA                                                   # noqa: E402

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('--port', default=os.environ.get('LEADER_PORT'))
ap.add_argument('--name', default=LA.DEFAULT_NAME)
args = ap.parse_args()
if not args.port:
    ap.error('give --port (ls -l /dev/serial/by-id/) or set LEADER_PORT')

ZERO_POSE = """ZERO POSE (the follower's zero: everything straight, as in the leader preview's first panel)
  - link 1 straight ahead, pointing AWAY from the stop rib (the rib is behind, at the dead end)
  - link 2 and the wrist in one straight line with link 1
  - gripper level (pointing straight out, not tipped up or down)
  - roll 0: the two jaw blades side by side, left and right (not one above the other)
  - finger loops closed (jaws touching)"""

# joint -> what to do for its POSITIVE direction (the follower URDF's: pans counter-clockwise from above,
# pitch = tip down, roll = +X right-hand rule, gripper = open)
POSITIVE = {
    'J1': 'turn the WHOLE ARM to the LEFT (counter-clockwise seen from above), about 45 deg',
    'J2': 'bend link 2 to the LEFT at the elbow (counter-clockwise seen from above), about 45 deg',
    'J3': 'turn the wrist to the LEFT (counter-clockwise seen from above), about 45 deg',
    'wrist_pitch': 'tip the gripper DOWN, about 45 deg',
    'wrist_roll': 'turn the gripper CLOCKWISE as you see it from BEHIND (looking along the arm), about 45 deg',
    'gripper': 'open the finger loops FULLY and hold them open',
}


def wait_enter():
    input()


def deg_from(raw, zero):
    return LA.wrap180((raw - zero) * 360.0 / LA.TICKS)


def live(bus, cal, until_enter, show_follower=False):
    """Print angles until Enter; returns the per-joint min / max seen."""
    L = LA.Leader(bus=bus, calib=cal)
    lo, hi = {}, {}
    print()
    while True:
        d = L.degrees()
        for j, v in d.items():
            lo[j] = min(lo.get(j, v), v); hi[j] = max(hi.get(j, v), v)
        txt = '  '.join(f'{j} {v:+7.1f}' for j, v in d.items())
        if show_follower:
            c = L.command(d)
            txt += '   -> follower gripper %.0f mm' % (c['gripper.pos'] * 1000)
        sys.stdout.write('\r' + txt + '   '); sys.stdout.flush()
        if until_enter and select.select([sys.stdin], [], [], 0.03)[0]:
            sys.stdin.readline()
            print()
            return lo, hi


bus = LA.Bus(args.port)
homing = bus.homing_offsets()
print(f'connected: {len(bus.ids)} servos on {args.port}, torque OFF (hold the arm). Homing offsets: {homing}')
if any(homing.values()):
    print('  (non-zero homing offsets from an earlier lerobot-calibrate: fine, they are stored and checked later)')

print('\n1/4  ' + ZERO_POSE + '\n     Put the leader in this pose, hold it still, press Enter.')
wait_enter()
zero_raw = bus.read_avg(30)
cal = {'name': args.name, 'date': str(date.today()), 'port': args.port, 'homing_offsets': homing, 'joints': {}}
for j, sid, _, _ in LA.JOINTS:
    cal['joints'][j] = {'id': sid, 'zero': round(zero_raw[sid], 1), 'sign': 1}

print('\n2/4  Directions. For each joint, start from the zero pose, move ONLY that joint as asked, press Enter.')
for j, sid, _, _ in LA.JOINTS:
    while True:
        print(f'  {j} (servo {sid}): {POSITIVE[j]}, then Enter')
        wait_enter()
        d = deg_from(bus.read_avg(10)[sid], zero_raw[sid])
        if abs(d) >= 15:
            break
        print(f'    it moved only {d:+.1f} deg: move it further (at least 15 deg)')
    cal['joints'][j]['sign'] = 1 if d > 0 else -1
    if j == 'gripper':
        cal['joints'][j]['open_deg'] = round(abs(d), 1)
        print(f'    sign {cal["joints"][j]["sign"]:+d}, fully open = {abs(d):.1f} deg (design: 40)')
        if not 25 <= abs(d) <= 50:
            print('    !! far from the design 40 deg: were the loops fully open / closed in the zero pose?')
    else:
        print(f'    sign {cal["joints"][j]["sign"]:+d} ({d:+.1f} deg). Bring it back to zero.')

print('\n3/4  Ranges. Move every joint slowly to both ends of its travel (J1 against both end stops,\n'
      '     the roll to +-90 or more, the loops fully closed and open). Press Enter when done.')
lo, hi = live(bus, cal, True)
cal['ranges_deg'] = {j: [round(lo[j], 1), round(hi[j], 1)] for j in lo}
print('     joint          leader range            follower limit')
bad = False
for j, _, _, lim in LA.JOINTS:
    note = ''
    if j == 'J1':
        if max(abs(lo[j]), abs(hi[j])) > 176:
            note, bad = '!! past the +-174 end stop: the zero pose was not straight ahead. Calibrate again.', True
        elif min(abs(lo[j]), abs(hi[j])) < 165:
            note = '(did you reach both stops? expected about -174 .. +174)'
    lim_txt = f'{lim[0]:+.0f} .. {lim[1]:+.0f}' if lim else '0 .. 90 mm'
    print(f'     {j:12s} {lo[j]:+7.1f} .. {hi[j]:+7.1f}      {lim_txt}  {note}')
print('     (leader ranges wider than the follower\'s are fine: commands are clamped to the follower limits)')

print('\n4/4  Check: move the arm around; the zero pose must read ~0 everywhere, every joint must turn the\n'
      '     way you moved it in step 2. Press Enter.')
live(bus, cal, True, show_follower=True)
bus.close()
if bad:
    sys.exit('not saved (see the !! above)')
ans = input(f'Save to {LA.calib_path(args.name)}? [y/N] ').strip().lower()
if ans == 'y':
    LA.CALIB_DIR.mkdir(exist_ok=True)
    LA.calib_path(args.name).write_text(json.dumps(cal, indent=1) + '\n')
    print('saved. Check it any time with: python train_pipeline/leader/check_leader.py')
else:
    print('not saved')
