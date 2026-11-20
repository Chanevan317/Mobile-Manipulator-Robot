"""Live view of the calibrated leader: its joint angles and the follower command they make (Ctrl+C to stop).

    python train_pipeline/leader/check_leader.py [--port ...] [--name leader_mono]

A '*' marks a joint the follower can't follow that far (the command is clamped there). Also prints the read rate:
teleop records at 30 Hz, so it must stay well above that.
"""
import argparse, math, os, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import leader_arm as LA                                                   # noqa: E402

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('--port', default=None)
ap.add_argument('--name', default=LA.DEFAULT_NAME)
args = ap.parse_args()

L = LA.Leader(port=args.port, name=args.name)
print('leader', args.name, '(calibrated', L.cal['date'] + ')')
print('  ' + '  '.join(f'{name:>16s}' for _, _, name, _ in LA.JOINTS))
n, t0 = 0, time.time()
try:
    while True:
        d = L.degrees()
        c = L.command(d)
        cells = []
        for j, _, name, lim in LA.JOINTS:
            if j == 'gripper':
                cells.append(f'{d[j]:+6.1f}d {c[name] * 1000:4.0f}mm')
            else:
                star = '*' if not lim[0] <= d[j] <= lim[1] else ' '
                cells.append(f'{d[j]:+7.1f} -> {math.degrees(c[name]):+5.0f}{star}')
        n += 1
        hz = n / max(time.time() - t0, 1e-6)
        sys.stdout.write('\r  ' + '  '.join(f'{x:>16s}' for x in cells) + f'   {hz:5.0f} Hz'); sys.stdout.flush()
except KeyboardInterrupt:
    print()
finally:
    L.close()
