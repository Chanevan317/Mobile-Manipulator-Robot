"""Give the leader's servos their ids 1..6 and the bus speed (new or replaced servos only; run in lerobot_env).

    conda activate lerobot_env
    python train_pipeline/leader/setup_ids.py --port /dev/serial/by-id/usb-1a86_..._-if00

The same as `lerobot-setup-motors --teleop.type=so100_leader` (same ids, same order, 1 Mbaud), but it asks for the
servos by THIS arm's joint names. A leader assembled with the SO-100 leader's servos in the SO-100 order already has
the right ids: skip this. Connect ONE servo at a time when asked, gripper first.
"""
import argparse, os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import leader_arm as LA                                                   # noqa: E402

ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
ap.add_argument('--port', default=os.environ.get('LEADER_PORT'))
args = ap.parse_args()
if not args.port:
    ap.error('give --port (ls -l /dev/serial/by-id/) or set LEADER_PORT')

from lerobot.motors import Motor, MotorNormMode                           # noqa: E402
from lerobot.motors.feetech import FeetechMotorsBus                       # noqa: E402

WHERE = {'J1': 'the shoulder servo (turns with link 1, on the hub)', 'J2': 'the elbow servo (end of link 1)',
         'J3': 'the wrist-pan servo (end of link 2)', 'wrist_pitch': 'the pitch servo (in the wrist)',
         'wrist_roll': 'the roll servo (on the pitch fork)', 'gripper': 'the gripper servo (moving finger loop)'}
bus = FeetechMotorsBus(port=args.port, motors={j: Motor(sid, 'sts3215', MotorNormMode.DEGREES)
                                               for j, sid, _, _ in LA.JOINTS})
for j, sid, _, _ in reversed(LA.JOINTS):
    input(f'Connect the board to {WHERE[j]} ONLY (id {sid}), then press Enter.')
    bus.setup_motor(j)
    print(f"  {j}: id {sid}, {bus.default_baudrate} baud")
print('done. Now run calibrate.py.')
