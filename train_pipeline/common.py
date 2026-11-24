"""Settings shared by every train_pipeline step (plain Python: imported by the Isaac recorder AND the LeRobot env).

One frame = the arm's state, the command sent to it (the action) and both camera images, at FPS.

State / action vector, in this order, SI units (rad, m). The real robot driver must use the same order and units.
    lift.pos           lift height (m, 0 .. 0.871)
    shoulder.pos       arm shoulder (URDF left_shoulder_pan, rad)
    elbow.pos          (URDF left_elbow_pan, rad)
    wrist_pan.pos      (URDF left_wrist_pan, rad)
    (since 2026-11-23, the mono HOLDER robot: the arm's own 3 pan joints. The ring robot's datasets used ring / shoulder /
    elbow for the same three URDF joints, shifted by one: they are not compatible.)
    wrist_pitch.pos    (rad, + = tip down)
    wrist_roll.pos     (rad)
    gripper.pos        jaw opening (m, 0 .. 0.09 = both fingers)
Action = the joint command the controller sent at that frame (proto_task's rate-limited q_cmd), not the measured
position: the same rule as the SO-101 project ("recorded action = leader command"), so the policy learns to command
and the gripper squeezes.
"""
from pathlib import Path

FPS = 30                      # recorded frames per second (the sim controls at 60 Hz: every 2nd step is kept)
ROBOT_TYPE = 'mobile_manipulator_mono'

# (LeRobot name, URDF joint(s)); the gripper is the sum of the two finger joints
JOINTS = [
    ('lift.pos', 'lift'),
    ('shoulder.pos', 'left_shoulder_pan'),
    ('elbow.pos', 'left_elbow_pan'),
    ('wrist_pan.pos', 'left_wrist_pan'),
    ('wrist_pitch.pos', 'left_wrist_pitch'),
    ('wrist_roll.pos', 'left_wrist_roll'),
    ('gripper.pos', ('left_finger_right', 'left_finger_left')),
]
JOINT_NAMES = [n for n, _ in JOINTS]

# LeRobot image key -> camera name in scripts/common/cameras.py
CAMERAS = {
    'observation.images.scene': 'scene',          # Orbbec Gemini 2 colour, 640 x 360, on the carriage mast
    'observation.images.wrist': 'left_wrist',     # OV2710 wrist camera, 640 x 360
}

ROOT = Path(__file__).resolve().parent
RAW_DIR = ROOT / 'data' / 'raw'               # episode_XXXXXX.h5 per episode, one folder per recording set
LEROBOT_DIR = ROOT / 'data' / 'lerobot'       # converted LeRobotDatasets
OUTPUTS_DIR = ROOT / 'outputs'                # training runs (checkpoints)
REPO = ROOT.parent


def task_text(kind, product, where):
    """The language instruction stored with each episode."""
    if kind == 'pick':
        return f'pick the {product.lower()} and put it in the tote'
    return f'take the {product.lower()} out of the tote and put it back on the {where}'
