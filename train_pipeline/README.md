# Training pipeline (LeRobot)

Record demonstrations in the Isaac store scene, check them, turn them into a LeRobot dataset, and train ACT.
The split (decided 2026-11-19): **our own code** for IK and motion planning (`scripts/common/scara_ik.py`,
`collide.py`, `scripts/scenes/scene_v0/lib/pick_place.py`), **LeRobot** for the dataset format and the policies.

Two conda environments, as in the SO-101 project, so LeRobot never has to live in the Isaac environment:

| Step | Environment | Script |
|---|---|---|
| 1. Record episodes | `env_isaaclab` (Isaac Sim 5.1) | `record/record_sim.py` |
| 2. Check them | `env_isaaclab` | `tools/inspect_raw.py`, `tools/replay_episode.py` |
| 3. Convert to a LeRobotDataset | `lerobot_env` (LeRobot 0.6.1) | `convert/to_lerobot.py` |
| 4. Train ACT | `lerobot_env` | `train/train_act.sh` |
| 5. Run the policy in the sim | not built yet | next step (see below) |

Run every command from the repo root.

## Folders

```
train_pipeline/
  common.py              settings every step shares: joint order + units, cameras, frame rate, paths, instructions
  record/record_sim.py   Isaac: run episodes, write raw HDF5
  tools/inspect_raw.py   list episodes, contact sheet of one episode (no Isaac)
  tools/replay_episode.py  Isaac: replay the recorded actions from the saved start, check the result
  convert/to_lerobot.py  raw HDF5 -> LeRobotDataset (AV1 videos)
  train/train_act.sh     lerobot-train with ACT
  data/raw/<name>/       episode_000000.h5 ...              (git-ignored)
  data/lerobot/<name>/   converted datasets                  (git-ignored)
  outputs/<job>/         training runs, checkpoints          (git-ignored)
```

## 1. Record

```bash
conda activate env_isaaclab
OMNI_KIT_ACCEPT_EULA=YES python train_pipeline/record/record_sim.py --episodes 50 --place --name pick_tote_v1
```

- **Who does the task:** for now the planner (`--source planner`): the same offline-planned pick and place as the
  "Pick to tote" panel. The leader arm (`--source leader`) plugs in once it is built.
- **One episode:** from the robot parked at the station, arm folded, to the arm folded again. Driving between
  stations is not recorded (navigation stays scripted). Before every episode the moved products go back to their spots.
- **`--place`:** after each pick, putting the product back is recorded as its own episode with its own instruction,
  so one dataset teaches two tasks.
- **Stations:** every station that picks reliably (`pick_place.PICKABLE`, 16 of 19), chosen at random;
  `--stations a,b` for a few.
- **Randomised per episode** so the policy can't memorise one picture (`--no_jitter` turns it off): where the robot stops
  (±2 cm, ±3°), the product (±1 cm, ±8°), the light (±25 %).
- Only successful episodes are kept (product in the tote / back on its spot, nothing else knocked over).
- Headless by default (`--window` to watch). Plans are cached, so the first episode at a station is slower.

Each frame (30 per second):

| Key | What |
|---|---|
| `observation.state` | 7 measured values, see below |
| `action` | the 7 values **commanded** at that moment (not the measured ones: the policy learns to command, and the gripper squeezes) |
| `observation.images.scene` | OAK-D Pro W colour image, 640 × 400 (JPEG in the raw file, AV1 video after converting) |
| `observation.images.wrist` | wrist camera, 640 × 360 |
| `observation.depth.scene` | only with `--depth`: depth in mm, for point-cloud policies later (not converted yet) |

The 7 values, in this order and in SI units. **The real robot's driver must use the same order and units.**

| # | Name | Joint in the URDF | Unit |
|---|---|---|---|
| 0 | `lift.pos` | `lift` | m (0–0.81) |
| 1 | `ring.pos` | `left_shoulder_pan` (the ring around the column) | rad |
| 2 | `shoulder.pos` | `left_elbow_pan` | rad |
| 3 | `elbow.pos` | `left_wrist_pan` | rad |
| 4 | `wrist_pitch.pos` | `left_wrist_pitch` | rad |
| 5 | `wrist_roll.pos` | `left_wrist_roll` | rad |
| 6 | `gripper.pos` | `left_finger_right` + `left_finger_left` | m, jaw opening (0–0.09) |

(The mono arm's URDF joint names are shifted by one because the ring took the shoulder's name; see CLAUDE.md.)

Each file also stores what is needed to rebuild its start (every joint, the base pose, the product's pose, the light),
the instruction (for example "pick the milk carton and put it in the tote") and the randomisation used.

## 2. Check

```bash
python train_pipeline/tools/inspect_raw.py pick_tote_v1               # one line per episode
python train_pipeline/tools/inspect_raw.py pick_tote_v1 --sheet 0     # 8 moments of episode 0, both cameras -> PNG
OMNI_KIT_ACCEPT_EULA=YES python train_pipeline/tools/replay_episode.py pick_tote_v1 --episodes 0,1
```

The replay rebuilds each episode's start and sends **only the recorded actions**. If the product still ends up in
the tote (or back on its spot), the actions are enough to reproduce the demonstration. The replay also reports how far
the replayed joints drift from the recorded ones.

## 3. Convert

```bash
conda activate lerobot_env
python train_pipeline/convert/to_lerobot.py --raw pick_tote_v1            # -> data/lerobot/pick_tote_v1
python train_pipeline/convert/to_lerobot.py --raw pick_tote_v1 --kind pick --out pick_only_v1
```

Failed episodes are skipped. Planner and leader recordings can't be mixed in one dataset (they would teach two
meanings of "action"); convert them separately.

## 4. Train ACT

```bash
conda activate lerobot_env
DATASET=pick_tote_v1 ./train_pipeline/train/train_act.sh
DATASET=pick_tote_v1 STEPS=200 SAVE_FREQ=100 JOB=smoke ./train_pipeline/train/train_act.sh   # quick check first
```

Defaults: 100 000 steps, batch 32, chunk 100 (3.3 s at 30 fps), checkpoints every 10 000 steps in
`train_pipeline/outputs/<JOB>/`. Change them with `STEPS=`, `BATCH_SIZE=`, `CHUNK=`, `GPU=`, or add any
`lerobot-train` option at the end. Resume: see the top of `train_act.sh`.

## 5. Next: run the policy in the sim

Not built yet. The plan: a policy runner in Isaac built on `replay_episode.py` (same start, same 7-value command
every frame), with the policy asked for actions from the two camera images and the state, as the SO-101 project's
policy server does. Then the leader-arm source for recording.

## Things to know

- **Real time:** the recorder runs headless and does not need to keep up with real time; the episode's clock is
  the sim's. Teleop with the leader arm will need about real time; check the frame rate with the cameras on first.
- **Cameras:** the wrist camera's 120° lens is assumed until the real one is calibrated. Data recorded before that
  might need recording again.
- **Sim-to-real gaps:** servo gains are not measured from real servos, the sim gripper is a simple slide, product masses are
  estimates and the products are rigid.
