#!/usr/bin/env bash
# Train ACT on a converted dataset with LeRobot (step 4 of the pipeline).
#
#   conda activate lerobot_env
#   DATASET=pick_tote_v1 ./train_pipeline/train/train_act.sh
#   DATASET=pick_tote_v1 STEPS=200 SAVE_FREQ=100 JOB=smoke ./train_pipeline/train/train_act.sh     # quick check
#
# Settings come from env vars (defaults below); extra arguments go straight to lerobot-train, e.g.
#   ./train_pipeline/train/train_act.sh --policy.chunk_size=50 --policy.n_action_steps=50
# Resume a stopped run:
#   lerobot-train --config_path=train_pipeline/outputs/<JOB>/checkpoints/last/pretrained_model/train_config.json --resume=true
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PIPE="$(cd "$HERE/.." && pwd)"

DATASET="${DATASET:?set DATASET=<folder name in train_pipeline/data/lerobot>}"
DATASET_ROOT="${DATASET_ROOT:-$PIPE/data/lerobot/$DATASET}"
REPO_ID="${REPO_ID:-local/$DATASET}"            # only a label for local datasets
JOB="${JOB:-act_$DATASET}"
OUTPUT_DIR="${OUTPUT_DIR:-$PIPE/outputs/$JOB}"
STEPS="${STEPS:-100000}"
BATCH_SIZE="${BATCH_SIZE:-32}"                   # ACT + 2 cameras at 640 x 400 / 360: fits a 16 GB GPU
SAVE_FREQ="${SAVE_FREQ:-10000}"
LOG_FREQ="${LOG_FREQ:-200}"
NUM_WORKERS="${NUM_WORKERS:-4}"                  # video decoding happens on the CPU
SEED="${SEED:-1000}"
GPU="${GPU:-0}"
# ACT chunk: 100 actions = 3.3 s at 30 fps (LeRobot's default); the arm's moves last 1-4 s
CHUNK="${CHUNK:-100}"
# FAST=1 (default): bf16 mixed precision + torch.compile through lerobot_train_fast.py, ~2x faster on the RTX 5060 Ti
# (measured 2026-11-20, see that file). FAST=0 = plain lerobot-train in float32.
FAST="${FAST:-1}"

if [[ ! -d "$DATASET_ROOT" ]]; then
  echo "no dataset at $DATASET_ROOT (convert first: python train_pipeline/convert/to_lerobot.py --raw <raw name>)" >&2
  exit 1
fi
if [[ -e "$OUTPUT_DIR" ]]; then
  echo "$OUTPUT_DIR exists: pick another JOB=..., delete it, or resume with --config_path (see the top of this file)" >&2
  exit 1
fi

export CUDA_VISIBLE_DEVICES="$GPU"
export PYTORCH_CUDA_ALLOC_CONF="${PYTORCH_CUDA_ALLOC_CONF:-expandable_segments:True}"

if [[ "$FAST" == "1" ]]; then
  TRAIN=(python "$PIPE/train/lerobot_train_fast.py" --policy.use_amp=true)
else
  TRAIN=(lerobot-train)
fi

exec "${TRAIN[@]}" \
  --dataset.repo_id="$REPO_ID" \
  --dataset.root="$DATASET_ROOT" \
  --policy.type=act \
  --policy.device=cuda \
  --policy.push_to_hub=false \
  --policy.chunk_size="$CHUNK" \
  --policy.n_action_steps="$CHUNK" \
  --batch_size="$BATCH_SIZE" \
  --steps="$STEPS" \
  --save_freq="$SAVE_FREQ" \
  --log_freq="$LOG_FREQ" \
  --num_workers="$NUM_WORKERS" \
  --seed="$SEED" \
  --job_name="$JOB" \
  --output_dir="$OUTPUT_DIR" \
  --wandb.enable=false \
  "$@"
