#!/bin/bash
# Run this ON THE SERVER, after connecting via upload_and_connect.ps1:
#   bash train_remote.sh

set -e

BUILD_ZIP=~/Build_Linux.zip
BUILD_DIR=~/Build_Linux
ENV_PATH=~/Build_Linux/Build_Linux/test.x86_64   # adjust the exe name if different
CONFIG=~/config.yaml
NUM_ENVS=16
TB_PORT=6006
MLAGENTS=~/miniconda/envs/mlagents/bin/mlagents-learn
TENSORBOARD=~/miniconda/envs/mlagents/bin/tensorboard

RUN_ID="gfsx_$(date +%Y%m%d_%H%M%S)"

echo "1. Unzipping fresh build..."
rm -rf "$BUILD_DIR"
unzip -o "$BUILD_ZIP" -d "$BUILD_DIR"

echo "2. Setting executable permission..."
chmod +x "$ENV_PATH"

echo "3. Starting training (run-id=$RUN_ID) in tmux..."
tmux new-session -d -s "train_$RUN_ID" \
  "$MLAGENTS $CONFIG --run-id=$RUN_ID --force --env=$ENV_PATH --num-envs=$NUM_ENVS --no-graphics --env-args -logFile /dev/null; exec bash"

echo "4. Starting TensorBoard in tmux..."
tmux new-session -d -s tensorboard \
  "$TENSORBOARD --logdir ~/results --port $TB_PORT; exec bash"

echo ""
echo "Done. run-id = $RUN_ID"
echo "Reattach to training:   tmux attach -t train_$RUN_ID"
echo "Reattach to TensorBoard: tmux attach -t tensorboard"
echo ""
echo "To view TensorBoard in your browser, open a NEW LOCAL PowerShell window and run:"
echo "  ssh -i <key> $USER@<VM_IP> -L ${TB_PORT}:127.0.0.1:${TB_PORT}"
echo "then open http://localhost:${TB_PORT}"
