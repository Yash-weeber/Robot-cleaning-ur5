#!/bin/bash
#SBATCH -A grp_hbenamor
#SBATCH -N 1
#SBATCH -c 8
#SBATCH -t 0-10:30:00
#SBATCH -p public
#SBATCH -q public
#SBATCH --mem=16G
#SBATCH --gpus-per-node=a100:1
#SBATCH -o sessions/errors/template-1.%j.out
#SBATCH -e sessions/errors/template-1.%j.err
#SBATCH --mail-type=ALL
#SBATCH --mail-user=melmisti@asu.edu
#SBATCH --export=NONE

set -euo pipefail

set +u
source ~/miniconda3/etc/profile.d/conda.sh
conda activate robot_cleaning
set -u

# --- Start Ollama server on the compute node ---
if ! command -v ollama >/dev/null 2>&1; then
  echo "ERROR: 'ollama' not found in PATH on this node."
  exit 1
fi

export OLLAMA_HOST="127.0.0.1:11434"
# Optional: where models live (avoid writing to $HOME if needed)
# export OLLAMA_MODELS="/scratch/$USER/ollama/models"

# Start server in background
ollama serve > "sessions/errors/ollama.${SLURM_JOB_ID}.log" 2>&1 &
OLLAMA_PID=$!

# Ensure cleanup on exit
cleanup() {
  kill "$OLLAMA_PID" >/dev/null 2>&1 || true
}
trap cleanup EXIT

# Wait until server is listening
for i in {1..60}; do
  if curl -fsS "http://${OLLAMA_HOST}/api/tags" >/dev/null 2>&1; then
    echo "Ollama is up."
    break
  fi
  sleep 1
done

# (Optional) pull the model once, so first call is not blocked
# ollama pull gpt-oss:120b

# --- Run your program ---
python main_llm.py --config semantics-guided-gridcoverage-20x20-hist-30-sinusoid-x-warmup-5.yaml