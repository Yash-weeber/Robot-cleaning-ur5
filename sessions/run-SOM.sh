#!/bin/bash
#SBATCH -A grp_hbenamor
#SBATCH -N 1
#SBATCH -c 8
#SBATCH -t 0-6:30:00
#SBATCH -p public
#SBATCH -q public
#SBATCH --mem=16G
#SBATCH --gpus-per-node=a100:1
#SBATCH -o sessions/errors/SOM-1.%j.out
#SBATCH -e sessions/errors/SOM-1.%j.err
#SBATCH --mail-type=ALL
#SBATCH --mail-user=melmisti@asu.edu
#SBATCH --export=NONE

set -euo pipefail

set +u
source ~/miniconda3/etc/profile.d/conda.sh
conda activate robot_cleaning
set -u


# --- Run your program ---
python SOM_Commented.py