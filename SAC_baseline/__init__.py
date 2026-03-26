# SAC_baseline
# ============
# Stable-Baselines3 SAC baseline for the UR5e ball-clearing task.
#
# RL formulation
# --------------
#   Action      : flattened DMP weights  w ∈ R^(2 × n_bfs)
#   Observation : ee_site 3-D position (after last trajectory) +
#                 previous weight vector  →  R^(3 + 2·n_bfs)
#   Reward      : fraction of balls swept off the workspace in one sweep
#   Episode     : 1 MDP step  (choose weights → run full trajectory → done)
#
# Entry points
# ------------
#   python -m SAC_baseline.train_sac [--config config/sac_config.yaml]
#   python -m SAC_baseline.eval_sac  --model <checkpoint.zip> [--episodes N]
