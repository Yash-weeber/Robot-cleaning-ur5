#%%
from __future__ import annotations

import argparse
from pathlib import Path

import pandas as pd
import matplotlib.pyplot as plt

import wandb


def _out_of_bounds(traj_df: pd.DataFrame) -> bool:
    return (
        traj_df["x"].lt(-1.0).any()
        or traj_df["x"].gt(1.0).any()
        or traj_df["y"].lt(-0.6).any()
        or traj_df["y"].gt(0.6).any()
    )


def log_cost_history_wandb(cost_history_csv: str | Path, *, run: wandb.sdk.wandb_run.Run, waypoint_thresh: int = 1571):
    df = pd.read_csv(cost_history_csv)
    df["flag_waypoints_lt_thresh"] = df["traj_waypoints"] < waypoint_thresh

    # Log raw data as a W&B table (useful for interactive plots / filtering in UI)
    cost_table = wandb.Table(dataframe=df)
    run.log({"cost_history/table": cost_table})

    # Replicate your matplotlib plot and log it
    mask = df["traj_waypoints"] < waypoint_thresh
    fig = plt.figure(figsize=(10, 6))
    plt.plot(df["iter"], df["total_balls"], marker="o")
    plt.scatter(
        df.loc[mask, "iter"],
        df.loc[mask, "total_balls"],
        marker="o",
        color="red",
        label=f"Waypoints < {waypoint_thresh}",
    )
    plt.title("Cost History Over Iterations")
    plt.xlabel("Iteration")
    plt.ylabel("Cost")
    plt.grid(True)
    plt.legend()

    run.log({"plots/cost_history": wandb.Image(fig)})
    plt.close(fig)


def log_trajectories_wandb(
    trajectory_csv: str | Path,
    cost_csv: str | Path,
    *,
    run: wandb.sdk.wandb_run.Run,
):
    df_traj = pd.read_csv(trajectory_csv)
    df_cost = pd.read_csv(cost_csv)

    # Merge total_balls onto trajectory points (for tables / later querying)
    df_cost_small = df_cost[["iter", "total_balls"]].drop_duplicates()
    df_merged = df_traj.merge(df_cost_small, on="iter", how="left")

    # Log all trajectory points as a table (optional but useful)
    traj_table = wandb.Table(dataframe=df_merged)
    run.log({"trajectories/table": traj_table})

    # Replicate per-iteration PNG generation, but log to W&B instead
    for it in sorted(df_traj["iter"].unique()):
        traj_data = df_traj[df_traj["iter"] == it]
        tb_series = df_cost.loc[df_cost["iter"] == it, "total_balls"]
        total_balls = tb_series.iloc[0] if not tb_series.empty else None

        oob = _out_of_bounds(traj_data)
        color = "red" if oob else "blue"

        fig = plt.figure(figsize=(10, 6))
        plt.plot(traj_data["x"], traj_data["y"], color=color)
        plt.title(f"Trajectory Iter {it} (total_balls={total_balls})")
        plt.xlabel("X Position")
        plt.ylabel("Y Position")
        plt.xlim(-1.05, 1.05)
        plt.ylim(-0.65, 0.65)
        plt.grid(True)

        run.log(
            {
                "plots/trajectory": wandb.Image(fig, caption=f"iter={it}, total_balls={total_balls}, oob={oob}"),
                "metrics/iter": int(it),
                "metrics/total_balls": float(total_balls) if total_balls is not None else None,
                "metrics/out_of_bounds": bool(oob),
            }
        )
        plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cost_csv", required=True, help="Path to llm_iteration_log.csv")
    ap.add_argument("--traj_csv", required=True, help="Path to trajectory_feedback.csv")
    ap.add_argument("--project", default="Robot-cleaning-ur5")
    ap.add_argument("--run_name", default=None)
    ap.add_argument("--waypoint_thresh", type=int, default=1571)
    args = ap.parse_args()

    run = wandb.init(
        project=args.project,
        name=args.run_name,
        config={
            "cost_csv": str(args.cost_csv),
            "traj_csv": str(args.traj_csv),
            "waypoint_thresh": args.waypoint_thresh,
        },
    )

    log_cost_history_wandb(args.cost_csv, run=run, waypoint_thresh=args.waypoint_thresh)
    log_trajectories_wandb(args.traj_csv, args.cost_csv, run=run)

    run.finish()


if __name__ == "__main__":
    main()