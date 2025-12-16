#!/usr/bin/env python3
"""
Simple Parallel Launcher - No changes to testiing_2.py needed!

Just configure your experiments below and run:
    python simple_parallel_launcher.py
"""

import os
import sys
import subprocess
import time
from datetime import datetime
from pathlib import Path
import builtins

builtins.input = lambda *a, **k: "7"
# ============================================
# CONFIGURE YOUR EXPERIMENTS HERE
# ============================================

EXPERIMENTS = [
    {
        "name": "exp1_try2",
        "base_dir": "Y:/models/ur5hanibenpng/final/Robot-cleaning-ur51/Robot-cleaning-ur5/",
        "api_keys": [1, 2, 3],  # Use GOOGLE_API_KEY_1, 2, 3
        "max_iters": 400,
        "iter_sleep": 40,  # Seconds between iterations
    },
    {
        "name": "exp2_try3",
        "base_dir": "Y:/models/ur5hanibenpng/final/Robot-cleaning-ur51/Robot-cleaning-ur5-exp2/",
        "api_keys": [4, 5, 6],  # Use GOOGLE_API_KEY_4, 5, 6
        "max_iters": 400,
        "iter_sleep": 40,
    },
    # Add more experiments as needed
]

# Path to your wrapper script (enhancedll_parallel.py)
WRAPPER_SCRIPT = Path(__file__).parent / "enhancedll_parallel.py"


# ============================================
# Experiment Runner
# ============================================

def run_experiment(exp_config, log_file):
    """
    Start one experiment in a subprocess

    Args:
        exp_config: Dictionary with experiment configuration
        log_file: Path to log file for this experiment's output
    """

    name = exp_config["name"]
    base_dir = exp_config["base_dir"]

    # Build environment variables
    env = os.environ.copy()

    # Set headless mode
    env["HEADLESS"] = "1"
    # Don't set MUJOCO_GL on Windows - we're monkey-patching the viewer anyway
    # The monkey patch prevents any GL context from being created

    # Force UTF-8 encoding for Python subprocess (handles emojis on Windows)
    env["PYTHONIOENCODING"] = "utf-8"

    # Set experiment-specific variables
    env["EXPERIMENT_BASE_DIR"] = base_dir
    env["EXPERIMENT_NAME"] = name
    env["MAX_ITERS"] = str(exp_config["max_iters"])
    env["ITER_SLEEP"] = str(exp_config.get("iter_sleep", 40))

    # Clear all API keys first, then set only the ones for this experiment
    for i in range(1, 20):
        env.pop(f"GOOGLE_API_KEY_{i}", None)

    # Map the specified API key numbers to sequential keys (1, 2, 3...)
    for idx, key_num in enumerate(exp_config["api_keys"], start=1):
        source_key = f"GOOGLE_API_KEY_{key_num}"
        if source_key in os.environ:
            env[f"GOOGLE_API_KEY_{idx}"] = os.environ[source_key]
        else:
            print(f"⚠️  Warning: {source_key} not found in environment!")

    # Verify base directory exists
    if not Path(base_dir).exists():
        raise FileNotFoundError(f"Base directory does not exist: {base_dir}")

    # Create logs directory if needed
    log_dir = Path(base_dir) / "logs"
    log_dir.mkdir(exist_ok=True)

    # Start the subprocess
    print(f"🚀 Starting {name}...")
    print(f"   📁 Directory: {base_dir}")
    print(f"   🔑 API Keys: {exp_config['api_keys']}")
    print(f"   📄 Log file: {log_file}\n")

    with open(log_file, "w", encoding="utf-8") as f:
        proc = subprocess.Popen(
            [sys.executable, str(WRAPPER_SCRIPT)],
            env=env,
            stdout=f,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            encoding='utf-8',
        )

    return proc


# ============================================
# Status Monitor
# ============================================

def monitor_experiments(processes, log_files):
    """
    Monitor running experiments and print status updates

    Args:
        processes: List of (name, subprocess.Popen) tuples
        log_files: Dict mapping experiment names to log file paths
    """

    print("\n" + "=" * 80)
    print("📊 Monitoring Experiments (Ctrl+C to stop all)")
    print("=" * 80 + "\n")

    check_interval = 30  # seconds

    try:
        while processes:
            time.sleep(check_interval)

            print(f"\n{'=' * 80}")
            print(f"📊 Status Update - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
            print(f"{'=' * 80}")

            still_running = []

            for name, proc in processes:
                retcode = proc.poll()

                if retcode is None:
                    # Still running - try to extract iteration from log
                    iteration = extract_current_iteration(log_files[name])
                    print(f"▶️  {name:20s} | Running | Iteration: {iteration}")
                    still_running.append((name, proc))

                elif retcode == 0:
                    print(f"✅ {name:20s} | Completed successfully!")

                else:
                    print(f"❌ {name:20s} | Failed (exit code {retcode})")
                    # Print last few lines of log
                    print(f"   Last log lines:")
                    try:
                        with open(log_files[name], "r") as f:
                            lines = f.readlines()
                            for line in lines[-5:]:
                                print(f"      {line.rstrip()}")
                    except:
                        pass

            processes = still_running
            print("=" * 80)

            if not processes:
                print("\n🎉 All experiments finished!\n")
                break

    except KeyboardInterrupt:
        print("\n\n⚠️  Interrupted! Stopping all experiments...\n")
        for name, proc in processes:
            print(f"   Terminating {name}...")
            proc.terminate()
            time.sleep(1)
            if proc.poll() is None:
                proc.kill()
        print("✅ All experiments stopped.\n")


def extract_current_iteration(log_file):
    """Extract the latest iteration number from log file"""
    try:
        with open(log_file, "r") as f:
            # Read last 50 lines to find iteration
            lines = f.readlines()[-50:]
            for line in reversed(lines):
                if "Iteration" in line and "/" in line:
                    # Extract "Iteration X/Y"
                    import re
                    match = re.search(r'Iteration\s+(\d+)/(\d+)', line)
                    if match:
                        return f"{match.group(1)}/{match.group(2)}"
        return "?"
    except:
        return "?"


# ============================================
# Main
# ============================================

def main():
    print("\n" + "=" * 80)
    print("🚀 Simple Parallel Experiment Launcher")
    print("=" * 80)
    print(f"📝 Configured Experiments: {len(EXPERIMENTS)}")
    print(f"🕐 Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 80 + "\n")

    # Verify wrapper script exists
    if not WRAPPER_SCRIPT.exists():
        print(f"❌ Error: Wrapper script not found: {WRAPPER_SCRIPT}")
        print(f"   Make sure 'headless_wrapper.py' is in the same directory!")
        sys.exit(1)

    # Verify API keys are set
    api_keys_found = [k for k in os.environ if k.startswith("GOOGLE_API_KEY_")]
    print(f"🔑 Found {len(api_keys_found)} API keys in environment:")
    for key in sorted(api_keys_found):
        print(f"   ✅ {key}")
    print()

    if not api_keys_found:
        print("⚠️  WARNING: No GOOGLE_API_KEY_* variables found!")
        print("   Set them like: set GOOGLE_API_KEY_1=your_key_here")
        print()

    # Create logs directory for launcher
    launcher_log_dir = Path("parallel_runs") / datetime.now().strftime("%Y%m%d_%H%M%S")
    launcher_log_dir.mkdir(parents=True, exist_ok=True)

    # Start all experiments
    processes = []
    log_files = {}

    for exp in EXPERIMENTS:
        log_file = launcher_log_dir / f"{exp['name']}.log"
        log_files[exp['name']] = log_file

        try:
            proc = run_experiment(exp, log_file)
            processes.append((exp['name'], proc))
            time.sleep(3)  # Stagger starts to avoid resource conflicts

        except Exception as e:
            print(f"❌ Failed to start {exp['name']}: {e}\n")
            import traceback
            traceback.print_exc()

    if not processes:
        print("❌ No experiments started successfully!")
        sys.exit(1)

    print(f"✅ Started {len(processes)} experiment(s)")
    print(f"📁 Logs saved to: {launcher_log_dir}\n")

    # Monitor experiments
    monitor_experiments(processes, log_files)

    # Final summary
    print("\n" + "=" * 80)
    print("📋 FINAL SUMMARY")
    print("=" * 80)

    for name, log_file in log_files.items():
        if log_file.exists():
            file_size = log_file.stat().st_size / 1024  # KB
            print(f"📄 {name:20s} | Log: {log_file.name} ({file_size:.1f} KB)")

    print(f"\n📁 All logs in: {launcher_log_dir}")
    print("=" * 80 + "\n")


if __name__ == "__main__":
    main()