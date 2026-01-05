#!/usr/bin/env python3
"""
Print all 4 issues with fixes to terminal
Run: python fix_issues.py
"""


def print_issue_1():
    print("\n" + "=" * 80)
    print("🔴 ISSUE #1: EMPTY FEEDBACK TEXT")
    print("=" * 80)
    print("\nFile: llm_main_runner.py")
    print("Lines: 82-98")
    print("Severity: 🔴 CRITICAL")
    print("Time: 15 minutes")
    print("\n" + "-" * 80)
    print("CURRENT CODE (WRONG) ❌")
    print("-" * 80)
    print("""
# PROMPT BUILDING LOGIC (Restored from enhancedll_Mohamed.py)
feedback_text = ""

# Logic to iterate through executed weights and bounds checks to build the string
# [Simplified for space, but must include iteration info, weights, and f(weights)]

if it >= 0:
    # This section would dynamically build the feedback_text variable
    # by checking if it in traj_feedback_data and calculating bounds failures.
    prompt = llm.render_prompt(it + 1, feedback_text, bounds)

    try:
        response = llm.call_ollama(prompt)
        w_next = parse_ollama_weights(response, n_bfs)
        save_dialog(config['logs']['dialog_dir'], it + 1, prompt, response)
    except Exception as e:
        print(f"LLM Error: {e}. Reusing current weights.")
        w_next = w2.copy()
else:
    # Random exploration during warmup
    np.random.seed(config['llm_settings']['seed_number'] + it)
    w_next = w2 + np.random.randn(2, n_bfs) * config['dmp_params']['random_scale']
    """)

    print("\n" + "-" * 80)
    print("FIXED CODE (CORRECT) ✅ - COPY THIS AND REPLACE LINES 82-98")
    print("-" * 80)
    print("""
# ========== PROMPT BUILDING LOGIC - Build detailed feedback for LLM ==========
feedback_text = ""

if it >= 0:
    # Build feedback from recent iteration history
    for prev_it in range(max(0, it - feedback_window), it):
        if prev_it in iter_log_data:
            iter_info = iter_log_data[prev_it]
            total_balls = iter_info['total_balls']
            traj_waypoints = iter_info['traj_waypoints']

            # Get trajectory data for bounds violation analysis
            if prev_it in traj_feedback_data:
                traj_points = traj_feedback_data[prev_it]

                # Count bounds violations for this iteration
                x_violations = 0
                y_violations = 0
                for p in traj_points:
                    if p['x'] < bounds['xmin'] or p['x'] > bounds['xmax']:
                        x_violations += 1
                    if p['y'] < bounds['ymin'] or p['y'] > bounds['ymax']:
                        y_violations += 1

                total_violations = x_violations + y_violations
                total_points = len(traj_points)

                # Calculate violation rate
                if total_points > 0:
                    violation_rate = (total_violations / total_points) * 100
                else:
                    violation_rate = 0

                # Get trajectory performance analysis
                if prev_it in trajectory_analysis:
                    analysis = trajectory_analysis[prev_it]
                    smoothness = analysis.get('smoothness', 0)
                    coverage_x = analysis.get('x_coverage', 0) * 100
                    coverage_y = analysis.get('y_coverage', 0) * 100
                else:
                    smoothness = 0
                    coverage_x = 0
                    coverage_y = 0

                # Build feedback text for this iteration
                feedback_text += (
                    f"Iteration {prev_it}: "
                    f"Balls_remaining={total_balls}, "
                    f"Trajectory_waypoints={traj_waypoints}, "
                    f"Bounds_violations={violation_rate:.1f}%, "
                    f"X_coverage={coverage_x:.1f}%, "
                    f"Y_coverage={coverage_y:.1f}%, "
                    f"Smoothness={smoothness:.3f}\\n"
                )

    # Fallback: if no history, provide minimal guidance
    if not feedback_text:
        feedback_text = "First iteration or insufficient history. Starting weight optimization."

    # Generate prompt with feedback
    prompt = llm.render_prompt(it + 1, feedback_text, bounds)

    try:
        response = llm.call_ollama(prompt)
        w_next = parse_ollama_weights(response, n_bfs)
        save_dialog(config['logs']['dialog_dir'], it + 1, prompt, response)
    except Exception as e:
        print(f"LLM Error at iteration {it}: {e}. Reusing current weights.")
        w_next = w2.copy()
else:
    # Random exploration during warmup phase (it < 0)
    np.random.seed(config['llm_settings']['seed_number'] + it)
    w_next = w2 + np.random.randn(2, n_bfs) * config['dmp_params']['random_scale']
    """)

    print("\n✅ WHAT TO DO:")
    print("   1. Open llm_main_runner.py")
    print("   2. Go to lines 82-98")
    print("   3. Select all code in that section")
    print("   4. Delete it")
    print("   5. Copy the FIXED CODE above")
    print("   6. Paste it")
    print("   7. Save the file")


def print_issue_3():
    print("\n\n" + "=" * 80)
    print("🟠 ISSUE #3: WEAK ERROR HANDLING")
    print("=" * 80)
    print("\nFile: llm_main_runner.py")
    print("Lines: Around 87-98 (after fixing Issue #1)")
    print("Severity: 🟠 HIGH")
    print("Time: 10 minutes")
    print("\n" + "-" * 80)
    print("CURRENT CODE (WEAK) ❌")
    print("-" * 80)
    print("""
try:
    response = llm.call_ollama(prompt)
    w_next = parse_ollama_weights(response, n_bfs)
    save_dialog(config['logs']['dialog_dir'], it + 1, prompt, response)
except Exception as e:
    print(f"LLM Error: {e}. Reusing current weights.")
    w_next = w2.copy()
    """)

    print("\n" + "-" * 80)
    print("FIXED CODE (ROBUST) ✅ - COPY THIS AND REPLACE THE TRY-EXCEPT BLOCK")
    print("-" * 80)
    print("""
# Ensure dialog directory exists
os.makedirs(config['logs']['dialog_dir'], exist_ok=True)

w_next = w2.copy()  # Default to current weights

try:
    # Call Ollama
    response = llm.call_ollama(prompt)

    # Validate response is not empty
    if response and len(response.strip()) > 0:
        try:
            # Try to parse weights
            w_next = parse_ollama_weights(response, n_bfs)

            # Save successful dialog
            save_dialog(config['logs']['dialog_dir'], it + 1, prompt, response)
            print(f"✓ Iteration {it}: LLM weight update successful")

        except (ValueError, IndexError) as parse_err:
            print(f"⚠ Warning at iteration {it}: Could not parse LLM weights: {parse_err}")
            print(f"  Falling back to previous weights.")
            w_next = w2.copy()
    else:
        print(f"⚠ Warning at iteration {it}: Received empty response from LLM")
        w_next = w2.copy()

except RuntimeError as llm_err:
    print(f"❌ LLM Error at iteration {it}: {llm_err}")
    print(f"  Falling back to previous weights.")
    w_next = w2.copy()

except Exception as unexpected_err:
    print(f"❌ Unexpected error at iteration {it}: {unexpected_err}")
    w_next = w2.copy()
    """)

    print("\n✅ WHAT TO DO:")
    print("   1. Open llm_main_runner.py")
    print("   2. Find the try-except block for LLM calls")
    print("   3. Delete the old try-except block")
    print("   4. Copy the FIXED CODE above")
    print("   5. Paste it")
    print("   6. Save the file")


def print_issue_4():
    print("\n\n" + "=" * 80)
    print("🟠 ISSUE #4: MISSING SAFETY CHECKS")
    print("=" * 80)
    print("\nFile: llm_analysis.py")
    print("Function: analyze_trajectory_performance()")
    print("Severity: 🟠 HIGH")
    print("Time: 10 minutes")
    print("\n" + "-" * 80)
    print("FIXED CODE (SAFE) ✅ - REPLACE THE ENTIRE analyze_trajectory_performance() FUNCTION")
    print("-" * 80)
    print("""
def analyze_trajectory_performance(trajectory_data, bounds):
    \"\"\"Analyze trajectory quality: coverage, bounds compliance, and smoothness.\"\"\"
    if not trajectory_data:
        return {}

    analysis = {}
    for iter_num, traj_points in trajectory_data.items():
        # Safety check: need at least 2 points
        if not traj_points or len(traj_points) < 2:
            continue

        xs = [p["x"] for p in traj_points]
        ys = [p["y"] for p in traj_points]

        # Bounds compliance
        x_in_bounds = all(bounds["xmin"] <= x <= bounds["xmax"] for x in xs)
        y_in_bounds = all(bounds["ymin"] <= y <= bounds["ymax"] for y in ys)

        # Coverage metrics - with safety checks
        x_range = max(xs) - min(xs)
        y_range = max(ys) - min(ys)
        bounds_width = bounds["xmax"] - bounds["xmin"]
        bounds_height = bounds["ymax"] - bounds["ymin"]

        # Avoid division by zero
        if bounds_width > 0:
            x_range_covered = x_range / bounds_width
        else:
            x_range_covered = 0

        if bounds_height > 0:
            y_range_covered = y_range / bounds_height
        else:
            y_range_covered = 0

        # Smoothness (path length vs direct distance)
        path_length = sum(np.sqrt((xs[i + 1] - xs[i]) ** 2 + (ys[i + 1] - ys[i]) ** 2)
                         for i in range(len(xs) - 1)) if len(xs) > 1 else 0

        direct_distance = np.sqrt((xs[-1] - xs[0]) ** 2 + (ys[-1] - ys[0]) ** 2) if len(xs) > 1 else 0

        # Avoid division by zero
        if path_length > 0:
            smoothness = direct_distance / path_length
        else:
            smoothness = 0

        analysis[iter_num] = {
            "bounds_compliant": x_in_bounds and y_in_bounds,
            "x_coverage": x_range_covered,
            "y_coverage": y_range_covered,
            "smoothness": smoothness,
            "path_length": path_length,
            "waypoint_count": len(traj_points)
        }

    return analysis
    """)

    print("\n✅ WHAT TO DO:")
    print("   1. Open llm_analysis.py")
    print("   2. Find the analyze_trajectory_performance() function")
    print("   3. Delete the entire old function")
    print("   4. Copy the FIXED CODE above")
    print("   5. Paste it")
    print("   6. Save the file")


def print_issue_5():
    print("\n\n" + "=" * 80)
    print("🟠 ISSUE #5: CONFIG PATH CONFLICTS")
    print("=" * 80)
    print("\nFile: llm_main_runner.py")
    print("Multiple locations")
    print("Severity: 🟠 MEDIUM")
    print("Time: 5 minutes")
    print("\n" + "-" * 80)
    print("WHAT TO FIX")
    print("-" * 80)
    print("""
SEARCH FOR (old paths - don't use):
    config['move_csv_path']
    config['simulation']['move_csv_path']

REPLACE WITH (new paths - use these):
    config['logs']['root']
    config['logs']['move_csv']
    config['logs']['weights_txt']
    config['logs']['iter_log_csv']
    config['logs']['weight_history_csv']
    config['logs']['dmp_trajectory_csv']
    config['logs']['dialog_dir']

VERIFY THESE LINES ARE CORRECT:

Line ~67:
    weights_csv = os.path.join(config['logs']['root'], "weights.csv")  ✅

Line ~80:
    weights_csv = os.path.join(config['logs']['root'], "weights.csv")  ✅

Line ~91:
    save_trajectory_data(it, dmp_task_trajectory, config['logs']['dmp_trajectory_csv'])  ✅

Line ~96:
    log_iteration_data(it, grid, total_balls, len(joint_traj), config['logs']['iter_log_csv'])  ✅
    """)

    print("\n✅ WHAT TO DO:")
    print("   1. Open llm_main_runner.py")
    print("   2. Use Ctrl+F to search for 'move_csv_path'")
    print("   3. If you find any, replace with config['logs'][...] paths")
    print("   4. Verify lines ~67, ~80, ~91, ~96 use NEW paths")
    print("   5. Save the file")


def print_summary():
    print("\n\n" + "=" * 80)
    print("📋 SUMMARY - 4 ISSUES TO FIX")
    print("=" * 80)
    print("""
Issue #1: Empty feedback_text
    File: llm_main_runner.py
    Lines: 82-98
    Time: 15 minutes
    ✅ Code provided above - copy and paste

Issue #3: Weak error handling
    File: llm_main_runner.py
    Lines: ~87-98 (after Issue #1 fix)
    Time: 10 minutes
    ✅ Code provided above - copy and paste

Issue #4: Missing safety checks
    File: llm_analysis.py
    Function: analyze_trajectory_performance()
    Time: 10 minutes
    ✅ Code provided above - copy and paste

Issue #5: Config path conflicts
    File: llm_main_runner.py
    Multiple lines
    Time: 5 minutes
    ✅ Instructions provided above

TOTAL TIME: 40 minutes to fix, 20-30 minutes to test = 1.5 hours

NEXT STEPS:
1. Fix Issue #1
2. Fix Issue #3
3. Fix Issue #4
4. Fix Issue #5
5. Run: python main_llm.py
6. Verify it works
    """)


def print_instructions():
    print("\n" + "=" * 80)
    print("🚀 HOW TO USE THIS")
    print("=" * 80)
    print("""
1. Read the issue you want to fix
2. Find the "FIXED CODE" section
3. Copy the entire code block
4. Open your file in editor
5. Find the old code
6. Delete the old code
7. Paste the new code
8. Make sure indentation looks correct
9. Save the file
10. Repeat for next issue

IMPORTANT: Copy the code exactly - indentation matters!
    """)


if __name__ == "__main__":
    print("\n")
    print("#" * 80)
    print("# 🎯 FIXING YOUR 4 ISSUES - ALL CODE PROVIDED BELOW")
    print("#" * 80)

    print_instructions()
    print_issue_1()
    print_issue_3()
    print_issue_4()
    print_issue_5()
    print_summary()

    print("\n\n" + "=" * 80)
    print("✅ ALL INFORMATION PROVIDED ABOVE")
    print("=" * 80)
    print("\nNow:")
    print("1. Copy the code from above")
    print("2. Make the fixes in your files")
    print("3. Run: python main_llm.py")
    print("4. Test that it works")
    print("\nGood luck! 🚀\n")