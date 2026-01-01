import numpy as np
import mujoco


def count_balls_in_grid(model, data, x_min, x_max, y_min, y_max, num_x_segments, num_y_segments, num_balls):
    x_edges = np.linspace(x_min, x_max, num_x_segments + 1)
    y_edges = np.linspace(y_min, y_max, num_y_segments + 1)
    grid_counts = np.zeros((num_x_segments, num_y_segments), dtype=int)

    ball_names = [f"ball_{i + 1}" for i in range(num_balls)]
    ball_positions = []
    for name in ball_names:
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body_id != -1:
            pos = data.xpos[body_id][:2]  # x, y position
            # Only include balls inside the grid bounds
            if (x_min <= pos[0] <= x_max) and (y_min <= pos[1] <= y_max):
                ball_positions.append(pos)
    ball_positions = np.array(ball_positions)

    for pos in ball_positions:
        x, y = pos
        # Find which grid cell (i, j) the ball is in
        i = np.searchsorted(x_edges, x, side='right') - 1
        j = np.searchsorted(y_edges, y, side='right') - 1
        # Clamp indices to valid range
        i = min(max(i, 0), num_x_segments - 1)
        j = min(max(j, 0), num_y_segments - 1)
        grid_counts[i, j] += 1

    # reverse columns then transpose to match visual layout
    grid_counts = grid_counts[:, ::-1]

    # Print results
    for i in range(num_x_segments):
        for j in range(num_y_segments):
            print(
                f"Grid cell ({i + 1},{j + 1}) x:[{x_edges[i]:.2f},{x_edges[i + 1]:.2f}] y:[{y_edges[j]:.2f},{y_edges[j + 1]:.2f}]: {grid_counts[i, j]} balls")

    print("total balls counted:", np.sum(grid_counts))
    return grid_counts