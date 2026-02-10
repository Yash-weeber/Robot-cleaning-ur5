import numpy as np
from config.loader import load_config

config = load_config(config_path="config/config.yaml")

# Rectangle keep-in zone boundaries/parameters
ws_center = config["simulation"]["ws_center"]
ws_width = config["simulation"]["ws_width"]
ws_length = config["simulation"]["ws_length"]

XMIN = ws_center[0] - ws_width / 2.0
XMAX = ws_center[0] + ws_width / 2.0
YMIN = ws_center[1] - ws_length / 2.0
YMAX = ws_center[1] + ws_length / 2.0

# Obstacle points inside the rectangle
INTERNAL_OBSTACLES = np.array([[0.0, 0.5],
                               [0.0, 0.525],
                               [0.0, 0.55],
                               [0.0, 0.575]], dtype=float)


def _project_into_rect(y):
    """Hard projection into keep-in rectangle."""
    y = np.asarray(y, dtype=float).reshape(2,)
    return np.array([np.clip(y[0], XMIN, XMAX), np.clip(y[1], YMIN, YMAX)], dtype=float)


def _keep_in_rect_force(y, *, d0_x=0.14, d0_y=0.06, eta=0.002, k_out=200.0):
    """
    Wall-based keep-in force for an axis-aligned rectangle.
    - Inside but within d0 of a wall: smooth repulsion away from the wall.
    - Outside: strong linear push back inside (k_out).
    """
    y = np.asarray(y, dtype=float).reshape(2,)
    p = np.zeros(2, dtype=float)
    eps = 1e-9

    # X walls
    if y[0] < XMIN:
        p[0] += k_out * (XMIN - y[0])
    else:
        d = y[0] - XMIN
        if d < d0_x:
            dd = d + eps
            p[0] += eta * (1.0 / dd - 1.0 / d0_x) * (1.0 / (dd * dd))

    if y[0] > XMAX:
        p[0] -= k_out * (y[0] - XMAX)
    else:
        d = XMAX - y[0]
        if d < d0_x:
            dd = d + eps
            p[0] -= eta * (1.0 / dd - 1.0 / d0_x) * (1.0 / (dd * dd))

    # Y walls
    if y[1] < YMIN:
        p[1] += k_out * (YMIN - y[1])
    else:
        d = y[1] - YMIN
        if d < d0_y:
            dd = d + eps
            p[1] += eta * (1.0 / dd - 1.0 / d0_y) * (1.0 / (dd * dd))

    if y[1] > YMAX:
        p[1] -= k_out * (y[1] - YMAX)
    else:
        d = YMAX - y[1]
        if d < d0_y:
            dd = d + eps
            p[1] -= eta * (1.0 / dd - 1.0 / d0_y) * (1.0 / (dd * dd))

    return p


def _repulsive_point_obstacles_force(y, obstacles_xy, *, d0=0.20, eta=0.02):
    """
    Distance-based repulsive potential field for point obstacles.
    Only active within radius d0.
    """
    y = np.asarray(y, dtype=float).reshape(2,)
    p = np.zeros(2, dtype=float)
    eps = 1e-9

    if obstacles_xy is None or len(obstacles_xy) == 0:
        return p

    obstacles_xy = np.asarray(obstacles_xy, dtype=float).reshape(-1, 2)

    for o in obstacles_xy:
        r = y - o
        d = np.linalg.norm(r) + eps
        if d >= d0:
            continue

        mag = eta * (1.0 / d - 1.0 / d0) * (1.0 / (d * d))
        p += (r / d) * mag

    return p


def avoid_obstacles(
    y,
    dy,
    goal,
    *,
    # keep-in rectangle params
    rect_d0_x=0.14,
    rect_d0_y=0.06,
    rect_eta=0.2,
    rect_k_out=200.0,
    # internal obstacle params
    obs_d0=0.25,
    obs_eta=5,
    # global clamp
    max_force=50.0,
):
    """
    Combined coupling term:
    - keep-in rectangle (walls)
    - keep-away internal point obstacles 
    """
    y = np.asarray(y, dtype=float).reshape(2,)
    p = np.zeros(2, dtype=float)

    p += _keep_in_rect_force(y, d0_x=rect_d0_x, d0_y=rect_d0_y, eta=rect_eta, k_out=rect_k_out)
    p += _repulsive_point_obstacles_force(y, INTERNAL_OBSTACLES, d0=obs_d0, eta=obs_eta)

    # Clamp for stability
    eps = 1e-9
    n = np.linalg.norm(p)
    if n > max_force:
        p *= (max_force / (n + eps))

    return p