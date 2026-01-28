import numpy as np
import matplotlib.pyplot as plt

def circle_trajectory(center, radius, num_points=100, plot=True, color='b', linestyle='-'):
    """
    Generate and optionally plot a circular trajectory.
    Returns: x, y arrays of trajectory points (shape: [num_points])
    """
    theta = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
    x = center[0] + radius * np.cos(theta)
    y = center[1] + radius * np.sin(theta)
    if plot:
        plt.plot(x, y, color=color, linestyle=linestyle)
        # plt.axis('equal')
        plt.grid(True)
    return x, y

def elipsoid_trajectory(center, axes_lengths, angle=0, num_points=100, plot=True, color='g', linestyle='-'):
    """
    Generate and optionally plot an ellipsoid trajectory.
    Returns: x, y arrays of trajectory points (shape: [num_points])
    """
    t = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
    x = axes_lengths[0] * np.cos(t)
    y = axes_lengths[1] * np.sin(t)
    R = np.array([[np.cos(angle), -np.sin(angle)],
                  [np.sin(angle),  np.cos(angle)]])
    ellipse = R @ np.vstack((x, y))
    X = center[0] + ellipse[0, :]
    Y = center[1] + ellipse[1, :]
    if plot:
        plt.plot(X, Y, color=color, linestyle=linestyle)
        # plt.axis('equal')
        plt.grid(True)
    return X, Y

def square_trajectory(center, side_length, num_points=100, plot=True, color='m', linestyle=':'):
    """
    Generate and optionally plot a square trajectory.
    Returns: x, y arrays of trajectory points (shape: [num_points])
    """
    half_side = side_length / 2
    corners = np.array([
        [center[0] - half_side, center[1] - half_side],
        [center[0] + half_side, center[1] - half_side],
        [center[0] + half_side, center[1] + half_side],
        [center[0] - half_side, center[1] + half_side],
        [center[0] - half_side, center[1] - half_side]
    ])
    points_per_edge = max(1, num_points // 4)
    x_traj, y_traj = [], []
    for i in range(4):
        start = corners[i]
        end = corners[i+1]
        xs = np.linspace(start[0], end[0], points_per_edge, endpoint=False)
        ys = np.linspace(start[1], end[1], points_per_edge, endpoint=False)
        x_traj.extend(xs)
        y_traj.extend(ys)
    x_traj.append(corners[0][0])
    y_traj.append(corners[0][1])
    if plot:
        plt.plot(x_traj, y_traj, color=color, linestyle=linestyle)
        # plt.axis('equal')
        plt.grid(True)
    return np.array(x_traj), np.array(y_traj)

def rectangle_trajectory(center, width, height, num_points=100, plot=True, color='y', linestyle='-.'):
    """
    Generate and optionally plot a rectangle trajectory.
    Returns: x, y arrays of trajectory points (shape: [num_points])
    """
    half_w = width / 2
    half_h = height / 2
    corners = np.array([
        [center[0] + half_w, center[1] + half_h],   # top right
        [center[0] + half_w, center[1] - half_h],   # bottom right
        [center[0] - half_w, center[1] - half_h],   # bottom left
        [center[0] - half_w, center[1] + half_h],   # top left
        [center[0] + half_w, center[1] + half_h]    # close loop to top right
    ])
    points_per_edge = max(1, num_points // 4)
    x_traj, y_traj = [], []
    for i in range(4):
        start = corners[i]
        end = corners[i+1]
        xs = np.linspace(start[0], end[0], points_per_edge, endpoint=False)
        ys = np.linspace(start[1], end[1], points_per_edge, endpoint=False)
        x_traj.extend(xs)
        y_traj.extend(ys)
    x_traj.append(corners[0][0])
    y_traj.append(corners[0][1])
    if plot:
        plt.plot(x_traj, y_traj, color=color, linestyle=linestyle)
        # plt.axis('equal')
        plt.grid(True)
    return np.array(x_traj), np.array(y_traj)

def triangle_trajectory(center, side_length, num_points=100, plot=True, color='c', linestyle='--'):
    """
    Generate and optionally plot a triangle trajectory.
    Returns: x, y arrays of trajectory points (shape: [num_points])
    """
    height = (np.sqrt(3) / 2) * side_length
    corners = np.array([
        [center[0] - side_length / 2, center[1] - height / 3],
        [center[0] + side_length / 2, center[1] - height / 3],
        [center[0], center[1] + 2 * height / 3],
        [center[0] - side_length / 2, center[1] - height / 3]
    ])
    points_per_edge = max(1, num_points // 3)
    x_traj, y_traj = [], []
    for i in range(3):
        start = corners[i]
        end = corners[i+1]
        xs = np.linspace(start[0], end[0], points_per_edge, endpoint=False)
        ys = np.linspace(start[1], end[1], points_per_edge, endpoint=False)
        x_traj.extend(xs)
        y_traj.extend(ys)
    x_traj.append(corners[0][0])
    y_traj.append(corners[0][1])
    if plot:
        plt.plot(x_traj, y_traj, color=color, linestyle=linestyle)
        # plt.axis('equal')
        plt.grid(True)
    return np.array(x_traj), np.array(y_traj)

def infinity_trajectory(center, size=(1.0, 1.0), num_points=200, plot=True, color='orange', linestyle='-'):
    """
    Generate and optionally plot an infinity (figure-eight) trajectory.
    Returns: x, y arrays of trajectory points (shape: [num_points])
    The parametric equation used is:
        x = a * sin(t)
        y = a * sin(t) * cos(t)
    """
    t = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
    a = size[0] / 2.0  # scale to match other shapes
    b = size[1] / 2.0
    x = center[0] + a * np.sin(t)
    y = center[1] + b * np.sin(t) * np.cos(t)
    if plot:
        plt.plot(x, y, color=color, linestyle=linestyle)
        # plt.axis('equal')
        plt.grid(True)
    return x, y