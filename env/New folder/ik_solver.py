# env/ik_solver.py
import numpy as np
import mujoco
import math
from .robot_logic import clamp_limits

def interpolate_path(p0, p1, max_step=0.03):
    p0 = np.asarray(p0, float)
    p1 = np.asarray(p1, float)
    dist = np.linalg.norm(p1 - p0)
    if dist <= max_step:
        return [p1]
    if math.isnan(dist) or math.isinf(dist):
        integer_value = int(0.0)
    n = int(np.ceil(dist / max_step))
    alphas = np.linspace(0.0, 1.0, n + 1)[1:]
    return [p0 * (1 - a) + p1 * a for a in alphas]

def enhanced_ik_solver(model, data, site_id, goal_pos_3d, joint_names,
                       step_clip=0.2, max_wp_step=0.03, max_iters_per_wp=300,
                       lam_init=0.1, lam_inc=2.0, lam_dec=0.85,
                       tol=1e-3, print_every=60):
    dof_cols = []
    for jn in joint_names:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        dof_cols.append(model.jnt_dofadr[jid])
    dof_cols = np.asarray(dof_cols, int)

    start = np.array(data.site_xpos[site_id])
    waypoints = interpolate_path(start, goal_pos_3d, max_step=max_wp_step)

    for wpi, wp in enumerate(waypoints, 1):
        lam = lam_init
        mujoco.mj_forward(model, data)
        prev_err = np.linalg.norm(wp - np.array(data.site_xpos[site_id]))
        stalled = 0

        for it in range(1, max_iters_per_wp + 1):
            mujoco.mj_forward(model, data)
            Jp = np.zeros((3, model.nv))
            Jr = np.zeros((3, model.nv))
            mujoco.mj_jacSite(model, data, Jp, Jr, site_id)
            J = Jp[:, dof_cols]
            e = wp - np.array(data.site_xpos[site_id])

            A = J.T @ J + lam * np.eye(J.shape[1])
            b = J.T @ e

            try:
                dq = np.linalg.solve(A, b)
            except np.linalg.LinAlgError:
                dq = np.linalg.pinv(A) @ b

            nq = np.linalg.norm(dq)
            if nq > step_clip:
                dq *= (step_clip / (nq + 1e-12))

            qpos_before = data.qpos.copy()
            for k, jn in enumerate(joint_names):
                jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
                qadr = model.jnt_qposadr[jid]
                data.qpos[qadr] += dq[k]

            clamp_limits(model, data.qpos, joint_names)
            mujoco.mj_forward(model, data)
            new_err = np.linalg.norm(wp - np.array(data.site_xpos[site_id]))

            if new_err < prev_err - 1e-6:
                prev_err = new_err
                lam = max(1e-6, lam * lam_dec)
                stalled = 0
            else:
                data.qpos[:] = qpos_before
                mujoco.mj_forward(model, data)
                lam *= lam_inc
                stalled += 1

            if prev_err < tol or stalled >= 30:
                break

        if prev_err >= tol:
            print(f" Waypoint {wpi} failed with error {prev_err:.6f} m")
            return False, prev_err

    final_err = np.linalg.norm(goal_pos_3d - np.array(data.site_xpos[site_id]))
    return True, final_err

print("[CONFIRMATION]: IK Solver module initialized.")