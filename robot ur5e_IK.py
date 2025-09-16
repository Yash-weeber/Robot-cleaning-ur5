# ur5e_ik_repl_autoviewer.py
# UR5e IK REPL (position-only LM) + auto viewer backend (mujoco.viewer or mujoco-python-viewer)

import time
import numpy as np
import mujoco

# ---------------------------------------------------------
# Viewer adapter: tries mujoco.viewer first, falls back to mujoco-python-viewer
# ---------------------------------------------------------
class ViewerAdapter:
    def __init__(self, model, data, title="MuJoCo"):
        self.model = model
        self.data = data
        self.backend = None
        self.viewer = None
        self._dm_context_mgr = None

        # Try DeepMind's built-in viewer (MuJoCo >= 3.1)
        try:
            import mujoco.viewer as mview
            self.backend = "dm"
            self.viewer = mview.launch_passive(model, data)  # returns a viewer object
            self._dm_context_mgr = self.viewer
            print("[Viewer] Using mujoco.viewer (DeepMind).")
            return
        except Exception:
            pass

        # Try community viewer
        try:
            import mujoco_viewer
            self.backend = "community"
            self.viewer = mujoco_viewer.MujocoViewer(model, data, hide_menus=False)
            print("[Viewer] Using mujoco-python-viewer.")
            return
        except Exception:
            pass

        print("[Viewer] No viewer available. Running headless.")
        self.backend = "none"

    def is_running(self):
        if self.backend == "dm":
            return self.viewer.is_running()
        elif self.backend == "community":
            return not self.viewer.closed
        else:
            return False

    def draw(self):
        if self.backend == "dm":
            self.viewer.sync()
        elif self.backend == "community":
            self.viewer.render()
        else:
            pass

    def close(self):
        if self.backend == "dm":
            try:
                self._dm_context_mgr.close()
            except Exception:
                pass
        elif self.backend == "community":
            try:
                self.viewer.close()
            except Exception:
                pass
        # headless: no-op


# ---------------- IK config ----------------
XML_PATH    = "ur5e_with_mop_and_dust_fixed.xml"
SITE_NAME   = "ee_site"  # change if your tip site is different (e.g., "mop_tip")
UR5E_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]

INIT_LAMBDA  = 0.15   # LM damping (lambda)
TOL          = 1e-3   # meters; stop when |pos error| < TOL
PRINT_EVERY  = 60
# -------------------------------------------

def _joint_cols(model, joint_names):
    cols = []
    for jn in joint_names:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        if jid == -1:
            raise RuntimeError(f"Joint '{jn}' not found in model.")
        if model.jnt_type[jid] != mujoco.mjtJoint.mjJNT_HINGE:
            raise RuntimeError(f"Joint '{jn}' must be a hinge.")
        cols.append(model.jnt_dofadr[jid])
    return np.asarray(cols, dtype=int)

def _clamp_limits(model, qpos, joint_names):
    for jn in joint_names:
        jid  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        qadr = model.jnt_qposadr[jid]
        if model.jnt_limited[jid]:
            lo, hi = model.jnt_range[jid]
            qpos[qadr] = np.clip(qpos[qadr], lo, hi)

def _interpolate_path(p0, p1, max_step=0.05):
    """Linear path from p0 to p1 with <= max_step meters between waypoints."""
    p0 = np.asarray(p0, float); p1 = np.asarray(p1, float)
    dist = np.linalg.norm(p1 - p0)
    if dist <= max_step:
        return [p1]
    n = int(np.ceil(dist / max_step))
    alphas = np.linspace(0.0, 1.0, n + 1)[1:]  # skip p0
    return [p0*(1-a) + p1*a for a in alphas]

def move_tip_to(model, data, site_id, goal_pos_world,
                joint_names=("shoulder_pan","shoulder_lift","elbow","wrist_1","wrist_2","wrist_3"),
                step_clip=0.25,         # max joint step (rad) per IK iteration
                max_wp_step=0.05,       # waypoint spacing (m)
                max_iters_per_wp=200,   # IK iterations per waypoint
                lam_init=0.15, lam_inc=2.5, lam_dec=0.8,
                tol=1e-3, print_every=60):
    """
    Moves the SITE to goal_pos_world via waypoints + trust-region LM.
    Returns (success, final_err).
    """
    # compute DOF columns for the six hinge joints
    dof_cols = []
    for jn in joint_names:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        if jid == -1: raise RuntimeError(f"Joint '{jn}' not found.")
        if model.jnt_type[jid] != mujoco.mjtJoint.mjJNT_HINGE:
            raise RuntimeError(f"Joint '{jn}' must be a hinge.")
        dof_cols.append(model.jnt_dofadr[jid])
    dof_cols = np.asarray(dof_cols, int)

    def clamp_limits():
        _clamp_limits(model, data.qpos, joint_names)

    mujoco.mj_forward(model, data)
    start = np.array(data.site_xpos[site_id])
    waypoints = _interpolate_path(start, goal_pos_world, max_step=max_wp_step)

    for wpi, wp in enumerate(waypoints, 1):
        lam = lam_init
        mujoco.mj_forward(model, data)
        prev_err = np.linalg.norm(wp - np.array(data.site_xpos[site_id]))
        stalled = 0

        for it in range(1, max_iters_per_wp + 1):
            mujoco.mj_forward(model, data)

            # world-frame site Jacobian
            Jp = np.zeros((3, model.nv)); Jr = np.zeros((3, model.nv))
            mujoco.mj_jacSite(model, data, Jp, Jr, site_id)
            J = Jp[:, dof_cols]                                  # (3,6)
            e = wp - np.array(data.site_xpos[site_id])           # (3,)

            # LM step: dq = (JᵀJ + λI)⁻¹ Jᵀ e
            A = J.T @ J + lam * np.eye(J.shape[1])
            b = J.T @ e
            try:
                dq = np.linalg.solve(A, b)
            except np.linalg.LinAlgError:
                dq = np.linalg.pinv(A) @ b

            # trust-region: clip ‖Δq‖
            nq = np.linalg.norm(dq)
            if nq > step_clip:
                dq *= (step_clip / (nq + 1e-12))

            # tentative update with rollback
            qpos_before = data.qpos.copy()
            for k, jn in enumerate(joint_names):
                jid  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
                qadr = model.jnt_qposadr[jid]
                data.qpos[qadr] += dq[k]

            clamp_limits()
            mujoco.mj_forward(model, data)

            new_err = np.linalg.norm(wp - np.array(data.site_xpos[site_id]))

            if new_err < prev_err - 1e-6:
                prev_err = new_err
                lam = max(1e-6, lam * lam_dec)
                stalled = 0
            else:
                # rollback, increase damping
                data.qpos[:] = qpos_before
                mujoco.mj_forward(model, data)
                lam *= lam_inc
                stalled += 1

            if it % print_every == 0:
                print(f"  [wp {wpi:02d}/{len(waypoints)} | it {it:03d}] "
                      f"|e|={prev_err:.6f} m, λ={lam:.3g}")

            if prev_err < tol:
                break
            if stalled >= 25:  # not improving
                break

        # if this waypoint failed, bail out
        if prev_err >= tol:
            return False, prev_err

    # success on final goal
    final_err = np.linalg.norm(goal_pos_world - np.array(data.site_xpos[site_id]))
    return True, final_err

def _print_joint_solution(model, data):
    print("Joint solution (rad / deg):")
    for jn in UR5E_JOINTS:
        jid  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        qadr = model.jnt_qposadr[jid]
        q = float(data.qpos[qadr])
        print(f"  {jn:12s}: {q:+.6f}   ({np.degrees(q):+7.3f}°)")

def _hold_with_actuators_if_present(model, data):
    # If you have '<joint>_act' position actuators, set ctrl=qpos to "hold" pose.
    act_ids = []
    for jn in UR5E_JOINTS:
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, jn + "_act")
        if aid == -1:
            return
        act_ids.append(aid)
    for k, jn in enumerate(UR5E_JOINTS):
        jid  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
        qadr = model.jnt_qposadr[jid]
        data.ctrl[act_ids[k]] = float(data.qpos[qadr])

def main():
    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data  = mujoco.MjData(model)

    site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, SITE_NAME)
    if site_id == -1:
        raise RuntimeError(f"Site '{SITE_NAME}' not found; update SITE_NAME to your tip site.")
    mujoco.mj_forward(model, data)

    vw = ViewerAdapter(model, data, title="UR5e IK")

    print("\nUR5e IK REPL (xyz in meters, world frame). Example:")
    print("  -0.80 0.72 0.56")
    print("Type 'q' to quit.\n")

    while True:
        try:
            line = input("xyz> ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not line or line.lower() == "q":
            break

        parts = line.replace(",", " ").split()
        if len(parts) != 3:
            print("Please enter exactly 3 numbers: x y z")
            continue

        try:
            goal = np.array(list(map(float, parts)), dtype=float)
        except ValueError:
            print("Could not parse numbers.")
            continue

        start = np.array(data.site_xpos[site_id])
        print(f"Start: {np.round(start,3)}  ->  Goal: {np.round(goal,3)}")

        # Robust IK with waypoints + trust region
        ok, err = move_tip_to(model, data, site_id, goal,
                              step_clip=0.25,
                              max_wp_step=0.05,
                              max_iters_per_wp=200,
                              lam_init=INIT_LAMBDA,
                              tol=TOL,
                              print_every=PRINT_EVERY)
        print(f"Result: success={ok}, |pos_err|={err:.6f} m")
        _print_joint_solution(model, data)
        _hold_with_actuators_if_present(model, data)

        # brief display/settle
        t0 = time.time()
        while time.time() - t0 < 0.5:
            mujoco.mj_forward(model, data)
            vw.draw()
            time.sleep(0.01)

    vw.close()

if __name__ == "__main__":
    main()
