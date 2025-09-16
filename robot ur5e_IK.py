# # interactive_dust_simulation.py
# # MuJoCo 3.x interactive teleop + per-frame IK for UR5e end-effector (mop/gripper)
# # - Keys move a world-frame target; IK solves UR5e joint angles each frame.
# # - Uses mujoco-python-viewer; no viewer.is_running() used.
# #
# # Controls:
# #   W / S : +Y / -Y
# #   A / D : -X / +X
# #   R / F : +Z / -Z
# #   Z / X : decrease / increase nudge step
# #   C     : toggle actuator hold (if <joint>_act exist)
# #   H     : print help
# #   ESC   : quit

# import time
# import numpy as np
# import mujoco
# import mujoco_viewer
# from pynput import keyboard
# from dataclasses import dataclass

# # ---------- CONFIG ----------
# XML_PATH    = "ur5e_with_mop_and_dust_fixed.xml"   # update path if needed
# SITE_NAME   = "ee_site"                            # end-effector site (tip of mop/gripper)
# UR5E_JOINTS = ["shoulder_pan","shoulder_lift","elbow","wrist_1","wrist_2","wrist_3"]

# # IK hyperparameters (Levenberg–Marquardt)
# STEP_GAIN    = 0.55     # scale for joint update (0.3..0.8 typical)
# INIT_LAMBDA  = 0.12     # initial damping
# LAM_INC      = 2.5
# LAM_DEC      = 0.8
# POS_TOL      = 1e-3     # meters
# MAX_IK_ITERS = 24       # inner iterations per frame

# # Teleop target nudge (meters)
# NUDGE_DEFAULT = 0.01
# NUDGE_MIN     = 0.001
# NUDGE_MAX     = 0.05
# # ---------------------------

# HELP = """\
# Controls:
#   W / S : +Y / -Y
#   A / D : -X / +X
#   R / F : +Z / -Z
#   Z / X : decrease / increase nudge step
#   C     : toggle actuator hold
#   H     : show help
#   ESC   : quit
# """

# @dataclass
# class TeleopState:
#     running: bool = True
#     nudge: float = NUDGE_DEFAULT
#     hold:  bool  = True
#     keys:  dict  = None

# def get_joint_cols(model, joint_names):
#     cols = []
#     for name in joint_names:
#         jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
#         if jid == -1:
#             raise RuntimeError(f"Joint '{name}' not found.")
#         if model.jnt_type[jid] != mujoco.mjtJoint.mjJNT_HINGE:
#             raise RuntimeError(f"Joint '{name}' must be hinge.")
#         cols.append(model.jnt_dofadr[jid])
#     return np.asarray(cols, dtype=int)

# def clamp_limits(model, qpos, joint_names):
#     for name in joint_names:
#         jid  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
#         qadr = model.jnt_qposadr[jid]
#         if model.jnt_limited[jid]:
#             lo, hi = model.jnt_range[jid]
#             qpos[qadr] = np.clip(qpos[qadr], lo, hi)

# def lm_step(J, e, lam):
#     A = J.T @ J + lam * np.eye(J.shape[1])
#     b = J.T @ e
#     try:
#         return np.linalg.solve(A, b)
#     except np.linalg.LinAlgError:
#         return np.linalg.pinv(A) @ b

# def ik_step_frame(model, data, site_id, joint_names, goal_pos):
#     """A few LM iterations this frame to pull the site toward goal_pos (position-only)."""
#     dof_cols = get_joint_cols(model, joint_names)
#     lam = INIT_LAMBDA

#     for _ in range(MAX_IK_ITERS):
#         mujoco.mj_forward(model, data)

#         # world-frame Jacobians at site
#         Jp = np.zeros((3, model.nv))
#         Jr = np.zeros((3, model.nv))
#         mujoco.mj_jacSite(model, data, Jp, Jr, site_id)

#         J = Jp[:, dof_cols]                                   # (3,6)
#         e = goal_pos - np.asarray(data.site_xpos[site_id])    # (3,)
#         err0 = np.linalg.norm(e)

#         dq = lm_step(J, e, lam)                               # (6,)
#         qpos_prev = data.qpos.copy()

#         # apply joint update
#         for k, name in enumerate(joint_names):
#             jid  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
#             qadr = model.jnt_qposadr[jid]
#             data.qpos[qadr] += STEP_GAIN * dq[k]

#         clamp_limits(model, data.qpos, joint_names)
#         mujoco.mj_forward(model, data)

#         e_new = goal_pos - np.asarray(data.site_xpos[site_id])
#         err1 = np.linalg.norm(e_new)

#         if err1 < err0:
#             lam = max(1e-6, lam * LAM_DEC)
#         else:
#             data.qpos[:] = qpos_prev
#             mujoco.mj_forward(model, data)
#             lam *= LAM_INC

#         if err1 < POS_TOL:
#             break

# def hold_with_actuators(model, data, joint_names, enable=True):
#     """If '<joint>_act' actuators exist, set ctrl=qpos to hold pose."""
#     act_ids = []
#     for name in joint_names:
#         aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name + "_act")
#         if aid == -1:
#             return  # silently skip if any are missing
#         act_ids.append(aid)

#     if not enable:
#         data.ctrl[:] = 0
#         return

#     for k, name in enumerate(joint_names):
#         jid  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
#         qadr = model.jnt_qposadr[jid]
#         data.ctrl[act_ids[k]] = float(data.qpos[qadr])

# def main():
#     print(HELP)

#     # Load model & data
#     model = mujoco.MjModel.from_xml_path(XML_PATH)
#     data  = mujoco.MjData(model)
#     mujoco.mj_forward(model, data)

#     site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, SITE_NAME)
#     if site_id == -1:
#         raise RuntimeError(f"Site '{SITE_NAME}' not found in model.")

#     # Teleop state and target
#     tele = TeleopState(keys={"w":False,"s":False,"a":False,"d":False,"r":False,"f":False})
#     target = np.array(data.site_xpos[site_id], dtype=float)

#     # Keyboard listeners
#     def on_press(key):
#         try:
#             ch = key.char.lower()
#         except Exception:
#             ch = None
#         if ch in tele.keys:
#             tele.keys[ch] = True
#         elif ch == 'z':
#             tele.nudge = max(NUDGE_MIN, tele.nudge*0.5)
#             print(f"[nudge] {tele.nudge:.4f} m")
#         elif ch == 'x':
#             tele.nudge = min(NUDGE_MAX, tele.nudge*2.0)
#             print(f"[nudge] {tele.nudge:.4f} m")
#         elif ch == 'c':
#             tele.hold = not tele.hold
#             print(f"[hold] {tele.hold}")
#         elif ch == 'h':
#             print(HELP)

#     def on_release(key):
#         if key == keyboard.Key.esc:
#             tele.running = False
#             return False
#         try:
#             ch = key.char.lower()
#         except Exception:
#             ch = None
#         if ch in tele.keys:
#             tele.keys[ch] = False

#     listener = keyboard.Listener(on_press=on_press, on_release=on_release)
#     listener.start()

#     # Viewer (no is_running used)
#     viewer = mujoco_viewer.MujocoViewer(model, data, hide_menus=False)

#     last_print = 0.0
#     try:
#         while tele.running:
#             moved = False
#             if tele.keys["w"]: target[1] += tele.nudge; moved = True
#             if tele.keys["s"]: target[1] -= tele.nudge; moved = True
#             if tele.keys["a"]: target[0] -= tele.nudge; moved = True
#             if tele.keys["d"]: target[0] += tele.nudge; moved = True
#             if tele.keys["r"]: target[2] += tele.nudge; moved = True
#             if tele.keys["f"]: target[2] -= tele.nudge; moved = True

#             # IK toward target
#             ik_step_frame(model, data, site_id, UR5E_JOINTS, target)

#             # Optional: hold pose with actuators
#             hold_with_actuators(model, data, UR5E_JOINTS, enable=tele.hold)

#             # Render one frame
#             mujoco.mj_forward(model, data)
#             viewer.render()
#             time.sleep(0.01)

#             # occasional console status
#             now = time.time()
#             if moved and (now - last_print) > 0.25:
#                 ee = data.site_xpos[site_id]
#                 print(f"[target] {target[0]:+.3f}, {target[1]:+.3f}, {target[2]:+.3f} m  "
#                       f"|  ee=({ee[0]:+.3f}, {ee[1]:+.3f}, {ee[2]:+.3f}) m")
#                 last_print = now
#     finally:
#         viewer.close()
#         listener.stop()

# if __name__ == "__main__":
#     main()



# ur5e_ik_repl_autoviewer.py
# UR5e IK REPL (position-only LM) + auto viewer backend (mujoco.viewer or mujoco-python-viewer)

import time
import numpy as np
import mujoco

# ---------------------------------------------------------
# Viewer adapter: tries mujoco.viewer first, falls back to mujoco-python-viewer
# Usage:
#   vw = ViewerAdapter(model, data, title="UR5e IK")
#   for ...:
#       mujoco.mj_forward(model, data)
#       vw.draw()
#   vw.close()
# ---------------------------------------------------------
class ViewerAdapter:
    def __init__(self, model, data, title="MuJoCo"):
        self.model = model
        self.data = data
        self.backend = None
        self.viewer = None

        # Try DeepMind's built-in viewer (MuJoCo >= 3.1)
        try:
            import mujoco.viewer as mview
            self.backend = "dm"
            # context manager interface
            self.viewer = mview.launch_passive(model, data)
            # The CM returns a viewer; we’ll keep it, but we also need the context manager for clean close.
            # Wrap it so we can enter/exit when closing.
            self._dm_context_mgr = self.viewer  # save for closing
            self._dm_entered = True
            print("[Viewer] Using mujoco.viewer (DeepMind).")
            return
        except Exception as e:
            # print(f"[Viewer] mujoco.viewer failed: {e}")
            pass

        # Try community viewer
        try:
            import mujoco_viewer
            self.backend = "community"
            self.viewer = mujoco_viewer.MujocoViewer(model, data, hide_menus=False)
            print("[Viewer] Using mujoco-python-viewer.")
            return
        except Exception as e:
            # print(f"[Viewer] mujoco-python-viewer failed: {e}")
            pass

        print("[Viewer] No viewer available. Running headless.")
        self.backend = "none"

    def is_running(self):
        if self.backend == "dm":
            # built-in viewer has .is_running()
            return self.viewer.is_running()
        elif self.backend == "community":
            # community viewer stops when .closed becomes True
            return not self.viewer.closed
        else:
            # headless mode: pretend window is "running" until caller decides to stop
            return False

    def draw(self):
        if self.backend == "dm":
            # keep FK updated outside; just sync frames
            self.viewer.sync()
        elif self.backend == "community":
            self.viewer.render()
        else:
            # headless: nothing to draw
            pass

    def close(self):
        if self.backend == "dm":
            # close the context manager cleanly
            try:
                # viewer from launch_passive supports context-manager protocol;
                # if we are here outside of a 'with', ensure we close it.
                self._dm_context_mgr.close()
            except Exception:
                pass
        elif self.backend == "community":
            try:
                self.viewer.close()
            except Exception:
                pass
        # no-op for headless


# ---------------- IK config ----------------
XML_PATH    = "ur5e_with_mop_and_dust_fixed.xml"
SITE_NAME   = "ee_site"  # change if your tip site is different (e.g., "mop_tip")
UR5E_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]

STEP_SIZE    = 0.55   # scale on dq (0.3..0.8 typical)
INIT_LAMBDA  = 0.15   # LM damping (lambda)
TOL          = 1e-3   # meters; stop when |pos error| < TOL
MAX_ITERS    = 600
LAM_INC      = 2.5
LAM_DEC      = 0.8
PRINT_EVERY  = 50
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

def _lm_step(J, e, lam):
    # dq = (J^T J + lam I)^-1 J^T e
    A = J.T @ J + lam * np.eye(J.shape[1])
    b = J.T @ e
    try:
        return np.linalg.solve(A, b)
    except np.linalg.LinAlgError:
        return np.linalg.pinv(A) @ b

def solve_ik_to_site(model, data, site_id, target_pos_world):
    """Adaptive LM (position-only) to move SITE to target_pos_world."""
    dof_cols = _joint_cols(model, UR5E_JOINTS)
    lam = INIT_LAMBDA

    mujoco.mj_forward(model, data)
    prev_err = np.linalg.norm(target_pos_world - np.array(data.site_xpos[site_id]))

    for it in range(1, MAX_ITERS + 1):
        mujoco.mj_forward(model, data)

        # world-frame Jacobian at the site
        Jp = np.zeros((3, model.nv))
        Jr = np.zeros((3, model.nv))
        mujoco.mj_jacSite(model, data, Jp, Jr, site_id)

        J = Jp[:, dof_cols]  # (3,6)
        e = target_pos_world - np.array(data.site_xpos[site_id])  # (3,)

        dq = _lm_step(J, e, lam)  # (6,)

        # tentative step
        qpos_before = data.qpos.copy()
        for k, jn in enumerate(UR5E_JOINTS):
            jid  = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jn)
            qadr = model.jnt_qposadr[jid]
            data.qpos[qadr] += STEP_SIZE * dq[k]

        _clamp_limits(model, data.qpos, UR5E_JOINTS)
        mujoco.mj_forward(model, data)

        new_err = np.linalg.norm(target_pos_world - np.array(data.site_xpos[site_id]))
        if new_err < prev_err:
            prev_err = new_err
            lam = max(1e-6, lam * LAM_DEC)
        else:
            # rollback and increase damping
            data.qpos[:] = qpos_before
            mujoco.mj_forward(model, data)
            lam *= LAM_INC

        if it % PRINT_EVERY == 0:
            print(f"  [iters {it:3d}] |e_pos|={prev_err:.6f} m, lambda={lam:.4f}")

        if prev_err < TOL:
            return True, prev_err

    return False, prev_err

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

        ok, err = solve_ik_to_site(model, data, site_id, goal)
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
