import math
import time
from collections import deque

import numpy as np
import mujoco
from mujoco import mjtObj
import mujoco_viewer
import glfw

# ========= PATH TO YOUR XML =========
XML_PATH = r"F:/Robotics Automation system AI/SEM 3/APPLIED PROJECT/NEW XMLS FILESSSSSSSSSSSSSSSSSSSSSSSSSSSS/Robot-cleaning-ur5-master/ur5e_with_table_merged.xml"

# ========= JOINTS & LIMITS (no 360 on wrist_3) =========
JOINT_ORDER = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
JOINT_LIMITS = {
    "shoulder_pan":  (-4.84, -2.52),
    "shoulder_lift": (-3.14, -0.126),
    "elbow":         (-3.0,  2.4),
    "wrist_1":       (-2.967,  2.967),  # ~±170°
    "wrist_2":       (-2.967,  2.967),
    "wrist_3":       (-2.967,  2.967),  # ~±170°
}

# ========= TABLE AREA (half-sizes in table frame) =========
TABLE_HALF_X = 1.00   # meters
TABLE_HALF_Y = 0.45   # meters

# ========= CONTROLS & GAINS =========
NUDGE = np.deg2rad(2.0)                # manual step per key
MAX_TARGET_RATE = np.deg2rad(90.0)     # rad/s max change to targets

# Contact “press” (positive pushes further into the table; small values!)
PRESS_DEPTH = 0.0015                   # meters (0.5–1.5 mm is good)

# --- Smooth, damped Z-contact control (no stomping) ---
KZ = 6.0                 # Z proportional gain
DZ = 2.0                 # Z damping gain (on vertical velocity)
MAX_Z_RATE = 0.03        # m/s max commanded vertical speed
Z_PLANE_ALPHA = 0.10     # low-pass filter for table height estimate

# --- Lateral cleaning controller (side-to-side) ---
K_XY = 6.0               # XY gain during cleaning
DWELL_DT = 0.10          # seconds per waypoint
PITCH = 0.15             # raster spacing (meters)

# --- IK damping ---
LAMBDA = 1e-3            # small damping for pseudoinverses


# ========= UTILS =========
def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v
def mid(lo, hi): return 0.5*(lo+hi)

def viewer_is_alive(viewer):
    alive_attr = getattr(viewer, "is_alive", None)
    return alive_attr() if callable(alive_attr) else bool(alive_attr)

def build_name_maps(model):
    joint_id, qpos_adr, dof_idx, act_id = {}, {}, {}, {}
    for name in JOINT_ORDER:
        jid = mujoco.mj_name2id(model, mjtObj.mjOBJ_JOINT, name)
        if jid < 0:
            raise RuntimeError(f"Joint '{name}' not found in model.")
        joint_id[name] = jid
        qpos_adr[name] = model.jnt_qposadr[jid]
        dof_idx[name]  = model.jnt_dofadr[jid]  # 1 DoF per revolute joint
        aid = mujoco.mj_name2id(model, mjtObj.mjOBJ_ACTUATOR, f"{name}_act")
        if aid < 0:
            aid = mujoco.mj_name2id(model, mjtObj.mjOBJ_ACTUATOR, name)
            if aid < 0:
                raise RuntimeError(f"Actuator for '{name}' not found.")
        act_id[name] = aid
    return joint_id, qpos_adr, dof_idx, act_id

def set_ctrl(data, act_id, targets):
    for n, v in targets.items():
        data.ctrl[act_id[n]] = float(v)

def raycast_table_z(model, data, x, y, z_above=5.0, bodyexclude=-1):
    """
    Cast a vertical ray downward from (x,y,z_above).
    Returns (z_hit, geomid) or (None, -1) if nothing hit.
    Matches mujoco.mj_ray signature that requires bodyexclude and a writable geomid array.
    """
    pnt = np.array([x, y, z_above], dtype=np.float64)
    vec = np.array([0.0, 0.0, -1.0], dtype=np.float64)
    geomgroup = np.array([1, 1, 1, 1, 1, 1], dtype=np.uint8)  # all 6 groups
    flg_static = 1
    geomid_out = np.array([-1], dtype=np.int32)

    dist = mujoco.mj_ray(model, data, pnt, vec, geomgroup, flg_static, int(bodyexclude), geomid_out)
    hit_id = int(geomid_out[0])
    if hit_id >= 0 and np.isfinite(dist):
        return z_above - dist, hit_id
    return None, -1

def table_axes_world(data, bodyid_table):
    """Return table origin and its x,y axes (world)."""
    o = data.xpos[bodyid_table].copy()
    R = data.xmat[bodyid_table].reshape(3, 3).copy()
    ex = R[:, 0]
    ey = R[:, 1]
    return o, ex, ey

def raster_path_points(center, ex, ey, half_x, half_y, pitch=0.15):
    """Yield world (x,y) waypoints that raster the rectangle on the table."""
    xs = np.linspace(-half_x, half_x, max(2, int(2*half_x/pitch)+1))
    ys = np.linspace(-half_y, half_y, max(2, int(2*half_y/pitch)+1))
    flip = False
    for y in ys:
        seq = xs[::-1] if flip else xs
        for x in seq:
            p = center + ex * x + ey * y
            yield p[0], p[1]
        flip = not flip


# ========= MAIN =========
def main():
    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data = mujoco.MjData(model)
    dt = model.opt.timestep

    joint_id, qpos_adr, dof_idx, act_id = build_name_maps(model)

    # Bodies
    bodyid_mop   = mujoco.mj_name2id(model, mjtObj.mjOBJ_BODY, "mop_head")
    bodyid_table = mujoco.mj_name2id(model, mjtObj.mjOBJ_BODY, "table")
    if bodyid_mop < 0:   raise RuntimeError("Body 'mop_head' not found.")
    if bodyid_table < 0: raise RuntimeError("Body 'table' not found.")

    # DOF column indices (nv) for our 6 joints in JOINT_ORDER
    dof_cols = [dof_idx[j] for j in JOINT_ORDER]

    # Targets and filtered targets
    targets = {j: mid(*JOINT_LIMITS[j]) for j in JOINT_ORDER}
    filt_targets = dict(targets)

    # Modes and toggles
    MODE_SIM, MODE_ACTIVE = 0, 1
    mode = MODE_ACTIVE            # start manual so you can position, then press B to place
    contact_lock = False          # press B or C to engage
    z_lock_only = False           # when True: hold Z contact only, ignore XY tracking
    cleaning = False              # G toggles raster cleaning
    placed = False                # becomes True after first "B" place step

    # Cleaning path state
    dwell_dt = DWELL_DT
    pitch = PITCH
    dwell_time = 0.0

    # Press depth (positive pushes more)
    press_depth = PRESS_DEPTH

    # XY anchor (used when z_lock_only == True)
    xy_anchor = None  # (x, y) world coords

    # Filtered tabletop Z estimate and previous mop Z (for velocity)
    z_plane_est = None
    mop_z_prev = None

    # Viewer + keyboard
    viewer = mujoco_viewer.MujocoViewer(model, data)
    if hasattr(viewer, "_render_every_frame"):
        viewer._render_every_frame = True

    # Prepare key event queue
    q = deque()
    def key_cb(window, key, scancode, action, mods):
        if action in (glfw.PRESS, glfw.REPEAT):
            q.append(key)

    win = getattr(viewer, "window", None) or getattr(viewer, "_window", None)
    if win is None:
        ctx = getattr(viewer, "context", None)
        win = getattr(ctx, "window", None) if ctx is not None else None
    if win:
        glfw.set_key_callback(win, key_cb)

    print("""
Keys:
  B = Place mop (lower to table & lock Z)
  Z = Toggle Z-lock only (maintain contact while you move around)
  U = Set XY anchor to current mop position (used when Z-lock is ON)
  G = Start/stop cleaning sweep (raster over table with contact)
  M = SIM/ACTIVE mode toggle     C = Contact-lock toggle
  [ / ] = decrease/increase press depth (contact pressure)
  1..6 + ←/→ = select/nudge joint (ACTIVE mode)
  R = reset mid-range    Q/Esc = quit
""")

    # Jacobian buffer
    Jp = np.zeros((3, model.nv))

    # Build initial raster path generator
    center, ex, ey = table_axes_world(data, bodyid_table)
    path_gen = raster_path_points(center, ex, ey, TABLE_HALF_X, TABLE_HALF_Y, pitch=pitch)
    try:
        des_x, des_y = next(path_gen)
    except StopIteration:
        des_x, des_y = center[0], center[1]

    # Selected joint (for ACTIVE nudges)
    selected_idx = 5  # default: wrist_3
    t0 = time.time()

    while viewer_is_alive(viewer):
        # ---------- Keyboard ----------
        while q:
            key = q.popleft()
            if key in (glfw.KEY_Q, glfw.KEY_ESCAPE):
                viewer.close()
                break

            elif key == glfw.KEY_M:
                mode = MODE_ACTIVE if mode == MODE_SIM else MODE_SIM
                print("[MODE]", "ACTIVE" if mode == MODE_ACTIVE else "SIM")

            elif key == glfw.KEY_C:
                contact_lock = not contact_lock
                print("[CONTACT-LOCK]", "ON" if contact_lock else "OFF")

            elif key == glfw.KEY_B:
                # PLACE: lock contact at current XY and engage Z tracking
                mop_pos = data.xpos[bodyid_mop].copy()
                z_hit, _ = raycast_table_z(model, data, mop_pos[0], mop_pos[1], z_above=5.0, bodyexclude=bodyid_mop)
                if z_hit is None:
                    z_hit = data.xpos[bodyid_table][2] + 0.40  # fallback
                xy_anchor = (mop_pos[0], mop_pos[1])
                z_plane_est = z_hit                         # init plane filter here
                mop_z_prev = mop_pos[2]                    # init velocity estimate
                contact_lock = True
                z_lock_only = True
                placed = True
                cleaning = False
                print(f"[PLACE] Anchored at XY=({xy_anchor[0]:.3f},{xy_anchor[1]:.3f}), Z locked to tabletop.")

            elif key == glfw.KEY_Z:
                z_lock_only = not z_lock_only
                print("[Z-LOCK ONLY]", "ON" if z_lock_only else "OFF")

            elif key == glfw.KEY_U:
                mop_pos = data.xpos[bodyid_mop].copy()
                xy_anchor = (mop_pos[0], mop_pos[1])
                print(f"[ANCHOR] XY anchor set to ({xy_anchor[0]:.3f},{xy_anchor[1]:.3f})")

            elif key == glfw.KEY_G:
                cleaning = not cleaning
                if cleaning:
                    # regen path from current table pose
                    center, ex, ey = table_axes_world(data, bodyid_table)
                    path_gen = raster_path_points(center, ex, ey, TABLE_HALF_X, TABLE_HALF_Y, pitch=pitch)
                    try:
                        des_x, des_y = next(path_gen)
                    except StopIteration:
                        des_x, des_y = center[0], center[1]
                    mode = MODE_SIM
                    z_lock_only = False  # follow full XY during sweep
                    contact_lock = True
                    placed = True
                print("[CLEANING]", "ON" if cleaning else "OFF")

            elif key == glfw.KEY_LEFT and mode == MODE_ACTIVE:
                name = JOINT_ORDER[selected_idx]
                lo, hi = JOINT_LIMITS[name]
                targets[name] = clamp(targets[name] - NUDGE, lo, hi)

            elif key == glfw.KEY_RIGHT and mode == MODE_ACTIVE:
                name = JOINT_ORDER[selected_idx]
                lo, hi = JOINT_LIMITS[name]
                targets[name] = clamp(targets[name] + NUDGE, lo, hi)

            elif glfw.KEY_1 <= key <= glfw.KEY_6:
                selected_idx = key - glfw.KEY_1
                print(f"[SELECT] {selected_idx+1}: {JOINT_ORDER[selected_idx]}")

            elif key == glfw.KEY_R:
                for j in JOINT_ORDER:
                    lo, hi = JOINT_LIMITS[j]
                    targets[j] = mid(lo, hi)
                print("[RESET] mid-range")

            elif key == glfw.KEY_LEFT_BRACKET:   # '['
                PRESS_MIN, PRESS_MAX = 0.0, 0.01
                press_depth = max(PRESS_MIN, press_depth - 0.0005)
                print(f"[PRESS] depth = {press_depth:.4f} m")

            elif key == glfw.KEY_RIGHT_BRACKET:  # ']'
                PRESS_MIN, PRESS_MAX = 0.0, 0.01
                press_depth = min(PRESS_MAX, press_depth + 0.0005)
                print(f"[PRESS] depth = {press_depth:.4f} m")

        # ---------- Desired mop (x,y,z) ----------
        mop_pos = data.xpos[bodyid_mop].copy()

        # Desired XY
        if mode == MODE_SIM and cleaning:
            dwell_time += dt
            if dwell_time >= dwell_dt:
                dwell_time = 0.0
                try:
                    des_x, des_y = next(path_gen)
                except StopIteration:
                    center, ex, ey = table_axes_world(data, bodyid_table)
                    path_gen = raster_path_points(center, ex, ey, TABLE_HALF_X, TABLE_HALF_Y, pitch=pitch)
                    des_x, des_y = next(path_gen)
            des_xy = (des_x, des_y)
        else:
            if z_lock_only and xy_anchor is not None:
                des_xy = xy_anchor   # hold XY at anchor; keep contact in Z
            else:
                des_xy = (mop_pos[0], mop_pos[1])  # no XY pull

        # Desired Z (contact) - filtered table plane, gentle press
        if contact_lock:
            z_hit, _ = raycast_table_z(model, data, des_xy[0], des_xy[1], z_above=5.0, bodyexclude=bodyid_mop)
            if z_hit is None:
                z_hit = data.xpos[bodyid_table][2] + 0.40  # fallback
            if z_plane_est is None:
                z_plane_est = z_hit
            else:
                z_plane_est = (1.0 - Z_PLANE_ALPHA) * z_plane_est + Z_PLANE_ALPHA * z_hit
            des_z = z_plane_est - press_depth  # small press into plane (solver / contacts will resist)
        else:
            des_z = mop_pos[2]

        # ---------- Resolved-rate IK (damped Z + optional XY) ----------
        # Jacobian at mop COM
        Jp = np.zeros((3, model.nv))
        mujoco.mj_jacBodyCom(model, data, Jp, None, bodyid_mop)  # 3 x nv
        J = Jp[:, dof_cols]  # 3 x 6

        # Estimate vertical velocity from finite differences (robust across MuJoCo versions)
        if mop_z_prev is None:
            vz = 0.0
        else:
            vz = (mop_pos[2] - mop_z_prev) / max(dt, 1e-6)
        mop_z_prev = mop_pos[2]

        # Task-space PD in Z to avoid stomping
        err_z = float(des_z - mop_pos[2])
        v_z_des = KZ * err_z - DZ * vz                  # desired vertical velocity (m/s)
        v_z_des = clamp(v_z_des, -MAX_Z_RATE, MAX_Z_RATE)

        # Map Z velocity to a joint step via damped pseudoinverse of Jz
        Jz = J[2:3, :]                                   # 1 x 6
        denom = float(Jz @ Jz.T + LAMBDA)                # scalar
        qdot_z = (Jz.T * (v_z_des / denom)).reshape(-1)  # 6,
        dq_z = qdot_z * dt                               # convert vel -> step

        # Optional lateral (x,y) tracking for cleaning sweep
        dq_xy = np.zeros(6)
        if (mode == MODE_SIM and cleaning and not z_lock_only):
            err_xy = np.array([des_xy[0] - mop_pos[0], des_xy[1] - mop_pos[1]])
            Jxy = J[0:2, :]                              # 2 x 6
            Axy = Jxy @ Jxy.T + LAMBDA * np.eye(2)
            dq_xy = (Jxy.T @ np.linalg.solve(Axy, K_XY * err_xy)).reshape(-1)

        # Combine steps
        dq = dq_z + dq_xy

        # Integrate dq into joint targets (rate limit + clamp)
        max_step = MAX_TARGET_RATE * dt
        for i, jname in enumerate(JOINT_ORDER):
            lo, hi = JOINT_LIMITS[jname]
            step = clamp(float(dq[i]), -max_step, max_step)
            targets[jname] = clamp(targets[jname] + step, lo, hi)

        # OPTIONAL: tiny wrist scrub during cleaning (keeps motion lively; wrist_3 still clamped)
        if mode == MODE_SIM and cleaning:
            t = time.time() - t0
            for w in ("wrist_2",):
                lo, hi = JOINT_LIMITS[w]
                osc = 0.2 * math.sin(2 * math.pi * 1.0 * t)
                targets[w] = clamp(targets[w] + osc * dt, lo, hi)

        # Smooth into filtered targets
        for jn in JOINT_ORDER:
            lo, hi = JOINT_LIMITS[jn]
            delta = targets[jn] - filt_targets[jn]
            step = clamp(delta, -max_step, max_step)
            filt_targets[jn] = clamp(filt_targets[jn] + step, lo, hi)

        # Send controls and step
        set_ctrl(data, act_id, filt_targets)
        mujoco.mj_step(model, data)

        # Render viewer
        viewer.render()

        # Keep real-time-ish
        time.sleep(dt * 0.25)

    # Cleanup
    viewer.close()


if __name__ == "__main__":
    main()
