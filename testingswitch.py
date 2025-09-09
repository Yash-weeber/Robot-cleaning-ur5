# import math
# import time
# from collections import deque

# import numpy as np
# import mujoco
# from mujoco import mjtObj
# import mujoco_viewer
# import glfw  # <- keyboard input

# # ======== USER: set your XML path here ========
# XML_PATH = r"F:/Robotics Automation system AI/SEM 3/APPLIED PROJECT/NEW XMLS FILESSSSSSSSSSSSSSSSSSSSSSSSSSSS/Robot-cleaning-ur5-master/ur5e_with_table_merged.xml"

# # ======== Joint config ========
# JOINT_ORDER = [
#     "shoulder_pan",
#     "shoulder_lift",
#     "elbow",
#     "wrist_1",
#     "wrist_2",
#     "wrist_3",
# ]

# # Your requested ranges (radians). Wrists are 360°; we treat them as wrap-around in [-pi, pi].
# JOINT_LIMITS = {
#     "shoulder_pan":  (-4.84, -2.52),
#     "shoulder_lift": (-3.14, -0.126),
#     "elbow":         (-3.0,  2.4),
#     "wrist_1":       (-math.pi, math.pi),
#     "wrist_2":       (-math.pi, math.pi),
#     "wrist_3":       (-math.pi, math.pi),
# }

# # Step size for manual nudging (radians)
# NUDGE = np.deg2rad(2.0)  # 2 degrees per key press


# def clamp(val, lo, hi):
#     return max(lo, min(hi, val))


# def mid_of_range(lo, hi):
#     return 0.5 * (lo + hi)


# def wrap_to_pi(x):
#     """Wrap angle to [-pi, pi]."""
#     return (x + math.pi) % (2 * math.pi) - math.pi


# def build_name_maps(model):
#     """Get ids and qpos addresses for our joints and actuators."""
#     joint_id = {}
#     qpos_adr = {}
#     act_id = {}

#     for name in JOINT_ORDER:
#         jid = mujoco.mj_name2id(model, mjtObj.mjOBJ_JOINT, name)
#         if jid < 0:
#             raise RuntimeError(f"Joint '{name}' not found in model.")
#         joint_id[name] = jid
#         qpos_adr[name] = model.jnt_qposadr[jid]

#         # actuators in your XML are "<joint>_act"
#         aid = mujoco.mj_name2id(model, mjtObj.mjOBJ_ACTUATOR, f"{name}_act")
#         if aid < 0:
#             # fallback: try same name as joint
#             aid = mujoco.mj_name2id(model, mjtObj.mjOBJ_ACTUATOR, name)
#             if aid < 0:
#                 raise RuntimeError(f"Actuator for '{name}' not found (tried '{name}_act' and '{name}').")
#         act_id[name] = aid

#     return joint_id, qpos_adr, act_id


# def set_ctrl_from_targets(data, act_id, targets):
#     """Write desired joint position targets into position actuators."""
#     for name, target in targets.items():
#         data.ctrl[act_id[name]] = float(target)


# def viewer_is_alive(viewer):
#     """Handle both property and callable forms across versions."""
#     alive_attr = getattr(viewer, "is_alive", None)
#     if callable(alive_attr):
#         return alive_attr()
#     return bool(alive_attr)


# def main():
#     model = mujoco.MjModel.from_xml_path(XML_PATH)
#     data = mujoco.MjData(model)

#     dt = model.opt.timestep

#     # Build maps
#     joint_id, qpos_adr, act_id = build_name_maps(model)

#     # Initialize targets at the middle of each range
#     targets = {name: mid_of_range(*JOINT_LIMITS[name]) for name in JOINT_ORDER}

#     # Modes
#     MODE_SIM = 0
#     MODE_ACTIVE = 1
#     mode = MODE_SIM

#     # SIM mode signal
#     t0 = time.time()
#     sim_amp_frac = 0.35  # 35% of each joint range
#     sim_speed = {
#         "shoulder_pan":  0.25,
#         "shoulder_lift": 0.20,
#         "elbow":         0.30,
#         "wrist_1":       0.50,
#         "wrist_2":       0.60,
#         "wrist_3":       0.70,
#     }

#     # Keyboard state
#     selected_idx = 0  # index into JOINT_ORDER

#     # Viewer
#     viewer = mujoco_viewer.MujocoViewer(model, data)
#     # force drawing each frame for smoother interaction
#     if hasattr(viewer, "_render_every_frame"):
#         viewer._render_every_frame = True

#     # Key event queue (we'll push into this from a GLFW callback)
#     event_queue = deque()

#     def key_callback(window, key, scancode, action, mods):
#         # Record key presses and repeats; ignore releases
#         if action in (glfw.PRESS, glfw.REPEAT):
#             event_queue.append((key, mods, action))

#     # Attach callback to the GLFW window used by the viewer
#     window = getattr(viewer, "window", None) or getattr(viewer, "_window", None)
#     if window is None:
#         # Some versions expose .context.window
#         context = getattr(viewer, "context", None)
#         window = getattr(context, "window", None) if context is not None else None
#     if window is None:
#         print("[WARN] Could not locate GLFW window in viewer; keyboard control disabled.")
#     else:
#         glfw.set_key_callback(window, key_callback)

#     print("\n=== Controls ===")
#     print("M : toggle SIM <-> ACTIVE")
#     print("1..6 : select joint (1=shoulder_pan ... 6=wrist_3)")
#     print("<-/-> : decrease/increase selected joint target")
#     print("R : reset to mid-range pose")
#     print("Q or ESC : quit")
#     print("================\n")

#     # Main loop
#     while viewer_is_alive(viewer):
#         # ---- Keyboard handling ----
#         while event_queue:
#             key, mods, action = event_queue.popleft()

#             if key in (glfw.KEY_Q, glfw.KEY_ESCAPE):
#                 viewer.close()
#                 break

#             elif key in (glfw.KEY_M,):
#                 mode = MODE_ACTIVE if mode == MODE_SIM else MODE_SIM
#                 print(f"[MODE] {'ACTIVE' if mode == MODE_ACTIVE else 'SIM'}")

#             elif key in (glfw.KEY_R,):
#                 for jn in JOINT_ORDER:
#                     lo, hi = JOINT_LIMITS[jn]
#                     targets[jn] = mid_of_range(lo, hi)
#                 print("[RESET] targets set to mid-range.")

#             elif glfw.KEY_1 <= key <= glfw.KEY_6:
#                 selected_idx = key - glfw.KEY_1
#                 print(f"[SELECT] joint {selected_idx+1}: {JOINT_ORDER[selected_idx]}")

#             elif key == glfw.KEY_LEFT:
#                 name = JOINT_ORDER[selected_idx]
#                 lo, hi = JOINT_LIMITS[name]
#                 new_val = targets[name] - NUDGE
#                 if name.startswith("wrist_"):
#                     new_val = wrap_to_pi(new_val)
#                 targets[name] = clamp(new_val, lo, hi)

#             elif key == glfw.KEY_RIGHT:
#                 name = JOINT_ORDER[selected_idx]
#                 lo, hi = JOINT_LIMITS[name]
#                 new_val = targets[name] + NUDGE
#                 if name.startswith("wrist_"):
#                     new_val = wrap_to_pi(new_val)
#                 targets[name] = clamp(new_val, lo, hi)

#         # ---- SIM mode target update ----
#         if mode == MODE_SIM:
#             t = time.time() - t0
#             for jn in JOINT_ORDER:
#                 lo, hi = JOINT_LIMITS[jn]
#                 center = 0.5 * (lo + hi)
#                 amp = sim_amp_frac * (hi - lo) * 0.5
#                 w = sim_speed.get(jn, 0.3)
#                 desired = center + amp * math.sin(2 * math.pi * w * t)
#                 if jn.startswith("wrist_"):
#                     desired = wrap_to_pi(desired)
#                 targets[jn] = clamp(desired, lo, hi)

#         # ---- Write controls & step ----
#         set_ctrl_from_targets(data, act_id, targets)
#         mujoco.mj_step(model, data)

#         # ---- Render ----
#         viewer.render()

#         # Keep real-time-ish without starving the event loop
#         time.sleep(dt * 0.25)

#     viewer.close()


# if __name__ == "__main__":
#     main()


# import math
# import time
# from collections import deque

# import numpy as np
# import mujoco
# from mujoco import mjtObj
# import mujoco_viewer
# import glfw

# # ---- Optional UI (Dear ImGui). Falls back to keyboard-only if not present. ----
# IMGUIMODE = True
# try:
#     import imgui
#     from imgui.integrations.glfw import GlfwRenderer
# except Exception:
#     IMGUIMODE = False


# # ========= PATH TO YOUR XML =========
# XML_PATH = r"F:/Robotics Automation system AI/SEM 3/APPLIED PROJECT/NEW XMLS FILESSSSSSSSSSSSSSSSSSSSSSSSSSSS/Robot-cleaning-ur5-master/ur5e_with_table_merged.xml"

# # ========= JOINTS & LIMITS (no 360 on wrist_3) =========
# JOINT_ORDER = ["shoulder_pan", "shoulder_lift", "elbow", "wrist_1", "wrist_2", "wrist_3"]
# JOINT_LIMITS = {
#     "shoulder_pan":  (-4.84, -2.52),
#     "shoulder_lift": (-3.14, -0.126),
#     "elbow":         (-3.0,  2.4),
#     "wrist_1":       (-2.967,  2.967),  # ~±170°
#     "wrist_2":       (-2.967,  2.967),
#     "wrist_3":       (-2.967,  2.967),  # ~±170°
# }

# # ========= TABLE AREA (half-sizes in table frame) =========
# TABLE_HALF_X = 1.00   # meters (half-length in table-X)
# TABLE_HALF_Y = 0.45   # meters (half-length in table-Y)

# # ========= CONTROL GAINS / RATES =========
# CONTACT_OFFSET = 0.002                 # small gap above table
# NUDGE = np.deg2rad(2.0)                # manual step per key
# MAX_TARGET_RATE = np.deg2rad(90.0)     # rad/s
# K_POS_DEFAULT = 10.0                   # IK position gain
# LAMBDA_DEFAULT = 1e-3                  # DLS damping
# DWELL_DT_DEFAULT = 0.10                # seconds per waypoint
# PITCH_DEFAULT = 0.15                   # raster spacing

# # ========= UTILS =========
# def clamp(v, lo, hi): return lo if v < lo else hi if v > hi else v
# def mid(lo, hi): return 0.5*(lo+hi)

# def viewer_is_alive(viewer):
#     alive_attr = getattr(viewer, "is_alive", None)
#     return alive_attr() if callable(alive_attr) else bool(alive_attr)

# def build_name_maps(model):
#     joint_id, qpos_adr, dof_idx, act_id = {}, {}, {}, {}
#     for name in JOINT_ORDER:
#         jid = mujoco.mj_name2id(model, mjtObj.mjOBJ_JOINT, name)
#         if jid < 0:
#             raise RuntimeError(f"Joint '{name}' not found in model.")
#         joint_id[name] = jid
#         qpos_adr[name] = model.jnt_qposadr[jid]
#         dof_idx[name]  = model.jnt_dofadr[jid]  # 1 DoF per revolute joint
#         aid = mujoco.mj_name2id(model, mjtObj.mjOBJ_ACTUATOR, f"{name}_act")
#         if aid < 0:
#             aid = mujoco.mj_name2id(model, mjtObj.mjOBJ_ACTUATOR, name)
#             if aid < 0:
#                 raise RuntimeError(f"Actuator for '{name}' not found.")
#         act_id[name] = aid
#     return joint_id, qpos_adr, dof_idx, act_id

# def set_ctrl(data, act_id, targets):
#     for n, v in targets.items():
#         data.ctrl[act_id[n]] = float(v)

# def raycast_table_z(model, data, x, y, z_above=5.0, bodyexclude=-1):
#     """
#     Cast a vertical ray downward from (x,y,z_above).
#     Returns (z_hit, geomid) or (None, -1) if nothing hit.
#     Matches mujoco.mj_ray signature that requires bodyexclude and a writable geomid array.
#     """
#     pnt = np.array([x, y, z_above], dtype=np.float64)
#     vec = np.array([0.0, 0.0, -1.0], dtype=np.float64)
#     geomgroup = np.array([1, 1, 1, 1, 1, 1], dtype=np.uint8)  # all 6 groups
#     flg_static = 1
#     geomid_out = np.array([-1], dtype=np.int32)

#     dist = mujoco.mj_ray(model, data, pnt, vec, geomgroup, flg_static, int(bodyexclude), geomid_out)
#     hit_id = int(geomid_out[0])
#     if hit_id >= 0 and np.isfinite(dist):
#         return z_above - dist, hit_id
#     return None, -1

# def table_axes_world(data, bodyid_table):
#     """Return table origin and its x,y axes (world)."""
#     o = data.xpos[bodyid_table].copy()
#     R = data.xmat[bodyid_table].reshape(3, 3).copy()
#     ex = R[:, 0]
#     ey = R[:, 1]
#     return o, ex, ey

# def raster_path_points(center, ex, ey, half_x, half_y, pitch=0.15):
#     """Yield world (x,y) waypoints that raster the rectangle on the table."""
#     xs = np.linspace(-half_x, half_x, max(2, int(2*half_x/pitch)+1))
#     ys = np.linspace(-half_y, half_y, max(2, int(2*half_y/pitch)+1))
#     flip = False
#     for y in ys:
#         seq = xs[::-1] if flip else xs
#         for x in seq:
#             p = center + ex * x + ey * y
#             yield p[0], p[1]
#         flip = not flip


# # ========= OPTIONAL: Separate Dear ImGui Control Panel =========
# class ControlPanel:
#     def __init__(self, title="Control Panel", size=(380, 480)):
#         if not IMGUIMODE:
#             self.alive = False
#             return
#         if not glfw.init():
#             # glfw already initialized by viewer; proceed anyway
#             pass
#         self.window = glfw.create_window(size[0], size[1], title, None, None)
#         if not self.window:
#             self.alive = False
#             return
#         glfw.make_context_current(self.window)
#         imgui.create_context()
#         self.impl = GlfwRenderer(self.window)
#         self.alive = True
#         self._first_frame = True

#     def render(self, state):
#         """Draw UI and mutate state dict. Returns False if window closed."""
#         if not self.alive:
#             return False

#         glfw.make_context_current(self.window)
#         glfw.poll_events()
#         self.impl.process_inputs()
#         imgui.new_frame()

#         imgui.begin("UR5 Cleaning Control", True)

#         changed, state["show_panel"] = imgui.checkbox("Show Panel", state["show_panel"])
#         if not state["show_panel"]:
#             imgui.end()
#             imgui.render()
#             self.impl.render(imgui.get_draw_data())
#             glfw.swap_buffers(self.window)
#             return True

#         # Mode toggles
#         if imgui.button("SIM  \u21C4  ACTIVE"):
#             state["mode"] = 1 - state["mode"]
#         imgui.same_line()
#         clicked, state["contact_lock"] = imgui.checkbox("Contact-Lock", state["contact_lock"])
#         imgui.same_line()
#         clicked, state["cleaning"] = imgui.checkbox("Cleaning Sweep", state["cleaning"])

#         imgui.separator()

#         # Path parameters
#         changed, state["pitch"] = imgui.slider_float("Raster Pitch (m)", state["pitch"], 0.05, 0.40)
#         changed, state["dwell_dt"] = imgui.slider_float("Waypoint Dwell (s)", state["dwell_dt"], 0.02, 0.40)
#         changed, state["contact_offset"] = imgui.slider_float("Contact Offset (m)", state["contact_offset"], 0.0, 0.01)
#         changed, state["k_pos"] = imgui.slider_float("IK Gain", state["k_pos"], 1.0, 30.0)
#         changed, state["lambda"] = imgui.slider_float("Damping (lambda)", state["lambda"], 1e-5, 1e-2, format="%.5f")

#         imgui.separator()

#         # Joint control
#         current = state["selected_idx"]
#         changed, current = imgui.combo(
#             "Selected Joint",
#             current,
#             [f"{i+1}. {n}" for i, n in enumerate(JOINT_ORDER)]
#         )
#         if changed:
#             state["selected_idx"] = current

#         if imgui.button("<< Nudge Left"):
#             jn = JOINT_ORDER[state["selected_idx"]]
#             lo, hi = JOINT_LIMITS[jn]
#             state["targets"][jn] = clamp(state["targets"][jn] - state["nudge"], lo, hi)
#         imgui.same_line()
#         if imgui.button("Nudge Right >>"):
#             jn = JOINT_ORDER[state["selected_idx"]]
#             lo, hi = JOINT_LIMITS[jn]
#             state["targets"][jn] = clamp(state["targets"][jn] + state["nudge"], lo, hi)

#         if imgui.button("Reset Mid-Range"):
#             for j in JOINT_ORDER:
#                 lo, hi = JOINT_LIMITS[j]
#                 state["targets"][j] = mid(lo, hi)

#         imgui.separator()

#         # Live readout
#         imgui.text_colored("Live Targets (rad):", 0.8, 0.9, 1.0)
#         for j in JOINT_ORDER:
#             imgui.text(f"{j:>12s}: {state['targets'][j]: .3f}")

#         imgui.end()
#         imgui.render()
#         self.impl.render(imgui.get_draw_data())
#         glfw.swap_buffers(self.window)

#         # Keep alive unless user closed it
#         self.alive = not glfw.window_should_close(self.window)
#         return self.alive

#     def close(self):
#         if not self.alive:
#             return
#         self.impl.shutdown()
#         glfw.destroy_window(self.window)
#         self.alive = False


# # ========= MAIN =========
# def main():
#     model = mujoco.MjModel.from_xml_path(XML_PATH)
#     data = mujoco.MjData(model)
#     dt = model.opt.timestep

#     joint_id, qpos_adr, dof_idx, act_id = build_name_maps(model)

#     # Bodies
#     bodyid_mop   = mujoco.mj_name2id(model, mjtObj.mjOBJ_BODY, "mop_head")
#     bodyid_table = mujoco.mj_name2id(model, mjtObj.mjOBJ_BODY, "table")
#     if bodyid_mop < 0:   raise RuntimeError("Body 'mop_head' not found.")
#     if bodyid_table < 0: raise RuntimeError("Body 'table' not found.")

#     # Indices for our 6 dofs in model nv order
#     dof_cols = [dof_idx[j] for j in JOINT_ORDER]

#     # Targets and filtered targets
#     targets = {j: mid(*JOINT_LIMITS[j]) for j in JOINT_ORDER}
#     filt_targets = dict(targets)

#     # Modes
#     MODE_SIM, MODE_ACTIVE = 0, 1
#     mode = MODE_SIM
#     contact_lock = True
#     cleaning = True

#     k_pos = K_POS_DEFAULT
#     lam = LAMBDA_DEFAULT
#     dwell_dt = DWELL_DT_DEFAULT
#     pitch = PITCH_DEFAULT
#     contact_offset = CONTACT_OFFSET

#     # Viewer + keyboard
#     viewer = mujoco_viewer.MujocoViewer(model, data)
#     if hasattr(viewer, "_render_every_frame"):
#         viewer._render_every_frame = True

#     # Prepare key event queue
#     q = deque()
#     def key_cb(window, key, scancode, action, mods):
#         if action in (glfw.PRESS, glfw.REPEAT):
#             q.append(key)

#     win = getattr(viewer, "window", None) or getattr(viewer, "_window", None)
#     if win is None:
#         ctx = getattr(viewer, "context", None)
#         win = getattr(ctx, "window", None) if ctx is not None else None
#     if win:
#         glfw.set_key_callback(win, key_cb)

#     print("\nKeys: P toggle panel • G start/stop cleaning • M SIM/ACTIVE • C contact-lock • 1..6 select • ←/→ nudge • R reset • Q/Esc quit\n")

#     # Optional: start control panel
#     panel = ControlPanel() if IMGUIMODE else None

#     # Jacobians
#     Jp = np.zeros((3, model.nv))
#     Jr_dummy = np.zeros((3, model.nv))

#     # Raster path init
#     center, ex, ey = table_axes_world(data, bodyid_table)
#     path_gen = raster_path_points(center, ex, ey, TABLE_HALF_X, TABLE_HALF_Y, pitch=pitch)
#     try:
#         des_x, des_y = next(path_gen)
#     except StopIteration:
#         des_x, des_y = center[0], center[1]
#     dwell_time = 0.0

#     # Selected joint (for ACTIVE nudges)
#     selected_idx = 5  # default: wrist_3

#     # Shared state for UI
#     state = {
#         "mode": mode,
#         "contact_lock": contact_lock,
#         "cleaning": cleaning,
#         "selected_idx": selected_idx,
#         "targets": targets,
#         "nudge": float(NUDGE),
#         "pitch": float(pitch),
#         "dwell_dt": float(dwell_dt),
#         "contact_offset": float(contact_offset),
#         "k_pos": float(k_pos),
#         "lambda": float(lam),
#         "show_panel": True,
#     }

#     t0 = time.time()

#     while viewer_is_alive(viewer):
#         # ---------- Panel (if available) ----------
#         if panel and panel.alive and state["show_panel"]:
#             panel.render(state)

#         # ---------- Keyboard ----------
#         while q:
#             key = q.popleft()
#             if key in (glfw.KEY_Q, glfw.KEY_ESCAPE):
#                 viewer.close()
#                 if panel: panel.close()
#                 break
#             elif key == glfw.KEY_P and panel:
#                 state["show_panel"] = not state["show_panel"]
#             elif key == glfw.KEY_M:
#                 state["mode"] = MODE_ACTIVE if state["mode"] == MODE_SIM else MODE_SIM
#                 print("[MODE]", "ACTIVE" if state["mode"] == MODE_ACTIVE else "SIM")
#             elif key == glfw.KEY_C:
#                 state["contact_lock"] = not state["contact_lock"]
#                 print("[CONTACT]", "ON" if state["contact_lock"] else "OFF")
#             elif key == glfw.KEY_G:
#                 state["cleaning"] = not state["cleaning"]
#                 print("[CLEANING]", "ON" if state["cleaning"] else "OFF")
#                 if state["cleaning"]:
#                     center, ex, ey = table_axes_world(data, bodyid_table)
#                     path_gen = raster_path_points(center, ex, ey, TABLE_HALF_X, TABLE_HALF_Y, pitch=state["pitch"])
#                     try: des_x, des_y = next(path_gen)
#                     except StopIteration: pass
#                 dwell_time = 0.0
#             elif key == glfw.KEY_R:
#                 for j in JOINT_ORDER:
#                     lo, hi = JOINT_LIMITS[j]
#                     state["targets"][j] = mid(lo, hi)
#                 print("[RESET]")
#             elif glfw.KEY_1 <= key <= glfw.KEY_6:
#                 state["selected_idx"] = key - glfw.KEY_1
#                 print(f"[SELECT] {state['selected_idx']+1}: {JOINT_ORDER[state['selected_idx']]}")
#             elif state["mode"] == MODE_ACTIVE and key in (glfw.KEY_LEFT, glfw.KEY_RIGHT):
#                 name = JOINT_ORDER[state["selected_idx"]]
#                 lo, hi = JOINT_LIMITS[name]
#                 step = -state["nudge"] if key == glfw.KEY_LEFT else state["nudge"]
#                 state["targets"][name] = clamp(state["targets"][name] + step, lo, hi)

#         # Copy live params out of state
#         mode = state["mode"]
#         contact_lock = state["contact_lock"]
#         cleaning = state["cleaning"]
#         selected_idx = state["selected_idx"]
#         targets = state["targets"]
#         pitch = state["pitch"]
#         dwell_dt = state["dwell_dt"]
#         contact_offset = state["contact_offset"]
#         k_pos = state["k_pos"]
#         lam = state["lambda"]

#         # If pitch changed, rebuild path next time cleaning toggles on
#         # (to avoid regenerating every frame).

#         # ---------- Desired mop (x,y,z) ----------
#         mop_pos = data.xpos[bodyid_mop].copy()

#         # (x,y): follow raster if SIM+cleaning, else hold
#         if mode == MODE_SIM and cleaning:
#             dwell_time += dt
#             if dwell_time >= dwell_dt:
#                 dwell_time = 0.0
#                 try:
#                     des_x, des_y = next(path_gen)
#                 except StopIteration:
#                     center, ex, ey = table_axes_world(data, bodyid_table)
#                     path_gen = raster_path_points(center, ex, ey, TABLE_HALF_X, TABLE_HALF_Y, pitch=pitch)
#                     des_x, des_y = next(path_gen)
#         else:
#             des_x, des_y = mop_pos[0], mop_pos[1]

#         # (z): raycast to table under desired (x,y)
#         z_hit, _ = raycast_table_z(model, data, des_x, des_y, z_above=5.0, bodyexclude=bodyid_mop)
#         if z_hit is None:
#             # Fallback: approximate top as table origin + 0.40 (consistent with your XML inertial)
#             z_hit = data.xpos[bodyid_table][2] + 0.40
#         des_z = z_hit + (contact_offset if contact_lock else 0.0)

#         # ---------- Resolved-rate IK on mop_head position ----------
#         Jp = np.zeros((3, model.nv))
#         mujoco.mj_jacBodyCom(model, data, Jp, None, bodyid_mop)  # 3 x nv
#         J = Jp[:, dof_cols]  # 3 x 6
#         err = np.array([des_x - mop_pos[0], des_y - mop_pos[1], des_z - mop_pos[2]])
#         # Damped least squares
#         A = J @ J.T + lam * np.eye(3)
#         dq = J.T @ np.linalg.solve(A, k_pos * err)  # 6 x 1

#         # Integrate dq into targets (rate limit + clamp)
#         max_step = MAX_TARGET_RATE * dt
#         for i, jname in enumerate(JOINT_ORDER):
#             lo, hi = JOINT_LIMITS[jname]
#             step = clamp(float(dq[i]), -max_step, max_step)
#             targets[jname] = clamp(targets[jname] + step, lo, hi)

#         # Smooth into filtered targets
#         for jn in JOINT_ORDER:
#             lo, hi = JOINT_LIMITS[jn]
#             delta = targets[jn] - filt_targets[jn]
#             step = clamp(delta, -max_step, max_step)
#             filt_targets[jn] = clamp(filt_targets[jn] + step, lo, hi)

#         # Send controls and step
#         set_ctrl(data, act_id, filt_targets)
#         mujoco.mj_step(model, data)

#         # Render viewer
#         viewer.render()

#         # Keep real-time-ish
#         time.sleep(dt * 0.25)

#     # Cleanup
#     viewer.close()
#     if panel: panel.close()


# if __name__ == "__main__":
#     main()





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

# ========= CONTROL GAINS / RATES =========
NUDGE = np.deg2rad(2.0)                # manual step per key
MAX_TARGET_RATE = np.deg2rad(90.0)     # rad/s max change
K_POS = 10.0                           # IK gain
LAMBDA = 1e-3                          # damping
DWELL_DT = 0.10                        # seconds per waypoint
PITCH = 0.15                           # raster spacing (m)
PRESS_DEPTH = 0.0015                   # press into table (m), positive = more press

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

def dls_step(Juse, err_vec, lam, gain):
    """
    Robust damped least-squares step:
      - Juse: (m x 6)
      - err_vec: (m,)  (IMPORTANT: m must match Juse rows)
    Returns dq: (6,)
    """
    Juse = np.asarray(Juse, dtype=float)
    err_vec = np.asarray(err_vec, dtype=float).reshape(-1)
    m = Juse.shape[0]
    assert err_vec.shape[0] == m, f"err_vec len {err_vec.shape[0]} must equal rows {m}"

    if m == 1:
        # Scalar case: J is 1x6; A is scalar
        denom = float(Juse @ Juse.T + lam)  # (1x1)
        alpha = gain * err_vec[0] / denom if denom != 0.0 else 0.0
        dq = (Juse.T * alpha).reshape(-1)
        return dq
    else:
        A = Juse @ Juse.T + lam * np.eye(m)
        rhs = gain * err_vec
        dq = Juse.T @ np.linalg.solve(A, rhs)
        return dq.reshape(-1)

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

    # Jacobians
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
                contact_lock = True
                z_lock_only = True
                placed = True
                cleaning = False
                print(f"[PLACE] Anchored at XY=({xy_anchor[0]:.3f},{xy_anchor[1]:.3f}), Z target set to table top.")

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
            if dwell_time >= DWELL_DT:
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

        # Desired Z (contact)
        if contact_lock:
            z_hit, _ = raycast_table_z(model, data, des_xy[0], des_xy[1], z_above=5.0, bodyexclude=bodyid_mop)
            if z_hit is None:
                z_hit = data.xpos[bodyid_table][2] + 0.40  # fallback
            des_z = z_hit - press_depth  # push slightly into the table; contact will resist
        else:
            des_z = mop_pos[2]

        # ---------- Resolved-rate IK ----------
        # Build Jacobian at mop COM
        Jp = np.zeros((3, model.nv))
        mujoco.mj_jacBodyCom(model, data, Jp, None, bodyid_mop)  # 3 x nv
        J = Jp[:, dof_cols]  # 3 x 6

        # Choose Z-only or full XYZ control
        if z_lock_only or (mode == MODE_ACTIVE and not cleaning):
            # Z only: 1x6 Jacobian, 1-d error
            Juse = J[2:3, :]  # (1 x 6)
            err_vec = np.array([des_z - mop_pos[2]])  # (1,)
        else:
            # Full XYZ
            Juse = J          # (3 x 6)
            err_vec = np.array([des_xy[0] - mop_pos[0],
                                des_xy[1] - mop_pos[1],
                                des_z     - mop_pos[2]])  # (3,)

        dq = dls_step(Juse, err_vec, LAMBDA, K_POS)  # (6,)

        # Integrate dq into targets (rate limit + clamp)
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
