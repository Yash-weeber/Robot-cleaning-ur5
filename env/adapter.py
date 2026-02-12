import mujoco
import os

class ViewerAdapter:
    def __init__(self, model, data, title="MuJoCo DMP Controller"):
        self.model = model
        self.data = data
        self.backend = None
        self.viewer = None
        
        # Guard: Check if a display (monitor) is actually available
        has_display = "DISPLAY" in os.environ

        if has_display:
            # 1. Try DeepMind's built-in passive viewer (MuJoCo >= 3.1)
            try:
                import mujoco.viewer as mview # Moved inside to prevent crash
                self.backend = "dm"
                self.viewer = mview.launch_passive(model, data)
                print("[Viewer] Using mujoco.viewer (DeepMind).")
                return
            except Exception:
                pass

            # 2. Try community viewer fallback
            try:
                import mujoco_viewer
                self.backend = "community"
                self.viewer = mujoco_viewer.MujocoViewer(model, data, hide_menus=False)
                print("[Viewer] Using mujoco-python-viewer.")
                return
            except Exception:
                pass

        # 3. Headless Fallback: If no display or viewers fail
        print("[Viewer] No display detected or viewers failed. Running in HEADLESS mode (EGL).")
        self.backend = "none"

    def is_running(self):
        """
        Keeps the simulation loop alive. 
        In headless mode, we always return True.
        """
        if self.backend == "dm":
            return self.viewer.is_running()
        elif self.backend == "community":
            return not self.viewer.closed
        return True # Headless mode is 'always running'

    def draw(self):
        """Syncs the visuals only if a viewer backend is active."""
        if self.backend == "dm":
            self.viewer.sync()
        elif self.backend == "community":
            self.viewer.render()

    def close(self):
        """Gracefully closes active viewer contexts."""
        if self.backend == "dm":
            try:
                self.viewer.close()
            except Exception:
                pass
        elif self.backend == "community":
            try:
                self.viewer.close()
            except Exception:
                pass