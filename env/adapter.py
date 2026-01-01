import mujoco
import os
import mujoco.viewer

class ViewerAdapter:
    def __init__(self, model, data, title="MuJoCo DMP Controller"):
        self.model = model
        self.data = data
        self.backend = None
        self.viewer = None
        self._dm_context_mgr = None

        # Try DeepMind's built-in viewer (MuJoCo >= 3.1)
        try:
            import mujoco.viewer as mview
            self.backend = "dm"
            self.viewer = mview.launch_passive(model, data)
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