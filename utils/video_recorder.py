"""
Off-screen video recorder for MuJoCo simulations.

Uses mujoco.Renderer (EGL / OSMesa, no window required) to capture frames
and writes them to an MP4 via imageio-ffmpeg.

Usage
-----
    from utils.video_recorder import VideoRecorder

    with VideoRecorder(model, data, "run.mp4", fps=30) as rec:
        for joints in joint_traj:
            ...  # advance simulation
            rec.capture()
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Optional

import mujoco
import numpy as np


class VideoRecorder:
    """
    Captures MuJoCo frames off-screen and writes an MP4.

    Parameters
    ----------
    model       : mujoco.MjModel
    data        : mujoco.MjData
    filepath    : output path (e.g. "videos/run.mp4")
    fps         : frames per second in the output video
    width/height: render resolution (pixels)
    camera_name : named camera in the XML, or -1 for the free camera
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        filepath: str = "recording.mp4",
        fps: int = 30,
        width: int = 1280,
        height: int = 720,
        camera_name: Optional[str] = None,
        azimuth: float = 180.0,
        elevation: float = -30.0,
        distance: float = 1.9,
        lookat: tuple = (0.6, 0.0, 0.5),
    ):
        self.model = model
        self.data = data
        self.filepath = str(filepath)
        self.fps = fps
        self.width = width
        self.height = height

        # Build the free-camera object with the same angles as the viewer.
        # If a named camera is requested we use that instead.
        if camera_name is not None:
            cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
            if cam_id == -1:
                print(f"[VideoRecorder] WARNING: camera '{camera_name}' not found, using free camera.")
                camera_name = None

        if camera_name is None:
            self._cam = mujoco.MjvCamera()
            self._cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            self._cam.azimuth = azimuth
            self._cam.elevation = elevation
            self._cam.distance = distance
            self._cam.lookat[:] = lookat
        else:
            # Named camera — let MuJoCo handle positioning
            self._cam = cam_id

        # Clamp to the model's offscreen framebuffer size to avoid ValueError.
        fb_w = model.vis.global_.offwidth
        fb_h = model.vis.global_.offheight
        if width > fb_w or height > fb_h:
            print(
                f"[VideoRecorder] Requested {width}x{height} exceeds framebuffer "
                f"{fb_w}x{fb_h}. Clamping to framebuffer size."
            )
            width = min(width, fb_w)
            height = min(height, fb_h)
        self.width = width
        self.height = height
        self._renderer = mujoco.Renderer(model, height=height, width=width)
        self._frames: list[np.ndarray] = []
        self._writer = None

        # Ensure output directory exists
        Path(self.filepath).parent.mkdir(parents=True, exist_ok=True)

    # ------------------------------------------------------------------
    # Context-manager helpers
    # ------------------------------------------------------------------

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.save()
        self.close()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def capture(self):
        """Render the current simulation state and store the frame."""
        self._renderer.update_scene(self.data, camera=self._cam)
        frame = self._renderer.render()          # returns (H, W, 3) uint8 RGB
        self._frames.append(frame.copy())

    def save(self):
        """Write all captured frames to the output file."""
        if not self._frames:
            print("[VideoRecorder] No frames captured — nothing to save.")
            return

        try:
            import imageio
        except ImportError:
            raise ImportError(
                "imageio is required for video saving.\n"
                "Install with:  pip install imageio[ffmpeg]"
            )

        print(f"[VideoRecorder] Saving {len(self._frames)} frames → {self.filepath}")
        imageio.mimwrite(
            self.filepath,
            self._frames,
            fps=self.fps,
            codec="libx264",
            quality=8,
            output_params=["-pix_fmt", "yuv420p"],  # broad player compatibility
        )
        print(f"[VideoRecorder] Saved: {os.path.abspath(self.filepath)}")

    def close(self):
        """Release the off-screen renderer."""
        try:
            self._renderer.close()
        except Exception:
            pass
        self._frames.clear()
