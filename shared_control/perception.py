import cv2
import cv2.aruco as aruco
import numpy as np
import mujoco
from transforms3d.quaternions import qmult, mat2quat
from typing import Optional, Tuple


class PerceptionSystem:
    def __init__(
        self,
        model,
        data,
        marker_size: float = 0.10,
        image_width: int = 1920,
        image_height: int = 1080,
    ):
        """
        model: MuJoCo model
        data: MuJoCo data
        marker_size: Size of ArUco markers in meters
        image_width: Camera image width
        image_height: Camera image height
        """
        self.model = model
        self.data = data
        self.marker_size = marker_size
        self.image_width = image_width
        self.image_height = image_height

        # Initialize ArUco detector
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)
        aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        # Camera calibration parameters (assuming no distortion)
        self.dist_coeffs = np.zeros((4, 1), dtype=np.float32)
        self.renderer = None

    def get_camera_intrinsics(self, camera_name: str = "main_arm") -> np.ndarray:
        cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
        fov_y = self.model.cam_fovy[cam_id]  # Field of view in degrees

        # Calculate focal length based on field of view
        fov_y_rad = np.deg2rad(fov_y)
        focal_length_y = (self.image_height / 2.0) / np.tan(fov_y_rad / 2.0)
        focal_length_x = focal_length_y

        # Principal point at image center
        cx = self.image_width / 2.0
        cy = self.image_height / 2.0

        camera_matrix = np.array(
            [[focal_length_x, 0.0, cx], [0.0, focal_length_y, cy], [0.0, 0.0, 1.0]],
            dtype=np.float32,
        )

        return camera_matrix

    def get_camera_image(self, camera_name: str = "main_arm") -> np.ndarray:
        cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)

        if self.renderer is None:
            self.renderer = mujoco.Renderer(
                self.model, width=self.image_width, height=self.image_height
            )

        self.renderer.update_scene(self.data, camera=cam_id)
        rgb_image = self.renderer.render()
        return rgb_image

    def _estimate_aruco_pose(
        self, marker_idx: int, corners: list, camera_matrix: np.ndarray
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        if not corners:
            return None, None

        half_size = self.marker_size / 2.0
        object_points = np.array(
            [
                [-half_size, -half_size, 0],  # Bottom-left
                [half_size, -half_size, 0],  # Bottom-right
                [half_size, half_size, 0],  # Top-right
                [-half_size, half_size, 0],  # Top-left
            ],
            dtype=np.float32,
        )

        # Get the 2D image points for this marker
        image_points = corners[marker_idx].reshape(-1, 2)

        # Solve PnP to get pose
        success, rvec, tvec = cv2.solvePnP(
            object_points, image_points, camera_matrix, self.dist_coeffs
        )

        if not success:
            print("Failed to estimate marker pose")
            return None, None

        rvec = rvec.flatten()
        tvec = tvec.flatten()
        return rvec, tvec

    def detect_aruco_marker(
        self, image: np.ndarray, marker_id: int = 0, camera_name: str = "main_arm"
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Detect ArUco marker in image and return its position and orientation.

        Args:
            image: Input RGB image
            marker_id: ID of the ArUco marker to detect
            camera_name: Name of the camera for intrinsic parameters

        Returns:
            Tuple of (position, quaternion) in camera frame, or (None, None) if not detected
        """
        # Convert to grayscale for ArUco detection
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        corners, ids, _ = self.aruco_detector.detectMarkers(gray)

        if ids is not None and len(ids) > 0:
            # Find the specific marker we're looking for
            marker_idx = None
            for i, detected_id in enumerate(ids.flatten()):
                if detected_id == marker_id:
                    marker_idx = i
                    break

            if marker_idx is not None:
                camera_matrix = self.get_camera_intrinsics(camera_name)
                rvec, tvec = self._estimate_aruco_pose(
                    marker_idx, corners, camera_matrix
                )

                if rvec is not None and tvec is not None:
                    # Convert rotation vector to rotation matrix, then to quaternion
                    rotation_matrix, _ = cv2.Rodrigues(rvec)
                    quat_wxyz = mat2quat(rotation_matrix)

                    tvec_corrected = tvec.copy()
                    tvec_corrected[2] = -tvec_corrected[2]  # Flip Z coordinate
                    return tvec_corrected, quat_wxyz
                else:
                    print(f"Failed to estimate pose for marker ID {marker_id}")
                    return None, None
            else:
                print(f"Marker ID {marker_id} not detected")
                return None, None
        else:
            print("No ArUco markers detected")
            return None, None

    def camera_to_world_transform(
        self,
        camera_pos: np.ndarray,
        camera_quat: np.ndarray,
        camera_name: str = "main_arm",
    ) -> Tuple[np.ndarray, np.ndarray]:
        # Get camera ID
        cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)

        # Get camera world position and orientation from model
        cam_world_pos = self.model.cam_pos[cam_id].copy()
        cam_rotation_matrix = self.model.cam_mat0[cam_id].reshape(3, 3).copy()

        # Convert rotation matrix to quaternion
        cam_world_quat = mat2quat(cam_rotation_matrix)

        # Transform position from camera frame to world frame
        world_pos = cam_world_pos + cam_rotation_matrix @ camera_pos

        # Transform quaternion from camera frame to world frame
        world_quat = qmult(cam_world_quat, camera_quat)

        return world_pos, world_quat
