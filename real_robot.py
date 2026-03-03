"""
hometest-old.py
===============
Dual-robot mop control system for two Universal Robots (UR) arms.

Sequence
--------
1. Robot 1 performs an IK-based mopping trajectory (manual X/Y input or
   CSV waypoint file), then returns to its rest position.
2. After a 6-second cooldown, Robot 2 executes a hardcoded fixed joint
   trajectory (scoop-and-dump task), then returns to its rest position.

ROS 2 topics / services used
-----------------------------
- /compute_ik                                      (MoveIt IK service)
- /scaled_joint_trajectory_controller/joint_trajectory  (UR1 publisher)
- /ur_2_controller/joint_trajectory                (UR2 publisher)
- /joint_states                                    (joint state subscriber)
- /ur_2_controller/follow_joint_trajectory         (UR2 action client)
"""

import sys  # Standard library — sys.exit if needed
import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # Base class for all ROS 2 nodes
from rclpy.duration import Duration  # ROS 2 duration helper
from rclpy.action import ActionClient  # ROS 2 action client
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint  # Trajectory message types
from control_msgs.action import FollowJointTrajectory  # Action type for joint trajectory execution
from moveit_msgs.srv import GetPositionIK  # MoveIt IK service type
from moveit_msgs.msg import MoveItErrorCodes  # MoveIt error code constants
from geometry_msgs.msg import PoseStamped  # Stamped 3-D pose message
from sensor_msgs.msg import JointState  # Joint position/velocity/effort message
from builtin_interfaces.msg import Duration as BuiltinDuration  # Low-level ROS duration (sec + nanosec)
import csv  # Standard CSV reader
import os  # OS path utilities
import numpy as np  # Numerical operations (used implicitly via math)
import time  # Wall-clock sleep / timing
import threading  # Background spin thread + Event synchronisation
import math  # deg → rad conversion


class DualMopControlSystem(Node):
    """
    ROS 2 node that coordinates two UR robot arms for a mop/scoop task.

    Robot 1 (UR1)
        - Uses MoveIt inverse kinematics (/compute_ik) to move its mop tip
          over a set of X/Y coordinates supplied either interactively or
          from a CSV file.
        - Publishes directly to the scaled_joint_trajectory_controller.

    Robot 2 (UR2)
        - Executes a fixed, pre-recorded joint-space trajectory via the
          FollowJointTrajectory action server.
        - Only starts after Robot 1 has been confirmed back at rest.
    """

    def __init__(self):
        """
        Initialise the node, create all ROS 2 interfaces, and define
        joint names, poses, and trajectory waypoints for both robots.
        """
        # Initialise the parent Node with a unique name visible in ros2 node list
        super().__init__('dual_mop_control_system')

        # ── ROS 2 Clients & Publishers ─────────────────────────────────
        # Service client for MoveIt inverse kinematics
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')

        # Publisher: sends JointTrajectory commands to Robot 1's controller
        # self.ur1_pub = self.create_publisher(
        #     JointTrajectory,
        #     '/scaled_joint_trajectory_controller/joint_trajectory',
        #     10)   # queue depth of 10
        self.ur1_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10)  # queue depth of 10

        # Publisher: sends JointTrajectory commands to Robot 2's controller
        self.ur2_pub = self.create_publisher(
            JointTrajectory,
            '/ur_2_controller/joint_trajectory',
            10)  # queue depth of 10

        # Subscriber: listens to the combined /joint_states topic for both robots
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_cb, 10)

        # ── Robot 2 Action Client ──────────────────────────────────────
        # Used to send the full fixed trajectory and wait for completion
        self.ur2_action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/ur_2_controller/follow_joint_trajectory')

        # ── Joint Name Lists ───────────────────────────────────────────
        # Ordered list of joint names for Robot 1 — must match the controller config
        self.ur1_joints = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]

        # Ordered list of joint names for Robot 2 — prefixed with 'ur_2_'
        self.ur2_joints = [
            'ur_2_shoulder_pan_joint',
            'ur_2_shoulder_lift_joint',
            'ur_2_elbow_joint',
            'ur_2_wrist_1_joint',
            'ur_2_wrist_2_joint',
            'ur_2_wrist_3_joint',
        ]

        # ── Named Poses in Degrees (converted to radians below) ────────
        # Robot 1 safe resting configuration (clear of workspace)
        rest_qs_r1 = [13.53, -57.08, 64.66, -144.75, -88.89, 28.75]
        # Robot 2 safe resting configuration
        rest_qs_r2 = [-37.32, -51.07, 99.88, -245.13, -66.17, 268.34]
        # Robot 1 home/ready pose above the mopping area
        # home_qs_r1 = [-59.17, -69.74, 96.13, -122.94, -89.73, 9.94]
        home_qs_r1 = [-59.17, -69.74, 96.13, -122.94, -89.73, 195.46]
        # Robot 2 home/ready pose above the scooping area
        # home_qs_r2 = [-16.57, -71.79, 127.75, -230.10, -86.40, 192.74]
        home_qs_r2 = [-37.32, -51.07, 99.88, -245.13, -66.17, 268.34]

        # Convert all poses from degrees to radians for the controller
        self.home_pose_r1 = self.deg_to_rad(home_qs_r1)  # Robot 1 home (rad)
        self.home_pose_r2 = self.deg_to_rad(home_qs_r2)  # Robot 2 home (rad)
        self.rest_pose_r1 = self.deg_to_rad(rest_qs_r1)  # Robot 1 rest (rad)
        self.rest_pose_r2 = self.deg_to_rad(rest_qs_r2)  # Robot 2 rest (rad)

        # ── Robot 2 Fixed Trajectory Waypoints (degrees) ──────────────
        # Each key is a human-readable label; values are 6-DOF joint angles
        # in degrees. These are converted to radians when the trajectory is built.
        self.ur2_fixed_trajectory = {
            'home_rest': home_qs_r2,  # Safe start/end
            'home_rest_safe': [-36.81, -79.65, 68.67, -183.56, -91.73, 267.84],  # Safe start/end
            'work_safe': [84.32, -40.01, 44.64, -194.52, -92.74, 282.35],  # Hover above table
            # 'work_point': [ 88.82,   2.22,   0.01, -176.72, -96.61, 166.75],  # Touch-down on table
            'work_point': [89.07, 1.7, 1.95, -182.16, -96.4, 282.35],  # Touch-down on table
            # 'motion_pt1': [ 62.82,   1.7,   1.95, -182.16, -96.4, 166.75],  # Sweep right along table
            # 'motion_pt2': [117.30,   1.7,   1.95, -182.16, -96.4, 166.75],  # Sweep left along table
            'motion_pt1': [62.82, 2.22, 0.01, -176.72, -96.61, 282.35],  # Sweep right along table
            'motion_pt2': [117.30, 2.22, 0.01, -176.72, -96.61, 282.35],  # Sweep left along table
            # 'motion_pt3': [-17.66, -73.09, 121.40, -231.08, -92.83, 181.31],  # Hover over collection box
            'hover_over_box': [-37.23, -77.44, 87.32, -193.87, -63.46, 282.35],  # Hover over collection box
            'hover_over_box_2': [-33.72, -52.45, 105.97, -246.43, -51.14, 279.17],  # Hover over box just before scoop
            # 'motion_pt6': [-17.74, -76.03, 122.21, -150.27, -92.92, 181.31],  # Begin scoop approach
            # 'scoop':      [-11.11, -60.74, 124.15, -194.51, -92.74, 174.65],  # Scoop beads
            'motion_pt6': [-33.27, -48.98, 106.98, -246.14, -51.12, 195.04],  # Begin scoop approach
            # 'motion_pt7': [ 33.27, -75.79,  95.34, -212.50, -95.39, 179.87],  # Carry beads over table
            # 'motion_pt8': [ 33.27, -75.79,  95.34, -212.50, -95.39, 179.87],  # Carry beads over table
            'scoop': [-15,-61.45,126.78,-236.87,-98.61,171.68],  # Scoop beads
            'scoop_done_safe': [-22.98,-61.43,132.58,-247.18,-96.47,254.43],  # Safe hover after scoop
            'work_safe_2': [84.32, -40.01, 44.64, -194.52, -92.74, 282.35],  # Hover above table
            'dump_pre_table': [75.80, -51.39, 86.01, -214.61, -63.46, 282.35],  # Hover above table ready to dump
            'dumpping_balls': [55.75,-45.37,80.79,-215.68,-83.03,179.10],#[76.16, -50.42, 86.06, -216.78, -63.44, 190.56],  # Dump beads over table
            'dump_done': [36.82, -91.00, 86.03, -196.63, -72.45, 181.56],  # Safe hover after dump
            # 'motion_pt4': [ 72.91, -63.50, 101.99, -217.47, -92.74,  98.49],  # Dump beads
        }

        # ── Constant Z heights used for IK targets ─────────────────────
        self.ur1_const_z = 0.76  # Robot 1 mop tip height above floor (metres)
        self.ur2_const_z = 1.0  # Robot 2 tool height above floor (metres)

        # ── Live joint state cache (populated by joint_cb) ─────────────
        self.current_joints_1 = None  # Latest joint positions for Robot 1
        self.current_joints_2 = None  # Latest joint positions for Robot 2

        # Threading event used to block the main thread until Robot 2's
        # action server reports the trajectory is complete
        self.ur2_done_event = threading.Event()

    # ──────────────────────────────────────────────────────────────────
    # Helper Methods
    # ──────────────────────────────────────────────────────────────────

    def deg_to_rad(self, degrees_list):
        """
        Convert a list of joint angles from degrees to radians.

        Parameters
        ----------
        degrees_list : list[float]
            Joint angles expressed in degrees.

        Returns
        -------
        list[float]
            The same angles expressed in radians.
        """
        return [math.radians(d) for d in degrees_list]  # Apply math.radians to every element

    def joint_cb(self, msg):
        """
        Callback for the /joint_states topic.

        Extracts and caches the current joint positions for each robot
        by looking up each expected joint name in the incoming message.
        Silently ignores messages that do not contain the expected joints
        (e.g. during startup before all controllers are active).

        Parameters
        ----------
        msg : sensor_msgs.msg.JointState
            The incoming joint state message containing name/position arrays
            for all joints currently published on /joint_states.
        """
        try:
            # Build Robot 1 joint list by finding each name's index in the message
            self.current_joints_1 = [
                msg.position[msg.name.index(n)] for n in self.ur1_joints
            ]
        except (ValueError, IndexError):
            # Joint not yet present in the message — keep previous value
            pass

        try:
            # Build Robot 2 joint list by finding each name's index in the message
            self.current_joints_2 = [
                msg.position[msg.name.index(n)] for n in self.ur2_joints
            ]
        except (ValueError, IndexError):
            # Joint not yet present in the message — keep previous value
            pass

    def verify_at_rest(self, robot_id):
        """
        Block the calling thread until the specified robot reaches its
        predefined rest position within a small angular tolerance.

        This is used as a hard synchronisation gate — Robot 2 will not
        start until this function returns for Robot 1.

        Parameters
        ----------
        robot_id : int
            1 for Robot 1, 2 for Robot 2.
        """
        # Select the correct target rest pose for this robot
        rest = self.rest_pose_r1 if robot_id == 1 else self.rest_pose_r2

        print(f"Waiting for Robot {robot_id} to reach REST position...")

        tolerance = 0.01  # Acceptable angular error per joint (radians ≈ 0.57°)

        while rclpy.ok():  # Keep checking as long as ROS 2 is running
            # Select the correct live joint cache
            current = self.current_joints_1 if robot_id == 1 else self.current_joints_2

            # Check every joint is within tolerance of its rest value
            if current and all(abs(j - k) < tolerance for j, k in zip(current, rest)):
                print(f"--- CONFIRMED: ROBOT {robot_id} IS AT REST POSITION ---")
                break  # Exit the loop once rest is confirmed

            time.sleep(0.2)  # Poll at 5 Hz to avoid busy-waiting

    def _reorder_positions(self, target_names, source_names, source_positions):
        """
        Reorder a list of joint positions to match a target joint ordering.

        MoveIt's IK service returns joints in an arbitrary order that may
        differ from the order expected by the hardware controller.  This
        method maps the IK result into the correct controller order.

        Parameters
        ----------
        target_names : list[str]
            Joint names in the order required by the controller.
        source_names : list[str]
            Joint names as returned by the IK service.
        source_positions : list[float]
            Joint positions corresponding to source_names.

        Returns
        -------
        list[float]
            Joint positions reordered to match target_names.

        Raises
        ------
        KeyError
            If a joint in target_names is not present in source_names.
        """
        # Build a name → position lookup from the IK result
        src = dict(zip(source_names, source_positions))

        # Return positions in the order demanded by the controller
        return [src[name] for name in target_names]

    def is_at_pose(self, target_pose, robot_id, tolerance=0.01):
        """
        Check whether a robot is currently within tolerance of a target pose.

        Parameters
        ----------
        target_pose : list[float]
            Target joint angles in radians to compare against.
        robot_id : int
            1 for Robot 1, 2 for Robot 2.
        tolerance : float, optional
            Maximum allowed angular error per joint in radians. Defaults to 0.01 (~0.57°).

        Returns
        -------
        bool
            True if all joints are within tolerance of the target, False otherwise.
        """
        # Fetch the correct live joint cache for this robot
        current = self.current_joints_1 if robot_id == 1 else self.current_joints_2

        # Cannot verify pose if no joint state has been received yet
        if current is None:
            return False

        # All joints must be within tolerance for the pose to be confirmed
        return all(abs(j - k) < tolerance for j, k in zip(current, target_pose))

    def move_to_home_verified(self, robot_id, max_attempts=3, timeout=10.0):
        """
        Send Robot 1 to its home pose and block until the pose is physically
        confirmed via joint state feedback. Retries up to ``max_attempts``
        times if the robot does not reach the home pose within ``timeout``
        seconds per attempt.

        This guards against the race condition where the trajectory publisher
        fires before the controller is ready to accept commands, causing the
        first move to be silently dropped.

        Parameters
        ----------
        robot_id : int
            1 for Robot 1, 2 for Robot 2.
        max_attempts : int, optional
            Number of times to resend the home command before giving up.
            Defaults to 3.
        timeout : float, optional
            Seconds to wait for the robot to reach home on each attempt.
            Defaults to 10.0 s.

        Returns
        -------
        bool
            True if home pose was confirmed, False if all attempts failed.
        """
        # Select the correct home pose for this robot
        home_pose = self.home_pose_r1 if robot_id == 1 else self.home_pose_r2

        for attempt in range(1, max_attempts + 1):
            print(f"[HOME] Sending Robot {robot_id} to home pose "
                  f"(attempt {attempt}/{max_attempts})...")

            # Publish the home trajectory — 4 second move duration
            self.move_to_joints(home_pose, robot_id=robot_id, duration=4.0)

            # Poll joint states until home is confirmed or timeout is exceeded
            deadline = time.time() + timeout
            while time.time() < deadline:
                if self.is_at_pose(home_pose, robot_id):
                    print(f"[HOME] ✓ Robot {robot_id} confirmed at home pose.")
                    return True  # Pose verified — safe to proceed
                time.sleep(0.1)  # Poll at 10 Hz

            # Timeout elapsed without confirmation — will retry if attempts remain
            print(f"[HOME] ✗ Robot {robot_id} did NOT reach home within "
                  f"{timeout}s — retrying...")

        # All attempts exhausted without confirmation
        print(f"[HOME] ERROR: Robot {robot_id} failed to reach home pose after "
              f"{max_attempts} attempts. Aborting phase.")
        return False  # Caller should handle this failure

    # ──────────────────────────────────────────────────────────────────
    # Robot 1 — IK-based Motion
    # ──────────────────────────────────────────────────────────────────

    def get_ik(self, x, y, z, seed, robot_id=1):
        """
        Call the MoveIt /compute_ik service to solve inverse kinematics
        for a Cartesian target pose.

        The solver is seeded with the robot's current joint positions to
        encourage a solution close to the present configuration, which
        helps avoid large unexpected joint jumps.

        Parameters
        ----------
        x : float
            Target X position in the 'world' frame (metres).
        y : float
            Target Y position in the 'world' frame (metres).
        z : float
            Target Z position in the 'world' frame (metres).
        seed : list[float] or None
            Current joint angles (radians) used to seed the IK solver.
            If None the call is skipped and None is returned.
        robot_id : int, optional
            1 = Robot 1 (mop_tip link), 2 = Robot 2 (ur_2_tool0 link).
            Defaults to 1.

        Returns
        -------
        list[float] or None
            Joint angles (radians) in controller order on success,
            or None if the IK service timed out or returned an error.
        """
        # Cannot solve IK without a seed — bail out early
        if seed is None:
            return None

        # Construct the IK service request
        req = GetPositionIK.Request()
        req.ik_request.avoid_collisions = True  # Ask MoveIt to reject self-colliding solutions

        # Configure the request fields that differ between the two robots
        if robot_id == 1:
            req.ik_request.group_name = "ur_manipulator"  # MoveIt planning group for Robot 1
            req.ik_request.ik_link_name = "mop_tip"  # End-effector link for Robot 1
            req.ik_request.robot_state.joint_state.name = self.ur1_joints
            controller_joint_order = self.ur1_joints  # Order expected by Robot 1 controller
        else:
            req.ik_request.group_name = "ur_2_manipulator"  # MoveIt planning group for Robot 2
            req.ik_request.ik_link_name = "ur_2_tool0"  # End-effector link for Robot 2
            req.ik_request.robot_state.joint_state.name = self.ur2_joints
            controller_joint_order = self.ur2_joints  # Order expected by Robot 2 controller

        # Provide the seed joint positions to guide the IK solver
        req.ik_request.robot_state.joint_state.position = seed

        # Build the target Cartesian pose
        target = PoseStamped()
        target.header.frame_id = "world"  # All targets are expressed in the world frame
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = z
        angle = math.radians(70)  # 70° → radians
        # target.pose.orientation.x = math.sin(angle / 2)   # ≈ 0.5736
        # target.pose.orientation.y = 0.0
        # target.pose.orientation.z = 0.0
        # target.pose.orientation.w = math.cos(angle / 2)   # ≈ 0.8192
        target.pose.orientation.x = 0.97611  # 0.9988
        target.pose.orientation.y = -0.030738  # -0.03325
        target.pose.orientation.z = -0.016869  # -0.010995
        target.pose.orientation.w = 0.21443  # 0.03412
        req.ik_request.pose_stamped = target
        # 0.97611; -0.030738; -0.016869; 0.21443

        # Send the request asynchronously (non-blocking call)
        future = self.ik_client.call_async(req)

        # Poll until the future resolves or a 2-second timeout is exceeded
        start = time.time()
        while not future.done():
            time.sleep(0.01)  # 10 ms polling interval
            if time.time() - start > 2.0:
                self.get_logger().error("IK Service Timeout!")
                return None  # Give up — service is unresponsive

        # Inspect the result
        res = future.result()
        if res is None or res.error_code.val != MoveItErrorCodes.SUCCESS:
            # Log the failure with the target coordinates and MoveIt error code
            self.get_logger().error(
                f"IK failed for robot {robot_id} at ({x:.3f},{y:.3f},{z:.3f}) "
                f"— code: {res.error_code.val if res else 'None'}"
            )
            return None

        # Extract joint names and positions from the IK solution
        names = list(res.solution.joint_state.name)
        positions = list(res.solution.joint_state.position)

        try:
            # Reorder the IK solution to match the hardware controller's joint order
            return self._reorder_positions(controller_joint_order, names, positions)
        except KeyError as e:
            # A required joint was absent from the IK solution
            self.get_logger().error(f"IK result missing joint: {e}")
            return None

    def move_to_joints(self, joint_goal, robot_id=1, duration=3.0):
        """
        Publish a single-point JointTrajectory command to move a robot
        to a target joint configuration.

        The function blocks for ``duration + 0.1`` seconds after publishing
        so the caller can treat it as a synchronous move.

        Parameters
        ----------
        joint_goal : list[float]
            Target joint angles in radians.  Length must equal 6.
        robot_id : int, optional
            1 = Robot 1, 2 = Robot 2.  Defaults to 1.
        duration : float, optional
            Time (seconds) the controller should take to reach the goal.
            Defaults to 3.0 s.
        """
        # Select the correct joint name list for this robot
        joint_names = self.ur1_joints if robot_id == 1 else self.ur2_joints

        # Validate that we have the right number of joint values
        if len(joint_goal) != len(joint_names):
            self.get_logger().error(
                f"Joint goal length mismatch for robot {robot_id}.")
            return  # Abort — publishing a malformed message could damage hardware

        # Build the JointTrajectory message
        msg = JointTrajectory()
        msg.joint_names = joint_names  # Tell the controller which joints to move

        # Create a single trajectory point representing the final target
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in joint_goal]  # Target joint angles
        pt.time_from_start = Duration(seconds=duration).to_msg()  # Arrival time
        msg.points.append(pt)  # Only one point — move directly to goal

        # Select the correct publisher and send the message
        publisher = self.ur1_pub if robot_id == 1 else self.ur2_pub
        publisher.publish(msg)

        # Block until the motion should be complete (duration + small buffer)
        time.sleep(duration + 0.1)

    def run_robot1_phase(self):
        """
        Execute the full operational phase for Robot 1.

        Flow
        ----
        1. Move Robot 1 to its home pose and VERIFY it has arrived before
           proceeding (retries up to 3 times with a 10-second timeout each).
        2. Prompt the operator to choose between:
           - Option 1: Interactive mode — enter X/Y targets manually.
           - Option 2: CSV mode — read waypoints from motion_safe.csv,
             pre-solve all IK solutions ('bake' the trajectory), then
             execute the full trajectory as a single multi-point message.
        3. Return Robot 1 to its rest position and block until confirmed.
        """
        print("\n--- STARTING PHASE FOR ROBOT 1 ---")

        # ── Verified home move ─────────────────────────────────────────
        # Unlike a plain move_to_joints call, this blocks until the robot
        # is physically confirmed at the home pose before the IK phase starts.
        # If home cannot be confirmed after max_attempts, the phase is aborted
        # to prevent running the trajectory from an unknown start position.
        if not self.move_to_home_verified(robot_id=1):
            print("[ABORT] Robot 1 could not reach home. Skipping IK phase.")
            # Still return to rest so Robot 2 can start safely
            self.move_to_joints(self.rest_pose_r1, robot_id=1, duration=5.0)
            self.verify_at_rest(robot_id=1)
            return

        # ── Mode selection ─────────────────────────────────────────────
        print("1: Manual X/Y  |  2: CSV Sequence")
        choice = input("Selection for Robot 1: ").strip()

        if choice == '1':
            # ── Interactive mode ──────────────────────────────────────
            while rclpy.ok():
                inp = input("Robot 1 Target (x y) or 'exit': ").split()

                # Exit condition: empty input or the word 'exit'
                if not inp or inp[0].lower() == 'exit':
                    break

                try:
                    x, y = float(inp[0]), float(inp[1])  # Parse X and Y from user input

                    # Solve IK for this target at the constant mopping height
                    sol = self.get_ik(
                        x, y, self.ur1_const_z,
                        self.current_joints_1,  # Use current pose as IK seed
                        robot_id=1)

                    if sol:
                        print(f"IK Success! Moving to ({x}, {y})...")
                        self.move_to_joints(sol, robot_id=1)  # Execute the move
                    else:
                        print("IK Failed for these coordinates.")  # No valid solution found

                except (ValueError, IndexError):
                    print("Invalid input — enter two numbers.")  # Non-numeric input guard

        elif choice == '2':
            # ── CSV trajectory mode ───────────────────────────────────
            csv_path = '/mnt/sdc/GitHub/ros2/data/motion_safe.csv'

            if not os.path.exists(csv_path):
                print(f"Error: CSV not found at {csv_path}")
            else:
                points = []  # Accumulates valid IK solutions
                last_seed = self.current_joints_1  # Seed first IK with current joints

                print("Baking trajectory for Robot 1...")

                with open(csv_path, 'r') as f:
                    for row in csv.DictReader(f):  # Iterate over each CSV row
                        sol = self.get_ik(
                            float(row['x']),  # Target X from CSV
                            float(row['y']),  # Target Y from CSV
                            self.ur1_const_z,  # Constant mopping height
                            last_seed,  # Warm-start seed from previous solution
                            robot_id=1)

                        if sol:
                            points.append(sol)  # Store valid solution
                            last_seed = sol  # Use this solution as seed for next point
                        # Rows with no IK solution are silently skipped

                if points:
                    print(f"Executing {len(points)} waypoints...")

                    # Build a multi-point trajectory message for smooth execution
                    msg = JointTrajectory()
                    msg.joint_names = self.ur1_joints

                    for i, p in enumerate(points):
                        # Space waypoints 0.2 seconds apart
                        msg.points.append(JointTrajectoryPoint(
                            positions=p,
                            time_from_start=Duration(seconds=(i + 1) * 0.2).to_msg()))

                    self.ur1_pub.publish(msg)  # Send the full trajectory in one message

                    # Wait for the entire trajectory to finish executing
                    time.sleep(len(points) * 0.2 + 2.0)
                else:
                    print("No valid IK solutions found.")

        # ── Mandatory return to rest ───────────────────────────────────
        # Robot 1 MUST be at rest before Robot 2 is allowed to start
        print("Robot 1 action complete. Returning to REST position...")
        self.move_to_joints(self.rest_pose_r1, robot_id=1, duration=5.0)
        self.verify_at_rest(robot_id=1)  # Block until rest is physically confirmed

    # ──────────────────────────────────────────────────────────────────
    # Robot 2 — Fixed Trajectory Execution
    # ──────────────────────────────────────────────────────────────────

    def run_robot2_phase(self):
        """
        Execute the full operational phase for Robot 2.

        Robot 2 performs a pre-recorded scoop-and-dump task using hardcoded
        joint-space waypoints.  The trajectory is sent as a single
        FollowJointTrajectory action goal so the controller handles
        smooth interpolation between waypoints.

        Flow
        ----
        1. Wait for the FollowJointTrajectory action server to become available.
        2. Build a trajectory message from the ordered waypoint sequence,
           converting degrees to radians and assigning absolute time stamps.
        3. Send the goal and block until the action server reports completion.
        4. Return Robot 2 to its rest position and block until confirmed.
        """
        print("\n--- STARTING PHASE FOR ROBOT 2 (FIXED TRAJECTORY) ---")

        # Block until the ur_2_controller action server is online
        self.get_logger().info("Waiting for ur_2_controller action server...")
        self.ur2_action_client.wait_for_server()

        # Shorthand alias to keep the sequence definition readable
        t = self.ur2_fixed_trajectory

        # Ordered sequence of (label, joint_angles_deg) pairs.
        # The order defines the physical motion — do not reorder without
        # verifying collision-free paths on the real robot.
        sequence = [
            ('home_rest', t['home_rest']),  # 1. Move to safe home position
            ('home_rest_safe', t['home_rest_safe']),  # 2. Move to safe home position
            ('work_safe', t['work_safe']),  # 3. Hover above the table surface
            ('work_point', t['work_point']),  # 4. Lower end-effector to table
            ('motion_pt1', t['motion_pt1']),  # 5. Sweep right to gather beads
            ('motion_pt2', t['motion_pt2']),  # 6. Sweep left to gather beads
            ('hover_over_box', t['hover_over_box']),  # 7. Hover over collection box
            ('hover_over_box_2', t['hover_over_box_2']),  # 8. Hover over box just before scoop
            ('motion_pt6', t['motion_pt6']),  # 9. Begin scoop approach
            ('scoop', t['scoop']),  # 10. Scoop beads into end-effector
            ('scoop_done_safe', t['scoop_done_safe']),  # 11. Safe hover after scoop
            ('home_rest_safe', t['home_rest_safe']),  # 12. Return to safe home position after scoop

            ('dump_pre_table', t['dump_pre_table']),  # 13. Hover above table ready to dump
            ('dumpping_balls', t['dumpping_balls']),  # 14. Dump beads over table
            ('dump_done', t['dump_done']),  # 15. Safe hover after dump
            ('home_rest_safe', t['home_rest_safe']),  # 16. Return to safe home position after dump
            ('home_rest', t['home_rest']),  # 17. Return to safe home position
        ]

        # Absolute time stamps (seconds from trajectory start) for each waypoint.
        # 5-second spacing gives the controller sufficient time between poses.
        # time_steps = [5.0, 10.0, 15.0, 20.0, 25.0, 30.0, 35.0, 40.0, 45.0, 50.0, 55.0]
        time_steps = [(i + 1) * 5.0 for i in range(len(sequence))]  # [5.0, 10.0, ..., 80.0]

        # Create the FollowJointTrajectory goal message
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.ur2_joints  # Tell controller which joints to move

        # Populate trajectory points by pairing each waypoint with its time stamp
        for (label, deg), t_sec in zip(sequence, time_steps):
            pt = JointTrajectoryPoint()
            pt.positions = self.deg_to_rad(deg)  # Convert degrees → radians for the controller

            # Build a BuiltinDuration from the float time stamp
            d = BuiltinDuration()
            d.sec = int(t_sec)  # Whole seconds
            d.nanosec = int((t_sec - int(t_sec)) * 1e9)  # Fractional seconds as nanoseconds
            pt.time_from_start = d

            goal_msg.trajectory.points.append(pt)
            print(f"  Waypoint '{label}' @ t={t_sec}s")  # Progress feedback

        print("Sending fixed trajectory to Robot 2...")

        # Send the goal asynchronously — returns a future for the goal handle
        send_future = self.ur2_action_client.send_goal_async(goal_msg)

        # Block (spin-wait) until the action server accepts or rejects the goal
        while not send_future.done():
            time.sleep(0.01)  # 10 ms polling interval

        goal_handle = send_future.result()

        if not goal_handle.accepted:
            # Goal was rejected — log error and abort without crashing
            self.get_logger().error("Robot 2 trajectory REJECTED by controller!")
            return

        self.get_logger().info("Robot 2 trajectory accepted — executing...")

        # Request the final result asynchronously and register our callback
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._ur2_result_callback)

        # Block the main thread here until _ur2_result_callback sets the event
        self.ur2_done_event.wait()
        self.ur2_done_event.clear()  # Reset the event for potential future use

        # Return Robot 2 to rest position after the task is complete
        print("Robot 2 trajectory complete. Returning to REST position...")
        self.move_to_joints(self.rest_pose_r2, robot_id=2, duration=5.0)
        self.verify_at_rest(robot_id=2)  # Block until rest is physically confirmed

    def _ur2_result_callback(self, future):
        """
        Called by the ROS 2 action client when Robot 2's trajectory
        execution finishes (success or failure).

        Logs the MoveIt error code and unblocks ``run_robot2_phase``
        by setting the ``ur2_done_event`` threading event.

        Parameters
        ----------
        future : rclpy.task.Future
            The completed future wrapping the FollowJointTrajectory result.
        """
        result = future.result().result  # Unwrap the action result object

        # Log the completion status for operator visibility
        self.get_logger().info(
            f"Robot 2 motion finished — error code: {result.error_code}")

        # Signal run_robot2_phase that it can continue past the wait() call
        self.ur2_done_event.set()


# ──────────────────────────────────────────────────────────────────────
# Entry Point
# ──────────────────────────────────────────────────────────────────────

def main():
    """
    Program entry point.

    Initialises ROS 2, starts the DualMopControlSystem node, and runs
    the relay sequence:

    1. Spin rclpy in a background daemon thread so that topic callbacks
       and service responses are processed without blocking the main thread.
    2. Wait until live joint states are available for both robots.
    3. Run Robot 1's IK-based mopping phase (blocks until done + at rest).
    4. Apply a 6-second cooldown to ensure hardware has settled.
    5. Run Robot 2's fixed scoop-and-dump phase (blocks until done + at rest).
    6. Shut down ROS 2 cleanly.
    """
    rclpy.init()  # Initialise the ROS 2 Python client library

    node = DualMopControlSystem()  # Create and configure the control node

    # Launch rclpy.spin in a background thread so callbacks are processed
    # while the main thread runs the blocking motion logic
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # Busy-wait until /joint_states has populated both robot caches
    print("Waiting for joint states from both robots...")
    while node.current_joints_1 is None or node.current_joints_2 is None:
        time.sleep(0.1)  # Poll at 10 Hz — avoids hammering the CPU
    print("Joint states received. Starting relay sequence.\n")

    # ── STEP 1: Robot 1 mopping phase ─────────────────────────────────
    # Blocks until Robot 1 is confirmed back at its rest position
    node.run_robot1_phase()

    # ── STEP 2: Safety cooldown ────────────────────────────────────────
    # Gives the hardware time to settle and ensures no residual motion
    print("Robot 1 confirmed at rest. Cooling down 6 seconds before Robot 2...")
    time.sleep(6.0)

    # ── STEP 3: Robot 2 scoop-and-dump phase ──────────────────────────
    # Blocks until Robot 2 is confirmed back at its rest position
    node.run_robot2_phase()

    print("\nRelay sequence complete. Both robots at rest.")

    rclpy.shutdown()  # Signal ROS 2 to stop all nodes and executors
    spin_thread.join()  # Wait for the background spin thread to exit cleanly


if __name__ == '__main__':
    main()  # Run main() when the script is executed directly