#!/usr/bin/env python3
"""
slam_evaluator.py

Estimates the extrinsic rotation R between the LiDAR/SLAM body frame and the
raw IMU sensor frame by comparing angular velocities from two sources:

  - slam/state  (nav_msgs/Odometry) : SLAM-estimated body pose. Consecutive
                 poses are numerically differentiated to get ω_slam in the
                 body frame.
  - rotorcraft/imu (sensor_msgs/Imu) : raw gyroscope readings ω_imu in the
                 IMU sensor frame.

If extrinsic_R is perfectly calibrated the two angular velocities should be
identical (they live in the same frame). A systematic rotation between them
indicates that extrinsic_R in the YAML is wrong.

The calibration uses the SVD-based rotation estimator (Kabsch algorithm):
  minimise  Σ || ω_imu - R_ext @ ω_slam ||²
  solution: H = Σ ω_imu ω_slam^T,  SVD(H) = U S V^T
            R_ext = U diag(1,1,det(UV^T)) V^T

Usage (standalone):
  rosrun fast_livo slam_evaluator.py

ROS params (all optional):
  ~slam_topic        default: slam/state
  ~imu_topic         default: rotorcraft/imu
  ~min_pairs         default: 300   (pairs before first estimate is printed)
  ~omega_min_norm    default: 0.05  (rad/s, skip near-zero motion)
  ~calibrate_period  default: 10.0  (seconds between re-estimation)
  ~max_dt_slam       default: 0.5   (s, discard SLAM intervals longer than this)
  ~max_dt_imu_match  default: 0.05  (s, max lag allowed when syncing IMU sample)
"""

import os
import threading
from collections import deque
from datetime import datetime

import numpy as np
import rospy
import rospkg
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as Rot
from sensor_msgs.msg import Imu


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def quat_to_R(q) -> np.ndarray:
    """Convert geometry_msgs/Quaternion to 3x3 rotation matrix."""
    return Rot.from_quat([q.x, q.y, q.z, q.w]).as_matrix()


def angular_velocity_from_rots(R1: np.ndarray, R2: np.ndarray, dt: float) -> np.ndarray:
    """
    Body-frame angular velocity (rad/s) between two consecutive rotations.
    ω = Log(R1^T R2) / dt
    """
    dR = R1.T @ R2
    rotvec = Rot.from_matrix(dR).as_rotvec()  # axis-angle in body frame
    return rotvec / dt


def estimate_rotation_svd(omega_slam_list, omega_imu_list) -> np.ndarray:
    """
    Kabsch / SVD rotation estimator.
    Finds R_ext = argmin Σ || ω_imu - R_ext @ ω_slam ||²

    Returns a valid SO(3) matrix.
    """
    H = np.zeros((3, 3))
    for os, oi in zip(omega_slam_list, omega_imu_list):
        H += np.outer(oi, os)          # 3x3 cross-covariance

    U, _, Vt = np.linalg.svd(H)
    # Ensure proper rotation (det = +1, not reflection)
    sign_mat = np.diag([1.0, 1.0, np.linalg.det(U @ Vt)])
    R_ext = U @ sign_mat @ Vt
    return R_ext


# ---------------------------------------------------------------------------
# Main node class
# ---------------------------------------------------------------------------

class SlamEvaluator:

    def __init__(self):
        # ---- ROS params ----
        self.slam_topic       = rospy.get_param('~slam_topic',       'slam/state')
        self.imu_topic        = rospy.get_param('~imu_topic',        'rotorcraft/imu')
        self.min_pairs        = int(rospy.get_param('~min_pairs',        300))
        self.omega_min_norm   = float(rospy.get_param('~omega_min_norm',   0.05))
        self.calib_period     = float(rospy.get_param('~calibrate_period', 10.0))
        self.max_dt_slam      = float(rospy.get_param('~max_dt_slam',      0.5))
        self.max_dt_imu_match = float(rospy.get_param('~max_dt_imu_match', 0.05))

        # ---- Output directory ----
        pkg_path = rospkg.RosPack().get_path('fast_livo')
        default_log_dir = os.path.join(pkg_path, 'Log')
        self.log_dir = rospy.get_param('~log_dir', default_log_dir)
        os.makedirs(self.log_dir, exist_ok=True)

        # ---- Internal state ----
        self._lock = threading.Lock()
        self._calib_count = 0  # how many times calibration has run

        # Ring buffers for incoming data
        self._slam_buf: deque = deque(maxlen=200)   # (stamp, R)
        self._imu_buf:  deque = deque(maxlen=2000)  # (stamp, omega 3-vec)

        # Accumulated (ω_slam, ω_imu) pairs ready for estimation
        self._omega_slam: list = []
        self._omega_imu:  list = []

        # ---- Subscribers ----
        rospy.Subscriber(self.slam_topic, Odometry,
                         self._slam_cb, queue_size=200)
        rospy.Subscriber(self.imu_topic, Imu,
                         self._imu_cb,  queue_size=2000)

        # ---- Periodic calibration ----
        rospy.Timer(rospy.Duration(self.calib_period), self._calibrate_cb)

        rospy.loginfo(
            "[slam_evaluator] Listening on:\n"
            "  SLAM: %s\n"
            "  IMU : %s\n"
            "Will run calibration every %.1f s (needs %d pairs).",
            self.slam_topic, self.imu_topic,
            self.calib_period, self.min_pairs
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _imu_cb(self, msg: Imu):
        t = msg.header.stamp.to_sec()
        omega = np.array([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        ])
        with self._lock:
            self._imu_buf.append((t, omega))

    def _slam_cb(self, msg: Odometry):
        t = msg.header.stamp.to_sec()
        R = quat_to_R(msg.pose.pose.orientation)

        with self._lock:
            self._slam_buf.append((t, R))

            # Try to form a pair from the last two SLAM poses
            if len(self._slam_buf) < 2:
                return

            t1, R1 = self._slam_buf[-2]
            t2, R2 = self._slam_buf[-1]
            dt = t2 - t1

            if dt <= 0.0 or dt > self.max_dt_slam:
                return

            # Angular velocity in body frame derived from SLAM
            omega_slam = angular_velocity_from_rots(R1, R2, dt)
            if np.linalg.norm(omega_slam) < self.omega_min_norm:
                return   # skip near-zero rotation (noisy & uninformative)

            # Find the IMU sample closest to the midpoint of the SLAM interval
            t_mid = 0.5 * (t1 + t2)
            best_omega, best_gap = self._nearest_imu(t_mid)
            if best_omega is None or best_gap > self.max_dt_imu_match:
                return   # no close-enough IMU sample

            self._omega_slam.append(omega_slam)
            self._omega_imu.append(best_omega)

    def _nearest_imu(self, t_query: float):
        """Return (omega, dt) of the IMU sample nearest to t_query.
        Must be called with self._lock held."""
        best_omega = None
        best_dt    = float('inf')
        for t, omega in self._imu_buf:
            gap = abs(t - t_query)
            if gap < best_dt:
                best_dt    = gap
                best_omega = omega.copy()
        return best_omega, best_dt

    # ------------------------------------------------------------------
    # Calibration
    # ------------------------------------------------------------------

    def _calibrate_cb(self, _event):
        with self._lock:
            n              = len(self._omega_slam)
            omega_slam_arr = list(self._omega_slam)
            omega_imu_arr  = list(self._omega_imu)

        rospy.loginfo("[slam_evaluator] Pairs collected so far: %d / %d",
                      n, self.min_pairs)

        if n < self.min_pairs:
            return

        R_ext = estimate_rotation_svd(omega_slam_arr, omega_imu_arr)

        # Sanity metrics
        det = np.linalg.det(R_ext)
        residuals = [
            np.linalg.norm(oi - R_ext @ os)
            for os, oi in zip(omega_slam_arr, omega_imu_arr)
        ]
        mean_res = float(np.mean(residuals))
        std_res  = float(np.std(residuals))

        # Axis-angle representation of estimated rotation
        angle_deg = np.degrees(
            Rot.from_matrix(R_ext).magnitude()
        )

        # ---- Print results ----
        sep = "=" * 62
        rospy.loginfo(sep)
        rospy.loginfo("[slam_evaluator]  EXTRINSIC ROTATION CALIBRATION RESULT")
        rospy.loginfo(sep)
        rospy.loginfo("  Pairs used          : %d", n)
        rospy.loginfo("  det(R_ext)          : %.6f  (ideal = +1.000000)", det)
        rospy.loginfo("  Rotation magnitude  : %.3f deg", angle_deg)
        rospy.loginfo("  Mean residual       : %.4f rad/s", mean_res)
        rospy.loginfo("  Std  residual       : %.4f rad/s", std_res)
        rospy.loginfo("")
        rospy.loginfo("  Estimated R_ext (row-major, 3x3):")
        for row in R_ext:
            rospy.loginfo("    [%10.6f, %10.6f, %10.6f]",
                          row[0], row[1], row[2])
        rospy.loginfo("")
        rospy.loginfo("  --- YAML snippet (paste into robosense_airy.yaml) ---")
        rospy.loginfo(
            "  extrinsic_R: [%.6f, %.6f, %.6f,\n"
            "               %.6f, %.6f, %.6f,\n"
            "               %.6f, %.6f, %.6f]",
            R_ext[0, 0], R_ext[0, 1], R_ext[0, 2],
            R_ext[1, 0], R_ext[1, 1], R_ext[1, 2],
            R_ext[2, 0], R_ext[2, 1], R_ext[2, 2],
        )
        rospy.loginfo(sep)

        # ---- Save to file ----
        self._calib_count += 1
        self._save_result(R_ext, n, det, angle_deg, mean_res, std_res)

    def _save_result(self, R_ext, n_pairs, det, angle_deg, mean_res, std_res):
        """Write a timestamped summary + YAML snippet to Log/."""
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = os.path.join(self.log_dir,
                                f'extrin_calib_{ts}_{self._calib_count:03d}.txt')

        lines = [
            "# slam_evaluator — extrinsic rotation calibration",
            f"# Generated : {datetime.now().isoformat()}",
            f"# Pairs used: {n_pairs}",
            f"# det(R_ext): {det:.6f}  (ideal = +1.0)",
            f"# Rotation magnitude: {angle_deg:.3f} deg",
            f"# Mean residual: {mean_res:.4f} rad/s",
            f"# Std  residual: {std_res:.4f} rad/s",
            "",
            "# --- paste into your YAML config ---",
            "extrin_calib:",
            f"  extrinsic_R: [{R_ext[0,0]:.6f}, {R_ext[0,1]:.6f}, {R_ext[0,2]:.6f},",
            f"               {R_ext[1,0]:.6f}, {R_ext[1,1]:.6f}, {R_ext[1,2]:.6f},",
            f"               {R_ext[2,0]:.6f}, {R_ext[2,1]:.6f}, {R_ext[2,2]:.6f}]",
            "",
            "# --- raw matrix rows ---",
        ]
        for i, row in enumerate(R_ext):
            lines.append(f"# R[{i}] = [{row[0]:.8f}, {row[1]:.8f}, {row[2]:.8f}]")

        with open(filename, 'w') as f:
            f.write('\n'.join(lines) + '\n')

        rospy.loginfo("[slam_evaluator] Result saved to: %s", filename)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    rospy.init_node('slam_evaluator', anonymous=False)
    node = SlamEvaluator()
    rospy.spin()
