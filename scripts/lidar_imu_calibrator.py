#!/usr/bin/env python3
"""
lidar_imu_calibrator.py

Estimates extrinsic_R (LiDAR-to-IMU rotation) by comparing:
  - angular velocity derived from consecutive SLAM poses  (ω_slam)
  - raw IMU gyroscope readings                            (ω_imu)

Method: SVD / Kabsch
  minimise  Σ || ω_imu - R_ext @ ω_slam ||²
  H = Σ ω_imu ω_slam^T
  U,S,Vt = SVD(H)
  R_ext = U diag(1,1,det(U Vt)) Vt

Results are printed to console AND written to:
  <package_root>/Log/lidar_imu_calib_<timestamp>.txt

ROS params (all optional):
  ~slam_topic        default: /aft_mapped_to_init
  ~imu_topic         default: /minithex/imu
  ~min_pairs         default: 100
  ~omega_min_norm    default: 0.05   rad/s  (skip near-static intervals)
  ~calibrate_period  default: 10.0   s      (re-estimate interval)
  ~max_dt_slam       default: 0.5    s      (discard long SLAM gaps)
  ~max_dt_imu_match  default: 0.05   s      (max sync lag to IMU sample)
  ~log_dir           default: <pkg>/Log/
"""

import os
import threading
from collections import deque
from datetime import datetime

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation as Rot
from sensor_msgs.msg import Imu


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _default_log_dir() -> str:
    """Derive Log/ path from the script's own location — no rospkg needed."""
    script_dir = os.path.dirname(os.path.realpath(__file__))
    log_dir = os.path.join(script_dir, '..', 'Log')
    return os.path.normpath(log_dir)


def quat_to_R(q) -> np.ndarray:
    return Rot.from_quat([q.x, q.y, q.z, q.w]).as_matrix()


def omega_from_rots(R1: np.ndarray, R2: np.ndarray, dt: float) -> np.ndarray:
    """Body-frame angular velocity: ω = Log(R1^T R2) / dt"""
    return Rot.from_matrix(R1.T @ R2).as_rotvec() / dt


def kabsch_rotation(A_list, B_list) -> np.ndarray:
    """
    Find R = argmin Σ||B_i - R A_i||²  via SVD.
    A_list: list of 3-vecs (omega_slam)
    B_list: list of 3-vecs (omega_imu)
    Returns valid SO(3) matrix.
    """
    H = sum(np.outer(b, a) for a, b in zip(A_list, B_list))
    U, _, Vt = np.linalg.svd(H)
    R = U @ np.diag([1.0, 1.0, np.linalg.det(U @ Vt)]) @ Vt
    return R


# ---------------------------------------------------------------------------
# Node
# ---------------------------------------------------------------------------

class LidarImuCalibrator:

    def __init__(self):
        # ---- params ----
        self.slam_topic       = rospy.get_param('~slam_topic',       '/aft_mapped_to_init')
        self.imu_topic        = rospy.get_param('~imu_topic',        '/minithex/imu')
        self.min_pairs        = int(rospy.get_param('~min_pairs',        100))
        self.omega_min_norm   = float(rospy.get_param('~omega_min_norm',   0.05))
        self.calib_period     = float(rospy.get_param('~calibrate_period', 10.0))
        self.max_dt_slam      = float(rospy.get_param('~max_dt_slam',      0.5))
        self.max_dt_imu_match = float(rospy.get_param('~max_dt_imu_match', 0.05))

        # ---- log dir: use ~log_dir param, then script-relative fallback ----
        fallback = _default_log_dir()
        self.log_dir = rospy.get_param('~log_dir', fallback)
        try:
            os.makedirs(self.log_dir, exist_ok=True)
            rospy.loginfo("[lidar_imu_calibrator] Saving results to: %s", self.log_dir)
        except OSError as e:
            rospy.logwarn("[lidar_imu_calibrator] Cannot create log_dir %s: %s — "
                          "results will only print to console.", self.log_dir, e)
            self.log_dir = None

        # ---- state ----
        self._lock        = threading.Lock()
        self._slam_buf    = deque(maxlen=200)   # (stamp, R)
        self._imu_buf     = deque(maxlen=2000)  # (stamp, omega)
        self._omega_slam  = []
        self._omega_imu   = []
        self._run_count   = 0

        # ---- subscriptions ----
        rospy.Subscriber(self.slam_topic, Odometry, self._slam_cb, queue_size=200)
        rospy.Subscriber(self.imu_topic,  Imu,      self._imu_cb,  queue_size=2000)

        rospy.Timer(rospy.Duration(self.calib_period), self._calibrate_cb)

        rospy.loginfo(
            "[lidar_imu_calibrator] Started.\n"
            "  SLAM : %s\n  IMU  : %s\n"
            "  Need %d pairs, estimating every %.0f s.",
            self.slam_topic, self.imu_topic, self.min_pairs, self.calib_period)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _imu_cb(self, msg: Imu):
        t     = msg.header.stamp.to_sec()
        omega = np.array([msg.angular_velocity.x,
                          msg.angular_velocity.y,
                          msg.angular_velocity.z])
        with self._lock:
            self._imu_buf.append((t, omega))

    def _slam_cb(self, msg: Odometry):
        t = msg.header.stamp.to_sec()
        R = quat_to_R(msg.pose.pose.orientation)

        with self._lock:
            self._slam_buf.append((t, R))
            if len(self._slam_buf) < 2:
                return

            t1, R1 = self._slam_buf[-2]
            t2, R2 = self._slam_buf[-1]
            dt = t2 - t1

            if dt <= 0.0 or dt > self.max_dt_slam:
                return

            w_slam = omega_from_rots(R1, R2, dt)
            if np.linalg.norm(w_slam) < self.omega_min_norm:
                return

            t_mid = 0.5 * (t1 + t2)
            w_imu, gap = self._nearest_imu_locked(t_mid)
            if w_imu is None or gap > self.max_dt_imu_match:
                return

            self._omega_slam.append(w_slam)
            self._omega_imu.append(w_imu)

    def _nearest_imu_locked(self, t_q):
        best, best_gap = None, float('inf')
        for t, w in self._imu_buf:
            g = abs(t - t_q)
            if g < best_gap:
                best_gap = g
                best = w.copy()
        return best, best_gap

    # ------------------------------------------------------------------
    # Calibration
    # ------------------------------------------------------------------

    def _calibrate_cb(self, _event):
        with self._lock:
            n      = len(self._omega_slam)
            w_slam = list(self._omega_slam)
            w_imu  = list(self._omega_imu)

        rospy.loginfo("[lidar_imu_calibrator] Pairs: %d / %d", n, self.min_pairs)
        if n < self.min_pairs:
            return

        R_ext = kabsch_rotation(w_slam, w_imu)

        det      = np.linalg.det(R_ext)
        residuals = [np.linalg.norm(wi - R_ext @ ws) for ws, wi in zip(w_slam, w_imu)]
        mean_res = float(np.mean(residuals))
        std_res  = float(np.std(residuals))
        angle_deg = float(np.degrees(Rot.from_matrix(R_ext).magnitude()))

        self._run_count += 1
        self._print_result(R_ext, n, det, angle_deg, mean_res, std_res)
        self._save_result(R_ext, n, det, angle_deg, mean_res, std_res)

    def _print_result(self, R, n, det, angle_deg, mean_res, std_res):
        sep = "=" * 64
        rospy.loginfo(sep)
        rospy.loginfo("[lidar_imu_calibrator]  LIDAR-IMU EXTRINSIC ROTATION")
        rospy.loginfo(sep)
        rospy.loginfo("  Pairs            : %d", n)
        rospy.loginfo("  det(R)           : %.6f  (ideal +1)", det)
        rospy.loginfo("  Angle magnitude  : %.3f deg", angle_deg)
        rospy.loginfo("  Mean residual    : %.4f rad/s", mean_res)
        rospy.loginfo("  Std  residual    : %.4f rad/s", std_res)
        rospy.loginfo("")
        rospy.loginfo("  extrinsic_R (row-major):")
        for row in R:
            rospy.loginfo("    [%10.6f, %10.6f, %10.6f]", *row)
        rospy.loginfo("")
        rospy.loginfo("  -- YAML (robosense_airy.yaml / sim.yaml) --")
        rospy.loginfo("  extrinsic_R: [%.6f, %.6f, %.6f,", R[0,0], R[0,1], R[0,2])
        rospy.loginfo("               %.6f, %.6f, %.6f,", R[1,0], R[1,1], R[1,2])
        rospy.loginfo("               %.6f, %.6f, %.6f]", R[2,0], R[2,1], R[2,2])
        rospy.loginfo(sep)

    def _save_result(self, R, n, det, angle_deg, mean_res, std_res):
        if self.log_dir is None:
            return

        ts       = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = os.path.join(self.log_dir,
                                f'lidar_imu_calib_{ts}_{self._run_count:03d}.txt')

        content = "\n".join([
            "# lidar_imu_calibrator — extrinsic rotation result",
            f"# Generated  : {datetime.now().isoformat()}",
            f"# Pairs used : {n}",
            f"# det(R)     : {det:.6f}",
            f"# Angle mag  : {angle_deg:.3f} deg",
            f"# Mean resid : {mean_res:.4f} rad/s",
            f"# Std  resid : {std_res:.4f} rad/s",
            "",
            "# --- paste into your YAML config ---",
            "extrin_calib:",
            f"  extrinsic_R: [{R[0,0]:.6f}, {R[0,1]:.6f}, {R[0,2]:.6f},",
            f"               {R[1,0]:.6f}, {R[1,1]:.6f}, {R[1,2]:.6f},",
            f"               {R[2,0]:.6f}, {R[2,1]:.6f}, {R[2,2]:.6f}]",
            "",
            "# --- full precision rows ---",
            f"# R[0] = [{R[0,0]:.10f}, {R[0,1]:.10f}, {R[0,2]:.10f}]",
            f"# R[1] = [{R[1,0]:.10f}, {R[1,1]:.10f}, {R[1,2]:.10f}]",
            f"# R[2] = [{R[2,0]:.10f}, {R[2,1]:.10f}, {R[2,2]:.10f}]",
        ]) + "\n"

        try:
            with open(filename, 'w') as f:
                f.write(content)
            rospy.loginfo("[lidar_imu_calibrator] Saved: %s", filename)
        except OSError as e:
            rospy.logerr("[lidar_imu_calibrator] Failed to save: %s", e)


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

if __name__ == '__main__':
    rospy.init_node('lidar_imu_calibrator', anonymous=False)
    LidarImuCalibrator()
    rospy.spin()
