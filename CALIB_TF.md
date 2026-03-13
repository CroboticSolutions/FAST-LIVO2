# Coordinate Frame & Calibration Documentation
## Robosense Airy on Minithex

---

## 1. Extrinsic Calibration in `config/robosense_airy.yaml`

### `extrinsic_T` and `extrinsic_R`

```yaml
extrinsic_T: [ 0.004156, 0.004265, -0.004468 ]
extrinsic_R: [-0.007468, -0.999971,  0.001664,
              -0.999966, -0.007474, -0.003502,
               0.003515, -0.001637, -0.999992]
```

These are the **internal hardware calibration** of the Robosense Airy sensor —
the fixed geometric relationship between the LiDAR scan origin and the internal
IMU chip, both physically located inside the sensor housing.

| Parameter | Meaning |
|---|---|
| `extrinsic_T` | Position of the LiDAR scan origin **expressed in the IMU frame** (metres) |
| `extrinsic_R` | 3×3 rotation matrix (row-major) that rotates a vector **from LiDAR frame into IMU frame** |

In code (`src/IMU_Processing.cpp`):
```cpp
Lid_offset_to_IMU = extrinsic_T   // used in motion undistortion
Lid_rot_to_IMU    = extrinsic_R   // used in motion undistortion
```

FAST-LIVO2 uses these **internally** for IMU-LiDAR motion compensation. They
do **not** describe robot mounting — only the sensor's internal geometry.

The rotation (~180° around the XY-diagonal axis) reflects that the Airy's
internal IMU chip axes are significantly rotated relative to the LiDAR scan
frame axes.

### `Rcl` and `Pcl`

Camera-to-LiDAR extrinsic (used for VIO when `img_en: 1`). Not relevant for
LiDAR-only operation.

---

## 2. Static TF Publishers in `launch/mapping_robosense.launch`

### TF 1: `rslidar_imu → rslidar`

```xml
<node pkg="tf" type="static_transform_publisher" name="rslidar_imu_to_rslidar"
     args="0.004156 0.004265 -0.004468 0.70711 -0.70709 0.001831 0.000660
           rslidar_imu rslidar 100" />
```

**What it represents:** The same relationship as `extrinsic_T`/`extrinsic_R`,
but published as a TF so the ROS TF tree is complete for visualization.

- Translation `[0.004156, 0.004265, -0.004468]` = `extrinsic_T` directly
  (position of `rslidar` origin in `rslidar_imu` frame)
- Quaternion `(x=0.70711, y=-0.70709, z=0.001831, w=0.000660)` = `extrinsic_R`
  converted to quaternion (rotates from `rslidar` frame to `rslidar_imu` frame)

This TF is **static** because the internal IMU-LiDAR geometry is fixed hardware.

### TF 2: `rslidar → minithex_base`

```xml
<node pkg="tf" type="static_transform_publisher" name="rslidar_to_minithex_base"
     args="-0.000011 0.03847 -0.03931 3.14 0.0 -2.35619
           rslidar minithex_base 100" />
```

**What it represents:** Where the Robosense Airy is mounted on the minithex
robot body, derived from the CAD model.

The Airy is mounted **upside down** on the minithex, offset ~55 mm along one
axis. The original CAD-model transform was `minithex_base → rslidar`:

```
original: args="-0.000011 -0.055 -0.0006 3.14 0.0 -2.35619 minithex_base rslidar"
```

This has been **inverted** (parent/child swapped, translation rotated) to fit
the TF tree direction. The rotation matrix for this transform is self-inverse
(R = Rᵀ, i.e. R² = I), so the rotation angles are identical in both directions.
Only the translation changes:

```
t_inverse = -R * t_original = [-0.000011, 0.03847, -0.03931]
```

---

## 3. TF Tree

```
world
  └── rslidar_imu          (dynamic, published by FAST-LIVO2 at LiDAR scan rate)
        └── rslidar         (static TF 1: internal Airy IMU-LiDAR calibration)
              └── minithex_base  (static TF 2: robot mounting from CAD model)
```

### Who publishes what

| TF edge | Publisher | Type | Source |
|---|---|---|---|
| `world → rslidar_imu` | FAST-LIVO2 (`TF_BASE = "rslidar_imu"`) | Dynamic | EKF state estimate |
| `rslidar_imu → rslidar` | `static_transform_publisher` | Static | `extrinsic_T`/`extrinsic_R` (factory calibration) |
| `rslidar → minithex_base` | `static_transform_publisher` | Static | CAD model (inverted) |

### Key design decisions

- **`TF_BASE = "rslidar_imu"`** (`include/common_lib.h`): FAST-LIVO2 estimates
  and tracks the pose of the internal Airy IMU, so this is the honest name for
  the tracked frame. Previously `"minithex_base"` was used, which was misleading
  because the estimated pose was actually at the IMU (inside the LiDAR housing),
  not at the physical robot base.

- **`gravity_align_en: true`**: Because the sensor is mounted upside down, the
  IMU Z-axis points downward at startup. Gravity alignment rotates the world
  frame so Z points up, preventing horizontal motion from incorrectly appearing
  as diagonal motion in the world frame. Robot must be stationary at startup
  during IMU initialisation (~1-2 seconds).

- **World frame origin**: Defined at the IMU pose when FAST-LIVO2 first
  initialises (after gravity alignment). Not georeferenced — relative odometry
  only.
