# FAST-LIVO2 — NDT_1.0

This branch contains the FAST-LIVO2 modifications used in the
AutoPilotNDT project, integrating LiDAR-inertial odometry with
the simulated miniThex UAV and the GenoM/POM estimation pipeline.

## Implemented Changes

### 1. LiDAR Timestamp Synchronization

- Updated `launch/mapping_sim.launch` to use the restamped
  Gazebo LiDAR point cloud (`/lidar3d/points/restamped`).
- Resolved the timestamp mismatch between simulated LiDAR
  measurements and IMU data.

### 2. Odometry and Covariance Publication

Modified `src/LIVMapper.cpp` to:

- Publish position and orientation covariance information
  through ROS Odometry.
- Publish linear velocity in the odometry child frame.
- Publish the corresponding linear velocity covariance.
- Explicitly represent unavailable angular velocity uncertainty.

These changes provide additional state-estimation information
for integration with the GenoM/POM pipeline.

### 3. LiDAR Point Cloud Processing

Modified `src/pointcloud_to_livox.cpp` to:

- Filter invalid LiDAR points.
- Remove simulated maximum-range returns.
- Update the reported point count after filtering.

### 4. Simulation Sensor Configuration

Modified `config/sim.yaml`:

- Updated the LiDAR-IMU extrinsic translation to [0.0, 0.0, 0.1].
- Preserved the LiDAR-inertial configuration used in the
  working miniThex simulation.

## Known Limitations and Future Improvements

- Camera integration remains pending; the tested simulation
  currently uses LiDAR-inertial odometry.
- Sensor extrinsic calibration requires further verification.
- The covariance representations and their interpretation
  within POM require continued validation.
- Performance and accuracy under different operating
  conditions require additional evaluation.
- Physical UAV deployment requires separate validation.

These items are intentionally left unchanged in NDT_1.0.

## Deployment

This repository is part of the AutoPilotNDT software stack.

Docker build and deployment configuration will be maintained
in the `docker_files` repository.

Because this branch modifies C++ source files, FAST-LIVO2
must be rebuilt from the NDT_1.0 source for the changes
to take effect.

## Validation

The integrated system was tested in the existing AutoPilotNDT
Docker simulation environment on November 24, 2026.

A clean Docker image rebuild and cross-machine deployment
validation remain pending.
