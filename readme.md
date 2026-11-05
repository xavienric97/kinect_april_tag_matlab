# Kinect v2 Multi-Tag 3D Room Reconstruction Pipeline

An end-to-end MATLAB pipeline for 3D room mapping and reconstruction using a mobile Kinect v2 sensor. The system estimates camera poses via an overlapping AprilTag network (Tag Registry System) and fuses synchronized point clouds into a globally unified 3D reconstruction.

## 📌 Project Overview

This repository provides a complete workflow to capture, estimate poses, inspect, and reconstruct 3D environments. By utilizing a multi-tag network with spatial overlap, the system enables continuous tracking even when the camera moves freely through the room and loses sight of the initial global reference tag.

### Key Features

* **Streaming Memory Management:** Real-time frame acquisition with `flushdata` buffer clearing to prevent RAM overflow.
* **On-Screen GUI Controls:** Interactive mouse-click capture interface for controlled dataset building.
* **Overlapping Multi-Tag Pose Estimation:** Dynamic bridge network (`containers.Map`) using 4x4 homogeneous transformation matrices (`rigidtform3d`) that dynamically maps secondary tags relative to the initial origin tag.
* **Direct Point Cloud Fusion:** Noise filtering (`pcdenoise`), voxel grid downsampling (`pcdownsample`), and global coordinate alignment (`pctransform`) without coordinate mirror issues.
* **CloudCompare Export:** Automatic export of the final merged room point cloud as a binary `.ply` file.

## 📁 Repository Structure

```
.
├── data/
│   ├── cameraParams.mat               # Calibrated camera intrinsic parameters
│   ├── calibration_images/            # Captured checkerboard calibration images
│   ├── kinect_sequence.mat            # Saved RGB, Depth, and PointCloud sequence
│   ├── camera_poses.mat               # Computed 3D camera poses & Tag network data
│   └── global_room_reconstruction.ply # Exported global 3D room point cloud
├── pre_for_calib.m                    # Helper: Kinect RGB checkerboard image capture
├── a1_data_acquisition.m              # Script 01: Live acquisition & GUI mouse capture
├── a2_inspect_data.m                  # SubScript: Synchronized RGB & PointCloud inspection
├── b_estimate_apriltag_poses.m        # Script 02: Multi-tag pose estimation & network tracking
├── c_trajectory_visualization.m       # Script 03: 3D Kinect trajectory visualizer
└── d_pointcloud_fusion.m              # Script 04: Global point cloud fusion & PLY export
```

## 🛠️ Requirements & Setup

### Prerequisites

* **MATLAB** (R2022a or newer recommended)
* **Image Acquisition Toolbox**
* **Image Acquisition Toolbox Support Package for Kinect For Windows Sensor**
* **Computer Vision Toolbox** (with AprilTag detection support)
* **Lidar Camera Calibrator / Navigation Toolbox** (for point cloud operations)

### Target Family & Tag Setup

* **AprilTag Family:** `tag36h11` (Default)
* **Tag Size:** `0.173 meters` (Ensure physical measurements match your printed tags in `Script 02`).
* **Tag Placement:** Place tags around the room ensuring **at least 2-3 overlapping tags** are visible per frame during camera translation.

## 🚀 Execution Workflow

### Step 0: Camera Calibration (`pre_for_calib.m`)

1. Connect your Kinect v2 sensor.
2. Run `pre_for_calib.m` to start the live RGB preview stream.
3. Place a checkerboard calibration pattern in front of the camera and press **`SPACE`** to capture each photo (15 to 20 images recommended).
4. Press **`Q`** to finish capture. The images will be saved in `data/calibration_images/`.
5. Run MATLAB's `cameraCalibrator` tool using these images to generate your camera parameters and save the output object as `cameraParams.mat` inside `data/`.

### Step 1: Data Acquisition (`a1_data_acquisition.m`)

1. Run `a1_data_acquisition.m` to launch the live preview window.
2. Click the green **`CAPTURE PHOTO [CLICK]`** button on-screen to capture synchronized RGB images, depth maps, and point clouds while moving through the room.
3. Press **`Q`** or close the window to finalize the session. Data will be exported to `data/kinect_sequence.mat`.

### Step 2: Data Inspection (`a2_inspect_data.m`)

Run `a2_inspect_data.m` to review keyframes from the sequence. It displays side-by-side comparisons of horizontal-mirrored RGB images and top-down (YX plane) point clouds.

### Step 3: Pose Estimation (`b_estimate_apriltag_poses.m`)

Run `b_estimate_apriltag_poses.m` to compute camera poses:
1. The first AprilTag detected in the sequence automatically acts as the **Absolute Global Origin** $(0,0,0)$.
2. Co-visible tags in subsequent keyframes act as bridges, storing global 4x4 matrix transformations in `knownTagMatrices`.
3. Output camera poses are saved to `data/camera_poses.mat`.

### Step 4: Trajectory Visualization (`c_trajectory_visualization.m`)

Run `c_trajectory_visualization.m` to inspect the 3D path traversed by the Kinect sensor. Start and end points are highlighted to verify continuous movement without unexpected trajectory jumps.

### Step 5: Global Point Cloud Fusion (`d_pointcloud_fusion.m`)

Run `d_pointcloud_fusion.m` to reconstruct the full 3D environment:
1. Filters individual frame noise using `pcdenoise`.
2. Direct-transforms each point cloud into global coordinates via `pctransform`.
3. Merges all keyframes into a unified voxel grid (`pcmerge` at $0.01\text{ m}$ resolution).
4. Exports the complete 3D scene to `data/global_room_reconstruction.ply`.

## 📊 Summary of Pipeline Scripts

| Script File | Purpose | Output File |
| :--- | :--- | :--- |
| `pre_for_calib.m` | RGB checkerboard calibration capture | `data/calibration_images/*.png` |
| `a1_data_acquisition.m` | Live preview & GUI mouse capture | `data/kinect_sequence.mat` |
| `a2_inspect_data.m` | Synchronized RGB & 3D inspection | Visual figures |
| `b_estimate_apriltag_poses.m` | Multi-tag bridge pose estimation | `data/camera_poses.mat` |
| `c_trajectory_visualization.m` | 3D camera trajectory plotting | Visual figure |
| `d_pointcloud_fusion.m` | Global point cloud fusion & PLY export | `data/global_room_reconstruction.ply` |

## 🔧 Troubleshooting & Tips

* **Reconstruction Alignment Issues:** Ensure `tagSize` in `b_estimate_apriltag_poses.m` exactly matches the physical size of your printed AprilTag edges in meters.
* **Loss of Global Reference:** Ensure at least 2–3 tags overlap in each capture frame when transitioning between different areas of the room.
* **Point Cloud Density Adjustments:** Modify `voxelGridSize` in `d_pointcloud_fusion.m` (e.g., `0.005` for finer detail or `0.02` for faster rendering).