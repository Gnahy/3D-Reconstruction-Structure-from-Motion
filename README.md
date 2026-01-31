# 3D Reconstruction using Structure-from-Motion

This repository implements a camera-guided 3D reconstruction and geometric transformation pipeline using Structure-from-Motion (SfM) with COLMAP. The reconstructed point cloud is transformed using camera pose information and converted to metric scale with validation.

## Usage

1. Extract uniformly spaced frames from the input video (see first section of the script).
2. Run COLMAP to generate the sparse 3D reconstruction and camera poses (as described in the report).
3. Run the provided Python scripts to visualise the reconstruction (second part of the script) and apply camera-guided translation, rotation, and metric scaling to the 3D point cloud (third part of the script).

