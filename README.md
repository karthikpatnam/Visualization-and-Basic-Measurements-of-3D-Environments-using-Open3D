# AI for Robotics: ZED Point Cloud Capture & Measurement

A comprehensive Python toolkit for capturing, visualizing, and measuring point clouds using a ZED stereo camera. This project enables real-time 3D scene analysis with interactive measurement tools for robotics applications.

## Project Overview

This project provides a complete workflow for:
1. **Capturing** 3D point clouds from a ZED camera
2. **Visualizing** and processing point clouds with advanced filtering
3. **Measuring** distances between points interactively
4. **Analyzing** 3D scenes for robotics applications

## Features

✨ **Point Cloud Capture**
- Capture frames from ZED stereo camera at HD720 resolution
- Ultra-depth quality mode for accurate 3D reconstruction
- Configurable confidence threshold to filter low-confidence points
- Export to industry-standard PLY format (XYZRGBA)

📊 **Visualization & Processing**
- Interactive 3D point cloud visualization
- Voxel downsampling for performance optimization
- Automatic normal estimation
- RANSAC-based plane segmentation (useful for removing floors/walls)
- Mesh generation (Poisson or ball-pivoting algorithms)

📏 **Interactive Measurement**
- Click-based point picking in 3D viewer
- Automatic pairwise distance calculation
- JSON-based pick recording
- Batch measurement capabilities

## Requirements

### Hardware
- **ZED Camera** (ZED, ZED 2, or ZED 2i)
- USB 3.0+ port for camera connection
- GPU recommended for optimal depth processing

### Software
- Python 3.7+
- ZED SDK (with Python bindings)
- Open3D library
- NumPy

## Installation

### 1. Install ZED SDK

**Windows:**
1. Download the ZED SDK installer from [Stereolabs website](https://www.stereolabs.com/developers/release/)
2. Run the installer and follow the setup wizard
3. Ensure Python bindings (pyzed) are installed

**Verify installation:**
```powershell
python -c "import pyzed.sl as sl; print('ZED SDK installed successfully')"
```

### 2. Install Python Dependencies

Create a virtual environment (recommended):
```powershell
python -m venv venv
.\venv\Scripts\Activate.ps1
```

Install required packages:
```powershell
pip install open3d numpy
```

## Usage

### 1. Capture Point Cloud from ZED Camera

Capture 30 frames and save to a PLY file:

```powershell
python capture_zed_pointcloud.py --frames 30 --output output_pointcloud.ply
```

**Arguments:**
- `--frames` (int): Number of successful frames to capture [default: 30]
- `--output` (str): Output PLY file path [default: zed_pointcloud.ply]
- `--timeout` (float): Maximum capture time in seconds [default: 10.0]

**Example:**
```powershell
python capture_zed_pointcloud.py --frames 50 --output scene1.ply
```

### 2. Visualize and Measure Point Clouds

Basic visualization:
```powershell
python viz_and_measure.py --ply output_pointcloud.ply
```

With voxel downsampling:
```powershell
python viz_and_measure.py --ply output_pointcloud.ply --voxel 0.02
```

With plane segmentation (removes dominant plane like floor):
```powershell
python viz_and_measure.py --ply output_pointcloud.ply --segment
```

Generate mesh from point cloud:
```powershell
python viz_and_measure.py --ply output_pointcloud.ply --mesh poisson --poisson_depth 8
```

Interactive measurement mode:
```powershell
python viz_and_measure.py --ply output_pointcloud.ply --measure
```

**Arguments:**
- `--ply` (str): Path to PLY point cloud file
- `--voxel` (float): Voxel size for downsampling [optional]
- `--segment`: Enable plane segmentation [optional]
- `--mesh` (str): Mesh algorithm: 'poisson' or 'ball-pivoting' [optional]
- `--poisson_depth` (int): Poisson mesh reconstruction depth [default: 8]
- `--measure`: Enable interactive measurement mode [optional]

### 3. Interactive Point Picking and Measurement

For detailed point-by-point measurement and analysis:

```powershell
python pick_and_measure_fix.py --ply output_pointcloud.ply
```

**How to use:**
1. Left-click on points in the 3D viewer to select them
2. Use mouse wheel to zoom and rotate
3. Press 'Q' or Esc to finish picking
4. The tool automatically calculates:
   - Picked point indices and coordinates (XYZ)
   - Pairwise distances between all selected points
   - Saves results to JSON files

**Output files:**
- `picked_points_xyz.txt`: Coordinates of picked points
- `picked_distances.txt`: Distances between point pairs
- `selection.json`: Raw selection data

## Project Structure

```
AIforRobotics/
├── capture_zed_pointcloud.py      # ZED camera capture script
├── viz_and_measure.py              # Visualization & measurement tool
├── pick_and_measure_fix.py         # Interactive point picker
├── commands-for-projectrun.txt     # Quick reference commands
├── picked_points_xyz.txt           # Output: picked point coordinates
├── picked_distances.txt            # Output: calculated distances
├── ScreenCamera_*.json             # Camera calibration files
└── README.md                       # This file
```

## Quick Start Guide

### Workflow Example:

```powershell
# 1. Capture a point cloud from the ZED camera
python capture_zed_pointcloud.py --frames 30 --output scene1.ply

# 2. Visualize the point cloud
python viz_and_measure.py --ply scene1.ply --voxel 0.02

# 3. Interactive measurement
python pick_and_measure_fix.py --ply scene1.ply
```

## Key Concepts

### Point Cloud Formats
- **PLY Format**: Industry-standard 3D point cloud format storing XYZ coordinates and RGBA color
- **XYZRGBA**: Each point has 3D position and 4-channel color (Red, Green, Blue, Alpha)

### ZED Camera Parameters
- **Resolution**: HD720 (1280×720)
- **Depth Mode**: ULTRA (highest quality)
- **Coordinate Units**: Millimeters
- **Confidence Threshold**: 50/100 (filters unreliable depth estimates)

### Point Cloud Processing
- **Voxel Downsampling**: Reduces point count while preserving geometry
- **Normal Estimation**: Calculates surface normals for mesh generation
- **Plane Segmentation**: Uses RANSAC to identify and remove dominant planes

## Troubleshooting

### Camera not detected
```
ERROR: Could not connect to ZED camera
```
- Verify camera is connected via USB 3.0
- Check ZED SDK installation: `python -c "import pyzed.sl as sl"`
- Try reconnecting the camera

### PyZed import error
```
ERROR: could not import pyzed.sl
```
- Install ZED SDK from [Stereolabs website](https://www.stereolabs.com/developers/release/)
- Verify PYTHONPATH includes ZED SDK: `echo $env:PYTHONPATH`

### Point cloud is empty
```
RuntimeError: Point cloud is empty
```
- Ensure the PLY file exists and is in correct format
- The camera may not have captured valid depth data (check lighting)

### Selection.json not created
- The viewer may have closed before picks were registered
- Try again with the viewer open for longer
- Check file permissions in the project directory

## Dependencies Explained

| Package | Purpose |
|---------|---------|
| `pyzed` | ZED SDK Python bindings for camera control |
| `open3d` | 3D data processing and visualization |
| `numpy` | Numerical computations |
| `python` | Programming language (≥3.7) |

## Performance Tips

1. **Use voxel downsampling** for large point clouds (>1M points)
2. **Enable plane segmentation** to remove clutter like floors
3. **Adjust confidence threshold** in capture script for cleaner data
4. **Use GPU rendering** for smooth visualization (requires compatible GPU)

## File Outputs

After running measurement scripts, the following files are generated:
- `picked_points_xyz.txt`: CSV format with columns [Index, X, Y, Z]
- `picked_distances.txt`: Distances between consecutive picked points
- `selection.json`: Complete selection metadata

## References

- [ZED SDK Documentation](https://www.stereolabs.com/docs/)
- [Open3D Documentation](http://www.open3d.org/docs/)
- [PLY Format Specification](https://en.wikipedia.org/wiki/PLY_(file_format))

## License

This project is for educational purposes in the AI for Robotics course.

## Notes

- Always ensure adequate lighting for optimal ZED camera depth sensing
- Point cloud quality depends on scene geometry and lighting conditions
- For best results, keep objects 0.3-5 meters away from the camera

---

**Last Updated:** 2025-12-10
