# 3D Scene Reconstruction with Drone Pipeline

![Build](https://img.shields.io/badge/build-passing-brightgreen)  
![License](https://img.shields.io/badge/license-MIT-blue)  
![ROS2](https://img.shields.io/badge/ROS2-Humble-orange)  
![CUDA](https://img.shields.io/badge/CUDA-12.4-lightgrey)

---

## Authors
1. Mohammed Munawwar  
2. Swaraj Mundruppady Rao

This project provides a full pipeline for reconstructing 3D scenes using aerial imagery captured by drones.  
It leverages ROS 2, OpenCV, COLMAP (with CUDA), and MeshLab to process images and generate 3D models.

---

## Disclaimer and Deployment Overview

### Deployment Overview

This repository is designed to run in two parts:

- **Onboard the Drone**:  
  The ROS2 nodes (`trajectory` and `best_frames`) are meant to run directly on the drone’s onboard computer (such as VOXL2 or any compatible companion computer).  
  These nodes handle trajectory execution and selective frame saving during flight.

- **On Local Ground Device**:  
  The COLMAP reconstruction pipeline is intended to run **offline on a local workstation** after the flight.  
  Once the best frames are saved and transferred from the drone, use your local machine (with sufficient CPU/GPU) to run the COLMAP sequential pipeline and perform 3D reconstruction.

---

### Workflow Summary

1. Run ROS2 nodes onboard the drone during flight.
2. After the mission, transfer saved images (`best_frames`) to the ground station.
3. Run the COLMAP reconstruction pipeline locally to build the sparse 3D scene.

---

### About Hardcoded Paths

In this project, some scripts and commands use hardcoded file paths like:

```
/home/munawwar/Final Colmap/
```

If you are running this on your own machine, **please replace `munawwar`** with **your own system username**  
or adjust the paths to match where you store your project files.

For example, if your username is `john`:

```
/home/john/Final Colmap/
```

We recommend updating these paths in:
- ROS2 launch files or scripts
- Command-line instructions
- Any project configuration files

This ensures the code runs smoothly in your local environment.

---

## Prerequisites

- ROS 2 (Humble or compatible)
- Python 3.8+
- COLMAP (CUDA 12.4 compatible build)
- Open3D
- MeshLab
- cv_bridge (for converting ROS image messages to OpenCV)
- OpenCV and NumPy

### System Requirements

- **Onboard system**: VOXL2 or any companion computer running ROS 2
- **Ground system**: Linux desktop with CUDA-capable GPU for COLMAP

---

## Install Prerequisites

### Install CV-bridge, OpenCV, NumPy, and Open3D

```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-cv-bridge
pip3 install opencv-python numpy open3d
```

### Install COLMAP (CUDA 12.4 Compatible)

To use COLMAP with CUDA 12.4 and NVIDIA Driver 550+ on Ubuntu:

#### Clone COLMAP

```bash
git clone https://github.com/colmap/colmap.git
cd colmap
```

#### Install Dependencies

```bash
sudo apt install \
    build-essential cmake ninja-build \
    libboost-all-dev libglew-dev \
    freeglut3-dev libxmu-dev libxi-dev \
    libpng-dev libjpeg-dev libtiff-dev \
    qt5-default libqt5opengl5-dev \
    libsqlite3-dev libgoogle-glog-dev libgflags-dev \
    libatlas-base-dev libsuitesparse-dev \
    libcgal-dev libcgal-qt5-dev
```

#### Patch for CUDA 12.4 Compatibility

Edit `cmake/CMakeHelper.cmake`:

```bash
nano cmake/CMakeHelper.cmake
```

Inside the `COLMAP_ADD_LIBRARY` macro, add after the `add_library(...)` line:

```cmake
if(CUDA_ENABLED)
    set_target_properties(${COLMAP_ADD_LIBRARY_NAME} PROPERTIES
        CUDA_ARCHITECTURES 86)
endif()
```

#### Configure the Build

```bash
mkdir build
cd build
cmake .. -GNinja -DCMAKE_BUILD_TYPE=Release
```

#### Compile

```bash
ninja
```

#### (Optional) Symlink COLMAP

```bash
sudo ln -s ~/colmap/build/src/colmap/exe/colmap /usr/local/bin/colmap
```

#### Test Installation

```bash
colmap
```

Expected output:

```
COLMAP 3.12.0.dev0 -- Structure-from-Motion and Multi-View Stereo
(Commit xxxxxxxx with CUDA)
```

---

## Install This Repository

### Clone the Repository

Navigate to your ROS 2 workspace’s `src` directory and clone:

```bash
cd ~/reconstruction_ws/src
git clone https://github.com/mun0404/3D-Scene-Reconstruction-with-drone.git
```

### Build the Package

```bash
cd ~/reconstruction_ws
colcon build
```

### Source the Workspace

```bash
# For bash users
source install/setup.bash

# For zsh users
source install/setup.zsh
```

You can also add this to your `.bashrc` or `.zshrc` for automatic sourcing.

---

## Usage

### Step 1: Run ROS2 Circular Path Node

First, run the circular path node to command the drone along the desired trajectory:

```bash
ros2 run trajectory circular_path
```

### Step 2: Run ROS2 Best Frame Saver Node

This node listens to `/hires_small_color`, detects keyframes using ORB + FLANN,  
and saves selected frames into:

```
"/home/munawwar/Final Colmap/best_frames/"
```

Run it:

```bash
ros2 run best_frames best_frames
```

Saved files:
```
frame_0000.png, frame_0001.png, frame_0002.png, ...
```

---

### Step 3: Run COLMAP Sequential Pipeline (On Local Machine)

Process the saved frames after transferring them to your local machine:

```bash
cd "/home/munawwar/Final Colmap"

# Clean previous runs
rm -f database.db
mkdir -p sparse/0

# Step 1: Create COLMAP database
colmap database_creator --database_path database.db

# Step 2: Extract features
colmap feature_extractor \
    --database_path database.db \
    --image_path best_frames \
    --ImageReader.single_camera 1 \
    --ImageReader.camera_model PINHOLE

# Step 3: Sequential matching (for video frames)
colmap sequential_matcher --database_path database.db

# Step 4: Sparse reconstruction
colmap mapper \
    --database_path database.db \
    --image_path best_frames \
    --output_path sparse/0
```

---

### Step 4: Convert to PLY and Visualize in MeshLab

```bash
# Convert COLMAP binary model to .ply
colmap model_converter \
    --input_path sparse/0 \
    --output_path sparse/0 \
    --output_type PLY

# Open in MeshLab
meshlab sparse/0/points3D.ply
```

---

### Optional: Launch COLMAP GUI

```bash
colmap gui
```

---
## Results

Results can be found here : https://drive.google.com/drive/folders/1Y0t8Z0K-NtN3TRaBAsJMMehzYb-bh0QN?usp=drive_link

### Data Collection Phase

![Data Collection](gif/data_collection.gif)

This shows the trajectory execution and frame capture process during drone flight.

---

### Best Frames Selection

![Best Frames](gif/best_frames.gif)

This shows the ORB + FLANN-based optimal frame selection running onboard the drone.


---

### 3D Reconstruction Output

![3D Reconstruction](gif/reconstruction.gif)

This shows the final 3D sparse reconstruction result after running the COLMAP pipeline on the saved frames.

---
## Folder Structure

```
Final Colmap/
├── best_frames/
├── database.db
└── sparse/
    └── 0/
```

---

## Credits

- ROS2 Best Frame Saver: Mohammed Munawwar, Swaraj Mundruppady Rao  
- COLMAP Pipeline: https://colmap.github.io/  
- MeshLab: https://www.meshlab.net/

---

## License

This project is licensed under the MIT License. See [LICENSE](LICENSE) for details.
