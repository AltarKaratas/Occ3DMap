# 🗺️ 3D Occupancy Grid Mapping

This project implements a simple **3D occupancy grid mapping** system using LiDAR point clouds and camera poses. It constructs a probabilistic volumetric map using log-odds update rules and visualizes the result using Open3D.

---

## 📦 Features

- Raycasting with **3D Bresenham** algorithm
- Probabilistic occupancy update using **log-odds**
- Input: PLY point clouds + camera poses
- Downsampling with voxel grid
- Open3D-based visualization

---

## ⚙️ Parameters

You can control the behavior via two main flags:

- `voxel_size`: Size of each grid cell (e.g., `0.1`, `0.25`, `0.35`)
- `num_files`: Number of PLY files to process (default: all)

These can be passed as arguments or configured manually in code.

---

## 🚀 Usage

### 1. Clone the repository
```bash
git clone https://github.com/AltarKaratas/Occ3DMap.git
cd Occ3DMap
```

2. Build
Make sure you have CMake and a C++17-compatible compiler.
```bash
mkdir build && cd build
cmake ..
make
```

3. Run
You can run the binary with parameters:

```bash
cd <root folder>
build/main_exec --voxel_size <voxel_size> --num_files <num_files>
# Example:
./build_main_exec --voxel_size 0.25 --num_files 100
```

📊 Log-Odds
Occupancy updates are done with log-odds values:

occupied_logodds = +0.8

free_logodds = -0.4

Clamped between [-2.0, +3.5]

These values are updated efficiently during integration.

🧠 Optimization Notes
Downsampling reduces number of rays by merging points within the same voxel

Visited voxel cache prevents redundant log-odds updates

Processed ray cache avoids recomputing Bresenham rays

voxel_size has a direct effect on performance & map resolution


