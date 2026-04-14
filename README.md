# 3D Vision Simulators

A modular collection of educational and engineering simulators of different sensors: TLS, SLS, Camera, Photogrammetry, and Depth camera as well as some processing algorithms.

## Overview

This repository contains simulation, visualization, and algorithmic tools for:

- Camera photography parameter modeling
- Depth camera and LiDAR simulation
- KD-tree spatial search
- Photogrammetry
- Structured Light Scanning (SLS)
- Terrestrial Laser Scanning (TLS)

The project is intended for:

- education
- research prototyping
- algorithm explanation
- engineering demos
- scientific visualization

## Repository Structure

- `Camera_photography_parameters/`  
  Triangular of exposure: ISO, Aperture, and F-stops.

- `Depth_camera/`  
  LiDAR and depth camera simulation, time-of-flight, point cloud reconstruction, and interactive visualization.

- `kdtree/`  
  KD-tree implementations and spatial nearest-neighbor search tools for 2D and 3D.

- `Photogrammetry/`  
  Thin lens simulation, Pin-hole projection, Lens distortion, Scale Ambiguity, Epipolar, SfM.

- `SLS/`  
  Structured light scanning simulation workflow.

- `TLS/`  
  Terrestrial laser scanning simulation for ToF as well as Phase-shift.

- `docs/`  
  Project website and documentation for GitHub Pages.

- `examples/`  
  Example scripts for quick demonstrations.

- `tests/`  
  Test suite for validation and maintenance.

## Features

- 2D and 3D visualization
- LiDAR time-of-flight simulation
- point cloud generation
- animated scan build-up
- modular code organization
- educational plots and diagrams
- extendable scientific computing workflow

## Quick Start

### Clone the repository:

```bash
git clone https://github.com/yourusername/sensors_vision_simulators.git
cd sensors_vision_simulators
```
### Install the required packages

```bash
$ pip install -r requirements.txt
```

### Run an example

#### Create a new environment using the following command:
```bash
$ conda create -n sensor_sim python=3.11
```
#### Activate the environment:
```bash
$ conda activate sensor_sim
```
#### Run file:
```bash
python Depth_camera/src/lidar_depth_camera_live_sim.py
```

## Documentation

### Example Visualizations
![LiDAR Scan Demo](docs/assets/gifs/live_lidar_scan.gif)
![Depth Map Example](docs/assets/images/depth_map_example.png)
![Point Cloud Example](docs/assets/images/point_cloud_example.png)

## Author
Karam Mawas
Affiliation: Technical University of Braunschweig, Institute of Geodesy and Photogrammetry (IGP)
Email: karam.mawas@gmail.com / k.mawas@tu-bs.de
GitHub: @KaramMawas
Website: [Karam Mawas](https://karammawas.github.io/)
ORCID: [ORCID_KaramMawas](https://orcid.org/0000-0002-8608-7578)

## Citation
If you use this repository in research or teaching, please cite it using the metadata in CITATION.cff

## License
This project is licensed under the terms of the LICENSE file.
