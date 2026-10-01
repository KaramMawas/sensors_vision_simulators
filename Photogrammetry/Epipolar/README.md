# Epipolar Geometry

![Python](https://img.shields.io/badge/Python-3.x-blue.svg)
![NumPy](https://img.shields.io/badge/NumPy-supported-orange.svg)
![Matplotlib](https://img.shields.io/badge/Matplotlib-interactive-green.svg)
![Status](https://img.shields.io/badge/Status-Educational_Simulation-success.svg)
![License](https://img.shields.io/badge/License-MIT-lightgrey.svg)

An interactive 3D visualization of **epipolar geometry** for a two-camera system.

Move a scene point and independently rotate the cameras to explore the relationships between the baseline, epipolar plane, image projections, epipoles, and epipolar lines.

## Topics

- two-view camera geometry
- camera baseline
- projection rays
- epipolar plane
- epipolar lines
- epipoles and epipoles at infinity
- camera yaw
- ray–plane intersection

## Main Script

- `src/epipolar_geometry.py`

## Features

- two fixed camera centers with adjustable yaw
- interactive 3D scene point
- translucent virtual image planes
- highlighted epipolar-plane triangle
- projected points and epipolar lines
- finite epipole visualization

## Run

From this directory:

```bash
python src/epipolar_geometry.py
```

## Requirements

```bash
python -m pip install -r ../../requirements.txt
```

This script uses **NumPy** and **Matplotlib**. A graphical Matplotlib backend is required.

## Model Scope

The script constructs epipolar geometry directly from known camera positions and a known 3D point.

It does not estimate a fundamental matrix, match image features, rectify stereo images, or reconstruct unknown points.

## Documentation

[Simulator guide](https://karammawas.github.io/sensors_vision_simulators/photogrammetry/epipolar-geometry/)

## License

[MIT License](../../LICENSE)
