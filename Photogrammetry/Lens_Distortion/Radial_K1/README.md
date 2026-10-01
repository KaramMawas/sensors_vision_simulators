# Radial Distortion: Single-Coefficient Model

![Python](https://img.shields.io/badge/Python-3.x-blue.svg)
![NumPy](https://img.shields.io/badge/NumPy-supported-orange.svg)
![Matplotlib](https://img.shields.io/badge/Matplotlib-interactive-green.svg)
![Status](https://img.shields.io/badge/Status-Educational_Simulation-success.svg)
![License](https://img.shields.io/badge/License-MIT-lightgrey.svg)

An interactive visualization of **radial lens distortion** using the first radial coefficient.

Compare a distorted grid with its ideal pinhole reference while moving between barrel-like, undistorted, and pincushion-like configurations.

## Topics

- radial lens distortion
- first radial coefficient
- barrel and pincushion distortion
- normalized image coordinates
- distortion center
- ideal-grid comparison
- corner-point displacement

## Main Script

- `src/radial_distortion_k1.py`

## Features

- continuously adjustable radial coefficient
- curved grid visualization
- optional ideal-grid overlay
- center marker
- live corner-point calculation
- distortion-type title

## Run

From this directory:

```bash
python src/radial_distortion_k1.py
```

## Requirements

```bash
python -m pip install -r ../../../requirements.txt
```

This script uses **NumPy** and **Matplotlib**. A graphical Matplotlib backend is required.

## Model Scope

The script applies only the first radial term of a polynomial distortion model. It does not estimate calibration parameters or undistort photographs.

Extreme coefficient values can produce nonphysical, non-invertible grid deformations.

## Documentation

[Simulator guide](https://karammawas.github.io/sensors_vision_simulators/photogrammetry/radial-distortion-k1/)

## License

[MIT License](../../../LICENSE)
