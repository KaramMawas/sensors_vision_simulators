# Tangential / Decentering Distortion

![Python](https://img.shields.io/badge/Python-3.x-blue.svg)
![NumPy](https://img.shields.io/badge/NumPy-supported-orange.svg)
![Matplotlib](https://img.shields.io/badge/Matplotlib-interactive-green.svg)
![Status](https://img.shields.io/badge/Status-Educational_Simulation-success.svg)
![License](https://img.shields.io/badge/License-MIT-lightgrey.svg)

An interactive visualization of **tangential, or decentering, lens distortion**.

Adjust two distortion coefficients to explore asymmetric coordinate displacements and compare the resulting grid with an ideal pinhole reference.

## Topics

- tangential distortion
- decentering distortion
- asymmetric image deformation
- cross-coordinate coupling
- normalized image coordinates
- ideal-grid comparison
- corner-point displacement

## Main Script

- `src/tangential_distortion.py`

## Features

- two independently adjustable coefficients
- distorted-grid visualization
- optional ideal-grid overlay
- center marker
- live horizontal and vertical displacement calculations

## Run

From this directory:

```bash
python src/tangential_distortion.py
```

## Requirements

```bash
python -m pip install -r ../../../requirements.txt
```

This script uses **NumPy** and **Matplotlib**. A graphical Matplotlib backend is required.

## Model Scope

The simulator applies the tangential component of a commonly used camera-distortion model. It does not include radial distortion or estimate calibration parameters.

The coefficients describe image-coordinate deformation; they are not physical lens or sensor tilt angles.

## Documentation

[Simulator guide](https://karammawas.github.io/sensors_vision_simulators/photogrammetry/tangential-distortion/)

## License

[MIT License](../../../LICENSE)
