# Thin-Prism Distortion

![Python](https://img.shields.io/badge/Python-3.x-blue.svg)
![NumPy](https://img.shields.io/badge/NumPy-supported-orange.svg)
![Matplotlib](https://img.shields.io/badge/Matplotlib-interactive-green.svg)
![Status](https://img.shields.io/badge/Status-Educational_Simulation-success.svg)
![License](https://img.shields.io/badge/License-MIT-lightgrey.svg)

An interactive visualization of a **simplified thin-prism distortion model**.

Explore independent horizontal and vertical displacements whose magnitudes increase with squared distance from the image center.

## Topics

- thin-prism distortion concepts
- radius-weighted coordinate displacement
- asymmetric grid deformation
- independent horizontal and vertical coefficients
- normalized image coordinates
- ideal-grid comparison

## Main Script

- `src/thin_prism_distortion.py`

## Features

- two adjustable displacement coefficients
- distorted-grid visualization
- optional ideal-grid overlay
- center marker
- live corner-point displacement calculations

## Run

From this directory:

```bash
python src/thin_prism_distortion.py
```

## Requirements

```bash
python -m pip install -r ../../../requirements.txt
```

This script uses **NumPy** and **Matplotlib**. A graphical Matplotlib backend is required.

## Model Scope

The implementation uses one squared-radius term per coordinate. Its coefficient names are local to this script and differ from some calibration-library conventions.

Despite the interface title mentioning sensor tilt, this is not a geometric tilted-sensor projection model.

## Documentation

[Simulator guide](https://karammawas.github.io/sensors_vision_simulators/photogrammetry/thin-prism-distortion/)

## License

[MIT License](../../../LICENSE)
