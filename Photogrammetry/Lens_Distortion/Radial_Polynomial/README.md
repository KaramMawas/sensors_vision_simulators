# Radial Distortion: Three-Coefficient Model

![Python](https://img.shields.io/badge/Python-3.x-blue.svg)
![NumPy](https://img.shields.io/badge/NumPy-supported-orange.svg)
![Matplotlib](https://img.shields.io/badge/Matplotlib-interactive-green.svg)
![Status](https://img.shields.io/badge/Status-Educational_Simulation-success.svg)
![License](https://img.shields.io/badge/License-MIT-lightgrey.svg)

An interactive visualization of **polynomial radial lens distortion** using three radial coefficients.

Explore how lower- and higher-order terms combine to produce barrel-like, pincushion-like, and mixed “mustache” distortion patterns.

## Topics

- polynomial radial distortion
- first, second, and third radial coefficients
- squared-, fourth-, and sixth-power radius terms
- barrel and pincushion distortion
- mixed radial distortion
- edge amplification
- radial scaling-factor analysis

## Main Script

- `src/radial_distortion_polynomial.py`

## Features

- three independently adjustable coefficients
- smoothly sampled grid curves
- optional ideal-grid overlay
- live radial scaling-factor calculation
- corner-point displacement example
- heuristic distortion-type title

## Run

From this directory:

```bash
python src/radial_distortion_polynomial.py
```

## Requirements

```bash
python -m pip install -r ../../../requirements.txt
```

This script uses **NumPy** and **Matplotlib**. A graphical Matplotlib backend is required.

## Model Scope

The implementation contains the polynomial radial component only. It does not include tangential distortion, thin-prism terms, or calibration.

The displayed distortion category is a heuristic. Extreme coefficients can create folded or non-invertible mappings.

## Documentation

[Simulator guide](https://karammawas.github.io/sensors_vision_simulators/photogrammetry/radial-distortion-polynomial/)

## License

[MIT License](../../../LICENSE)
