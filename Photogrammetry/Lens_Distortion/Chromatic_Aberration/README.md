# Chromatic Aberration: RGB Channel Warping

![Python](https://img.shields.io/badge/Python-3.x-blue.svg)
![NumPy](https://img.shields.io/badge/NumPy-supported-orange.svg)
![Matplotlib](https://img.shields.io/badge/Matplotlib-interactive-green.svg)
![Status](https://img.shields.io/badge/Status-Educational_Simulation-success.svg)
![License](https://img.shields.io/badge/License-MIT-lightgrey.svg)

An interactive visualization of **lateral chromatic aberration-like color fringing** using independent radial warping of red, green, and blue image channels.

A synthetic checkerboard and a magnified corner view make channel misalignment easy to inspect.

## Topics

- lateral chromatic aberration concepts
- independent RGB channel warping
- radius-dependent image displacement
- color fringing
- reference-channel selection
- image resampling and boundary clipping

## Main Script

- `src/chromatic_aberration.py`

## Features

- generated 600 × 600 checkerboard image
- full-image and top-left zoom views
- independent red, green, and blue coefficients
- green-reference status message
- reset button

## Run

From this directory:

```bash
python src/chromatic_aberration.py
```

## Requirements

```bash
python -m pip install -r ../../../requirements.txt
```

This script uses **NumPy** and **Matplotlib**. A graphical Matplotlib backend is required.

## Model Scope

The simulator applies a simplified channel-dependent image warp. It does not model wavelength-dependent refraction, longitudinal chromatic aberration, or spectral image formation.

Keeping the green coefficient at zero leaves that channel unwarped; it does not establish physical camera calibration.

## Documentation

[Simulator guide](https://karammawas.github.io/sensors_vision_simulators/photogrammetry/chromatic-aberration/)

## License

[MIT License](../../../LICENSE)
