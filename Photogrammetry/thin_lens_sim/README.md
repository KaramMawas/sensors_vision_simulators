# Thin Lens: Focus and Blur

![Python](https://img.shields.io/badge/Python-3.x-blue.svg)
![NumPy](https://img.shields.io/badge/NumPy-supported-orange.svg)
![Matplotlib](https://img.shields.io/badge/Matplotlib-interactive-green.svg)
![Status](https://img.shields.io/badge/Status-Educational_Simulation-success.svg)
![License](https://img.shields.io/badge/License-MIT-lightgrey.svg)

An interactive **thin-lens ray diagram** illustrating image formation, sensor placement, and defocus.

Adjust the object height, object distance, focal length, and sensor position. Use the Auto-Focus button to place the sensor at the ideal real-image distance.

## Topics

- thin-lens equation
- object and image distance
- focal points
- principal-ray construction
- image inversion
- sensor placement
- geometric defocus
- real and virtual image conditions

## Main Script

- `src/thin_lens_simulator.py`

## Features

- interactive 2D optical diagram
- three principal rays
- movable sensor plane
- focal-point markers
- sharp-focus and out-of-focus indicators
- illustrative ray spread at the sensor
- analytical Auto-Focus button

## Run

From this directory:

```bash
python src/thin_lens_simulator.py
```

## Requirements

```bash
python -m pip install -r ../../requirements.txt
```

This script imports **NumPy** and **Matplotlib**. A graphical Matplotlib backend is required.

## Model Scope

The red sensor marker represents the spread of three illustrated rays. It is not a physically calibrated circle of confusion because aperture diameter and a full ray bundle are not modeled.

The simulator does not render photographs, diffraction, aberrations, or depth-of-field limits.

## Documentation

[Simulator guide](https://karammawas.github.io/sensors_vision_simulators/photogrammetry/thin-lens/)

## License

[MIT License](../../LICENSE)
