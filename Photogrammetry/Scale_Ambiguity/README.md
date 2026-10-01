# Scale Ambiguity

![Python](https://img.shields.io/badge/Python-3.x-blue.svg)
![NumPy](https://img.shields.io/badge/NumPy-supported-orange.svg)
![Matplotlib](https://img.shields.io/badge/Matplotlib-interactive-green.svg)
![Status](https://img.shields.io/badge/Status-Educational_Simulation-success.svg)
![License](https://img.shields.io/badge/License-MIT-lightgrey.svg)

An interactive demonstration of **size–distance ambiguity** in perspective imaging.

Explore how different object sizes and distances can produce the same projected square. Lock the projected shape to visualize ambiguity, or unlock it to explore ordinary perspective scaling.

## Topics

- perspective image scale
- object size and camera distance
- focal length and magnification
- single-view scale ambiguity
- coupled parameter adjustment
- physical image-plane inversion
- X-axis, Y-axis, and Z-axis viewing conventions

## Main Script

- `src/scale_ambiguity.py`

## Features

- coordinated 3D scene and 2D image view
- adjustable focal length, distance, and object size
- projected-shape lock
- three viewing-axis modes
- projection rays and image-plane visualization
- live numerical calculations

## Run

From this directory:

```bash
python src/scale_ambiguity.py
```

## Requirements

```bash
python -m pip install -r ../../requirements.txt
```

This script uses **NumPy** and **Matplotlib**. A graphical Matplotlib backend is required.

## Model Scope

The object is a frontoparallel square. Image dimensions are expressed in arbitrary image-plane units, despite the interface label “Pixel Size.”

This is a conceptual demonstration of scale ambiguity, not a reconstruction or camera-calibration algorithm.

## Documentation

[Simulator guide](https://karammawas.github.io/sensors_vision_simulators/photogrammetry/scale-ambiguity/)

## License

[MIT License](../../LICENSE)
