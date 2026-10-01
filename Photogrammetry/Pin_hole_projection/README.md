# Pinhole Camera Projection

![Python](https://img.shields.io/badge/Python-3.x-blue.svg)
![NumPy](https://img.shields.io/badge/NumPy-3D_Simulator-orange.svg)
![Matplotlib](https://img.shields.io/badge/Matplotlib-interactive-green.svg)
![Status](https://img.shields.io/badge/Status-Educational_Simulation-success.svg)
![License](https://img.shields.io/badge/License-MIT-lightgrey.svg)

Two interactive simulators demonstrating **pinhole camera geometry**, from a 2D image-formation diagram to perspective projection in 3D.

Explore how focal length, object size, and object distance affect the projected image, and compare physical and virtual image-plane conventions.

## Topics

- pinhole camera geometry
- perspective projection and perspective division
- focal length and image magnification
- object distance and projected size
- size–distance ambiguity
- image inversion
- physical and virtual image planes
- camera-centered coordinates
- projection along different viewing axes

## Simulators

### 2D Pinhole Camera

**Script:** `src/pinhole_sim.py`

A side-view diagram showing an object, a pinhole, an image plane behind the pinhole, and projection rays.

Features:

- adjustable focal length
- adjustable object distance and height
- inverted image visualization
- projection rays passing through the pinhole
- live projected-size calculation
- manual exploration of size–distance ambiguity

The displayed projected size is a positive magnitude. The image is drawn below the optical axis to represent its inverted orientation.

### 3D Pinhole Projection

**Script:** `src/pinhole_projection.py`

An interactive 3D scene showing how a point projects onto a virtual image plane in front of the camera.

Features:

- adjustable focal length and object coordinates
- Z-axis and X-axis viewing modes
- translucent image plane
- projected-point coordinates
- live numerical projection calculations

## Run

Run either script from this directory.

### 2D Pinhole Camera

```bash
python src/pinhole_sim.py
```

### 3D Pinhole Projection

```bash
python src/pinhole_projection.py
```

Each command opens a separate interactive Matplotlib window.

## Requirements

Install the shared dependencies:

```bash
python -m pip install -r ../../requirements.txt
```

- **2D simulator:** Matplotlib.
- **3D simulator:** NumPy and Matplotlib.
- A graphical Matplotlib backend is required for interactive controls.

See [requirements.txt](../../requirements.txt).

## Suggested Experiments

1. Increase focal length while keeping the object fixed.
2. Increase object distance and observe the projected image shrink.
3. In the 2D simulator, multiply object height and distance by the same factor to preserve the projected size.
4. Compare the inverted 2D image with the 3D virtual-plane projection.
5. Switch the 3D viewing axis and inspect the updated projection equations.

## Model Scope

Both scripts use ideal pinhole geometry and consistent arbitrary units.

They do not model:

- finite pinhole diameter or diffraction
- lens distortion or thin-lens focusing
- pixel sampling or calibrated sensor dimensions
- image-based camera calibration or reconstruction

The 2D simulator has independent sliders; it does not automatically lock projected size.

In the 3D simulator, projection is undefined at zero depth along the viewing axis. A small substitute denominator avoids division by zero but does not resolve the geometric singularity.

Both simulators use fixed plot limits, so some parameter combinations can place projected geometry outside the visible area.

## Documentation

[Guide to both pinhole simulators](https://karammawas.github.io/sensors_vision_simulators/photogrammetry/pinhole-projection/)

## License

[MIT License](../../LICENSE)
