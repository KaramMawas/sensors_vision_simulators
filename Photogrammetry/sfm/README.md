# Structure from Motion: Workflow Visualization

![Python](https://img.shields.io/badge/Python-3.x-blue.svg)
![NumPy](https://img.shields.io/badge/NumPy-supported-orange.svg)
![Matplotlib](https://img.shields.io/badge/Matplotlib-animated-green.svg)
![Status](https://img.shields.io/badge/Status-Conceptual_Demonstration-success.svg)
![License](https://img.shields.io/badge/License-MIT-lightgrey.svg)

An animated, interactive illustration of a **Structure from Motion (SfM) workflow**.

Explore camera registration, schematic multiscale feature search, residual visualization, view rejection, and point-support accumulation using a synthetic scene.

> This is a workflow visualization, not an SfM solver. Camera positions and scene points are predefined.

## Topics

- incremental camera registration concepts
- multiscale image-pyramid visualization
- feature-search concepts
- illustrative residual vectors
- accepted and rejected views
- point-support visualization
- interactive workflow progression

## Main Script

- `src/sfm_workflow_simulator.py`

## Features

- synthetic house-shaped point set
- predefined 12-position camera path
- animated candidate-camera motion
- three schematic pyramid levels
- scripted view acceptance and rejection
- changing point markers to illustrate accumulated support
- optimize and restart buttons

## Run

From this directory:

```bash
python src/sfm_workflow_simulator.py
```

## Requirements

```bash
python -m pip install -r ../../requirements.txt
```

This script uses **NumPy** and **Matplotlib**. A graphical Matplotlib backend is required.

## Model Scope

No images are loaded or processed. Feature locations, residuals, convergence behavior, and point-support updates are illustrative.

The script does not perform feature matching, pose estimation, triangulation, bundle adjustment, or dense reconstruction.

## Documentation

[Simulator guide](https://karammawas.github.io/sensors_vision_simulators/photogrammetry/structure-from-motion/)

## License

[MIT License](../../LICENSE)
