---
layout: page
title: Structure from Motion — Workflow Visualization
permalink: /photogrammetry/structure-from-motion/
---

# Structure from Motion: Workflow Visualization

> An animated introduction to camera registration, multiscale search, and view rejection.

[Back to Photogrammetry]({{ '/photogrammetry/' | relative_url }})

## Overview

This demonstration uses a predefined house-shaped point set and a 12-position camera path to illustrate selected ideas from a Structure from Motion workflow.

One panel shows the scene and camera states. A second panel displays a schematic three-level image pyramid with animated feature-search markers and residual vectors.

**This is not an SfM reconstruction algorithm.** The scene structure and camera positions are already known to the program.

## Topics

- incremental camera registration concepts
- multiscale feature-search visualization
- illustrative residual vectors
- candidate-camera convergence
- accepted and rejected views
- point-support accumulation
- workflow progression and restart

## Simulation Layout

### Global 3D Structure

- **Blue point markers:** predefined scene points.
- **Navy camera markers:** accepted camera positions.
- **Orange camera marker:** current candidate position with simulated jitter.
- **Orange lines:** illustrative viewing rays.
- **Pulsing red cross:** rejected view.

Point size and color vary with a synthetic support counter. They do not represent measured reconstruction uncertainty.

### Image Pyramid

Three grid levels represent nominal image scales:

- **L0:** full resolution.
- **L1:** half resolution.
- **L2:** quarter resolution.

The active level changes during animation. Search markers and residual vectors illustrate multiscale processing but are not calculated from actual images.

## Controls

### OPTIMIZE POSE

The button has two roles:

- While a view is being illustrated as “solving,” it accepts or rejects that view.
- After a decision, another click advances to the next view when one is available.

Despite its label, the button does not invoke numerical pose optimization.

### RESTART SFM

Resets the camera sequence, point-support counters, candidate state, and status message.

Randomness is not reseeded, so restarting does not reproduce an identical animation.

## How It Works

1. The first camera is marked as accepted.
2. A candidate camera moves around its predefined position using random jitter.
3. Simulated error decreases toward a selected floor.
4. A button click decides the current view's outcome.
5. Accepted views add their predefined camera positions and randomly increase point-support counters.
6. Rejected views are marked with a pulsing cross.
7. A subsequent click advances the sequence.

Camera indices 5 and 9 are explicitly designated as rejected views using zero-based indexing. Their rejection is scripted, not inferred from computed residuals.

## Animation

The requested animation interval is 50 milliseconds. Actual refresh rate depends on the computer and Matplotlib backend.

The displayed epipolar line and residuals are schematic and are not derived from the camera geometry.

## Assumptions and Limitations

The script does not implement:

- image loading or image-pyramid construction
- feature detection or descriptor extraction
- correspondence matching
- essential- or fundamental-matrix estimation
- camera-pose estimation
- triangulation
- bundle adjustment
- dense reconstruction

Use it to explain workflow concepts, not to evaluate SfM accuracy or performance.

## Run

From the repository root:

```bash
python Photogrammetry/sfm/src/sfm_workflow_simulator.py
```

## Requirements

```bash
python -m pip install -r requirements.txt
```

Requires NumPy, Matplotlib, and a graphical Matplotlib backend.

## Source and License

- [Source code](https://github.com/KaramMawas/sensors_vision_simulators/blob/main/Photogrammetry/sfm/src/sfm_workflow_simulator.py)
- [MIT License](https://github.com/KaramMawas/sensors_vision_simulators/blob/main/LICENSE)
