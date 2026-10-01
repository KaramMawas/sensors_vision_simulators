---
layout: page
title: Photogrammetry
permalink: /photogrammetry/
---

# Photogrammetry Simulators

![Python](https://img.shields.io/badge/Python-3.x-blue.svg)
![NumPy](https://img.shields.io/badge/NumPy-supported-orange.svg)
![Matplotlib](https://img.shields.io/badge/Matplotlib-interactive-green.svg)
![License](https://img.shields.io/badge/License-MIT-lightgrey.svg)

Interactive educational demonstrations of **camera projection, image formation, lens distortion, two-view geometry, and Structure from Motion concepts**.

The collection combines geometric visualizations with adjustable parameters and live numerical feedback.

## Camera Projection and Image Formation

- [Pinhole Projection]({{ '/photogrammetry/pinhole-projection/' | relative_url }})  
  Perspective projection of a 3D point onto a virtual image plane.

- [Scale Ambiguity]({{ '/photogrammetry/scale-ambiguity/' | relative_url }})  
  Different object sizes and distances producing the same projected shape.

- [Thin Lens: Focus and Blur]({{ '/photogrammetry/thin-lens/' | relative_url }})  
  Principal rays, image distance, sensor placement, and geometric defocus.

## Two-View Geometry and Reconstruction Concepts

- [Epipolar Geometry]({{ '/photogrammetry/epipolar-geometry/' | relative_url }})  
  Baseline, epipolar plane, epipoles, and epipolar lines.

- [Structure from Motion: Workflow Visualization]({{ '/photogrammetry/structure-from-motion/' | relative_url }})  
  A conceptual animation of camera registration, multiscale search, and view rejection.

## Lens Distortion and Chromatic Effects

- [Radial Distortion: Single Coefficient]({{ '/photogrammetry/radial-distortion-k1/' | relative_url }})
- [Radial Distortion: Three-Coefficient Polynomial]({{ '/photogrammetry/radial-distortion-polynomial/' | relative_url }})
- [Tangential / Decentering Distortion]({{ '/photogrammetry/tangential-distortion/' | relative_url }})
- [Thin-Prism Distortion]({{ '/photogrammetry/thin-prism-distortion/' | relative_url }})
- [Chromatic Aberration: RGB Channel Warping]({{ '/photogrammetry/chromatic-aberration/' | relative_url }})

## Suggested Learning Order

1. Start with pinhole projection.
2. Explore size–distance ambiguity.
3. Compare pinhole geometry with thin-lens image formation.
4. Study radial, tangential, thin-prism, and chromatic effects.
5. Explore epipolar geometry.
6. Finish with the conceptual SfM workflow.

## Installation

From the repository root:

```bash
python -m pip install -r requirements.txt
```

These simulators use **NumPy** and **Matplotlib**.

Run them locally with a graphical Matplotlib backend. The website documents the applications; it does not execute the Python interfaces in the browser.

Each simulator page provides its repository-root launch command.

## Scope

These tools are intended for learning and visual exploration.

They are not a complete camera-calibration, optical-design, or photogrammetric reconstruction package. In particular, the SfM demonstration uses predefined scene geometry and scripted outcomes rather than estimating a reconstruction from images.

## Source and License

- [Photogrammetry source directory](https://github.com/KaramMawas/sensors_vision_simulators/tree/main/Photogrammetry)
- [Shared requirements](https://github.com/KaramMawas/sensors_vision_simulators/blob/main/requirements.txt)
- [MIT License](https://github.com/KaramMawas/sensors_vision_simulators/blob/main/LICENSE)
