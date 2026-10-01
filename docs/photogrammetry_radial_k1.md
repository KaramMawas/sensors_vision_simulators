---
layout: page
title: Radial Distortion — Single Coefficient
permalink: /photogrammetry/radial-distortion-k1/
---

{% include mathjax.html %}

# Radial Distortion: Single-Coefficient Model

> Explore the first radial-distortion term and its effect on an ideal image grid.

[Back to Photogrammetry]({{ '/photogrammetry/' | relative_url }})

## Overview

This simulator transforms an ideal grid using one radial coefficient.

An optional reference overlay makes it easy to compare the original straight lines with the transformed curves. A calculation panel evaluates the mapping at the corner point.

## Topics

- radial lens distortion
- first radial coefficient
- barrel and pincushion distortion
- normalized image coordinates
- radial displacement
- ideal-grid comparison

## Simulation Layout

- **Blue curves:** transformed grid.
- **Gray dotted lines:** ideal reference grid.
- **Red cross:** origin, used as the distortion center.
- **Calculation panel:** numerical example for the corner point.

The grid contains 11 horizontal and 11 vertical lines, with 60 samples per line.

## Controls

- **Radial Coefficient:** −0.5 to 0.5; default −0.18.
- **Show Ideal Pinhole Grid:** enabled initially.

## Mathematical Model

For ideal normalized coordinates:

$$
r^2 = x^2+y^2.
$$

The forward mapping is:

$$
x_{\mathrm{dist}} = x(1+k_1r^2),
$$

$$
y_{\mathrm{dist}} = y(1+k_1r^2).
$$

Within a physically sensible parameter range:

- A negative coefficient moves points inward, producing barrel-like distortion.
- A positive coefficient moves points outward, producing pincushion-like distortion.
- A zero coefficient preserves the original grid.

For the sampled corner:

$$
(x,y)=(1,1),
\qquad r^2=2,
$$

so:

$$
x_{\mathrm{dist}}=y_{\mathrm{dist}}=1+2k_1.
$$

## Try It

1. Set the coefficient to zero.
2. Move gradually toward negative values.
3. Compare the center and outer grid regions.
4. Move toward positive values.
5. Toggle the reference grid to isolate the transformed pattern.

## Assumptions and Limitations

- The distortion center is fixed at the coordinate origin.
- Only the first radial term is implemented.
- Coordinates are normalized model coordinates, not measured pixels.
- No camera calibration or image resampling is performed.
- Large negative values can make the radial mapping non-monotonic; the full slider range is not guaranteed to represent a physically valid lens.
- Fixed plot limits can clip outward-displaced curves.
- The title uses a small coefficient threshold to label near-zero distortion; only an exactly zero coefficient gives the identity mapping.

## Related Simulator

[Three-coefficient radial distortion]({{ '/photogrammetry/radial-distortion-polynomial/' | relative_url }}) adds higher-order radial terms.

## Run

From the repository root:

```bash
python Photogrammetry/Lens_Distortion/Radial_K1/src/radial_distortion_k1.py
```

## Requirements

```bash
python -m pip install -r requirements.txt
```

Requires NumPy, Matplotlib, and a graphical Matplotlib backend.

## Source and License

- [Source code](https://github.com/KaramMawas/sensors_vision_simulators/blob/main/Photogrammetry/Lens_Distortion/Radial_K1/src/radial_distortion_k1.py)
- [MIT License](https://github.com/KaramMawas/sensors_vision_simulators/blob/main/LICENSE)
