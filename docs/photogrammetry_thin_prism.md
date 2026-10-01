---
layout: page
title: Thin-Prism Distortion
permalink: /photogrammetry/thin-prism-distortion/
---

{% include mathjax.html %}

# Thin-Prism Distortion

> Explore a simplified asymmetric distortion model with radius-weighted horizontal and vertical displacement.

[Back to Photogrammetry]({{ '/photogrammetry/' | relative_url }})

## Overview

Unlike radial scaling, this simulator adds separate coordinate offsets that grow with squared distance from the origin.

The resulting deformation illustrates a simplified thin-prism contribution to an image-coordinate distortion model.

## Topics

- thin-prism distortion concepts
- asymmetric coordinate displacement
- radius-dependent offsets
- independent horizontal and vertical coefficients
- ideal-grid comparison
- coefficient-convention differences

## Simulation Layout

- **Blue curves:** transformed grid.
- **Gray dotted lines:** ideal reference grid.
- **Red cross:** coordinate origin.
- **Calculation panel:** horizontal and vertical displacement at the corner point.

The grid contains 11 horizontal and 11 vertical lines, with 60 samples per line.

## Controls

- **Thin Prism s1:** −0.15 to 0.15; default 0.04.
- **Thin Prism s2:** −0.15 to 0.15; default 0.02.
- **Show Ideal Pinhole Grid:** enabled initially.

## Implemented Model

The script defines:

$$
r^2=x^2+y^2,
$$

$$
\Delta x=s_1r^2,
\qquad
\Delta y=s_2r^2.
$$

The transformed coordinates are:

$$
x_{\mathrm{dist}}=x+\Delta x,
\qquad
y_{\mathrm{dist}}=y+\Delta y.
$$

At the corner point:

$$
(x,y)=(1,1),
$$

the displacement becomes:

$$
\Delta x=2s_1,
\qquad
\Delta y=2s_2.
$$

Points at the same radius receive the same displacement vector, regardless of their angular position.

## Coefficient Convention

The script uses two coefficients: one for horizontal squared-radius displacement and one for vertical squared-radius displacement.

Some calibration libraries use a four-coefficient convention:

$$
\Delta x=s_1r^2+s_2r^4,
$$

$$
\Delta y=s_3r^2+s_4r^4.
$$

Under that convention, this script's second coefficient corresponds to the vertical squared-radius term, not the horizontal fourth-power term. Coefficient names should therefore not be transferred between implementations without checking their equations.

## Try It

1. Set both coefficients to zero.
2. Increase only the horizontal coefficient.
3. Increase only the vertical coefficient.
4. Reverse a coefficient's sign.
5. Compare points on opposite sides of the origin.

## Assumptions and Limitations

- The model includes only one squared-radius offset per coordinate.
- It does not combine radial or tangential distortion.
- No physical prism parameters or calibration procedure are implemented.
- The interface mentions “Sensor Tilt,” but the equations do not implement a geometric tilted-sensor projection.
- Fixed plot limits can hide parts of strongly displaced curves.

## Run

From the repository root:

```bash
python Photogrammetry/Lens_Distortion/Thin_Prism/src/thin_prism_distortion.py
```

## Requirements

```bash
python -m pip install -r requirements.txt
```

Requires NumPy, Matplotlib, and a graphical Matplotlib backend.

## Source and License

- [Source code](https://github.com/KaramMawas/sensors_vision_simulators/blob/main/Photogrammetry/Lens_Distortion/Thin_Prism/src/thin_prism_distortion.py)
- [MIT License](https://github.com/KaramMawas/sensors_vision_simulators/blob/main/LICENSE)
