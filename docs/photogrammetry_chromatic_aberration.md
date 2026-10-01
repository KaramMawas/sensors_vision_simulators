---
layout: page
title: Chromatic Aberration
permalink: /photogrammetry/chromatic-aberration/
---

{% include mathjax.html %}

# Chromatic Aberration: RGB Channel Warping

> Visualize color fringing caused by different spatial warps in the red, green, and blue channels.

[Back to Photogrammetry]({{ '/photogrammetry/' | relative_url }})

## Overview

A synthetic checkerboard is warped independently in each RGB channel.

Differences between the channel mappings create colored fringes near contrast boundaries, illustrating the appearance of lateral chromatic aberration.

A full-image view and a magnified top-left region show the effects at different scales.

## Topics

- lateral chromatic aberration concepts
- channel-dependent radial mapping
- color fringing at image edges
- reference-channel selection
- normalized image coordinates
- integer-index image resampling
- boundary clipping

## Simulation Layout

- **Full Sensor View:** the complete 600 × 600 image.
- **Edge Zoom:** the top-left region covering approximately the first 120 pixels along each axis.
- **Status title:** indicates whether the green channel is left unwarped.

## Controls

Each channel coefficient ranges from −0.1 to 0.1:

- **Red Scale:** default 0.03.
- **Green Scale:** default 0.
- **Blue Scale:** default −0.03.
- **Reset:** restores these initial values.

Reset therefore restores the initial color-fringing example, not an undistorted image.

## Implemented Warp

Output pixel coordinates are normalized to the range from −1 to 1.

For each channel:

$$
r^2 = x_{\mathrm{n}}^2+y_{\mathrm{n}}^2,
$$

$$
a_c = 1+k_c r^2,
$$

$$
x_{\mathrm{sample},c} = \frac{x_{\mathrm{n}}}{a_c},
\qquad
y_{\mathrm{sample},c} = \frac{y_{\mathrm{n}}}{a_c}.
$$

The resulting sampling coordinates are converted back to pixel indices, truncated to integers, and clipped to the image boundaries.

This is the sampling rule implemented by the script. It should not be interpreted as an exact analytical inverse of a general forward radial-distortion model.

## Interpreting the Green Reference

When the green coefficient is zero, the green channel retains the original coordinate mapping.

The interface calls this “stable geometry.” More precisely, it means that one channel remains an unwarped reference. It does not imply that the camera is calibrated or that the other channels are geometrically correct.

## Try It

1. Set all three coefficients to zero.
2. Increase only the red coefficient.
3. Give red and blue coefficients opposite signs.
4. Set all coefficients to the same value.

Equal coefficients apply the same spatial warp to every channel, removing relative RGB displacement even though the image may remain geometrically distorted.

## Assumptions and Limitations

- This is a simplified lateral color-misalignment demonstration.
- No wavelengths, dispersion properties, or optical materials are modeled.
- Longitudinal chromatic aberration and wavelength-dependent blur are not simulated.
- Integer sampling can introduce aliasing.
- Boundary clipping can create edge artifacts.
- No image-loading, correction, or calibration workflow is provided.

## Run

From the repository root:

```bash
python Photogrammetry/Lens_Distortion/Chromatic_Aberration/src/chromatic_aberration.py
```

## Requirements

```bash
python -m pip install -r requirements.txt
```

Requires NumPy, Matplotlib, and a graphical Matplotlib backend.

## Source and License

- [Source code](https://github.com/KaramMawas/sensors_vision_simulators/blob/main/Photogrammetry/Lens_Distortion/Chromatic_Aberration/src/chromatic_aberration.py)
- [MIT License](https://github.com/KaramMawas/sensors_vision_simulators/blob/main/LICENSE)
