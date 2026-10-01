---
layout: page
title: Thin Lens — Focus and Blur
permalink: /photogrammetry/thin-lens/
---

{% include mathjax.html %}

# Thin Lens: Focus and Blur

> Investigate how object distance, focal length, and sensor position determine focus.

[Back to Photogrammetry]({{ '/photogrammetry/' | relative_url }})

## Overview

This simulator draws a two-dimensional thin-lens ray diagram.

The object lies to the right of the lens, and the sensor lies to the left. Three principal rays illustrate image formation and their separation at the current sensor position.

## Topics

- thin-lens equation
- focal points
- principal rays
- image inversion
- real and virtual image conditions
- sensor placement
- geometric defocus

## Simulation Layout

- **Blue vertical line:** idealized lens at the origin.
- **Black object marker:** object height and position.
- **Red points:** focal points.
- **Colored lines:** three principal rays.
- **Gray vertical line:** sensor plane.
- **Red sensor rectangle:** illustrative ray spread during defocus.
- **Status panel:** image distance and focus state.

## Controls

- **Object Height:** 1 to 10; default 4.
- **Object Distance:** 6 to 30; default 15.
- **Focal Length:** 2 to 10; default 4.
- **Sensor Position:** 2 to 20; default 5.45.
- **Auto-Focus:** sets the sensor distance to the calculated real-image distance when the object is beyond the focal point.

## Thin-Lens Model

Using positive object and real-image distances:

$$
\frac{1}{f} = \frac{1}{Z}+\frac{1}{v}.
$$

Therefore:

$$
v = \frac{fZ}{Z-f}.
$$

For an object beyond the focal point, the real image forms at distance $$v$$ on the opposite side of the lens.

Its signed height is:

$$
Y_{\mathrm{image}} = -\frac{v}{Z}Y.
$$

The negative sign indicates inversion.

## Principal Rays

The diagram includes:

1. A ray parallel to the optical axis that refracts through the image-side focal point.
2. A ray through the lens center that continues undeviated.
3. A ray directed through the object-side focal point that emerges parallel to the optical axis.

## Focus Indicator

The interface displays **SHARP FOCUS** when:

$$
|S-v| < 0.15,
$$

where $$S$$ is the sensor distance.

Otherwise, the red rectangle spans the minimum and maximum heights of the three illustrated rays at the sensor.

## Try It

1. Press **Auto-Focus** with the initial object and focal-length settings.
2. Move the sensor away from the calculated image distance.
3. Change object distance and refocus.
4. Increase focal length while keeping the object beyond the focal point.

## Assumptions and Limitations

- The lens is idealized and has no thickness or aberrations.
- Aperture diameter is not modeled.
- The red rectangle is a ray-spread illustration, not a calibrated circle of confusion.
- The focus tolerance is a chosen simulation threshold, not an optical sharpness criterion.
- Near the focal singularity, the script perturbs the object distance numerically.
- Objects inside the focal distance produce a virtual-image condition, but the interface is primarily designed to illustrate real-image focusing.
- Auto-Focus is an analytical sensor-position update, not an image-based autofocus algorithm.
- Auto-Focus values are not explicitly restricted to the sensor slider's nominal range.

## Run

From the repository root:

```bash
python Photogrammetry/thin_lens_sim/src/thin_lens_simulator.py
```

## Requirements

```bash
python -m pip install -r requirements.txt
```

Requires NumPy, Matplotlib, and a graphical Matplotlib backend.

## Source and License

- [Source code](https://github.com/KaramMawas/sensors_vision_simulators/blob/main/Photogrammetry/thin_lens_sim/src/thin_lens_simulator.py)
- [MIT License](https://github.com/KaramMawas/sensors_vision_simulators/blob/main/LICENSE)
