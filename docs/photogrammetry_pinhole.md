---
layout: page
title: Pinhole Camera Projection
permalink: /photogrammetry/pinhole-projection/
---

{% include mathjax.html %}

# Pinhole Camera Projection

![Python](https://img.shields.io/badge/Python-3.x-blue.svg)
![Matplotlib](https://img.shields.io/badge/Matplotlib-interactive-green.svg)
![Status](https://img.shields.io/badge/Status-Educational_Simulation-success.svg)
![License](https://img.shields.io/badge/License-MIT-lightgrey.svg)

> Explore pinhole image formation using a 2D ray diagram and an interactive 3D projection scene.

[Back to Photogrammetry]({{ '/photogrammetry/' | relative_url }})

## Overview

This module contains two complementary simulators:

- **2D Pinhole Camera:** illustrates image inversion, projected size, and the relationship between object height and distance.
- **3D Pinhole Projection:** illustrates perspective division and projection onto different virtual image planes.

Both place the camera center at the origin and update the visualization as parameters change.

## Topics

- pinhole camera geometry
- perspective projection
- focal length and magnification
- object size and depth
- size–distance ambiguity
- image inversion
- physical and virtual image-plane conventions
- camera-centered coordinates
- viewing-axis selection

## Scripts

- **2D:** `Photogrammetry/Pin_hole_projection/src/pinhole_sim.py`
- **3D:** `Photogrammetry/Pin_hole_projection/src/pinhole_projection.py`

---

## 2D Pinhole Camera

### Purpose

The 2D simulator shows how light passing through a pinhole forms an inverted image on a plane behind the camera center.

Adjusting focal length, object distance, and object height demonstrates their effect on image magnification.

The interface is titled **Pinhole Camera Z-Ambiguity Simulator**. It supports manual exploration of size–distance ambiguity; it does not automatically preserve projected size.

### Simulation Layout

- **Black point:** pinhole at the origin.
- **Black dashed vertical line:** plane containing the pinhole.
- **Blue segment:** object in front of the camera.
- **Gray dotted vertical line:** image plane behind the pinhole.
- **Red segment:** inverted projected image.
- **Green dashed lines:** projection rays.
- **Calculation panel:** current projected-size magnitude.

The horizontal plot axis represents distance along the optical axis. The vertical plot axis represents height.

### Controls

- **Focal Length:** 2 to 15; default 5.
- **Distance:** 5 to 50; default 20.
- **Object Height:** 1 to 30; default 10.

All three sliders are independent.

### Projection Model

Let:

- $$X$$ be object height.
- $$Z$$ be object distance.
- $$f$$ be the distance from the pinhole to the image plane.

The object is placed at positive optical-axis distance, while the image plane is located at:

$$
z_{\mathrm{image}}=-f.
$$

By similar triangles, the signed image height is:

$$
x_{\mathrm{image}}=-f\frac{X}{Z}.
$$

The negative sign represents image inversion.

The interface displays the positive projected-size magnitude:

$$
s_{\mathrm{image}}
=
\left|x_{\mathrm{image}}\right|
=
f\frac{X}{Z},
$$

because the available object heights, distances, and focal lengths are positive.

At the initial settings:

$$
s_{\mathrm{image}}
=
5\frac{10}{20}
=
2.5.
$$

The red image segment therefore extends from zero to a height of:

$$
x_{\mathrm{image}}=-2.5.
$$

### Magnification

Signed magnification is:

$$
m=\frac{x_{\mathrm{image}}}{X}
=-\frac{f}{Z}.
$$

Consequently:

- Increasing focal length increases projected size.
- Increasing object distance decreases projected size.
- Increasing object height increases projected size.
- The negative magnification indicates inversion.

In this ideal pinhole model, the control labeled “Focal Length” represents the pinhole-to-image-plane distance. There is no focusing lens.

### Size–Distance Ambiguity

For fixed focal length, multiplying object height and distance by the same positive factor preserves the image:

$$
f\frac{\lambda X}{\lambda Z}
=
f\frac{X}{Z},
\qquad \lambda>0.
$$

For example, both configurations below produce a projected-size magnitude of 2.5:

- Focal length 5, object height 10, distance 20.
- Focal length 5, object height 20, distance 40.

The sliders must be adjusted separately, so the image may change temporarily between adjustments.

For an automatic projected-shape lock, see the [Scale Ambiguity simulator]({{ '/photogrammetry/scale-ambiguity/' | relative_url }}).

### Try It

1. Observe the initial inverted image.
2. Increase focal length while keeping the object fixed.
3. Restore focal length to 5.
4. Change object height to 20 and distance to 40.
5. Confirm that the final projected size matches the initial value.
6. Keep object height fixed and change distance alone.

### Limitations

- The diagram is a two-dimensional cross-section.
- All quantities use consistent arbitrary units, not calibrated pixels.
- The pinhole is idealized; aperture size and diffraction are absent.
- Lens focusing, defocus, and distortion are not modeled.
- The fixed vertical display range can clip large inverted images.
- Clipping by the plot boundary is not a physical sensor-boundary calculation.

---

## 3D Pinhole Projection

### Purpose

The 3D simulator shows how a movable scene point maps onto a virtual image plane in front of the camera.

Switching the viewing axis demonstrates that perspective projection follows the same principle under different coordinate conventions.

### Simulation Layout

- **Black point:** camera center at the origin.
- **Blue point:** scene point.
- **Green dashed segment:** line from the camera center to the scene point.
- **Translucent surface:** virtual image plane.
- **Red point:** projected position.
- **Coordinate label:** projected-point coordinates.
- **Calculation panel:** active equations and numerical results.

### Controls

- **Focal Length:** 2 to 15; default 5.
- **3D Point X:** −25 to 25; default 10.
- **3D Point Y:** −25 to 25; default 5.
- **3D Point Z:** 1 to 25; default 15.
- **Viewing axis:**
  - Z-axis: image plane parallel to X–Y.
  - X-axis: image plane parallel to Y–Z.

### Z-Axis Projection

The virtual image plane is located at:

$$
z=f.
$$

The projected coordinates are:

$$
x_{\mathrm{proj}}=f\frac{X}{Z},
\qquad
y_{\mathrm{proj}}=f\frac{Y}{Z},
\qquad
z_{\mathrm{proj}}=f.
$$

The coordinate along the viewing axis provides the depth denominator.

### X-Axis Projection

The virtual image plane is located at:

$$
x=f.
$$

The projected coordinates are:

$$
x_{\mathrm{proj}}=f,
\qquad
y_{\mathrm{proj}}=f\frac{Y}{X},
\qquad
z_{\mathrm{proj}}=f\frac{Z}{X}.
$$

Here, the X coordinate supplies the depth denominator.

### Try It

1. Keep the point fixed and increase focal length.
2. In Z-axis mode, increase depth and observe the projected point move toward the image center.
3. Switch to X-axis mode.
4. Compare the new projection equations with those in Z-axis mode.
5. Move the X coordinate close to zero and observe the rapidly increasing projected coordinates.

### Limitations

- Coordinates are expressed in arbitrary units.
- Pixel coordinates and camera-intrinsic calibration are not implemented.
- Lens distortion and finite sensor boundaries are absent.
- In X-axis mode, negative depths are permitted mathematically; points behind the camera are not rejected.
- At exactly zero depth, the code substitutes a small denominator. The underlying projection remains geometrically undefined.
- Fixed plot limits can hide large projected coordinates.
- The displayed ray segment ends at the object point and is not always extended to a projected point beyond that segment.

---

## Comparing the Image-Plane Conventions

The simulators use different image-plane placements intentionally.

### Physical Image Plane: 2D Simulator

The image plane lies behind the pinhole:

$$
z_{\mathrm{image}}=-f.
$$

The signed image height is:

$$
x_{\mathrm{image}}=-f\frac{X}{Z}.
$$

This convention illustrates the inverted image formed by a physical pinhole camera.

### Virtual Image Plane: 3D Simulator

For Z-axis viewing, the image plane lies in front of the camera:

$$
z_{\mathrm{image}}=f.
$$

The projected coordinate is:

$$
x_{\mathrm{proj}}=f\frac{X}{Z}.
$$

This is a common geometric convention that avoids image inversion in the projection equations.

Both conventions follow the same central-projection geometry. Their sign difference results from placing the image plane on opposite sides of the camera center.

---

## Run

Execute these commands from the repository root.

### 2D Pinhole Camera

```bash
python Photogrammetry/Pin_hole_projection/src/pinhole_sim.py
```

### 3D Pinhole Projection

```bash
python Photogrammetry/Pin_hole_projection/src/pinhole_projection.py
```

Each script opens its own interactive Matplotlib window.

## Requirements

Install the shared dependencies from the repository root:

```bash
python -m pip install -r requirements.txt
```

- **2D simulator:** Matplotlib.
- **3D simulator:** NumPy and Matplotlib.
- Both require a graphical Matplotlib backend.

These are local Python applications. This documentation page does not run the simulators in the browser.

## Related Simulators

- [Scale Ambiguity]({{ '/photogrammetry/scale-ambiguity/' | relative_url }}) — preserve the projected shape using coupled parameter adjustments.
- [Thin Lens: Focus and Blur]({{ '/photogrammetry/thin-lens/' | relative_url }}) — compare pinhole geometry with lens-based image formation.
- [Epipolar Geometry]({{ '/photogrammetry/epipolar-geometry/' | relative_url }}) — extend central projection to a two-camera configuration.

## Source and License

- [2D pinhole source code](https://github.com/KaramMawas/sensors_vision_simulators/blob/main/Photogrammetry/Pin_hole_projection/src/pinhole_sim.py)
- [3D projection source code](https://github.com/KaramMawas/sensors_vision_simulators/blob/main/Photogrammetry/Pin_hole_projection/src/pinhole_projection.py)
- [Shared requirements](https://github.com/KaramMawas/sensors_vision_simulators/blob/main/requirements.txt)
- [MIT License](https://github.com/KaramMawas/sensors_vision_simulators/blob/main/LICENSE)
