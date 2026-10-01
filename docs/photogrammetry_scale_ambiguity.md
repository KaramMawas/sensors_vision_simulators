---
layout: page
title: Scale Ambiguity
permalink: /photogrammetry/scale-ambiguity/
---

{% include mathjax.html %}

# Scale Ambiguity

> Discover why projected size alone does not uniquely determine an object's true size and distance.

[Back to Photogrammetry]({{ '/photogrammetry/' | relative_url }})

## Overview

This simulator displays a square object in a 3D scene and its projected footprint in a separate 2D view.

With the projected-shape lock enabled, selected parameters adjust together to keep the image unchanged. With the lock disabled, focal length, object size, and distance can be varied independently.

## Topics

- perspective image scale
- size–distance ambiguity
- focal length and magnification
- coupled parameter adjustment
- physical image-plane inversion
- viewing-axis conventions

## Simulation Layout

- **3D Scene Explorer:** camera center, square object, image plane, and projection rays.
- **2D Sensor Image:** projected square and numerical size calculation.

The image plane is placed behind the camera center, producing an inverted projection. Because the object is a centered square, inversion is not visually distinctive in the 2D footprint.

## Controls

- **Focal Length:** 5 to 30; default 15.
- **Distance:** 10 to 100; default 40.
- **True Size:** 1 to 50; default 25.
- **Lock Projected Shape:** enabled initially.
- **Viewing axis:** Z, X, or Y.

## Projection Model

For a frontoparallel object:

$$
s_{\mathrm{image}} = f\frac{S}{D},
$$

where:

- $$f$$ is focal length.
- $$S$$ is the object's side length.
- $$D$$ is depth along the selected viewing axis.
- $$s_{\mathrm{image}}$$ is the projected side length.

For fixed focal length, scaling size and distance by the same positive factor leaves the projection unchanged:

$$
f\frac{\lambda S}{\lambda D}
=
f\frac{S}{D},
\qquad \lambda > 0.
$$

Thus, projected size alone cannot distinguish a small nearby object from a proportionally larger distant one.

## How the Lock Works

When the lock is enabled:

- Changing **distance** adjusts object size.
- Changing **object size** adjusts distance.
- Changing **focal length** adjusts object size.

The script preserves the projected size stored when the lock is activated.

Changing focal length illustrates an additional parameter trade-off; focal length need not be unknown for size–distance ambiguity to exist.

## Try It

1. Leave the lock enabled and increase distance.
2. Observe the object grow while its projected square remains unchanged.
3. Disable the lock and repeat the distance change.
4. Switch viewing axes to see the same principle in different coordinate orientations.

## Assumptions and Limitations

- The object is a centered, frontoparallel square.
- The interface label “Pixel Size” refers to image-plane units, not calibrated pixels.
- This is a single-view illustration, not a full multiview reconstruction.
- Programmatically coupled values are not explicitly clamped to slider bounds.
- Fixed image-view limits can clip large projections.

## Run

From the repository root:

```bash
python Photogrammetry/Scale_Ambiguity/src/scale_ambiguity.py
```

## Requirements

```bash
python -m pip install -r requirements.txt
```

Requires NumPy, Matplotlib, and a graphical Matplotlib backend.

## Source and License

- [Source code](https://github.com/KaramMawas/sensors_vision_simulators/blob/main/Photogrammetry/Scale_Ambiguity/src/scale_ambiguity.py)
- [MIT License](https://github.com/KaramMawas/sensors_vision_simulators/blob/main/LICENSE)
