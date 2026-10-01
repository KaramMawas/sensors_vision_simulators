---
layout: page
title: Epipolar Geometry
permalink: /photogrammetry/epipolar-geometry/
---

{% include mathjax.html %}

# Epipolar Geometry

> Explore the geometric relationship between two cameras observing the same 3D point.

[Back to Photogrammetry]({{ '/photogrammetry/' | relative_url }})

## Overview

Two fixed camera centers observe an adjustable 3D point. Each camera has an independently adjustable yaw angle and a virtual image plane.

The simulator constructs the epipolar plane, projects the scene point into each image plane, and displays the corresponding epipolar lines and finite epipoles.

## Topics

- two-view geometry
- camera baseline
- epipolar plane
- projected image points
- epipolar lines
- finite and infinite epipoles
- camera yaw
- line–plane intersection

## Scene Elements

- **Black markers:** camera centers.
- **Black connecting line:** baseline.
- **Blue point:** scene point.
- **Green dashed lines:** projection rays.
- **Cyan triangle:** a finite patch of the epipolar plane.
- **Gray surfaces:** virtual image planes.
- **Red and orange lines:** epipolar lines.
- **Purple crosses:** finite epipoles.

## Controls

- **Point P — X:** −30 to 30; default 0.
- **Point P — Y:** −20 to 30; default 15.
- **Point P — Z:** 10 to 60; default 30.
- **Camera A Pan:** −45° to 45°; default 15°.
- **Camera B Pan:** −45° to 45°; default −15°.

Camera centers remain fixed at:

$$
\mathbf{C}_A = (-15,0,0),
\qquad
\mathbf{C}_B = (15,0,0).
$$

The focal length is fixed at 10 simulation units.

## How It Works

The camera centers and scene point define the epipolar plane. Its normal is proportional to:

$$
\mathbf{n}_{\mathrm{epi}}
\propto
(\mathbf{C}_B-\mathbf{C}_A)
\times
(\mathbf{P}-\mathbf{C}_A).
$$

The intersection of this plane with each image plane defines an epipolar line.

An epipole is the intersection of the baseline, extended as necessary, with an image plane. Equivalently, it is the projection of the other camera center.

Point projections and finite epipoles are calculated using line–plane intersection:

$$
t =
\frac{(\mathbf{S}-\mathbf{C})\cdot\mathbf{n}}
{\mathbf{d}\cdot\mathbf{n}},
\qquad
\mathbf{Q} = \mathbf{C}+t\mathbf{d},
$$

where the plane passes through $$\mathbf{S}$$ with normal $$\mathbf{n}$$.

## Try It

1. Move the scene point vertically and observe the epipolar plane rotate around the baseline.
2. Change one camera's yaw and inspect its image plane and epipole.
3. Set a camera's yaw to zero.

At zero yaw in this setup, the baseline is parallel to that camera's image plane. Its epipole is at infinity and is not drawn as a finite marker.

## Assumptions and Limitations

- Image planes are virtual planes in front of the cameras.
- The intersection function uses an unrestricted line parameter; it does not reject intersections behind a camera.
- Finite image-plane patches are illustrative, not enforced sensor boundaries.
- Epipolar lines are drawn as finite segments, not clipped to the image-plane patches.
- Fixed plot limits can hide distant epipoles.
- No fundamental-matrix estimation, image matching, rectification, or triangulation is performed.

## Run

From the repository root:

```bash
python Photogrammetry/Epipolar/src/epipolar_geometry.py
```

## Requirements

```bash
python -m pip install -r requirements.txt
```

Requires NumPy, Matplotlib, and a graphical Matplotlib backend.

## Source and License

- [Source code](https://github.com/KaramMawas/sensors_vision_simulators/blob/main/Photogrammetry/Epipolar/src/epipolar_geometry.py)
- [MIT License](https://github.com/KaramMawas/sensors_vision_simulators/blob/main/LICENSE)
