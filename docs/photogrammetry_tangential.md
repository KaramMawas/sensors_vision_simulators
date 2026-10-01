---
layout: page
title: Tangential / Decentering Distortion
permalink: /photogrammetry/tangential-distortion/
---

{% include mathjax.html %}

# Tangential / Decentering Distortion

> Explore asymmetric grid deformation produced by a two-coefficient decentering model.

[Back to Photogrammetry]({{ '/photogrammetry/' | relative_url }})

## Overview

This simulator applies the tangential component of a commonly used camera-distortion model to an ideal grid.

Unlike purely radial scaling, its coordinate offsets depend on both position and cross-coordinate terms. The result is an asymmetric deformation that changes as the two coefficients are adjusted.

## Topics

- tangential distortion
- decentering distortion
- asymmetric image deformation
- cross-coordinate coupling
- normalized coordinates
- ideal-grid comparison
- corner-point displacement

## Simulation Layout

- **Blue curves:** transformed grid.
- **Gray dotted lines:** ideal reference grid.
- **Red cross:** coordinate origin.
- **Calculation panel:** displacement and transformed coordinates at the corner point.

The grid contains 11 horizontal and 11 vertical lines, with 60 samples per line.

## Controls

- **Tangential p1:** −0.2 to 0.2; default 0.05.
- **Tangential p2:** −0.2 to 0.2; default 0.03.
- **Show Ideal Pinhole Grid:** enabled initially.

## Mathematical Model

For ideal normalized coordinates:

$$
r^2=x^2+y^2.
$$

The tangential offsets are:

$$
\Delta x=2p_1xy+p_2(r^2+2x^2),
$$

$$
\Delta y=p_1(r^2+2y^2)+2p_2xy.
$$

The transformed coordinates are:

$$
x_{\mathrm{dist}}=x+\Delta x,
\qquad
y_{\mathrm{dist}}=y+\Delta y.
$$

At the sampled corner:

$$
(x,y)=(1,1),
$$

the equations reduce to:

$$
\Delta x=2p_1+4p_2,
\qquad
\Delta y=4p_1+2p_2.
$$

Both coefficients influence both coordinate components.

## Physical Interpretation

Decentering distortion is commonly associated with imperfect alignment of optical elements.

This simulator demonstrates an image-coordinate model rather than a mechanical lens model. The coefficients are not physical tilt angles, and the interface's “Asymmetric Tilt” label should be interpreted qualitatively.

## Try It

1. Set both coefficients to zero.
2. Change only the first coefficient.
3. Return it to zero and change only the second.
4. Reverse the coefficient signs.
5. Compare the result with the radial-distortion simulator.

## Assumptions and Limitations

- Only the tangential component is implemented.
- Radial and thin-prism contributions are absent.
- No camera calibration or photograph correction is performed.
- Coordinates are normalized model coordinates, not measured pixels.
- The title treats small coefficients as approximately aligned; only two exactly zero coefficients remove the modeled distortion entirely.
- Fixed plot limits can clip strongly displaced grid curves.

## Related Simulators

- [Single-coefficient radial distortion]({{ '/photogrammetry/radial-distortion-k1/' | relative_url }})
- [Thin-prism distortion]({{ '/photogrammetry/thin-prism-distortion/' | relative_url }})

## Run

From the repository root:

```bash
python Photogrammetry/Lens_Distortion/Tangential_Decentering/src/tangential_distortion.py
```

## Requirements

```bash
python -m pip install -r requirements.txt
```

Requires NumPy, Matplotlib, and a graphical Matplotlib backend.

## Source and License

- [Source code](https://github.com/KaramMawas/sensors_vision_simulators/blob/main/Photogrammetry/Lens_Distortion/Tangential_Decentering/src/tangential_distortion.py)
- [MIT License](https://github.com/KaramMawas/sensors_vision_simulators/blob/main/LICENSE)
