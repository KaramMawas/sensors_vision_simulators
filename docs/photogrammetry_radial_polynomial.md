---
layout: page
title: Radial Distortion — Three-Coefficient Model
permalink: /photogrammetry/radial-distortion-polynomial/
---

{% include mathjax.html %}

# Radial Distortion: Three-Coefficient Model

> Investigate how multiple radial terms combine to reshape an image grid.

[Back to Photogrammetry]({{ '/photogrammetry/' | relative_url }})

## Overview

This simulator extends the single-coefficient radial model with fourth- and sixth-power radius terms.

Independent sliders reveal how competing terms can produce complex deformation, including mixed barrel-like and pincushion-like behavior across the image.

## Topics

- polynomial radial distortion
- first, second, and third radial coefficients
- higher-order edge effects
- barrel and pincushion patterns
- mixed “mustache” distortion
- radial scaling-factor analysis
- mapping validity

## Simulation Layout

- **Blue curves:** transformed grid.
- **Gray dotted lines:** ideal reference grid.
- **Red cross:** fixed distortion center.
- **Calculation panel:** radial scaling factor and corner-point example.

The grid contains 13 horizontal and 13 vertical lines, with 70 samples per line.

## Controls

All three coefficients range from −0.5 to 0.5:

- **First coefficient:** default −0.15.
- **Second coefficient:** default 0.10.
- **Third coefficient:** default 0.05.
- **Show Ideal Pinhole Grid:** enabled initially.

## Mathematical Model

Define the radial scaling factor:

$$
L(r)=1+k_1r^2+k_2r^4+k_3r^6,
$$

where:

$$
r^2=x^2+y^2.
$$

The forward mapping is:

$$
x_{\mathrm{dist}}=xL(r),
\qquad
y_{\mathrm{dist}}=yL(r).
$$

At the sampled corner:

$$
(x,y)=(1,1),
\qquad
r^2=2,
$$

so:

$$
L(r)=1+2k_1+4k_2+8k_3.
$$

This illustrates how higher-order terms can have a strong effect toward the outer image region.

## Understanding Mixed Distortion

Opposing coefficient signs can cause the radial scale to behave differently at different radii.

The initial settings combine a negative first coefficient with positive higher-order coefficients. This produces inward displacement over part of the domain and outward displacement farther from the center.

The interface's category label is based on coefficient thresholds, not a complete analysis of the radial mapping.

## Try It

1. Set the second and third coefficients to zero.
2. Explore the first coefficient alone.
3. Add a second coefficient with the opposite sign.
4. Adjust the third coefficient and inspect the outer grid.
5. Compare the numerical corner calculation with the visible deformation.

## Assumptions and Limitations

- Only polynomial radial distortion is modeled.
- “Complete Radial Model” in the interface refers to this three-term implementation, not every possible radial model.
- Tangential, thin-prism, rational-denominator, and fisheye terms are absent.
- The distortion center is fixed at the origin.
- Extreme settings can produce negative scale factors, folding, or non-invertible mappings.
- The script does not test calibration validity or invert the distortion.
- Fixed plot limits can clip displaced curves.

## Related Simulator

[Single-coefficient radial distortion]({{ '/photogrammetry/radial-distortion-k1/' | relative_url }}) provides a simpler introduction.

## Run

From the repository root:

```bash
python Photogrammetry/Lens_Distortion/Radial_Polynomial/src/radial_distortion_polynomial.py
```

## Requirements

```bash
python -m pip install -r requirements.txt
```

Requires NumPy, Matplotlib, and a graphical Matplotlib backend.

## Source and License

- [Source code](https://github.com/KaramMawas/sensors_vision_simulators/blob/main/Photogrammetry/Lens_Distortion/Radial_Polynomial/src/radial_distortion_polynomial.py)
- [MIT License](https://github.com/KaramMawas/sensors_vision_simulators/blob/main/LICENSE)
