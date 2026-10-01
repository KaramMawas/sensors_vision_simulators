---
layout: page
title: Camera Parameters
permalink: /camera-parameters/
---

{% include mathjax.html %}

# Photography Camera Parameters Simulator

![Python](https://img.shields.io/badge/Python-3.x-blue.svg)
![NumPy](https://img.shields.io/badge/NumPy-supported-orange.svg)
![SciPy](https://img.shields.io/badge/SciPy-image_processing-blue.svg)
![Matplotlib](https://img.shields.io/badge/Matplotlib-interactive-green.svg)
![Status](https://img.shields.io/badge/Status-Educational_Simulation-success.svg)
![License](https://img.shields.io/badge/License-MIT-lightgrey.svg)

> An interactive photography simulator for exposure, focus, depth of field, motion blur, panning, image noise, and histogram behavior.

[Back to Modules]({{ '/modules/' | relative_url }})

## Overview

This simulator provides an interactive synthetic photography scene containing:

- a star-shaped foreground subject
- a checkerboard background
- simulated focus blur
- exposure scaling
- Gaussian-like sensor noise
- horizontal motion blur
- rotational motion blur
- panning behavior

The interface also displays standard photographic equations for exposure, hyperfocal distance, and depth-of-field limits.

> Important: the image renderer and the depth-of-field equation panel are intentionally separate educational models. The visual blur is not derived directly from the displayed circle of confusion or the physical lens settings.

---

## Topics

- exposure triangle
- aperture and f-number
- shutter speed
- ISO sensitivity
- relative exposure
- sensor noise
- histogram interpretation
- highlight and shadow clipping
- focal length
- circle of confusion
- hyperfocal distance
- depth of field
- focus distance
- visual defocus
- motion blur
- panning
- subject rotation

---

## Simulation Layout

### Viewfinder Result

The primary image panel displays the synthetic camera result.

- The **star** represents the foreground subject.
- The **checkerboard** represents the background.
- Star and background use separate fixed scene depths.
- Changes to exposure, focus, motion, panning, and noise update the rendered image.

### Histogram

The histogram shows the distribution of grayscale values from `0` to `1`.

- Values near `0` correspond to dark image regions.
- Values near `1` correspond to bright image regions.
- The diagnostic panel reports the percentage of nearly black and nearly white pixels.

### Equation Panel

The left information panel displays:

- relative exposure calculation
- exposure multiplier relative to a reference exposure
- hyperfocal distance
- near and far depth-of-field limits
- lens and focus settings
- motion travel distance
- clipping diagnostics

---

## Scene Model

The image is rendered on a `420 × 420` grid.

The simulation uses two fixed scene depths:

| Element | Scene depth |
|---|---:|
| Star subject | `5` |
| Checkerboard background | `10` |

These values are used only by the simplified visual-blur renderer.

They are not automatically linked to the physical focus distance used in the hyperfocal and depth-of-field calculations.

---

## Controls

## Exposure Triangle

### Aperture: f-number

The f-number slider ranges from `f/1.4` to `f/22`.

- Lower f-number: wider aperture, more relative exposure, shallower physical depth of field.
- Higher f-number: narrower aperture, less relative exposure, deeper physical depth of field.

### Shutter Speed

The shutter-speed slider ranges from `0.001` to `0.5` seconds.

- Shorter exposure: less light and less motion blur.
- Longer exposure: more light and more motion blur.

Longer shutter speeds also increase the number of time samples used for the motion-rendering approximation.

### ISO

The ISO slider ranges from `50` to `6400`.

- Lower ISO: darker result with less synthetic noise.
- Higher ISO: brighter result with more synthetic noise.

## Focus Controls

### Scene Focus for Rendering

This slider ranges from `1` to `10`.

It controls the visual focus model used to blur the star and checkerboard layers.

- A scene focus near `5` keeps the star relatively sharper.
- A scene focus near `10` keeps the checkerboard relatively sharper.

This is a scene-depth control for visualization, not a physical focus-distance setting.

### Physical Lens Parameters

The following controls are used by the displayed photographic equations:

- **Focal length:** `14` to `200` mm.
- **Circle of confusion:** `0.005` to `0.050` mm.
- **Physical focus distance:** `0.2` to `30` m.

## Motion Controls

- **Speed:** horizontal subject speed from `0` to `3000`.
- **deg/s:** rotational speed from `0` to `3000` degrees per second.
- **start x:** initial horizontal subject position from `-5` to `5`.
- **y:** initial vertical subject position from `-3` to `3`.

## Motion and Tracking Options

- **Star is Moving:** enables horizontal motion during exposure.
- **Star is Spinning:** enables rotation during exposure.
- **Panning Mode:** tracks the moving star approximately while shifting the background.
- **Show Motion Path:** overlays the sampled star trajectory.

---

## Exposure Model

The simulator uses this relative exposure relationship:

$$
E \propto \frac{\mathrm{ISO}\cdot t}{N^2},
$$

where:

- $$E$$ is relative exposure.
- $$\mathrm{ISO}$$ is the ISO setting.
- $$t$$ is shutter duration in seconds.
- $$N$$ is f-number.

The renderer compares the current value to this reference:

$$
\mathrm{ISO}=100,
\qquad
t=0.008\ \mathrm{s},
\qquad
N=8.
$$

The relative exposure multiplier is:

$$
M
=
\frac{
\mathrm{ISO}\cdot t/N^2
}{
100\cdot0.008/8^2
}.
$$

A multiplier of:

- $$M=1$$ matches the reference exposure.
- $$M>1$$ is brighter before clipping.
- $$M<1$$ is darker before clipping.

## Example

For ISO 100, `1/125 s`, and `f/8`:

$$
E \propto \frac{100\cdot0.008}{8^2}
=
0.0125.
$$

For ISO 400, `1/125 s`, and `f/8`:

$$
E \propto \frac{400\cdot0.008}{8^2}
=
0.05.
$$

The second setting has four times the relative exposure in this simplified model.

---

## Noise and Clipping

After exposure scaling, the simulation adds random noise.

The approximate noise level is proportional to ISO:

$$
\sigma_{\mathrm{noise}}
=
\left(\frac{\mathrm{ISO}}{100}\right)\cdot0.012.
$$

An additional small read-noise term is also added.

Finally, pixel intensities are clipped to the range:

$$
[0,1].
$$

The diagnostics panel reports:

- **White clip:** percentage of pixels with values at least `0.999`.
- **Black clip:** percentage of pixels with values at most `0.001`.

Because new random noise is generated during each update, the image and histogram can change slightly even if the slider state appears unchanged.

---

## Visual Focus Model

The rendered defocus is intentionally simplified.

For an image layer at depth $$d$$ and scene focus $$d_f$$, the blur scale is:

$$
\sigma
=
\mathrm{clip}
\left(
|d-d_f|\frac{18}{N},
0,
10
\right).
$$

where $$N$$ is f-number.

This means:

- blur increases as a layer moves away from the selected scene-focus depth;
- blur increases when the f-number becomes smaller;
- blur is capped at a Gaussian sigma of `10`.

The star and checkerboard are blurred independently before compositing.

## Important Separation of Models

The visual focus slider does **not** use:

- focal length in millimeters
- physical focus distance in meters
- circle of confusion
- calculated near and far depth-of-field limits

Those physical controls are used in the equation panel only.

This separation is deliberate: it keeps the interface responsive and makes the visual effect easy to inspect, but it is not an optically exact depth-of-field renderer.

---

## Hyperfocal Distance

The displayed hyperfocal distance is calculated in millimeters:

$$
H
=
\frac{F^2}{N c}+F,
$$

where:

- $$H$$ is hyperfocal distance.
- $$F$$ is focal length in millimeters.
- $$N$$ is f-number.
- $$c$$ is circle of confusion in millimeters.

The interface converts the result to meters for display.

A larger focal length, a wider aperture, or a smaller circle of confusion generally increases hyperfocal distance.

---

## Depth of Field

For a physical focus distance $$s$$, the near depth-of-field limit is:

$$
D_{\mathrm{n}}
=
\frac{Hs}{H+(s-F)}.
$$

The far depth-of-field limit is:

$$
D_{\mathrm{f}}
=
\frac{Hs}{H-(s-F)},
\qquad
s<H.
$$

When:

$$
s \geq H,
$$

the simulator reports:

$$
D_{\mathrm{f}}=\infty.
$$

The depth-of-field equations require consistent units. Internally, the simulator converts focus distance from meters to millimeters before applying the formulas.

---

## Motion Blur

When subject motion is enabled, horizontal star position changes over exposure time:

$$
x(t)
=
x_0 + vt,
$$

where:

- $$x_0$$ is the start position.
- $$v$$ is horizontal speed.
- $$t$$ is time during exposure.

The implemented scene velocity is:

$$
v=\frac{\mathrm{speed}}{60}.
$$

The total horizontal travel is:

$$
\Delta x = vt_{\mathrm{shutter}}.
$$

The renderer approximates motion blur by averaging multiple star positions over the exposure interval.

The number of samples is:

$$
n
=
\mathrm{clip}
\left(
18+220t_{\mathrm{shutter}},
18,
180
\right).
$$

This is an illustrative temporal-sampling method, not a physically complete shutter model.

---

## Panning Mode

When both **Star is Moving** and **Panning Mode** are enabled:

- the background is shifted in the opposite direction of star motion;
- the star is rendered around its initial horizontal location;
- the effect resembles camera tracking.

This can make the star appear relatively sharper while creating streaks in the checkerboard background.

The implementation shifts a 2D background image and does not simulate full perspective camera motion, parallax, rolling shutter, or optical flow.

---

## Preset Scenarios

### Portrait

Typical settings:

- wide aperture
- stationary subject
- shallow physical depth-of-field settings

### Sports

Typical settings:

- fast shutter
- higher ISO
- moving subject
- visible motion path

### Night

Typical settings:

- wide aperture
- slow shutter
- elevated ISO

### Panning

Typical settings:

- moving subject
- slower shutter
- panning enabled
- background streaking

---

## Suggested Experiments

### Exposure Triangle

1. Start with the default state.
2. Increase shutter time.
3. Reduce ISO to compensate.
4. Compare brightness, noise, clipping percentage, and histogram.

### Aperture and Visual Blur

1. Set scene focus near the star depth of `5`.
2. Reduce the f-number.
3. Observe the relative blur behavior of the star and background.
4. Move scene focus toward `10`.
5. Compare the checkerboard sharpness.

### Hyperfocal Distance

1. Set focal length to `24 mm`.
2. Observe hyperfocal distance at `f/16`.
3. Change focal length to `200 mm`.
4. Compare the hyperfocal distance and depth-of-field limits.
5. Change the circle of confusion.

### Motion Freeze

1. Enable **Star is Moving**.
2. Set speed to a high value.
3. Use a slow shutter and observe blur.
4. Reduce shutter time.
5. Increase ISO to recover brightness.

### Panning

1. Enable **Star is Moving**.
2. Set a nonzero speed and moderate shutter speed.
3. Enable **Panning Mode**.
4. Compare the moving star and checkerboard background with panning disabled.

---

## Model Scope and Limitations

This application is designed for education and visual experimentation.

### Not Modeled

The simulator does not model:

- physical lens aperture geometry
- diffraction
- lens aberrations
- perspective depth variation
- sensor size or crop factor
- pixel pitch
- RAW camera response
- gamma encoding
- white balance
- color channels or demosaicing
- wavelength-dependent noise
- actual ISO gain stages
- rolling shutter
- physically correct motion integration
- a real autofocus algorithm
- circle-of-confusion-based image rendering

### Important Notes

- The rendered blur is not physically coupled to the hyperfocal and depth-of-field calculation.
- The star and background have fixed synthetic depths.
- The exposure model is relative, not radiometrically calibrated.
- Noise is intentionally random and varies between redraws.
- Long shutter speeds can require more temporal samples and slower redraws.
- The star can move beyond the visible frame.
- Preset values are educational examples, not universal camera recommendations.

---

## Run

From the repository root:

```bash
python Camera_parameters/src/photography_simulator.py
```

## Requirements

```bash
python -m pip install -r requirements.txt
```

Required packages:

- NumPy
- SciPy
- Matplotlib

A graphical Matplotlib backend is required because the simulator uses sliders, checkboxes, buttons, and an interactive figure window.

## Source and License

- [Source code](https://github.com/KaramMawas/sensors_vision_simulators/blob/main/Camera_parameters/src/photography_simulator.py)
- [Shared requirements](https://github.com/KaramMawas/sensors_vision_simulators/blob/main/requirements.txt)
- [MIT License](https://github.com/KaramMawas/sensors_vision_simulators/blob/main/LICENSE)
