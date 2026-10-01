# Photography Camera Parameters Simulator

![Python](https://img.shields.io/badge/Python-3.x-blue.svg)
![NumPy](https://img.shields.io/badge/NumPy-supported-orange.svg)
![SciPy](https://img.shields.io/badge/SciPy-image_processing-blue.svg)
![Matplotlib](https://img.shields.io/badge/Matplotlib-interactive-green.svg)
![Status](https://img.shields.io/badge/Status-Educational_Simulation-success.svg)
![License](https://img.shields.io/badge/License-MIT-lightgrey.svg)

An interactive photography simulator for exploring the relationship between **aperture, shutter speed, ISO, focus, depth of field, motion blur, panning, and image noise**.

The simulator renders a synthetic scene containing a star-shaped subject and a checkerboard background. Interactive controls update the viewfinder result, histogram, exposure calculation, hyperfocal distance, depth-of-field limits, motion state, and clipping diagnostics.

> The visual renderer is intentionally simplified for educational use. The displayed photographic equations use standard thin-lens depth-of-field relationships, while the rendered blur is a separate illustrative model.

---

## Topics

- exposure triangle
- aperture and f-number
- shutter speed
- ISO sensitivity
- relative exposure
- image noise
- highlight and shadow clipping
- focal length
- circle of confusion
- hyperfocal distance
- depth of field
- focus distance
- simplified visual defocus
- horizontal motion blur
- rotational motion blur
- camera panning
- image histogram analysis

---

## Main Script

- `src/photography_simulator.py`

---

## Features

- interactive synthetic viewfinder
- star-shaped foreground subject
- checkerboard background
- aperture, shutter-speed, and ISO controls
- relative exposure calculation
- simulated brightness and sensor noise
- highlight and shadow clipping statistics
- image histogram
- visual foreground/background focus control
- focal-length, circle-of-confusion, and physical-focus controls
- hyperfocal-distance calculation
- near and far depth-of-field limits
- horizontal subject motion
- rotational subject motion
- panning mode
- optional motion-path overlay
- Portrait, Sports, Night, and Panning presets

---

## Simulation Scene

The rendered scene contains two simplified depth layers:

- **Star subject:** scene depth `5`
- **Checkerboard background:** scene depth `10`

The star represents a foreground subject. The checkerboard makes background blur and panning streaks easy to observe.

---

## Run

From this directory:

```bash
python src/photography_simulator.py
```

Or, from the repository root:

```bash
python Camera_parameters/src/photography_simulator.py
```

---

## Requirements

Install the shared repository dependencies:

```bash
python -m pip install -r ../requirements.txt
```

The simulator requires:

- NumPy
- SciPy
- Matplotlib
- a graphical Matplotlib backend

---

## Controls

### Exposure Triangle

- **f-number**  
  Controls aperture size. A smaller f-number represents a wider aperture and increases relative exposure.

- **Shutter Speed**  
  Controls exposure time. A longer exposure increases brightness and motion blur.

- **ISO**  
  Controls the simulated sensitivity multiplier. Higher ISO increases brightness and image noise.

### Focus and Depth of Field

- **Scene Focus**  
  Controls the simplified visual blur applied to the rendered star and background.

- **Focal Length**  
  Used in hyperfocal-distance and depth-of-field equations.

- **CoC**  
  Circle of confusion used for the physical depth-of-field calculations.

- **Focus Distance**  
  Physical focus distance used in the depth-of-field equations.

### Motion and Position

- **Speed**  
  Horizontal speed of the star during exposure.

- **deg/s**  
  Angular speed of star rotation during exposure.

- **start x** and **y**  
  Initial star position.

- **Star is Moving**  
  Enables horizontal star motion.

- **Star is Spinning**  
  Enables star rotation during exposure.

- **Panning Mode**  
  Simulates camera tracking by holding the moving star near its initial horizontal position while shifting the background.

- **Show Motion Path**  
  Overlays the sampled trajectory of the moving star.

---

## Exposure Model

The simulator uses a relative exposure relationship:

$$
E \propto \frac{\mathrm{ISO} \cdot t}{N^2},
$$

where:

- $$E$$ is relative exposure.
- $$\mathrm{ISO}$$ is the ISO value.
- $$t$$ is shutter speed in seconds.
- $$N$$ is the f-number.

The reference configuration is:

- ISO 100
- $$t = 1/125\ \mathrm{s} = 0.008\ \mathrm{s}$$
- $$N = 8$$

This reference configuration has a relative exposure multiplier of `1.0`.

---

## Hyperfocal Distance and Depth of Field

The simulator calculates hyperfocal distance using:

$$
H = \frac{F^2}{N c} + F,
$$

where:

- $$H$$ is hyperfocal distance.
- $$F$$ is focal length in millimeters.
- $$N$$ is f-number.
- $$c$$ is circle of confusion in millimeters.

For focus distance $$s$$, the near depth-of-field limit is:

$$
D_{\mathrm{n}}
=
\frac{Hs}{H + (s-F)}.
$$

The far depth-of-field limit is:

$$
D_{\mathrm{f}}
=
\frac{Hs}{H - (s-F)},
\qquad s < H.
$$

When the focus distance is at or beyond hyperfocal distance, the simulator reports:

$$
D_{\mathrm{f}} = \infty.
$$

---

## Presets

- **Portrait**  
  Wide aperture, shallow depth of field, stationary star.

- **Sports**  
  Fast shutter speed, high ISO, fast-moving star, visible motion path.

- **Night**  
  Wide aperture, slow shutter speed, elevated ISO.

- **Panning**  
  Moving star with camera tracking enabled, illustrating a relatively sharper subject and streaked background.

---

## Model Scope and Limitations

This tool is an educational visualization, not a physically complete camera renderer.

### Exposure

- Exposure is based on a relative relationship, not calibrated luminance or irradiance.
- ISO brightens the rendered image and increases synthetic Gaussian noise.
- Camera response curves, RAW processing, white balance, and color science are not modeled.

### Focus

- The visual blur model is not calculated from the circle of confusion, focal length, or physical focus distance.
- The **Scene Focus** slider controls an independent educational Gaussian-blur model.
- The hyperfocal and depth-of-field formulas are calculated separately and shown for learning purposes.
- The rendered star depth and checkerboard depth are fixed at `5` and `10` scene units.

### Motion

- Motion blur is approximated by averaging multiple rendered star positions over the shutter interval.
- Panning shifts the background while holding the subject near its starting position.
- This is not a full camera-motion, perspective, or rolling-shutter simulation.
- The star may move beyond the displayed frame during long exposures or high-speed settings.

### Performance

The image resolution is `420 × 420`. Longer shutter times increase the number of temporal samples and can make updates slower.

---

## Documentation

[Photography Camera Parameters Simulator Guide](https://karammawas.github.io/sensors_vision_simulators/camera-parameters/)

---

## License

[MIT License](../LICENSE)
