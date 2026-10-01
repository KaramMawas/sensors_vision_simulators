"""
===============================================================================
Project: Sensor Vision Simulators
Module: Camera Parameters / Photography Simulator
File: photography_simulator.py

Author: Karam Mawas
Affiliation: Technical University of Braunschweig / Institute of Geodesy and Photogrammetry (IGP)
Email: karam.mawas@gmail.com
GitHub: https://github.com/KaramMawas
Website: https://karammawas.github.io/
ORCID: https://orcid.org/0000-0002-8608-7578

Created: 2026-04-14
Last Updated: 2026-10-01

Copyright (c) 2026 Karam Mawas
License: MIT

Description:
An interactive photography simulator for illustrating aperture, shutter speed,
ISO, relative exposure, image noise, visual focus, hyperfocal distance,
depth of field, subject motion, rotational motion, panning, and histogram
behavior using a synthetic star and checkerboard scene.
===============================================================================
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider, CheckButtons, Button
from scipy.ndimage import gaussian_filter, shift

# ============================================================
# Photography Simulator
# - Star subject only
# - Checkerboard background only
# - Exposure triangle
# - Motion blur / panning
# - Simplified visual focus model
# - Live photographic equations:
#     E ∝ ISO * t / N^2
#     H = F^2 / (N*c) + F
#     Dn, Df depth-of-field limits
# ============================================================

# ------------------------------------------------------------
# Scene grid
# ------------------------------------------------------------
resolution = 420
x = np.linspace(-5, 5, resolution)
y = np.linspace(-5, 5, resolution)
xx, yy = np.meshgrid(x, y)
dx = x[1] - x[0]

rng = np.random.default_rng(1234)

# ------------------------------------------------------------
# Background only: checkerboard
# ------------------------------------------------------------
bg_checker = (np.sin(xx * 9) * np.sin(yy * 9) > 0).astype(float) * 0.85

# Simplified scene depths used for visual blur rendering
DEPTH_SUBJECT = 5.0
DEPTH_BACKGROUND = 10.0


# ============================================================
# Star shape only
# ============================================================
def star_shape(xr, yr, angle=0.0, tips=10, outer_r=0.95, inner_r=0.34, softness=24):
    c, s = np.cos(angle), np.sin(angle)
    xrot = c * xr + s * yr
    yrot = -s * xr + c * yr

    theta = np.arctan2(yrot, xrot)
    r = np.sqrt(xrot**2 + yrot**2) + 1e-12

    mod = 0.5 * (1.0 + np.cos(tips * theta))
    r_boundary = inner_r + (outer_r - inner_r) * (mod ** 1.8)

    return 1.0 / (1.0 + np.exp(softness * (r - r_boundary)))


def render_subject(x0, y0, angle=0.0):
    xr = xx - x0
    yr = yy - y0
    return star_shape(xr, yr, angle=angle)


# ============================================================
# Visual blur / exposure helpers
# ============================================================
def blur_sigma_from_focus(depth, focus_depth, f_stop):
    """
    Simplified educational visual blur model:
    blur grows with distance from focus plane,
    blur grows when f-number is smaller.
    """
    return np.clip(abs(depth - focus_depth) * (18.0 / f_stop), 0.0, 10.0)


def exposure_multiplier(f_stop, shutter_speed, iso):
    """
    Reference exposure:
    f/8, 1/125 s (= 0.008 s), ISO 100 -> multiplier 1.0
    """
    base = (100.0 * 0.008) / (8.0**2)
    current = (iso * shutter_speed) / (f_stop**2)
    return current / base


# ============================================================
# Photographic equations
# ============================================================
def hyperfocal_distance_mm(F_mm, N, c_mm):
    """
    H = F^2 / (N*c) + F
    """
    return (F_mm**2) / (N * c_mm) + F_mm


def dof_limits_mm(F_mm, N, c_mm, s_mm):
    """
    H  = hyperfocal distance
    Dn = near limit
    Df = far limit

    H = F^2/(N*c) + F
    Dn = H*s / (H + (s - F))
    Df = H*s / (H - (s - F)) if s < H, else inf

    All units in mm.
    """
    H_mm = hyperfocal_distance_mm(F_mm, N, c_mm)

    Dn_mm = (H_mm * s_mm) / (H_mm + (s_mm - F_mm))

    if s_mm < H_mm:
        denom = H_mm - (s_mm - F_mm)
        if np.isclose(denom, 0.0):
            Df_mm = np.inf
        else:
            Df_mm = (H_mm * s_mm) / denom
    else:
        Df_mm = np.inf

    return H_mm, Dn_mm, Df_mm


def mm_to_m(x_mm):
    return x_mm / 1000.0


# ============================================================
# Main simulation
# ============================================================
def simulate_photo(
    f_stop,
    shutter_speed,
    iso,
    scene_focus_depth,
    motion_enabled,
    horizontal_speed,
    spin_enabled,
    spin_speed_deg,
    start_x,
    subject_y,
    panning_enabled,
    show_path
):
    # Background only
    sigma_bg = blur_sigma_from_focus(DEPTH_BACKGROUND, scene_focus_depth, f_stop)
    scene_base = gaussian_filter(bg_checker, sigma=sigma_bg)

    # Motion left to right when enabled
    vx = horizontal_speed / 60.0 if motion_enabled else 0.0
    omega = np.deg2rad(spin_speed_deg) if spin_enabled else 0.0

    samples = int(np.clip(18 + shutter_speed * 220, 18, 180))
    ts = np.linspace(0.0, shutter_speed, samples)

    fg_subject_acc = np.zeros_like(xx)
    bg_scene_acc = np.zeros_like(xx)
    path_overlay = np.zeros_like(xx)

    for t in ts:
        x_t = start_x + vx * t
        y_t = subject_y
        angle_t = omega * t

        subj_t = render_subject(x_t, y_t, angle=angle_t)
        sigma_subj = blur_sigma_from_focus(DEPTH_SUBJECT, scene_focus_depth, f_stop)
        subj_t = gaussian_filter(subj_t, sigma=sigma_subj)

        if panning_enabled and motion_enabled:
            world_shift = -(x_t - start_x)
            pixel_shift = world_shift / dx
            scene_t = shift(scene_base, shift=(0, pixel_shift), order=1, mode='nearest')

            subj_t = render_subject(start_x, y_t, angle=angle_t)
            subj_t = gaussian_filter(subj_t, sigma=sigma_subj)
        else:
            scene_t = scene_base

        fg_subject_acc += subj_t
        bg_scene_acc += scene_t

        if show_path and motion_enabled:
            path_overlay += np.exp(-((xx - x_t)**2 + (yy - y_t)**2) / 0.01)

    fg_subject = fg_subject_acc / samples
    bg_scene = bg_scene_acc / samples

    alpha = np.clip(fg_subject, 0, 1)
    img = alpha * fg_subject + (1 - alpha) * bg_scene

    if show_path and motion_enabled:
        img = np.clip(img + 0.12 * np.clip(path_overlay, 0, 1), 0, 1)

    # Exposure
    img *= exposure_multiplier(f_stop, shutter_speed, iso)

    # Noise
    noise_level = (iso / 100.0) * 0.012
    noise = rng.normal(0.0, noise_level, img.shape)
    read_noise = rng.normal(0.0, 0.004, img.shape)

    return np.clip(img + noise + read_noise, 0, 1)


# ============================================================
# Figure and layout
# ============================================================
fig = plt.figure(figsize=(18, 10))
fig.canvas.manager.set_window_title("Photography Simulator - Star, Checkerboard, and Live Equations")

# ------------------------------------------------------------
# Initial state
# ------------------------------------------------------------
init = dict(
    f_stop=8.0,
    shutter=0.008,
    iso=100,
    scene_focus_depth=5.0,
    focal_mm=50.0,
    coc_mm=0.030,
    focus_dist_m=3.0,
    motion=False,
    speed=0.0,
    spin=False,
    spin_speed=0.0,
    start_x=0.0,
    subject_y=0.0,
    panning=False,
    show_path=False
)

# ------------------------------------------------------------
# Left info blocks
# ------------------------------------------------------------
info_text = fig.text(
    0.02, 0.92, "",
    fontsize=6,
    family='monospace',
    va='top',
    bbox=dict(facecolor='#f4f4f4', edgecolor='gray', boxstyle='round,pad=0.5')
)

concept_text = fig.text(
    0.005, 0.4,#0.02, 0.38,
    "Concept notes:\n"
    "- The image renderer uses a simplified visual blur model.\n"
    "- The equations panel shows real photographic equations.\n"
    "- F is focal length, N is f-number, c is circle of confusion, \n s is focus distance.\n"
    "- Smaller f-number means wider aperture opening.\n"
    "- Larger f-number means narrower aperture opening.\n"
    "- Positive speed moves the star from left to right.\n"
    "- CoC = circle of confusion: \n the maximum blur spot still treated as acceptably sharp.\n"
    "Hyperfocal distance is the focus distance that \n gives acceptable sharpness from half that distance to infinity.\n"
    "- Depth of field limits are the near (Dn:near depth-of-field) \n and far (Df:depth-of-field limit) distances that are \n acceptably sharp for a given focus distance.\n",
    #"- Panning tries to keep the star sharper \n while the checkerboard streaks.",
    fontsize=6,
    va='top',
    bbox=dict(facecolor='#fbfbfb', edgecolor='lightgray', boxstyle='round,pad=0.4')
)

# ------------------------------------------------------------
# Main image
# ------------------------------------------------------------
ax_img = plt.axes([0.18, 0.14, 0.34, 0.64])

img0 = simulate_photo(
    init["f_stop"], init["shutter"], init["iso"], init["scene_focus_depth"],
    init["motion"], init["speed"], init["spin"], init["spin_speed"],
    init["start_x"], init["subject_y"], init["panning"], init["show_path"]
)

img_display = ax_img.imshow(
    img0, cmap='gray', vmin=0, vmax=1,
    origin='lower', extent=[-5, 5, -5, 5]
)
ax_img.set_title("Viewfinder Result")
ax_img.set_xticks([])
ax_img.set_yticks([])

# ------------------------------------------------------------
# Histogram
# ------------------------------------------------------------
#ax_hist = plt.axes([0.58, 0.82, 0.37, 0.12])
ax_hist = plt.axes([0.21, 0.89, 0.32, 0.08])

# ============================================================
# Right-side controls
# ============================================================

# Aperture
#fig.text(0.58, 0.955, "Aperture (f-number)", fontweight='bold', fontsize=12)
fig.text(0.58, 0.97, "Aperture (f-number)", fontweight='bold', fontsize=12)
fig.text(
    0.58, 0.938,
    "Smaller f-number: wider opening, more light, shallower depth of field.\n "
    "Larger f-number: narrower opening, less light, deeper depth of field.",
    fontsize=8, color='darkred'
)
ax_aperture = plt.axes([0.58, 0.910, 0.37, 0.020])
slider_aperture = Slider(ax_aperture, 'f-number', 1.4, 22.0, valinit=init["f_stop"])

# Shutter
fig.text(0.58, 0.880, "Shutter Speed (seconds)", fontweight='bold', fontsize=12)
fig.text(
    0.58, 0.863,
    "Smaller time: faster shutter, less light, less motion blur. "
    "Larger time: slower shutter, more light, more motion blur.",
    fontsize=8, color='darkblue'
)
ax_shutter = plt.axes([0.58, 0.835, 0.37, 0.020])
slider_shutter = Slider(ax_shutter, 'Sec.', 0.001, 0.5, valinit=init["shutter"])

# ISO
fig.text(0.58, 0.805, "ISO (sensor sensitivity)", fontweight='bold', fontsize=12)
fig.text(
    0.58, 0.788,
    "Lower ISO is cleaner but darker. Higher ISO is brighter but noisier.",
    fontsize=8, color='darkgreen'
)
ax_iso = plt.axes([0.58, 0.760, 0.37, 0.020])
slider_iso = Slider(ax_iso, 'ISO', 50, 6400, valinit=init["iso"], valstep=50)

# Visual scene focus
fig.text(0.58, 0.730, "Scene Focus for Rendering", fontweight='bold', fontsize=12)
fig.text(
    0.58, 0.713,
    "This controls the simplified visual focus model used in the image renderer.",
    fontsize=8, color='purple'
)
ax_focus = plt.axes([0.58, 0.685, 0.37, 0.020])
slider_focus = Slider(ax_focus, 'scene focus', 1.0, 10.0, valinit=init["scene_focus_depth"])
slider_focus.label.set_fontsize(9)

# Physical lens parameters
fig.text(0.58, 0.655, "Physical Lens Parameters", fontweight='bold', fontsize=12)
fig.text(
    0.58, 0.638,
    "These are used in the live hyperfocal and depth-of-field equations.",
    fontsize=8, color='teal'
)

ax_focal = plt.axes([0.58, 0.610, 0.37, 0.020])
slider_focal = Slider(ax_focal, 'focal mm', 14.0, 200.0, valinit=init["focal_mm"])

ax_coc = plt.axes([0.58, 0.575, 0.37, 0.020])
slider_coc = Slider(ax_coc, 'CoC mm', 0.005, 0.050, valinit=init["coc_mm"])

# Physical focus distance
fig.text(0.58, 0.545, "Physical Focus Distance", fontweight='bold', fontsize=12)
fig.text(
    0.58, 0.528,
    "This is the actual focus distance used in the photographic formulas.",
    fontsize=8.7, color='purple'
)
ax_focus_dist_m = plt.axes([0.58, 0.500, 0.37, 0.020])
slider_focus_dist_m = Slider(ax_focus_dist_m, 'focus m', 0.2, 30.0, valinit=init["focus_dist_m"])

# Motion
fig.text(0.58, 0.470, "Horizontal Star Motion", fontweight='bold', fontsize=12)
fig.text(
    0.58, 0.453,
    "If enabled, positive speed moves the star from left to right during exposure.",
    fontsize=8.7, color='brown'
)
ax_speed = plt.axes([0.58, 0.425, 0.37, 0.020])
slider_speed = Slider(ax_speed, 'speed', 0, 3000, valinit=init["speed"])

# Spin
fig.text(0.58, 0.395, "Star Spin", fontweight='bold', fontsize=12)
fig.text(
    0.58, 0.378,
    "If enabled, the star rotates around its center during exposure.",
    fontsize=8.7, color='darkorange'
)
ax_spin = plt.axes([0.58, 0.350, 0.37, 0.020])
slider_spin = Slider(ax_spin, 'deg/s', 0, 3000, valinit=init["spin_speed"])

# Position
fig.text(0.58, 0.320, "Star Position at Exposure Start", fontweight='bold', fontsize=12)
fig.text(
    0.58, 0.303,
    "Use start position, speed, and shutter together to control where the star appears.",
    fontsize=8.7, color='black'
)
ax_start_x = plt.axes([0.58, 0.275, 0.37, 0.020])
slider_start_x = Slider(ax_start_x, 'start x', -5.0, 5.0, valinit=init["start_x"])

ax_subject_y = plt.axes([0.58, 0.240, 0.37, 0.020])
slider_subject_y = Slider(ax_subject_y, 'y', -3.0, 3.0, valinit=init["subject_y"])

# ------------------------------------------------------------
# Bottom controls
# ------------------------------------------------------------
controls_text = fig.text(
    0.2, 0.02,#0.18, 0.045,
    "Control guide:\n"
    "- Checkboxes turn star motion, star spinning, panning, and motion-path overlay on or off.\n"
    "- Preset buttons load common photography scenarios.\n"
    "- Portrait: wide aperture, shallow depth of field, still star.\n"
    "- Sports: fast shutter to freeze a fast-moving star.\n"
    "- Night: wide aperture and slower shutter with higher ISO.\n"
    "- Panning: slower shutter with camera tracking to keep the star sharper.",
    fontsize=8.8,
    va='bottom',
    bbox=dict(facecolor='#fcfcfc', edgecolor='lightgray', boxstyle='round,pad=0.4')
)

# Motion and tracking options
fig.text(0.58, 0.205, "Motion / Tracking Options", fontweight='bold', fontsize=11)
fig.text(
    0.58, 0.190,
    "These toggles control whether the star moves, spins, is panned with the camera, "
    "or shows its motion path.",
    fontsize=8, color='dimgray'
)

ax_check = plt.axes([0.58, 0.085, 0.22, 0.09])
check = CheckButtons(
    ax_check,
    ['Star is Moving', 'Star is Spinning', 'Panning Mode', 'Show Motion Path'],
    [init["motion"], init["spin"], init["panning"], init["show_path"]]
)

# Predefined scenarios
"""fig.text(0.82, 0.205, "Preset Scenarios", fontweight='bold', fontsize=11)
fig.text(
    0.82, 0.190,
    "These buttons instantly load example settings.",
    fontsize=8.5, color='dimgray'
)

ax_p1 = plt.axes([0.82, 0.125, 0.06, 0.045])
ax_p2 = plt.axes([0.89, 0.125, 0.06, 0.045])
ax_p3 = plt.axes([0.82, 0.070, 0.06, 0.045])
ax_p4 = plt.axes([0.89, 0.070, 0.06, 0.045])"""

fig.text(0.58, 0.07, "Preset Scenarios", fontweight='bold', fontsize=11)
fig.text(
    0.58, 0.055,
    "These buttons instantly load example settings.",
    fontsize=8, color='dimgray'
)
#plt.axes([left, bottom, width, height])
x1, y1, w, h = 0.58, 0.005, 0.06, 0.045
gap = 0.02

ax_p1 = plt.axes([x1, y1, w, h])
ax_p2 = plt.axes([x1 + (w + gap), y1, w, h])
ax_p3 = plt.axes([x1 + 2 * (w + gap), y1, w, h])
ax_p4 = plt.axes([x1 + 3 * (w + gap), y1, w, h])

btn_portrait = Button(ax_p1, 'Portrait')
btn_sports = Button(ax_p2, 'Sports')
btn_night = Button(ax_p3, 'Night')
btn_panning = Button(ax_p4, 'Panning')


# ============================================================
# State helpers
# ============================================================
def get_state():
    motion_active, spin_active, panning_active, show_path_active = check.get_status()
    return {
        "f_stop": slider_aperture.val,
        "shutter": slider_shutter.val,
        "iso": slider_iso.val,
        "scene_focus_depth": slider_focus.val,
        "focal_mm": slider_focal.val,
        "coc_mm": slider_coc.val,
        "focus_dist_m": slider_focus_dist_m.val,
        "motion": motion_active,
        "speed": slider_speed.val,
        "spin": spin_active,
        "spin_speed": slider_spin.val,
        "start_x": slider_start_x.val,
        "subject_y": slider_subject_y.val,
        "panning": panning_active,
        "show_path": show_path_active
    }


def set_checkbutton(index, desired_state):
    current = check.get_status()[index]
    if current != desired_state:
        check.set_active(index)


def apply_preset(name):
    if name == 'Portrait':
        slider_aperture.set_val(2.0)
        slider_shutter.set_val(0.02)
        slider_iso.set_val(100)
        slider_focus.set_val(5.0)
        slider_focal.set_val(85.0)
        slider_coc.set_val(0.030)
        slider_focus_dist_m.set_val(2.0)
        slider_speed.set_val(0)
        slider_spin.set_val(0)
        slider_start_x.set_val(0.0)
        slider_subject_y.set_val(0.0)
        set_checkbutton(0, False)
        set_checkbutton(1, False)
        set_checkbutton(2, False)
        set_checkbutton(3, False)

    elif name == 'Sports':
        slider_aperture.set_val(2.8)
        slider_shutter.set_val(0.002)
        slider_iso.set_val(800)
        slider_focus.set_val(5.0)
        slider_focal.set_val(200.0)
        slider_coc.set_val(0.030)
        slider_focus_dist_m.set_val(20.0)
        slider_speed.set_val(1800)
        slider_spin.set_val(0)
        slider_start_x.set_val(-4.0)
        slider_subject_y.set_val(0.0)
        set_checkbutton(0, True)
        set_checkbutton(1, False)
        set_checkbutton(2, False)
        set_checkbutton(3, True)

    elif name == 'Night':
        slider_aperture.set_val(1.8)
        slider_shutter.set_val(0.15)
        slider_iso.set_val(1600)
        slider_focus.set_val(5.0)
        slider_focal.set_val(35.0)
        slider_coc.set_val(0.030)
        slider_focus_dist_m.set_val(3.0)
        slider_speed.set_val(0)
        slider_spin.set_val(0)
        slider_start_x.set_val(0.0)
        slider_subject_y.set_val(0.0)
        set_checkbutton(0, False)
        set_checkbutton(1, False)
        set_checkbutton(2, False)
        set_checkbutton(3, False)

    elif name == 'Panning':
        slider_aperture.set_val(8.0)
        slider_shutter.set_val(0.08)
        slider_iso.set_val(200)
        slider_focus.set_val(5.0)
        slider_focal.set_val(70.0)
        slider_coc.set_val(0.030)
        slider_focus_dist_m.set_val(10.0)
        slider_speed.set_val(2200)
        slider_spin.set_val(0)
        slider_start_x.set_val(-4.5)
        slider_subject_y.set_val(0.0)
        set_checkbutton(0, True)
        set_checkbutton(1, False)
        set_checkbutton(2, True)
        set_checkbutton(3, False)
    
    update()


# ============================================================
# Update
# ============================================================
def update(val=None):
    state = get_state()

    img = simulate_photo(
        state["f_stop"],
        state["shutter"],
        state["iso"],
        state["scene_focus_depth"],
        state["motion"],
        state["speed"],
        state["spin"],
        state["spin_speed"],
        state["start_x"],
        state["subject_y"],
        state["panning"],
        state["show_path"]
    )

    img_display.set_data(img)

    # Histogram
    ax_hist.clear()
    ax_hist.hist(img.ravel(), bins=40, range=(0, 1), color='gray')
    ax_hist.set_title("Histogram", fontsize=10)
    ax_hist.set_xlim(0, 1)
    ax_hist.set_ylim(bottom=0)

    # Equation values
    F_mm = state["focal_mm"]
    N = state["f_stop"]
    c_mm = state["coc_mm"]
    s_mm = state["focus_dist_m"] * 1000.0
    t = state["shutter"]
    iso = state["iso"]

    H_mm, Dn_mm, Df_mm = dof_limits_mm(F_mm, N, c_mm, s_mm)

    exposure_rel = (iso * t) / (N**2)
    base_exposure_rel = (100.0 * 0.008) / (8.0**2)
    exposure_mult = exposure_rel / base_exposure_rel

    if np.isinf(Df_mm):
        df_str = "∞"
    else:
        df_str = f"{mm_to_m(Df_mm):.3f} m"

    # Motion and clipping info
    travel_distance = (state["speed"] / 60.0) * state["shutter"] if state["motion"] else 0.0
    end_x = state["start_x"] + travel_distance
    white_clip = np.mean(img >= 0.999) * 100.0
    black_clip = np.mean(img <= 0.001) * 100.0

    info_text.set_text(
        "Live photographic equations\n"
        "──────────────────────────\n"
        "\n"
        "1) Exposure model used in simulator\n"
        "   E ∝ ISO·t/N²\n"
        f"   E ∝ {int(iso)}·{t:.4f}/({N:.2f})² = {exposure_rel:.6f}\n"
        f"   Relative exposure multiplier = {exposure_mult:.3f}\n"
        "\n"
        "2) Hyperfocal distance\n"
        "   H = F²/(N·c) + F\n"
        f"   H = ({F_mm:.1f}²)/({N:.2f}·{c_mm:.4f}) + {F_mm:.1f}\n"
        f"   H = {H_mm:.2f} mm = {mm_to_m(H_mm):.3f} m\n"
        "\n"
        "3) Depth of field\n"
        "   Dn = H·s / (H + (s - F))\n"
        "   Df = H·s / (H - (s - F)) if s < H, else ∞\n"
        f"   s = {state['focus_dist_m']:.3f} m\n"
        f"   Dn = {mm_to_m(Dn_mm):.3f} m\n"
        f"   Df = {df_str}\n"
        "\n"
        "Current values\n"
        f"   F = {F_mm:.1f} mm\n"
        f"   N = f/{N:.1f}\n"
        f"   c = {c_mm:.4f} mm\n"
        f"   scene focus = {state['scene_focus_depth']:.2f}\n"
        f"   start x = {state['start_x']:.2f}\n"
        f"   end x = {end_x:.2f}\n"
        f"   travel = {travel_distance:.2f}\n"
        "\n"
        "Image diagnostics\n"
        f"   white clip = {white_clip:.2f}%\n"
        f"   black clip = {black_clip:.2f}%"
    )

    fig.canvas.draw_idle()


# ============================================================
# Bind callbacks
# ============================================================
for s in [
    slider_aperture, slider_shutter, slider_iso, slider_focus,
    slider_focal, slider_coc, slider_focus_dist_m,
    slider_speed, slider_spin, slider_start_x, slider_subject_y
]:
    s.on_changed(update)

check.on_clicked(update)

btn_portrait.on_clicked(lambda event: apply_preset('Portrait'))
btn_sports.on_clicked(lambda event: apply_preset('Sports'))
btn_night.on_clicked(lambda event: apply_preset('Night'))
btn_panning.on_clicked(lambda event: apply_preset('Panning'))

update()
plt.show()
