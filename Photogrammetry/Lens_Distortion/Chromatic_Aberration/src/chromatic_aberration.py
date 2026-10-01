"""
===============================================================================
Project: Sensor Vision Simulators
Module: Photogrammetry - Chromatic Aberration
File: chromatic_aberration.py

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
An interactive chromatic-aberration visualization using independent radial
image warping of red, green, and blue channels to demonstrate lateral color
fringing on a synthetic checkerboard pattern.
===============================================================================
"""


import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider, Button

# --- Parameters ---
img_size = 600
# Initial states: R and B shifted, G is 0
init_kr, init_kg, init_kb = 0.03, 0.0, -0.03

# --- 1. Create Pattern ---
def create_pattern(size):
    re = np.indices((size, size)) // (size // 12)
    check = (re[0] + re[1]) % 2
    img = np.stack([check, check, check], axis=-1).astype(float)
    # Highlight edges for visibility
    img[0:10, :, :] = 0.5; img[-10:, :, :] = 0.5
    return img

original_img = create_pattern(img_size)

# --- 2. Setup Figure ---
fig = plt.figure(figsize=(14, 9))
plt.subplots_adjust(bottom=0.3)

ax_full = fig.add_subplot(121)
im_full = ax_full.imshow(original_img)
ax_full.set_title("Full Sensor View")
ax_full.axis('off')

ax_zoom = fig.add_subplot(122)
im_zoom = ax_zoom.imshow(original_img)
ax_zoom.set_title("Edge Zoom (Top-Left)")
ax_zoom.set_xlim(0, 120); ax_zoom.set_ylim(120, 0)
ax_zoom.axis('off')

# --- 3. UI Controls (Stacked Vertically) ---
ax_r = plt.axes([0.25, 0.20, 0.5, 0.03])
ax_g = plt.axes([0.25, 0.15, 0.5, 0.03])
ax_b = plt.axes([0.25, 0.10, 0.5, 0.03])
ax_reset = plt.axes([0.8, 0.02, 0.1, 0.04])

slider_r = Slider(ax_r, 'Red Scale (kr)', -0.1, 0.1, valinit=init_kr, color='red')
slider_g = Slider(ax_g, 'Green Scale (kg)', -0.1, 0.1, valinit=init_kg, color='green')
slider_b = Slider(ax_b, 'Blue Scale (kb)', -0.1, 0.1, valinit=init_kb, color='blue')
btn_reset = Button(ax_reset, 'Reset')

# --- 4. Warp Logic ---
def apply_unlocked_warp(img, kr, kg, kb):
    h, w, _ = img.shape
    y, x = np.indices((h, w))
    nx = 2.0 * x / (w - 1) - 1.0
    ny = 2.0 * y / (h - 1) - 1.0
    r2 = nx**2 + ny**2
    
    def warp(c_idx, k_val):
        scale = 1.0 + k_val * r2
        sx = ((nx / scale + 1.0) * (w - 1) / 2.0).astype(int)
        sy = ((ny / scale + 1.0) * (h - 1) / 2.0).astype(int)
        sx = np.clip(sx, 0, w - 1)
        sy = np.clip(sy, 0, h - 1)
        return img[sy, sx, c_idx]

    out = np.zeros_like(img)
    out[:, :, 0] = warp(0, kr)
    out[:, :, 1] = warp(1, kg)
    out[:, :, 2] = warp(2, kb)
    return out

# --- 5. Update Loop ---
def update(val=None):
    kr, kg, kb = slider_r.val, slider_g.val, slider_b.val
    warped = apply_unlocked_warp(original_img, kr, kg, kb)
    
    im_full.set_data(warped)
    im_zoom.set_data(warped)
    
    # Mathematical Note
    title = f"Independent Channel Warping\n"
    if kg != 0:
        title += f"WARNING: Green is NOT 0. Geometry is floating!"
    else:
        title += f"Green is 0. Geometry is stable."
    
    fig.suptitle(title, fontsize=12, fontweight='bold', color='darkred' if kg != 0 else 'black')
    fig.canvas.draw_idle()

def reset(event):
    slider_r.reset(); slider_g.reset(); slider_b.reset()

slider_r.on_changed(update)
slider_g.on_changed(update)
slider_b.on_changed(update)
btn_reset.on_clicked(reset)

update()
plt.show()