"""
===============================================================================
Project: Sensor Vision Simulators
Module: Photogrammetry - Tangential Decentering Distortion
File: tangential_distortion.py

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
An interactive tangential or decentering lens-distortion simulator illustrating
asymmetric image deformation through p1 and p2 coefficients, cross-coordinate
coupling, normalized image coordinates, and ideal-grid comparison.
===============================================================================
"""


import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider, CheckButtons
from matplotlib.lines import Line2D

#Tangential_(Decentering)_Distortion

# --- Physical Parameters ---
# Initial p1, p2 set to show a subtle asymmetric tilt
init_p1 = 0.05
init_p2 = 0.03
grid_res = 11   # Number of lines in the major grid axes
num_fine_points = 60 # Number of points used to make curved lines look smooth

# --- Scene Setup ---
fig = plt.figure(figsize=(14, 10))
ax = fig.add_subplot(111)
# Adjust layout: bottom margin for sliders, left margin for labels
plt.subplots_adjust(left=0.1, bottom=0.25, right=0.95, top=0.90)

# Set static axes properties (normalized coordinates -1 to 1)
lim = 1.3
ax.set_xlim(-lim, lim)
ax.set_ylim(-lim, lim)
ax.set_aspect('equal')
ax.set_xlabel("Normalized Sensor X")
ax.set_ylabel("Normalized Sensor Y")
ax.grid(True, which='both', linestyle='-', alpha=0.1)

# --- UI Controls (Sliders) ---
ax_p1 = plt.axes([0.25, 0.12, 0.50, 0.03])
ax_p2 = plt.axes([0.25, 0.07, 0.50, 0.03])
ax_check = plt.axes([0.25, 0.01, 0.50, 0.05], frameon=False)

slider_p1 = Slider(ax_p1, r'Tangential $p_1$ Coefficient', -0.2, 0.2, valinit=init_p1)
slider_p2 = Slider(ax_p2, r'Tangential $p_2$ Coefficient', -0.2, 0.2, valinit=init_p2)
check_ideal = CheckButtons(ax_check, ['Show Ideal Pinhole Grid (Reference)'], [True])

# --- Pre-calculate Ideal Grid ---
major_space = np.linspace(-1, 1, grid_res)
fine_space = np.linspace(-1, 1, num_fine_points)

ideal_lines = [] 
# Horizontal lines
for y in major_space:
    ideal_lines.append((fine_space, np.full_like(fine_space, y)))
# Vertical lines
for x in major_space:
    ideal_lines.append((np.full_like(fine_space, x), fine_space))

# --- Initialize Matplotlib Objects ---
ideal_line_objects = []
distorted_line_objects = []

# Distorted Grid (Solid Blue)
for _ in range(grid_res * 2):
    line, = ax.plot([], [], 'b-', lw=1.5, alpha=0.9)
    distorted_line_objects.append(line)

# Ideal Grid (Gray Dotted)
for _ in range(grid_res * 2):
    line, = ax.plot([], [], 'gray', linestyle=':', lw=1, alpha=0.6)
    ideal_line_objects.append(line)

# Principal Point (Center Crosshair)
ax.plot([0], [0], 'r+', markersize=12, lw=2)

# Calculation Text Panel (Dynamically updated)
# --- Updated Initialization (Change the Y position to 0.92 to move it up) ---
text_math = fig.text(0.12, 0.92, "", fontsize=9, fontweight='bold', va='top',
                    bbox=dict(facecolor='lightyellow', alpha=0.8, boxstyle='round,pad=0.3'))

# --- Legend Setup ---
custom_handles = [
    Line2D([0], [0], color='blue', lw=1.5, label='Actual Tangential Grid'),
    Line2D([0], [0], color='gray', linestyle=':', lw=1, label='Ideal Pinhole Grid'),
    Line2D([0], [0], color='r', marker='+', linestyle='', markersize=10, mew=2, label='Principal Point')
]
legend = ax.legend(handles=custom_handles, loc='upper right', bbox_to_anchor=(1.5, 1.05))

# --- Update Function ---
def update(val=None):
    p1 = slider_p1.val
    p2 = slider_p2.val
    show_ideal = check_ideal.get_status()[0]
    
    # Update title based on p1/p2
    if abs(p1) > 0.01 or abs(p2) > 0.01:
        ax.set_title(f"Tangential Distortion Model (Asymmetric Tilt: $p_1, p_2 \\neq 0$)", fontsize=14, pad=15)
    else:
        ax.set_title("Tangential Distortion Model (Perfect Alignment: $p_1, p_2 \\approx 0$)", fontsize=14, pad=15)
        
    # We sample a single vertex (the corner) to show the math in action
    samp_x, samp_y = 1.0, 1.0
    samp_r2 = samp_x**2 + samp_y**2
    
    # Apply Distortion Logic & Update Objects
    for i, ((x_ideal, y_ideal), d_obj, i_obj) in enumerate(zip(ideal_lines, distorted_line_objects, ideal_line_objects)):
        # Update Ideal Grid
        i_obj.set_visible(show_ideal)
        if show_ideal:
            i_obj.set_data(x_ideal, y_ideal)
            
        # Calculate & Update Distorted Grid
        r2 = x_ideal**2 + y_ideal**2
        
        # Tangential Model:
        # dx = [2p1xy + p2(r^2 + 2x^2)]
        # dy = [p1(r^2 + 2y^2) + 2p2xy]
        dx = 2*p1*x_ideal*y_ideal + p2*(r2 + 2*x_ideal**2)
        dy = p1*(r2 + 2*y_ideal**2) + 2*p2*x_ideal*y_ideal
        
        d_obj.set_data(x_ideal + dx, y_ideal + dy)
        
    # Update Math formula box with current sample
    samp_dx = 2*p1*samp_x*samp_y + p2*(samp_r2 + 2*samp_x**2)
    samp_dy = p1*(samp_r2 + 2*samp_y**2) + 2*p2*samp_x*samp_y
    
    # Use a more compact string format with fewer newlines
    math_str = (f"Tangential Distortion Model:\n"
                f"$\Delta x = 2p_1xy + p_2(r^2 + 2x^2)$\n"
                f"$\Delta y = p_1(r^2 + 2y^2) + 2p_2xy$\n"
                f"--- Corner Vertex (1,1) ---\n"
                f"$\Delta x = [2 \cdot {p1:.2f} \cdot 1] + [{p2:.2f} \cdot ({samp_r2:.1f} + 2)]$\n"
                f"$\Delta x = {samp_dx:.3f} \Rightarrow x' = {1+samp_dx:.3f}$\n"
                f"$\Delta y = [{p1:.2f} \cdot ({samp_r2:.1f} + 2)] + [2 \cdot {p2:.2f} \cdot 1]$\n"
                f"$\Delta y = {samp_dy:.3f} \Rightarrow y' = {1+samp_dy:.3f}$")
    
    text_math.set_text(math_str)
    fig.canvas.draw_idle()
    
    

# --- Connect UI Elements and Initialize ---
slider_p1.on_changed(update)
slider_p2.on_changed(update)
check_ideal.on_clicked(update)

# Trigger the initial draw
update()

plt.show()