import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider, CheckButtons
from matplotlib.lines import Line2D


#Radial Distortion

# --- Physical Parameters ---
init_k1 = -0.18 # Standard wide-angle barrel distortion
grid_res = 11   # Number of lines in the major grid axes
num_fine_points = 60 # Number of points used to make curved lines look smooth

# --- Scene Setup ---
fig = plt.figure(figsize=(12, 9))
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
ax.grid(True, which='both', linestyle='-', alpha=0.1) # Very subtle grid on the background canvas

# --- UI Controls ---
ax_k1 = plt.axes([0.25, 0.10, 0.50, 0.03])
ax_check = plt.axes([0.25, 0.02, 0.50, 0.06], frameon=False)

slider_k1 = Slider(ax_k1, r'Radial Coefficient $k_1$', -0.5, 0.5, valinit=init_k1)
# The default check status matches our initial k1 state
check_ideal = CheckButtons(ax_check, ['Show Ideal Pinhole Grid (Reference)'], [True])

# --- Pre-calculate Ideal Grid (Standard Straight Lines) ---
major_space = np.linspace(-1, 1, grid_res)
fine_space = np.linspace(-1, 1, num_fine_points)

ideal_lines = [] # Stores (x, y) coordinates of straight lines
# Horizontal lines
for y in major_space:
    ideal_lines.append((fine_space, np.full_like(fine_space, y)))
# Vertical lines
for x in major_space:
    ideal_lines.append((np.full_like(fine_space, x), fine_space))

# --- Initialize Matplotlib Objects ---
# These lists hold the actual line objects so they can be updated efficiently.
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

# Calculation Text Panel
text_math = fig.text(0.12, 0.88, "", fontsize=11, fontweight='bold',
                    bbox=dict(facecolor='wheat', alpha=0.7, boxstyle='round,pad=0.5'))

# --- Legend Setup ---
# plot_surface is tricky for legends, so we create custom handles
custom_handles = [
    Line2D([0], [0], color='blue', lw=1.5, label='Actual Distorted Grid'),
    Line2D([0], [0], color='gray', linestyle=':', lw=1, label='Ideal Pinhole Grid'),
    Line2D([0], [0], color='r', marker='+', linestyle='', markersize=10, mew=2, label='Principal Point')
]
legend = ax.legend(handles=custom_handles, loc='upper right', bbox_to_anchor=(1.0, 1.05))

# --- Update Function ---
def update(val=None):
    k1 = slider_k1.val
    show_ideal = check_ideal.get_status()[0]
    
    # -- 1. Update Ideal Grid Visibility --
    for line_obj in ideal_line_objects:
        line_obj.set_visible(show_ideal)
        if show_ideal:
            # Re-draw the pre-calculated ideal lines
            pass # We already set visibility, set_data is inside the loop below
            
    # -- 2. Apply Distortion Logic & Update Objects --
    # Iterates through both lists simultaneously using zip
    for i, ((x_ideal, y_ideal), d_obj, i_obj) in enumerate(zip(ideal_lines, distorted_line_objects, ideal_line_objects)):
        # -- 2a. Update Ideal Data --
        if show_ideal:
            i_obj.set_data(x_ideal, y_ideal)
            
        # -- 2b. Calculate & Update Distorted Data --
        # Radial Distortion Formula:
        # r^2 = x^2 + y^2
        # x' = x * (1 + k1 * r^2)
        # y' = y * (1 + k1 * r^2)
        r2 = x_ideal**2 + y_ideal**2
        
        x_distorted = x_ideal * (1 + k1 * r2)
        y_distorted = y_ideal * (1 + k1 * r2)
        
        d_obj.set_data(x_distorted, y_distorted)
        
    # -- 3. Update Title & Equation Box --
    # Dynamic Title
    if k1 < -0.01:
        ax.set_title(f"Lens Distortion Model: Barrel Distortion ($k_1 < 0$)", fontsize=14, pad=15)
    elif k1 > 0.01:
        ax.set_title(f"Lens Distortion Model: Pincushion Distortion ($k_1 > 0$)", fontsize=14, pad=15)
    else:
        ax.set_title("Lens Distortion Model: Perfect Pinhole ($k_1 \\approx 0$)", fontsize=14, pad=15)
        
    # Dynamic Equation and Current Calculation
    # We sample a single vertex (the corner) to show the math in action
    samp_x, samp_y = 1.0, 1.0
    samp_r2 = samp_x**2 + samp_y**2
    samp_x_dist = samp_x * (1 + k1 * samp_r2)
    samp_y_dist = samp_y * (1 + k1 * samp_r2)
    
    math_str = (f"Brown-Conrady Radial Model:\n\n"
                f"$\mathbf{{x_{{dist}}}} = \mathbf{{x}} \cdot (1 + \mathbf{{k_1}} \cdot \mathbf{{r^2}})$\n"
                f"Calculation for corner vertex:\n"
                f"x' = {samp_x:.1f} $\cdot$ (1 + {k1:.2f} $\cdot$ {samp_r2:.1f})\n"
                f"Distorted x' = {samp_x_dist:.2f}\n"
                f"r' = $\sqrt{{{samp_x_dist:.2f}^2 + {samp_y_dist:.2f}^2}} = \sqrt{{{samp_x_dist**2 + samp_y_dist**2:.2f}}}$")
    text_math.set_text(math_str)
    
    fig.canvas.draw_idle()

# --- Connect UI Elements and Initialize ---
slider_k1.on_changed(update)
check_ideal.on_clicked(update)

# Trigger the initial draw
update()

plt.show()