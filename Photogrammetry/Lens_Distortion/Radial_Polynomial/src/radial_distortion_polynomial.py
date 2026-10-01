import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider, CheckButtons
from matplotlib.lines import Line2D

# --- Physical Parameters ---
# INITIAL STATE: Complex 'Mustache' distortion
init_k1 = -0.15 # Strong barrel in the center
init_k2 = 0.10  # Counter-acts center, complex flex
init_k3 = 0.05  # Strong pincushion near extreme edges
grid_res = 13   # Number of major grid lines
num_fine_points = 70 # High density for smooth curves

# --- Scene Setup ---
fig = plt.figure(figsize=(14, 11))
ax = fig.add_subplot(111)
# Wide margins for stacked vertical sliders and the math box
plt.subplots_adjust(left=0.1, bottom=0.35, right=0.95, top=0.90)

# Set static axes properties (normalized sensor coordinates -1 to 1)
lim = 1.4
ax.set_xlim(-lim, lim)
ax.set_ylim(-lim, lim)
ax.set_aspect('equal')
ax.set_xlabel("Normalized Sensor X")
ax.set_ylabel("Normalized Sensor Y")
ax.grid(True, which='both', linestyle='-', alpha=0.1)

# --- UI Controls (Stacked Vertically) ---
ax_k1 = plt.axes([0.20, 0.20, 0.60, 0.03])
ax_k2 = plt.axes([0.20, 0.15, 0.60, 0.03])
ax_k3 = plt.axes([0.20, 0.10, 0.60, 0.03])
ax_check = plt.axes([0.20, 0.02, 0.60, 0.05], frameon=False)

slider_k1 = Slider(ax_k1, r'1st Coeff ($k_1$)', -0.5, 0.5, valinit=init_k1)
slider_k2 = Slider(ax_k2, r'2nd Coeff ($k_2$)', -0.5, 0.5, valinit=init_k2)
slider_k3 = Slider(ax_k3, r'3rd Coeff ($k_3$)', -0.5, 0.5, valinit=init_k3)
check_ideal = CheckButtons(ax_check, ['Show Ideal Pinhole Grid (Reference)'], [True])

# --- Pre-calculate Ideal Grid (Standard Straight Lines) ---
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
# Moved to the top-left (X=0.1, Y=0.92) and anchored 'top' so it grows downward
text_math = fig.text(0.1, 0.92, "", fontsize=9.5, fontweight='bold', va='top',
                    bbox=dict(facecolor='wheat', alpha=0.8, boxstyle='round,pad=0.3'))

# --- Legend Setup ---
custom_handles = [
    Line2D([0], [0], color='blue', lw=1.5, label='Actual Distorted Grid'),
    Line2D([0], [0], color='gray', linestyle=':', lw=1, label='Ideal Pinhole Grid'),
    Line2D([0], [0], color='r', marker='+', linestyle='', markersize=10, mew=2, label='Principal Point')
]
legend = ax.legend(handles=custom_handles, loc='upper right', bbox_to_anchor=(1.5, 1.05))

# --- Update Function ---
def update(val=None):
    k1 = slider_k1.val
    k2 = slider_k2.val
    k3 = slider_k3.val
    show_ideal = check_ideal.get_status()[0]
    
    # Simple logic to define the title based on the primary distortion
    if abs(k1) < 0.05 and abs(k2) < 0.05 and abs(k3) < 0.05:
        ax.set_title("Complete Radial Model: Ideal Pinhole Camera ($k_{1,2,3} \\approx 0$)", fontsize=15, pad=15)
    elif k1 < -0.10 and k2 > 0.05:
        ax.set_title("Complete Radial Model: Complex 'Mustache' Distortion", fontsize=15, pad=15)
    elif k1 < -0.01:
        ax.set_title(f"Complete Radial Model: Barrel Distortion ($k_1 < 0$)", fontsize=15, pad=15)
    elif k1 > 0.01:
        ax.set_title(f"Complete Radial Model: Pincushion Distortion ($k_1 > 0$)", fontsize=15, pad=15)
    else:
        ax.set_title("Complete Radial Model (Custom State)", fontsize=15, pad=15)
        
    # We sample a single vertex (the corner) to show the math in action
    samp_x, samp_y = 1.0, 1.0
    samp_r2 = samp_x**2 + samp_y**2
    samp_r4 = samp_r2**2
    samp_r6 = samp_r2**3
    
    # Calculate the Sampe Radial Scaling Factor L(r)
    # L(r) = (1 + k1*r^2 + k2*r^4 + k3*r^6)
    L_r = (1 + k1*samp_r2 + k2*samp_r4 + k3*samp_r6)
    samp_x_distorted = samp_x * L_r
    
    # --- Apply Distortion Logic & Update Objects ---
    # Iterates through lists simultaneously using zip
    for i, ((x_ideal, y_ideal), d_obj, i_obj) in enumerate(zip(ideal_lines, distorted_line_objects, ideal_line_objects)):
        # -- Update Ideal Grid --
        i_obj.set_visible(show_ideal)
        if show_ideal:
            i_obj.set_data(x_ideal, y_ideal)
            
        # -- Calculate & Update Distorted Grid --
        # Full polynomial for Radial Distortion
        r2 = x_ideal**2 + y_ideal**2
        r4 = r2**2
        r6 = r2**3
        
        radial_scale = (1 + k1*r2 + k2*r4 + k3*r6)
        
        x_distorted = x_ideal * radial_scale
        y_distorted = y_ideal * radial_scale
        
        d_obj.set_data(x_distorted, y_distorted)
        
    # Update Math formula box with current sample
    # Dynamic calculation display
    math_str = (f"Brown-Conrady Polynomial Radial Model:\n"
                f"L(r) = 1 + k_1r^2 + k_2r^4 + k_3r^6\n"
                f"$\mathbf{{x_{{dist}}}}$ = x $\cdot$ L(r)\n\n"
                f"Calc for corner vertex (1,1) where $r^2$=2:\n"
                f"L(r) = 1 + [{k1:.2f} $\cdot$ 2] + [{k2:.2f} $\cdot$ 4] + [{k3:.2f} $\cdot$ 8]\n"
                f"L(r) = 1 + ({k1*2:.2f}) + ({k2*4:.2f}) + ({k3*8:.2f})\n"
                f"L(r) = {L_r:.3f}\n"
                f"Projected x' = 1 $\cdot$ {L_r:.3f} = {samp_x_distorted:.3f}")
    
    text_math.set_text(math_str)
    
    fig.canvas.draw_idle()

# --- Connect UI Elements and Initialize ---
slider_k1.on_changed(update)
slider_k2.on_changed(update)
slider_k3.on_changed(update)
check_ideal.on_clicked(update)

# Trigger the initial draw
update()

plt.show()