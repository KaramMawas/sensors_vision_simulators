import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider, CheckButtons
from matplotlib.lines import Line2D

# --- Physical Parameters ---
init_s1 = 0.04  # Shift in X proportional to r^2
init_s2 = 0.02  # Shift in Y proportional to r^2
grid_res = 11
num_fine_points = 60

# --- Scene Setup ---
fig = plt.figure(figsize=(14, 10))
ax = fig.add_subplot(111)
plt.subplots_adjust(left=0.1, bottom=0.25, right=0.95, top=0.90)

ax.set_xlim(-1.4, 1.4)
ax.set_ylim(-1.4, 1.4)
ax.set_aspect('equal')
ax.set_xlabel("Normalized Sensor X")
ax.set_ylabel("Normalized Sensor Y")

# --- UI Controls ---
ax_s1 = plt.axes([0.25, 0.12, 0.50, 0.03])
ax_s2 = plt.axes([0.25, 0.07, 0.50, 0.03])
ax_check = plt.axes([0.25, 0.01, 0.50, 0.05], frameon=False)

slider_s1 = Slider(ax_s1, r'Thin Prism $s_1$', -0.15, 0.15, valinit=init_s1)
slider_s2 = Slider(ax_s2, r'Thin Prism $s_2$', -0.15, 0.15, valinit=init_s2)
check_ideal = CheckButtons(ax_check, ['Show Ideal Pinhole Grid'], [True])

# --- Pre-calculate Ideal Grid ---
major_space = np.linspace(-1, 1, grid_res)
fine_space = np.linspace(-1, 1, num_fine_points)
ideal_lines = [] 
for y in major_space:
    ideal_lines.append((fine_space, np.full_like(fine_space, y)))
for x in major_space:
    ideal_lines.append((np.full_like(fine_space, x), fine_space))

# --- Initialize Objects ---
ideal_line_objects = []
distorted_line_objects = []

for _ in range(grid_res * 2):
    line_d, = ax.plot([], [], 'b-', lw=1.5, alpha=0.9)
    line_i, = ax.plot([], [], 'gray', linestyle=':', lw=1, alpha=0.6)
    distorted_line_objects.append(line_d)
    ideal_line_objects.append(line_i)

ax.plot([0], [0], 'r+', markersize=12, lw=2)
text_math = fig.text(0.1, 0.92, "", fontsize=9.5, fontweight='bold', va='top',
                    bbox=dict(facecolor='aliceblue', alpha=0.8, boxstyle='round,pad=0.3'))

# --- Legend ---
custom_handles = [
    Line2D([0], [0], color='blue', lw=1.5, label='Thin Prism Distorted'),
    Line2D([0], [0], color='gray', linestyle=':', lw=1, label='Ideal Grid'),
    Line2D([0], [0], color='r', marker='+', linestyle='', label='Principal Point')
]
ax.legend(handles=custom_handles, loc='upper right', bbox_to_anchor=(1.5, 1.05))

# --- Update Function ---
def update(val=None):
    s1 = slider_s1.val
    s2 = slider_s2.val
    show_ideal = check_ideal.get_status()[0]
    
    ax.set_title("Thin Prism / Sensor Tilt Model ($s_1, s_2$)", fontsize=14, pad=15)
    
    # Vertex Sample for Math Box (1,1)
    samp_x, samp_y = 1.0, 1.0
    samp_r2 = samp_x**2 + samp_y**2
    samp_dx = s1 * samp_r2
    samp_dy = s2 * samp_r2

    for i, ((x_i, y_i), d_obj, i_obj) in enumerate(zip(ideal_lines, distorted_line_objects, ideal_line_objects)):
        i_obj.set_visible(show_ideal)
        if show_ideal: i_obj.set_data(x_i, y_i)
        
        # Prism Logic: Radial weighting shift
        r2 = x_i**2 + y_i**2
        x_dist = x_i + s1 * r2
        y_dist = y_i + s2 * r2
        d_obj.set_data(x_dist, y_dist)
        
    math_str = (f"Thin Prism Model:\n"
                f"$\Delta x = s_1 \cdot (x^2 + y^2)$\n"
                f"$\Delta y = s_2 \cdot (x^2 + y^2)$\n"
                f"--- Corner Vertex (1,1) ---\n"
                f"$\Delta x = {s1:.2f} \cdot {samp_r2:.1f} = {samp_dx:.2f}$\n"
                f"$\Delta y = {s2:.2f} \cdot {samp_r2:.1f} = {samp_dy:.2f}$\n"
                f"Projected: ({1+samp_dx:.2f}, {1+samp_dy:.2f})")
    text_math.set_text(math_str)
    fig.canvas.draw_idle()

slider_s1.on_changed(update)
slider_s2.on_changed(update)
check_ideal.on_clicked(update)

update()
plt.show()