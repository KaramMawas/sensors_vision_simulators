import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider, Button
from matplotlib.patches import Rectangle

# --- Initial Parameters ---
init_Y = 4.0
init_Z = 15.0
init_f = 4.0
init_S = 5.45

# --- Setup Figure and Plotting Area ---
fig, ax = plt.subplots(figsize=(12, 8))
plt.subplots_adjust(left=0.1, bottom=0.35, right=0.9, top=0.9)

# Axis limits (Left side for sensor/image, Right side for object)
x_min, x_max = -25, 35
y_min, y_max = -10, 15
ax.set_xlim(x_min, x_max)
ax.set_ylim(y_min, y_max)
ax.set_title("Thin Lens Camera Simulator: Focus & Blur", fontsize=14, pad=15)
ax.set_xlabel("Optical Axis Distance")
ax.set_ylabel("Height")
ax.grid(True, alpha=0.3)

# --- Draw Static Elements ---
# Optical Axis
ax.axhline(0, color='black', linestyle='--', linewidth=1)

# Lens (Represented as a stylized vertical blue shape at the origin)
ax.plot([0, 0], [y_min, y_max], color='lightblue', linewidth=8, alpha=0.6, label="Lens")
ax.text(0.5, y_max - 2, "Lens", color='tab:blue', fontweight='bold')

# --- Initialize Dynamic Elements ---
# Object (Arrow)
object_line, = ax.plot([], [], 'k-', linewidth=3, label='Object (Y)')
object_marker, = ax.plot([], [], 'kv', markersize=8) # Arrow tip

# Light Rays
ray1, = ax.plot([], [], 'g-', alpha=0.7, label='Ray 1 (Parallel -> Focal)')
ray2, = ax.plot([], [], 'orange', alpha=0.7, label='Ray 2 (Center)')
ray3, = ax.plot([], [], 'm-', alpha=0.7, label='Ray 3 (Focal -> Parallel)')

# Focal Points
focal_points, = ax.plot([], [], 'ro', markersize=6, label='Focal Points (F, F\')')
text_F1 = ax.text(0, 0.5, "-F", color='red', ha='center')
text_F2 = ax.text(0, 0.5, "+F", color='red', ha='center')

# Sensor Plane
sensor_line = ax.axvline(-init_S, color='gray', linewidth=4, label='Sensor Plane')
sensor_text = ax.text(-init_S, y_max - 1, 'Sensor', rotation=90, va='top', ha='right')

# Blur Circle (Red Rectangle)
blur_rect = Rectangle((0, 0), 1, 1, facecolor='red', alpha=0.0)
ax.add_patch(blur_rect)

# Text Displays
text_math = fig.text(0.12, 0.85, "", fontsize=11, bbox=dict(facecolor='white', alpha=0.8, boxstyle='round,pad=0.5'))
text_status = fig.text(0.12, 0.80, "", fontsize=14, fontweight='bold')

ax.legend(loc='upper right')

# --- UI Controls (Sliders & Buttons) ---
ax_Y = plt.axes([0.15, 0.20, 0.65, 0.03])
ax_Z = plt.axes([0.15, 0.15, 0.65, 0.03])
ax_f = plt.axes([0.15, 0.10, 0.65, 0.03])
ax_S = plt.axes([0.15, 0.05, 0.65, 0.03])
ax_btn = plt.axes([0.83, 0.04, 0.12, 0.05])

slider_Y = Slider(ax_Y, 'Object Height (Y)', 1.0, 10.0, valinit=init_Y)
slider_Z = Slider(ax_Z, 'Object Distance (Z)', 6.0, 30.0, valinit=init_Z)
slider_f = Slider(ax_f, 'Focal Length (f)', 2.0, 10.0, valinit=init_f)
slider_S = Slider(ax_S, 'Sensor Position', 2.0, 20.0, valinit=init_S)
btn_focus = Button(ax_btn, 'Auto-Focus', color='lightgreen', hovercolor='palegreen')

# --- Core Logic & Update Function ---
def update(val=None):
    Y = slider_Y.val
    Z = slider_Z.val
    f = slider_f.val
    S = slider_S.val
    
    # Safe division to prevent crashing if Z equals f exactly
    Z_safe = Z if abs(Z - f) > 0.01 else Z + 0.01 
    
    # Thin Lens Equation: 1/v = 1/f - 1/Z
    v = 1.0 / ((1.0 / f) - (1.0 / Z_safe))
    
    # Update Object
    object_line.set_data([Z, Z], [0, Y])
    object_marker.set_data([Z], [Y])
    
    # Update Focal Points
    focal_points.set_data([-f, f], [0, 0])
    text_F1.set_position((-f, 0.5))
    text_F2.set_position((f, 0.5))
    
    # Update Sensor
    sensor_line.set_xdata([-S, -S])
    sensor_text.set_position((-S - 0.5, y_max - 1))
    
    # Calculate Ray Paths (originating from right (+Z), moving left towards sensor)
    # Ray 1: Parallel to axis, hits lens at (0, Y), refracts through -f
    slope1 = Y / f
    y_sensor1 = slope1 * (-S) + Y
    ray1.set_data([Z, 0, x_min], [Y, Y, slope1 * x_min + Y])
    
    # Ray 2: Through center (0,0), straight line
    slope2 = Y / Z_safe
    y_sensor2 = slope2 * (-S)
    ray2.set_data([Z, 0, x_min], [Y, 0, slope2 * x_min])
    
    # Ray 3: Through +f, hits lens, travels parallel
    y_lens3 = -Y * f / (Z_safe - f)
    y_sensor3 = y_lens3
    ray3.set_data([Z, f, 0, x_min], [Y, 0, y_lens3, y_lens3])
    
    # Blur Circle Calculation (Height difference of rays exactly at the sensor plane)
    y_min_ray = min(y_sensor1, y_sensor2, y_sensor3)
    y_max_ray = max(y_sensor1, y_sensor2, y_sensor3)
    blur_height = y_max_ray - y_min_ray
    
    # Check Focus Status (Tolerance of 0.15 units)
    if abs(S - v) < 0.15:
        text_status.set_text("SHARP FOCUS")
        text_status.set_color("green")
        blur_rect.set_alpha(0) # Hide blur circle
    else:
        text_status.set_text("OUT OF FOCUS")
        text_status.set_color("red")
        blur_rect.set_alpha(0.3)
        # Draw red rectangle on sensor
        blur_rect.set_bounds(-S - 0.5, y_min_ray, 1.0, blur_height)
        
    # Update Math Text
    math_str = (f"Thin Lens Equation: 1/f = 1/Z + 1/v\n"
                f"Convergence Point (v) = {v:.2f}\n"
                f"Current Sensor Pos = {S:.2f}")
    text_math.set_text(math_str)
    
    fig.canvas.draw_idle()

# --- Auto-Focus Callback ---
def autofocus(event):
    Z = slider_Z.val
    f = slider_f.val
    if Z > f:
        # Calculate ideal v
        ideal_v = 1.0 / ((1.0 / f) - (1.0 / Z))
        # Snap sensor slider to ideal v (triggers update automatically)
        slider_S.set_val(ideal_v)

# Connect events
slider_Y.on_changed(update)
slider_Z.on_changed(update)
slider_f.on_changed(update)
slider_S.on_changed(update)
btn_focus.on_clicked(autofocus)

# Initialize
update()

plt.show()