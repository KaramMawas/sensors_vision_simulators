import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# --- Initial Parameters ---
init_X = 10.0  # Object height
init_Z = 20.0  # Object distance
init_f = 5.0   # Focal length

# --- Setup Figure and Axes ---
fig, ax = plt.subplots(figsize=(10, 6))
plt.subplots_adjust(left=0.1, bottom=0.35) # Make room for sliders at the bottom
ax.set_xlim(-20, 60)
ax.set_ylim(-15, 35)
ax.set_aspect('equal') # Ensures 1 unit on X axis equals 1 unit on Y axis
ax.set_title("Pinhole Camera Z-Ambiguity Simulator")
ax.set_xlabel("Distance along optical axis")
ax.set_ylabel("Height")

# --- Draw Static Elements ---
# The Pinhole plane (Z=0)
ax.axvline(0, color='black', linestyle='--', label='Pinhole Plane')
ax.plot(0, 0, 'ko', markersize=8, label='Pinhole')

# --- Initialize Dynamic Elements ---
# Image plane
image_plane_line = ax.axvline(-init_f, color='gray', linestyle=':', label='Image Plane')

# Object (Blue line), Projected Image (Red line), and Light Rays (Green dashed)
object_line, = ax.plot([init_Z, init_Z], [0, init_X], 'b-', lw=4, label='Real Object (X)')
image_line, = ax.plot([-init_f, -init_f], [0, -init_f * (init_X / init_Z)], 'r-', lw=4, label='Projected Image (x)')

ray_top, = ax.plot([init_Z, -init_f], [init_X, -init_f * (init_X / init_Z)], 'g--', alpha=0.6)
ray_bottom, = ax.plot([init_Z, -init_f], [0, 0], 'g--', alpha=0.6)

# Text box to show the real-time math
text_math = ax.text(0.05, 0.95, '', transform=ax.transAxes, va='top', fontsize=12,
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

ax.legend(loc='upper right')

# --- Setup Sliders ---
# Define the axes for the sliders [left, bottom, width, height]
ax_f = plt.axes([0.15, 0.2, 0.65, 0.03])
ax_Z = plt.axes([0.15, 0.15, 0.65, 0.03])
ax_X = plt.axes([0.15, 0.1, 0.65, 0.03])

# Create the slider objects
slider_f = Slider(ax_f, 'Focal Length (f)', 2.0, 15.0, valinit=init_f)
slider_Z = Slider(ax_Z, 'Distance (Z)', 5.0, 50.0, valinit=init_Z)
slider_X = Slider(ax_X, 'Object Height (X)', 1.0, 30.0, valinit=init_X)

# --- Update Function ---
def update(val):
    # Get current slider values
    f = slider_f.val
    Z = slider_Z.val
    X = slider_X.val
    
    # Calculate projected size (Notice it will be negative because pinhole images are inverted)
    x_proj = f * (X / Z)
    
    # Update the positions of all lines based on new values
    image_plane_line.set_xdata([-f, -f])
    
    object_line.set_data([Z, Z], [0, X])
    image_line.set_data([-f, -f], [0, -x_proj])
    
    ray_top.set_data([Z, -f], [X, -x_proj])
    ray_bottom.set_data([Z, -f], [0, 0])
    
    # Update the math text box
    text_math.set_text(f"Projected Size (x) = f * (X / Z)\n"
                       f"x = {f:.1f} * ({X:.1f} / {Z:.1f}) = {x_proj:.2f}")
    
    # Redraw the canvas
    fig.canvas.draw_idle()

# Register the update function with the sliders
slider_f.on_changed(update)
slider_Z.on_changed(update)
slider_X.on_changed(update)

# Initialize the text box with starting values
update(0)

# Display the plot
plt.show()