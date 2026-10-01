import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider, RadioButtons
from mpl_toolkits.mplot3d import Axes3D

# --- Initial Parameters ---
init_X = 10.0
init_Y = 5.0
init_Z = 15.0
init_f = 5.0

# --- Setup Figure and 3D Axes ---
fig = plt.figure(figsize=(14, 9))
ax = fig.add_subplot(111, projection='3d')

# Adjust layout to make room for sliders (bottom) and radio buttons (right)
plt.subplots_adjust(left=0.05, bottom=0.35, right=0.7)

# Set static axes limits
lim = 25
ax.set_xlim(-lim, lim)
ax.set_ylim(-lim, lim)
ax.set_zlim(-lim, lim)
ax.set_xlabel('X Axis')
ax.set_ylabel('Y Axis')
ax.set_zlabel('Z Axis')
ax.set_title("3D-to-2D Projection: Switching Planes")

# Ensure equal aspect ratio for realistic spatial representation
if hasattr(ax, 'set_box_aspect'):
    ax.set_box_aspect([1, 1, 1])

# Create a meshgrid for drawing the semi-transparent image planes
grid_range = np.linspace(-lim, lim, 10)
G1, G2 = np.meshgrid(grid_range, grid_range)

# --- Draw Static Elements & Initialize Dynamic Elements ---
# Camera Center (Origin)
ax.scatter([0], [0], [0], color='black', s=60, label='Camera Center (0,0,0)', zorder=5)

# Initialize dynamic plot objects with empty data
object_point, = ax.plot([], [], [], 'bo', markersize=10, label='Real 3D Point')
ray_line, = ax.plot([], [], [], 'g--', alpha=0.6, label='Light Ray')
proj_point, = ax.plot([], [], [], 'ro', markersize=8, label='Projected Point')
proj_text = ax.text(0, 0, 0, "", color='red', fontsize=11, fontweight='bold')
math_text = fig.text(0.72, 0.8, "", fontsize=12, bbox=dict(facecolor='wheat', alpha=0.5, boxstyle='round,pad=1'))

image_plane_surface = None

# Custom legend
ax.legend(loc='upper left')

# --- Setup UI Elements ---
# Slider Axes: [left, bottom, width, height]
ax_f = plt.axes([0.15, 0.25, 0.5, 0.03])
ax_X = plt.axes([0.15, 0.18, 0.5, 0.03])
ax_Y = plt.axes([0.15, 0.11, 0.5, 0.03])
ax_Z = plt.axes([0.15, 0.04, 0.5, 0.03])

# Radio Button Axes
ax_radio = plt.axes([0.72, 0.45, 0.25, 0.15], facecolor='lightgoldenrodyellow')

# Create UI Widgets
slider_f = Slider(ax_f, 'Focal Length (f)', 2.0, 15.0, valinit=init_f)
slider_X = Slider(ax_X, '3D Point X', -lim, lim, valinit=init_X)
slider_Y = Slider(ax_Y, '3D Point Y', -lim, lim, valinit=init_Y)
slider_Z = Slider(ax_Z, '3D Point Z', 1.0, lim, valinit=init_Z) # Prevent Z=0 initially

radio = RadioButtons(ax_radio, ('Z-Axis (Project to X-Y Plane)', 'X-Axis (Project to Y-Z Plane)'))

# --- Core Logic & Update Function ---
def update(val):
    global image_plane_surface
    
    # Read current slider values
    f = slider_f.val
    X = slider_X.val
    Y = slider_Y.val
    Z = slider_Z.val
    mode = radio.value_selected
    
    # Clean up the previous plane surface
    if image_plane_surface is not None:
        image_plane_surface.remove()

    # --- Math & Plane Generation based on active Axis ---
    if 'Z-Axis' in mode:
        # Looking down Z-axis. Image is on X-Y plane at depth f.
        Z_safe = Z if Z != 0 else 0.0001 # Prevent division by zero
        
        # Perspective Divide
        x_proj = f * (X / Z_safe)
        y_proj = f * (Y / Z_safe)
        z_proj = f
        
        # Draw Plane at Z=f
        surf_X = G1
        surf_Y = G2
        surf_Z = np.full_like(G1, f)
        image_plane_surface = ax.plot_surface(surf_X, surf_Y, surf_Z, color='gray', alpha=0.3)
        
        # Update text box with active formulas
        math_str = (f"Active Axis: Z\n\n"
                    f"x = f * (X / Z)\n"
                    f"x = {f:.1f} * ({X:.1f} / {Z:.1f}) = {x_proj:.2f}\n\n"
                    f"y = f * (Y / Z)\n"
                    f"y = {f:.1f} * ({Y:.1f} / {Z:.1f}) = {y_proj:.2f}")
        
    else:
        # Looking down X-axis. Image is on Y-Z plane at depth f.
        X_safe = X if X != 0 else 0.0001 
        
        # Perspective Divide
        x_proj = f
        y_proj = f * (Y / X_safe)
        z_proj = f * (Z / X_safe)
        
        # Draw Plane at X=f
        surf_X = np.full_like(G1, f)
        surf_Y = G1
        surf_Z = G2
        image_plane_surface = ax.plot_surface(surf_X, surf_Y, surf_Z, color='lightblue', alpha=0.3)
        
        # Update text box with active formulas
        math_str = (f"Active Axis: X\n\n"
                    f"y = f * (Y / X)\n"
                    f"y = {f:.1f} * ({Y:.1f} / {X:.1f}) = {y_proj:.2f}\n\n"
                    f"z = f * (Z / X)\n"
                    f"z = {f:.1f} * ({Z:.1f} / {X:.1f}) = {z_proj:.2f}")

    # --- Update Visual Positions ---
    # Update Real 3D Point
    object_point.set_data([X], [Y])
    object_point.set_3d_properties([Z])
    
    # Update Light Ray (connecting origin to real point)
    ray_line.set_data([0, X], [0, Y])
    ray_line.set_3d_properties([0, Z])
    
    # Update Projected Point on the plane
    proj_point.set_data([x_proj], [y_proj])
    proj_point.set_3d_properties([z_proj])
    
    # Update Coordinate Text near the projected point
    proj_text.set_position((x_proj, y_proj))
    proj_text.set_3d_properties(z_proj + 2, 'z')
    proj_text.set_text(f"({x_proj:.1f}, {y_proj:.1f}, {z_proj:.1f})")
    
    # Update Math formula box
    math_text.set_text(math_str)
    
    fig.canvas.draw_idle()

# --- Connect UI to Update Function ---
slider_f.on_changed(update)
slider_X.on_changed(update)
slider_Y.on_changed(update)
slider_Z.on_changed(update)
radio.on_clicked(update)

# Initialize the plot with default values
update(0)

# Show the interactive plot
plt.show()