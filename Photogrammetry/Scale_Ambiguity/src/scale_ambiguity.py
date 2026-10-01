import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider, CheckButtons, RadioButtons

# --- Initial Parameters ---
init_f = 15.0
init_D = 40.0  # Depth distance (can be X, Y, or Z depending on mode)
init_S = 25.0   # Object True Size
is_locked = True
target_x = init_f * (init_S / init_D) # The projected size we want to lock

is_updating = False

# --- Setup Figure and Layout ---
fig = plt.figure(figsize=(15, 8))
plt.subplots_adjust(left=0.05, right=0.95, bottom=0.35, top=0.9)

# Left Subplot: 3D Scene Explorer
ax1 = fig.add_subplot(121, projection='3d')
# Right Subplot: 2D Projected Image
ax2 = fig.add_subplot(122)

# --- UI Controls ---
ax_f = plt.axes([0.10, 0.20, 0.4, 0.03])
ax_D = plt.axes([0.10, 0.15, 0.4, 0.03])
ax_S = plt.axes([0.10, 0.10, 0.4, 0.03])

# Checkbox for Lock
ax_lock = plt.axes([0.60, 0.10, 0.25, 0.08])
# Radio buttons for Planes
ax_radio = plt.axes([0.60, 0.20, 0.35, 0.13])

s_f = Slider(ax_f, 'Focal Length (f)', 5.0, 30.0, valinit=init_f)
s_D = Slider(ax_D, 'Distance (Depth)', 10.0, 100.0, valinit=init_D)
s_S = Slider(ax_S, 'True Size', 1.0, 50.0, valinit=init_S)

check_lock = CheckButtons(ax_lock, ['Lock Projected Shape\n(Show Ambiguity)'], [is_locked])
# Note Lock is photogrammetry case since the object is already captured so we dont know the true size or distance, we only have the projected size. 
# So changing focal length or distance will change the other to maintain the same projected size.
# Whithout a lock is a normal real photography case where we can change the focal length and distance and the projected size changes accordingly.
radio = RadioButtons(ax_radio, ('Z-Axis (Image on X-Y Plane)', 
                                'X-Axis (Image on Y-Z Plane)', 
                                'Y-Axis (Image on X-Z Plane)'))

# --- Drawing & Logic Function ---
def draw_scene(f, d, S, axis_mode):
    # Calculate projected size (perspective divide)
    proj_size = f * (S / d)
    
    # -- 1. Update 3D Scene (ax1) --
    ax1.clear()
    ax1.set_title("3D Scene Explorer", fontsize=14)
    
    # Make a uniform cubic space so switching axes doesn't warp the view
    lim = 105
    ax1.set_xlim(-lim, lim)
    ax1.set_ylim(-lim, lim)
    ax1.set_zlim(-lim, lim)
    
    # Draw Camera Center
    ax1.scatter([0], [0], [0], color='black', s=50, label='Camera Center')
    
    plane_size = 20
    s2 = S / 2.0
    p2 = proj_size / 2.0
    
    # Determine coordinates based on selected axis
    if 'Z-Axis' in axis_mode:
        ax1.set_xlabel('X')
        ax1.set_ylabel('Y')
        ax1.set_zlabel('Depth (Z)', fontweight='bold')
        
        # Sensor Plane at Z = -f
        xx, yy = np.meshgrid([-plane_size, plane_size], [-plane_size, plane_size])
        zz = np.full_like(xx, -f)
        
        # Object and Projection Vertices
        obj_verts = [[s2, s2, d], [-s2, s2, d], [-s2, -s2, d], [s2, -s2, d], [s2, s2, d]]
        proj_verts = [[-p2, -p2, -f], [p2, -p2, -f], [p2, p2, -f], [-p2, p2, -f], [-p2, -p2, -f]]
        
    elif 'X-Axis' in axis_mode:
        ax1.set_xlabel('Depth (X)', fontweight='bold')
        ax1.set_ylabel('Y')
        ax1.set_zlabel('Z')
        
        # Sensor Plane at X = -f
        yy, zz = np.meshgrid([-plane_size, plane_size], [-plane_size, plane_size])
        xx = np.full_like(yy, -f)
        
        # Object and Projection Vertices
        obj_verts = [[d, s2, s2], [d, -s2, s2], [d, -s2, -s2], [d, s2, -s2], [d, s2, s2]]
        proj_verts = [[-f, -p2, -p2], [-f, p2, -p2], [-f, p2, p2], [-f, -p2, p2], [-f, -p2, -p2]]
        
    elif 'Y-Axis' in axis_mode:
        ax1.set_xlabel('X')
        ax1.set_ylabel('Depth (Y)', fontweight='bold')
        ax1.set_zlabel('Z')
        
        # Sensor Plane at Y = -f
        xx, zz = np.meshgrid([-plane_size, plane_size], [-plane_size, plane_size])
        yy = np.full_like(xx, -f)
        
        # Object and Projection Vertices
        obj_verts = [[s2, d, s2], [-s2, d, s2], [-s2, d, -s2], [s2, d, -s2], [s2, d, s2]]
        proj_verts = [[-p2, -f, -p2], [p2, -f, -p2], [p2, -f, p2], [-p2, -f, p2], [-p2, -f, -p2]]

    # Draw the calculated elements
    ax1.plot_surface(xx, yy, zz, color='gray', alpha=0.2)
    
    ox, oy, oz = zip(*obj_verts)
    ax1.plot(ox, oy, oz, color='blue', linewidth=2, label='True Object')
    
    px, py, pz = zip(*proj_verts)
    ax1.plot(px, py, pz, color='red', linewidth=2, label='Projection')
    
    # Draw Rays of Projection
    for i in range(4):
        ax1.plot([ox[i], px[i]], [oy[i], py[i]], [oz[i], pz[i]], 'g--', alpha=0.5)
        
    ax1.legend(loc='upper left')

    # -- 2. Update 2D Image View (ax2) --
    ax2.clear()
    ax2.set_title("2D Sensor Image (What the camera sees)", fontsize=14)
    ax2.set_xlim(-15, 15)
    ax2.set_ylim(-15, 15)
    ax2.set_aspect('equal')
    ax2.grid(True, linestyle=':', alpha=0.6)
    
    # Draw the projected footprint
    box = plt.Rectangle((-p2, -p2), proj_size, proj_size, edgecolor='red', facecolor='pink', alpha=0.5, lw=2)
    ax2.add_patch(box)
    
    # Add math text
    math_text = (f"Perspective Divide:\n"
                 f"Pixel Size = f * (True Size / Depth)\n"
                 f"Pixel Size = {f:.1f} * ({S:.1f} / {d:.1f})\n"
                 f"Pixel Size = {proj_size:.2f}")
    ax2.text(-14, 14, math_text, va='top', fontsize=11, bbox=dict(facecolor='white', alpha=0.8))


# --- Interaction Handlers ---
def update_logic(source):
    global is_updating, target_x, is_locked
    
    if is_updating:
        return
    is_updating = True
    
    f = s_f.val
    d = s_D.val
    S = s_S.val
    axis_mode = radio.value_selected
    
    if is_locked:
        if source == 'D':
            new_S = (target_x * d) / f
            s_S.set_val(new_S)
            S = new_S
        elif source == 'S':
            new_d = (f * S) / target_x
            s_D.set_val(new_d)
            d = new_d
        elif source == 'f':
            new_S = (target_x * d) / f
            s_S.set_val(new_S)
            S = new_S
    else:
        target_x = f * (S / d)
        
    draw_scene(f, d, S, axis_mode)
    fig.canvas.draw_idle()
    is_updating = False

def toggle_lock(label):
    global is_locked, target_x
    is_locked = not is_locked
    if is_locked:
        target_x = s_f.val * (s_S.val / s_D.val)

def radio_changed(label):
    update_logic('radio')

# Attach handlers
s_f.on_changed(lambda val: update_logic('f'))
s_D.on_changed(lambda val: update_logic('D'))
s_S.on_changed(lambda val: update_logic('S'))
check_lock.on_clicked(toggle_lock)
radio.on_clicked(radio_changed)

# Initial draw
draw_scene(init_f, init_D, init_S, radio.value_selected)

plt.show()