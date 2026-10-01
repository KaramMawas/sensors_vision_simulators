"""
===============================================================================
Project: Sensor Vision Simulators
Module: Photogrammetry - Epipolar Geometry
File: epipolar_geometry.py

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
An interactive three-dimensional epipolar geometry simulator illustrating two
camera centers, baseline, projection rays, epipolar planes, image planes,
epipoles, epipolar lines, and independently adjustable camera yaw angles.
===============================================================================
"""


import matplotlib.pyplot as plt
import numpy as np
from matplotlib.widgets import Slider
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# --- Initial Parameters ---
f = 10.0
init_A = np.array([-15.0, 0.0, 0.0])
init_B = np.array([15.0, 0.0, 0.0])
init_P = np.array([0.0, 15.0, 30.0])
init_yaw_A = 15.0  # Degrees (positive rotates right)
init_yaw_B = -15.0 # Degrees (negative rotates left)

# --- Setup Figure ---
fig = plt.figure(figsize=(14, 10))
ax = fig.add_subplot(111, projection='3d')
# Increased bottom margin to fit all stacked sliders safely
plt.subplots_adjust(left=0.1, bottom=0.4, right=0.95, top=0.95)

# --- UI Controls (Stacked Vertically) ---
# Axes format: [left, bottom, width, height]
ax_Px = plt.axes([0.2, 0.30, 0.65, 0.03])
ax_Py = plt.axes([0.2, 0.25, 0.65, 0.03])
ax_Pz = plt.axes([0.2, 0.20, 0.65, 0.03])
ax_yawA = plt.axes([0.2, 0.13, 0.65, 0.03])
ax_yawB = plt.axes([0.2, 0.08, 0.65, 0.03])

s_Px = Slider(ax_Px, 'Point P (X)', -30.0, 30.0, valinit=init_P[0])
s_Py = Slider(ax_Py, 'Point P (Y)', -20.0, 30.0, valinit=init_P[1])
s_Pz = Slider(ax_Pz, 'Point P (Z)', 10.0, 60.0, valinit=init_P[2])
s_yawA = Slider(ax_yawA, 'Cam A Pan (deg)', -45.0, 45.0, valinit=init_yaw_A)
s_yawB = Slider(ax_yawB, 'Cam B Pan (deg)', -45.0, 45.0, valinit=init_yaw_B)

# --- Helper Functions ---
def get_plane_basis(yaw_deg):
    """Returns normal vector and local u,v vectors for the sensor plane."""
    rad = np.radians(yaw_deg)
    n = np.array([np.sin(rad), 0, np.cos(rad)])
    u = np.array([np.cos(rad), 0, -np.sin(rad)])
    v = np.array([0, 1, 0])
    return n, u, v

def intersect_ray_plane(ray_origin, ray_dir, plane_center, plane_normal):
    """Find intersection of a ray with a plane."""
    denom = np.dot(ray_dir, plane_normal)
    if abs(denom) < 1e-6:
        return None # Ray is parallel to plane
    t = np.dot(plane_center - ray_origin, plane_normal) / denom
    return ray_origin + t * ray_dir

# --- Update Function ---
def update(val=None):
    ax.clear()
    
    P = np.array([s_Px.val, s_Py.val, s_Pz.val])
    yawA = s_yawA.val
    yawB = s_yawB.val
    
    if hasattr(ax, 'set_box_aspect'):
        ax.set_box_aspect([1, 1, 1])
        
    # 1. Cameras and Baseline
    ax.scatter(*init_A, color='black', s=60, label='Camera Center')
    ax.scatter(*init_B, color='black', s=60)
    ax.plot([init_A[0], init_B[0]], [init_A[1], init_B[1]], [init_A[2], init_B[2]], 'k-', lw=2, label='Baseline')
    
    # Text labels for Cameras
    ax.text(init_A[0], init_A[1]-3, init_A[2], "Camera A", color='black', fontweight='bold', ha='center')
    ax.text(init_B[0], init_B[1]-3, init_B[2], "Camera B", color='black', fontweight='bold', ha='center')
    
    # 2. Point P and Rays
    ax.scatter(*P, color='blue', s=80, label='3D Point P')
    ax.text(P[0], P[1]+2, P[2], "Point P", color='blue', fontweight='bold', ha='center')
    
    ax.plot([P[0], init_A[0]], [P[1], init_A[1]], [P[2], init_A[2]], 'g--', alpha=0.5, label='Projection Rays')
    ax.plot([P[0], init_B[0]], [P[1], init_B[1]], [P[2], init_B[2]], 'g--', alpha=0.5)
    
    # 3. Epipolar Plane Normal
    vec_AB = init_B - init_A
    vec_AP = P - init_A
    epipolar_normal = np.cross(vec_AB, vec_AP)
    norm_len = np.linalg.norm(epipolar_normal)
    if norm_len > 1e-6:
        epipolar_normal = epipolar_normal / norm_len
        
    # Draw Epipolar Triangle
    poly = Poly3DCollection([[init_A, init_B, P]], facecolors='cyan', edgecolors='none', alpha=0.2, label='Epipolar Plane')
    ax.add_collection3d(poly)
    
    # 4. Generate Image Planes, Epipoles, and Epipolar Lines
    plane_size = 15
    uu, vv = np.meshgrid(np.linspace(-plane_size, plane_size, 5), 
                         np.linspace(-plane_size, plane_size, 5))
    
    # Loop over both cameras
    for idx, (cam_center, yaw, color, name, baseline_dir) in enumerate([
        (init_A, yawA, 'red', 'A', init_B - init_A), 
        (init_B, yawB, 'orange', 'B', init_A - init_B)
    ]):
        n, u_vec, v_vec = get_plane_basis(yaw)
        sensor_center = cam_center + f * n
        
        # Draw the 3D grid for the sensor
        X_grid = sensor_center[0] + uu * u_vec[0] + vv * v_vec[0]
        Y_grid = sensor_center[1] + uu * u_vec[1] + vv * v_vec[1]
        Z_grid = sensor_center[2] + uu * u_vec[2] + vv * v_vec[2]
        # Only label the first sensor plane to avoid duplicate legend entries
        plane_label = 'Image Sensor Plane' if idx == 0 else ""
        ax.plot_surface(X_grid, Y_grid, Z_grid, color='gray', alpha=0.2, label=plane_label)
        
        # --- Project Point P onto this sensor ---
        ray_dir = P - cam_center
        P_proj = intersect_ray_plane(cam_center, ray_dir, sensor_center, n)
        
        # --- Calculate Epipole (Intersection of Baseline with Sensor) ---
        epipole = intersect_ray_plane(cam_center, baseline_dir, sensor_center, n)
        
        if epipole is not None:
            # Draw Epipole
            ax.scatter(*epipole, color='purple', s=60, marker='X', zorder=6, label='Epipole' if idx==0 else "")
            ax.text(epipole[0], epipole[1]-2, epipole[2], f"Epipole {name}", color='purple', fontsize=9)
            
        if P_proj is not None:
            # Draw Projected Point
            ax.scatter(*P_proj, color=color, s=50, zorder=5, label='Projected Point' if idx==0 else "")
            
            # Find Epipolar Line direction
            line_dir = np.cross(epipolar_normal, n)
            line_len = np.linalg.norm(line_dir)
            
            if line_len > 1e-6:
                line_dir = line_dir / line_len
                # Draw the line passing through P_proj
                L_start = P_proj - line_dir * plane_size * 1.5
                L_end = P_proj + line_dir * plane_size * 1.5
                ax.plot([L_start[0], L_end[0]], 
                        [L_start[1], L_end[1]], 
                        [L_start[2], L_end[2]], color=color, lw=2, label='Epipolar Line' if idx==0 else "")

    # Clean up duplicate legend entries
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    # Exclude empty labels from surface plots
    valid_handles_labels = {l: h for l, h in by_label.items() if l}
    ax.legend(valid_handles_labels.values(), valid_handles_labels.keys(), loc='upper left', bbox_to_anchor=(0.0, 1.05))

    # Lock axis limits at the end so it doesn't jump wildly if an epipole goes to infinity
    ax.set_xlim(-40, 40)
    ax.set_ylim(-20, 40)
    ax.set_zlim(-5, 65)
    ax.set_xlabel('X Axis')
    ax.set_ylabel('Y Axis')
    ax.set_zlabel('Z Axis (Depth)')
    ax.set_title("Epipolar Geometry: Converging Epipolar Lines & Epipoles", fontsize=15, pad=20)
    
    fig.canvas.draw_idle()

# --- Connect and Run ---
s_Px.on_changed(update)
s_Py.on_changed(update)
s_Pz.on_changed(update)
s_yawA.on_changed(update)
s_yawB.on_changed(update)

update()
plt.show()