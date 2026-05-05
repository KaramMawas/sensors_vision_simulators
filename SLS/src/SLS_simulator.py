"""
===============================================================================
Project: Sensor Vision Simulators
Module: Structured Light Scanner (SLS) Simulator 
File: lidar_depth_camera_live_sim.py

Author: Karam Mawas
Affiliation: Technical University of Braunschweig / Institute of Geodesy and Photogrammetry (IGP)
Email: karam.mawas@gmail.com
GitHub: https://github.com/KaramMawas
Website: https://karammawas.github.io/
ORCID: https://orcid.org/0000-0002-8608-7578

Created: 2026-04-14
Last Updated: 2026-05-05

Copyright (c) 2026 Karam Mawas
License: MIT

Description:
A visual and interactive simulator for a **Structured Light Scanner (SLS)** sensor.  
This project demonstrates how a **projector-camera system** uses projected stripe patterns to observe surface deformation and estimate 3D structure.
===============================================================================
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.widgets import Slider, CheckButtons
import matplotlib.animation as animation

def simulate_strobe_sls():
    # --- 1. DATA GENERATION ---
    x = np.linspace(-5, 5, 150) # Reduced resolution slightly for smoother animation
    y = np.linspace(-5, 5, 150)
    X, Y = np.meshgrid(x, y)
    
    Z = 2 * np.exp(-(X**2 + Y**2) / 5) + 0.5 * np.sin(X) * np.cos(Y)
    shift_factor = 0.7 

    fig = plt.figure(figsize=(16, 10))
    plt.subplots_adjust(left=0.15, bottom=0.25, hspace=0.3, wspace=0.2)

    ax1 = fig.add_subplot(2, 2, 1, projection='3d')
    ax2 = fig.add_subplot(2, 2, 2)
    ax3 = fig.add_subplot(2, 2, 3)
    ax4 = fig.add_subplot(2, 2, 4)

    # State Variables
    state = {
        'freq': 2.5,
        'angle': 0,
        'strobe_active': False,
        'blink_on': True
    }

    # UI Elements
    ax_freq = plt.axes([0.3, 0.1, 0.45, 0.03])
    ax_angle = plt.axes([0.3, 0.05, 0.45, 0.03])
    s_freq = Slider(ax_freq, 'Stripe Frequency', 0.5, 10.0, valinit=state['freq'])
    s_angle = Slider(ax_angle, 'Rotation (Deg)', 0, 360, valinit=state['angle'])

    ax_check = plt.axes([0.02, 0.45, 0.12, 0.08], facecolor='#f0f0f0')
    check = CheckButtons(ax_check, ['Strobe Mode'], [False])

    def compute_view(freq, angle_deg):
        angle_rad = np.radians(angle_deg)
        X_rot = X * np.cos(angle_rad) - Y * np.sin(angle_rad)
        pure = 0.5 * (1 + np.sin(freq * X_rot))
        deformed = 0.5 * (1 + np.sin(freq * (X_rot + Z * shift_factor)))
        return pure, deformed

    def draw_scene(frame):
        # Update state from sliders
        state['freq'] = s_freq.val
        state['angle'] = s_angle.val
        state['strobe_active'] = check.get_status()[0]

        # Logic for blinking: 
        # If strobe is OFF, it's always ON. 
        # If strobe is ON, it flips every frame (frame triggers every 500ms).
        if state['strobe_active']:
            state['blink_on'] = not state['blink_on']
        else:
            state['blink_on'] = True

        ax1.clear()
        ax3.clear()
        ax4.clear()

        proj_coords = np.array([-7, 0, 6])
        cam_coords = np.array([7, 0, 6])
        target_center = np.array([0, 0, 1])

        if state['blink_on']:
            pure_p, cam_v = compute_view(state['freq'], state['angle'])
            colors = cm.gray(cam_v)
            ax3.imshow(pure_p, extent=[-5, 5, -5, 5], cmap='gray')
            ax4.imshow(cam_v, extent=[-5, 5, -5, 5], cmap='gray')
            p_color = 'blue'
        else:
            colors = np.full((*X.shape, 4), 0.3) # Darker gray when off
            ax3.imshow(np.zeros_like(X), extent=[-5, 5, -5, 5], cmap='gray', vmin=0, vmax=1)
            ax4.imshow(np.zeros_like(X), extent=[-5, 5, -5, 5], cmap='gray', vmin=0, vmax=1)
            p_color = 'gray'

        # Redraw 3D Surface
        ax1.plot_surface(X, Y, Z, facecolors=colors, shade=state['blink_on'], antialiased=True, alpha=0.8)
        
        # Hardware & Legend
        ax1.plot([proj_coords[0], cam_coords[0]], [0, 0], [proj_coords[2], cam_coords[2]], 'k--', alpha=0.4, label='Baseline')
        ax1.scatter(*proj_coords, color='blue', s=100, marker='s', label='Projector')
        #ax1.scatter(*cam_coords, color='red', s=100, marker='o', label='Camera')
        ax1.quiver(proj_coords[0], proj_coords[1], proj_coords[2], *(target_center - proj_coords), 
                   color=p_color, length=4, normalize=True, arrow_length_ratio=0.2)
        
        # C. Draw Camera and Viewing Direction
        ax1.scatter(*cam_coords, color='red', s=100, marker='o', label='SLS Camera')
        # Vector from camera to target
        c_vec = target_center - cam_coords
        ax1.quiver(cam_coords[0], cam_coords[1], cam_coords[2], 
                   c_vec[0], c_vec[1], c_vec[2], 
                   color='red', length=4, normalize=True, arrow_length_ratio=0.2, label='Optical Axis')
        
        ax1.set_title(f"3D SLS Layout ({'BLINKING' if state['strobe_active'] else 'ALWAYS ON'})")
        ax1.set_zlim(0, 8)
        ax1.legend(loc='upper right', fontsize='x-small')
        ax3.set_title("Projector Pattern")
        ax4.set_title("Camera Capture")

    # Static Ground Truth
    im2 = ax2.imshow(Z, extent=[-5, 5, -5, 5], cmap='viridis')
    ax2.set_title("Ground Truth: Object Depth (Z)")
    fig.colorbar(im2, ax=ax2, fraction=0.046, pad=0.04)

    # Use FuncAnimation to handle the 500ms (0.5s) strobe interval
    ani = animation.FuncAnimation(fig, draw_scene, interval=500, cache_frame_data=False)

    plt.show()

if __name__ == "__main__":
    simulate_strobe_sls()