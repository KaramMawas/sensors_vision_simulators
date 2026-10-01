import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Button
from matplotlib.lines import Line2D

# --- 1. Data Generation ---
pts_house = np.array([
    [0,0,0], [2,0,0], [2,2,0], [0,2,0], [0,0,0],
    [0,0,2], [2,0,2], [2,2,2], [0,2,2], [0,0,2], [1,1,3]
])

def get_path():
    angles = np.linspace(-0.9, 0.9, 12)
    return np.array([[7*np.sin(a)+1, 7*np.cos(a)+1, 2.0] for a in angles])

cams_gt = get_path()

# --- 2. Setup Figure ---
fig = plt.figure(figsize=(16, 9))
ax3d = fig.add_subplot(121, projection='3d')
ax2d = fig.add_subplot(122, projection='3d') # 3D Projection for the Pyramid
plt.subplots_adjust(bottom=0.25, top=0.9)

class SfMEngine:
    def __init__(self, fig):
        self.fig = fig
        self.message_handle = self.fig.text(0.5, 0.18, "", ha='center', fontsize=12, fontweight='bold',
                                           bbox=dict(facecolor='white', edgecolor='none', alpha=0.9, pad=5))
        self.reset()

    def reset(self, event=None):
        self.idx = 1
        self.solving = True
        self.failed = False
        self.iter = 0
        self.aligned_cams = [cams_gt[0]]
        self.point_weights = np.zeros(len(pts_house))
        self.point_weights[:5] = 1 
        self.current_error = 1.0
        self.bad_indices = [5, 9] 
        self.update_status("INITIALIZING MULTI-SCALE PYRAMID...")

    def update_status(self, text):
        color = 'red' if ("ERROR" in text or "REJECTED" in text) else 'darkgreen'
        self.message_handle.set_text(text)
        self.message_handle.set_color(color)

    def step(self, event=None):
        if self.failed or not self.solving:
            if self.idx < len(cams_gt) - 1:
                self.idx += 1
                self.solving = True
                self.failed = False
                self.iter = 0
                self.current_error = 1.0
                self.update_status(f"PYRAMID SEARCH: VIEW {self.idx}...")
        else:
            if self.idx in self.bad_indices:
                self.failed = True
                self.solving = False
                self.update_status(f"ERROR: RESIDUALS EXCEED SCALE TOLERANCE")
            else:
                self.solving = False
                self.aligned_cams.append(cams_gt[self.idx])
                # Increase weights for points seen across images
                self.point_weights += np.random.choice([0, 1], size=len(pts_house), p=[0.4, 0.6])
                self.update_status(f"SUCCESS: POSE OPTIMIZED AT ALL SCALES")

engine = SfMEngine(fig)

def update(frame):
    ax3d.clear()
    ax2d.clear()
    engine.iter += 1
    idx = engine.idx
    gt_pos = cams_gt[idx]
    
    # --- 3D VIEW: GLOBAL RECONSTRUCTION ---
    ax3d.set_title("GLOBAL 3D STRUCTURE", fontweight='bold')
    ax3d.set_xlim(-5, 5); ax3d.set_ylim(-2, 10); ax3d.set_zlim(-1, 5)
    
    for i, p in enumerate(pts_house):
        w = engine.point_weights[i]
        # Transition from Cyan to Deep Blue
        color_val = max(0, 1 - (w / 5.0))
        color = (0.2, 0.5 * color_val, 1.0 - (0.5 * color_val))
        ax3d.scatter(p[0], p[1], p[2], color=color, s=40 + w*15, edgecolors='black')

    for c in engine.aligned_cams:
        ax3d.scatter(c[0], c[1], c[2], color='navy', marker='^', s=80)

    if engine.solving:
        noise = 0.6 if idx in engine.bad_indices else 0.05
        engine.current_error = max(noise, engine.current_error * 0.94)
        jitter = (np.random.rand(3)-0.5) * engine.current_error * 4
        curr_p = gt_pos + jitter
        ax3d.scatter(curr_p[0], curr_p[1], curr_p[2], color='orange', marker='^', s=150)
        # 3D Rays
        for p in pts_house[::2]:
            ax3d.plot([curr_p[0], p[0]], [curr_p[1], p[1]], [curr_p[2], p[2]], color='orange', alpha=0.1)
    
    elif engine.failed:
        # ANIMATED RED X EFFECT
        # Pulse size based on iteration
        pulse = 1.0 + 0.2 * np.sin(engine.iter * 0.5)
        ax3d.scatter(gt_pos[0], gt_pos[1], gt_pos[2], color='red', marker='x', s=300 * pulse, lw=4)
        ax3d.text(gt_pos[0], gt_pos[1], gt_pos[2]+1.2, "DROPPED", color='red', 
                  ha='center', fontweight='bold', fontsize=12)

    # --- 2D VIEW: THE SCALE-SPACE PYRAMID ---
    ax2d.set_title("IMAGE PYRAMID (SCALE-SPACE)", fontweight='bold')
    ax2d.set_zlim(0, 3); ax2d.set_xlim(-1, 1); ax2d.set_ylim(-1, 1)
    
    # Draw Pyramid Layers with Resolution Grids
    scales = [0, 1, 2] 
    for s in scales:
        size = 1.0 - (s * 0.2)
        grid_density = 12 // (2**s) 
        g_coords = np.linspace(-size, size, grid_density)
        
        # Grid lines for this resolution
        for g in g_coords:
            ax2d.plot([g, g], [-size, size], [s, s], color='black', alpha=0.1)
            ax2d.plot([-size, size], [g, g], [s, s], color='black', alpha=0.1)
        
        # Plane Border
        ax2d.plot([-size, size, size, -size, -size], [-size, -size, size, size, -size], s, color='black', alpha=0.4, lw=2)
        ax2d.text(size, size, s, f"L{s}: 1/{2**s}x Res", fontsize=9, fontweight='bold')

    if engine.solving:
        # Active Search jumps through levels
        current_s = (engine.iter // 10) % 3
        ly = np.sin(engine.iter * 0.1) * 0.3
        
        # Epipolar Line at current scale
        ax2d.plot([-0.8, 0.8], [ly, ly], [current_s, current_s], color='red', linestyle='--', lw=2)
        
        # Feature Search Markers
        for i in range(10):
            s_feat = i % 3
            bx, by = np.sin(i*0.7) * 0.4, np.cos(i*0.7) * 0.4
            
            if s_feat == current_s:
                err = engine.current_error * 0.3
                px, py = bx + np.random.normal(0, err), by + np.random.normal(0, err)
                ax2d.scatter(px, py, s_feat, marker='+', color='red', s=50)
                # Residual Vector
                ax2d.plot([bx, px], [by, py], [s_feat, s_feat], 'r-', alpha=0.3)
            else:
                ax2d.scatter(bx, by, s_feat, marker='o', color='gray', s=10, alpha=0.3)

    # --- LEGENDS ---
    legend_3d = [
        Line2D([0], [0], marker='^', color='w', markerfacecolor='navy', label='Aligned Cam'),
        Line2D([0], [0], marker='^', color='w', markerfacecolor='orange', label='Solving Cam'),
        Line2D([0], [0], marker='o', color='w', markerfacecolor=(0.1, 0.4, 0.9), label='Verified Point'),
        Line2D([0], [0], color='orange', alpha=0.3, label='Projection Ray')
    ]
    ax3d.legend(handles=legend_3d, loc='upper left', fontsize=8)

    legend_2d = [
        Line2D([0], [0], color='red', linestyle='--', label='Epipolar Line'),
        Line2D([0], [0], color='red', alpha=0.3, label='Residual Vector'),
        Line2D([0], [0], marker='+', color='red', linestyle='None', label='Feature Search')
    ]
    ax2d.legend(handles=legend_2d, loc='upper left', fontsize=8)

# --- Buttons ---
ax_next = plt.axes([0.3, 0.05, 0.2, 0.06])
ax_reset = plt.axes([0.55, 0.05, 0.2, 0.06])
btn_next = Button(ax_next, 'OPTIMIZE POSE')
btn_reset = Button(ax_reset, 'RESTART SFM')

btn_next.on_clicked(engine.step)
btn_reset.on_clicked(engine.reset)

ani = FuncAnimation(fig, update, interval=50, cache_frame_data=False)
plt.show()