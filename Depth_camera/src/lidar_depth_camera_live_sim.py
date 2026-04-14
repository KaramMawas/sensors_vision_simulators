"""
===============================================================================
Project: Sensor Vision Simulators
Module: LiDAR and Depth Camera Simulation
File: lidar_depth_camera_live_sim.py

Author: Karam Mawas
Affiliation: Technical University of Braunschweig / Institute of Geodesy and Photogrammetry (IGP)
Email: karam.mawas@gmail.com
GitHub: https://github.com/KaramMawas
Website: https://yourwebsite.com
ORCID: https://orcid.org/0000-0000-0000-0000

Created: 2026-04-14
Last Updated: 2026-04-14

Copyright (c) 2026 Karam Mawas
License: MIT

Description:
LiDAR and depth camera simulation, time-of-flight, point cloud reconstruction, and interactive visualization.
===============================================================================
"""

# Author information and metadata
AUTHOR_NAME = "Karam Mawas"
AUTHOR_AFFILIATION = "Technical University of Braunschweig / Institute of Geodesy and Photogrammetry (IGP)"
AUTHOR_EMAIL = "karam.mawas@gmail.com"
PROJECT_NAME = "Sensor Vision Simulators"
PROJECT_VERSION = "1.0.0"
COPYRIGHT_NOTICE = "© 2026 Karam Mawas. All rights reserved."


import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrowPatch
from matplotlib.widgets import Button
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ============================================================
# Constants
# ============================================================

C = 299_792_458.0  # speed of light [m/s]

# ============================================================
# Helpers
# ============================================================

def normalize(v):
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    return v if n < 1e-12 else v / n


def set_window_title(fig, title):
    try:
        fig.canvas.manager.set_window_title(title)
    except Exception:
        pass


def set_axes_equal_3d(ax):
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    y_range = abs(y_limits[1] - y_limits[0])
    z_range = abs(z_limits[1] - z_limits[0])

    x_middle = np.mean(x_limits)
    y_middle = np.mean(y_limits)
    z_middle = np.mean(z_limits)

    plot_radius = 0.5 * max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])


# ============================================================
# Scene objects
# ============================================================

class Sphere:
    def __init__(self, center, radius, reflectivity=0.8, name="sphere"):
        self.center = np.array(center, dtype=float)
        self.radius = float(radius)
        self.reflectivity = float(reflectivity)
        self.name = name

    def intersect(self, ray_o, ray_d):
        oc = ray_o - self.center
        a = np.dot(ray_d, ray_d)
        b = 2.0 * np.dot(oc, ray_d)
        c = np.dot(oc, oc) - self.radius**2
        disc = b*b - 4*a*c
        if disc < 0:
            return None
        s = np.sqrt(disc)
        t1 = (-b - s) / (2*a)
        t2 = (-b + s) / (2*a)
        ts = [t for t in [t1, t2] if t > 1e-6]
        if not ts:
            return None
        t = min(ts)
        p = ray_o + t * ray_d
        n = normalize(p - self.center)
        return {
            "t": t,
            "point": p,
            "normal": n,
            "reflectivity": self.reflectivity,
            "name": self.name
        }


class Box:
    def __init__(self, min_corner, max_corner, reflectivity=0.6, name="box"):
        self.min_corner = np.array(min_corner, dtype=float)
        self.max_corner = np.array(max_corner, dtype=float)
        self.reflectivity = float(reflectivity)
        self.name = name

    def intersect(self, ray_o, ray_d):
        tmin = -np.inf
        tmax = np.inf
        hit_axis = None
        hit_sign = 0

        for i in range(3):
            if abs(ray_d[i]) < 1e-12:
                if ray_o[i] < self.min_corner[i] or ray_o[i] > self.max_corner[i]:
                    return None
            else:
                t1 = (self.min_corner[i] - ray_o[i]) / ray_d[i]
                t2 = (self.max_corner[i] - ray_o[i]) / ray_d[i]
                tn = min(t1, t2)
                tf = max(t1, t2)

                if tn > tmin:
                    tmin = tn
                    hit_axis = i
                    hit_sign = -1 if t1 > t2 else 1

                tmax = min(tmax, tf)
                if tmin > tmax:
                    return None

        t = tmin if tmin > 1e-6 else tmax
        if t <= 1e-6:
            return None

        p = ray_o + t * ray_d
        n = np.zeros(3)
        if hit_axis is not None:
            n[hit_axis] = hit_sign

        return {
            "t": t,
            "point": p,
            "normal": n,
            "reflectivity": self.reflectivity,
            "name": self.name
        }


class GroundPlaneXZ:
    def __init__(self, y0=-1.0, reflectivity=0.3, name="ground"):
        self.y0 = float(y0)
        self.reflectivity = float(reflectivity)
        self.name = name
        self.normal = np.array([0.0, 1.0, 0.0])

    def intersect(self, ray_o, ray_d):
        denom = ray_d[1]
        if abs(denom) < 1e-12:
            return None
        t = (self.y0 - ray_o[1]) / denom
        if t <= 1e-6:
            return None
        p = ray_o + t * ray_d
        return {
            "t": t,
            "point": p,
            "normal": self.normal,
            "reflectivity": self.reflectivity,
            "name": self.name
        }


# ============================================================
# Sensor rig
# ============================================================

class SensorRig:
    def __init__(self):
        self.camera_center = np.array([-0.16, 0.10, 0.0])
        self.lidar_center = np.array([0.16, 0.18, 0.0])

        self.camera_size = np.array([0.30, 0.14, 0.12])
        self.lidar_size = np.array([0.24, 0.10, 0.10])


# ============================================================
# LiDAR simulator
# ============================================================

class LidarSimulator:
    def __init__(
        self,
        lidar_origin,
        width=100,
        height=70,
        hfov_deg=62,
        vfov_deg=40,
        max_range=15.0,
        depth_noise_std=0.012,
        timing_jitter_std=0.20e-9,
        ambient_level=0.03
    ):
        self.origin = np.array(lidar_origin, dtype=float)
        self.width = width
        self.height = height
        self.hfov = np.deg2rad(hfov_deg)
        self.vfov = np.deg2rad(vfov_deg)
        self.max_range = max_range
        self.depth_noise_std = depth_noise_std
        self.timing_jitter_std = timing_jitter_std
        self.ambient_level = ambient_level

        self.scan_order = [(v, u) for v in range(self.height) for u in range(self.width)]
        self.num_rays = len(self.scan_order)

    def pixel_to_ray(self, u, v):
        x = (u + 0.5) / self.width
        y = (v + 0.5) / self.height
        px = (2*x - 1) * np.tan(self.hfov / 2)
        py = (1 - 2*y) * np.tan(self.vfov / 2)
        pz = 1.0
        return normalize([px, py, pz])

    def trace_single_ray(self, u, v, objects):
        ray_o = self.origin
        ray_d = self.pixel_to_ray(u, v)

        nearest = None
        nearest_t = None

        for obj in objects:
            hit = obj.intersect(ray_o, ray_d)
            if hit is None:
                continue
            if hit["t"] <= self.max_range:
                if nearest_t is None or hit["t"] < nearest_t:
                    nearest_t = hit["t"]
                    nearest = hit

        if nearest is None:
            return {
                "u": u,
                "v": v,
                "ray_o": ray_o,
                "ray_d": ray_d,
                "hit": False,
                "end": ray_o + ray_d * self.max_range,
                "depth": np.nan,
                "tof": np.nan,
                "intensity": self.ambient_level,
                "point": np.array([np.nan, np.nan, np.nan]),
                "name": None
            }

        true_depth = nearest["t"]
        measured_depth = max(0.0, true_depth + np.random.normal(0, self.depth_noise_std))
        measured_tof = max(0.0, 2 * true_depth / C + np.random.normal(0, self.timing_jitter_std))

        n = normalize(nearest["normal"])
        incoming = -normalize(ray_d)
        cosine = max(0.0, np.dot(n, incoming))
        refl = nearest["reflectivity"]
        inten = refl * (0.2 + 0.8 * cosine) / (1 + 0.08 * true_depth**2) + self.ambient_level
        inten = np.clip(inten, 0.0, 1.0)

        return {
            "u": u,
            "v": v,
            "ray_o": ray_o,
            "ray_d": ray_d,
            "hit": True,
            "end": nearest["point"],
            "depth": measured_depth,
            "tof": measured_tof,
            "intensity": inten,
            "point": nearest["point"],
            "name": nearest["name"]
        }


# ============================================================
# Drawing helpers
# ============================================================

def draw_ground(ax, y0=-1.0, xlim=(-4, 4), zlim=(0, 14)):
    xx, zz = np.meshgrid(np.linspace(*xlim, 2), np.linspace(*zlim, 2))
    yy = np.full_like(xx, y0)
    ax.plot_surface(xx, yy, zz, color="lightgray", alpha=0.35, shade=False)


def draw_sphere(ax, center, radius, color="orange", alpha=0.55):
    u = np.linspace(0, 2*np.pi, 28)
    v = np.linspace(0, np.pi, 20)
    x = center[0] + radius * np.outer(np.cos(u), np.sin(v))
    y = center[1] + radius * np.outer(np.sin(u), np.sin(v))
    z = center[2] + radius * np.outer(np.ones_like(u), np.cos(v))
    ax.plot_surface(x, y, z, color=color, alpha=alpha, linewidth=0)


def draw_box(ax, center, size, color="tan", alpha=0.7):
    cx, cy, cz = center
    sx, sy, sz = size / 2.0
    corners = np.array([
        [cx-sx, cy-sy, cz-sz], [cx+sx, cy-sy, cz-sz],
        [cx+sx, cy+sy, cz-sz], [cx-sx, cy+sy, cz-sz],
        [cx-sx, cy-sy, cz+sz], [cx+sx, cy-sy, cz+sz],
        [cx+sx, cy+sy, cz+sz], [cx-sx, cy+sy, cz+sz]
    ])
    faces = [
        [corners[i] for i in [0, 1, 2, 3]],
        [corners[i] for i in [4, 5, 6, 7]],
        [corners[i] for i in [0, 1, 5, 4]],
        [corners[i] for i in [2, 3, 7, 6]],
        [corners[i] for i in [1, 2, 6, 5]],
        [corners[i] for i in [0, 3, 7, 4]],
    ]
    poly = Poly3DCollection(faces, facecolors=color, edgecolors="black", linewidths=0.7, alpha=alpha)
    ax.add_collection3d(poly)


# ============================================================
# Animated application
# ============================================================

class LiveLidarApp:
    def __init__(self):
        self.rig = SensorRig()
        self.objects = self.build_scene()
        self.lidar = LidarSimulator(self.rig.lidar_center)

        self.timer = None
        self.running = False
        self.scan_index = 0
        self.rays_per_tick = 60

        self._init_buffers()
        self._build_all_windows()
        self._connect_close_events()
        self._create_timer()

    def build_scene(self):
        return [
            GroundPlaneXZ(y0=-1.0, reflectivity=0.30, name="ground"),
            Sphere(center=[-1.7, -0.15, 5.5], radius=0.9, reflectivity=0.85, name="sphere_target"),
            Sphere(center=[1.6, 0.10, 8.2], radius=1.0, reflectivity=0.75, name="far_sphere"),
            Box(min_corner=[-0.4, -1.0, 6.2], max_corner=[0.9, 0.8, 7.4], reflectivity=0.55, name="box_obstacle"),
            Box(min_corner=[-2.8, -1.0, 9.5], max_corner=[-1.9, 1.4, 10.8], reflectivity=0.65, name="tall_box"),
        ]

    def _init_buffers(self):
        H, W = self.lidar.height, self.lidar.width
        self.depth = np.full((H, W), np.nan)
        self.tof = np.full((H, W), np.nan)
        self.intensity = np.full((H, W), np.nan)
        self.points = []
        self.active_ray_segments = []
        self.tof_values_ns = []

    def _build_all_windows(self):
        self._build_control_window()
        self._build_hardware_window()
        self._build_scene_window()
        self._build_measurement_window()
        self._build_pointcloud_window()
        self._build_tof_window()
        self._build_hist_window()
        self._build_pipeline_window()

    def _build_control_window(self):
        self.fig_control, ax = plt.subplots(figsize=(6.4, 3.2))
        set_window_title(self.fig_control, "LiDAR Depth Camera Simulation Control Panel")
        ax.axis("off")
        ax.set_title("LiDAR Depth Camera Simulation Control Panel", fontsize=14, fontweight="bold")

        self.status_text = ax.text(
            0.05, 0.76,
            "Status: Ready\nPress Start to begin live scan.",
            transform=ax.transAxes,
            fontsize=11,
            bbox=dict(facecolor="lightyellow", alpha=0.9)
        )

        ax_start = plt.axes([0.10, 0.10, 0.20, 0.18])
        ax_restart = plt.axes([0.39, 0.10, 0.20, 0.18])
        ax_exit = plt.axes([0.68, 0.10, 0.20, 0.18])
        
        # Project info
        ax.text(
            0.05, 0.55,
            f"{PROJECT_NAME} v{PROJECT_VERSION}\n"
            f"Author: {AUTHOR_NAME}\n"
            f"{AUTHOR_AFFILIATION}\n"
            f"{AUTHOR_EMAIL}",
            transform=ax.transAxes,
            fontsize=9,
            va="top",
            bbox=dict(facecolor="white", alpha=0.85)
        )

        self.btn_start = Button(ax_start, "Start")
        self.btn_restart = Button(ax_restart, "Restart")
        self.btn_exit = Button(ax_exit, "Exit")

        self.btn_start.on_clicked(self.start)
        self.btn_restart.on_clicked(self.restart)
        self.btn_exit.on_clicked(self.exit_all)

    def _build_hardware_window(self):
        self.fig_hw, ax = plt.subplots(figsize=(15, 6))
        set_window_title(self.fig_hw, "Hardware Architecture of a LiDAR-Based Depth Camera")
        ax.set_title("Hardware Architecture of a LiDAR-Based Depth Camera", fontsize=15, fontweight="bold")
        ax.set_xlim(0, 16)
        ax.set_ylim(0, 7)
        ax.axis("off")

        boxes = [
            (0.6, 4.9, 2.1, 1.0, "Laser Driver", "#dcecff"),
            (3.0, 4.9, 2.3, 1.0, "LiDAR Emitter\n& Beam Steering", "#dcecff"),
            (5.8, 4.9, 2.0, 1.0, "Transmit Optics", "#dcecff"),
            (8.3, 4.9, 2.0, 1.0, "Scene / Objects", "#fff2c9"),
            (5.8, 2.3, 2.0, 1.0, "Receiver Optics", "#e5f7de"),
            (8.3, 2.3, 2.1, 1.0, "Photodetector", "#e5f7de"),
            (10.9, 2.3, 2.0, 1.0, "TDC / Timing", "#e5f7de"),
            (13.2, 2.3, 2.1, 1.0, "Depth Processor", "#f2e1ff"),
            (10.9, 4.9, 2.0, 1.0, "Camera Sensor", "#ffe0e0"),
            (13.2, 4.9, 2.1, 1.0, "Fusion / Display", "#f2e1ff"),
        ]

        for x, y, w, h, label, color in boxes:
            ax.add_patch(Rectangle((x, y), w, h, facecolor=color, edgecolor="black"))
            ax.text(x+w/2, y+h/2, label, ha="center", va="center", fontsize=10)

        arrows = [
            ((2.7, 5.4), (3.0, 5.4), "drive"),
            ((5.3, 5.4), (5.8, 5.4), "beam"),
            ((7.8, 5.4), (8.3, 5.4), "emit"),
            ((9.3, 4.9), (6.8, 3.3), "reflection"),
            ((7.8, 2.8), (8.3, 2.8), "return"),
            ((10.4, 2.8), (10.9, 2.8), "pulses"),
            ((12.9, 2.8), (13.2, 2.8), "TOF"),
            ((11.9, 3.3), (11.9, 4.9), "sync"),
            ((12.9, 5.4), (13.2, 5.4), "image+depth"),
        ]

        for (x1, y1), (x2, y2), txt in arrows:
            arr = FancyArrowPatch((x1, y1), (x2, y2), arrowstyle="->", mutation_scale=15, linewidth=1.4)
            ax.add_patch(arr)
            ax.text((x1+x2)/2, (y1+y2)/2 + 0.15, txt, ha="center", fontsize=9)

        ax.text(12.0, 1.0, r"$d = \frac{c\,t}{2}$", fontsize=20)
        ax.text(
            0.7, 0.45,
            "Legend / Explanation:\n"
            "- Blue blocks: LiDAR transmit path\n"
            "- Green blocks: LiDAR receive path\n"
            "- Red block: camera imaging path\n"
            "- Purple blocks: processing and fusion",
            fontsize=10, va="bottom"
        )

    def _build_scene_window(self):
        self.fig_scene = plt.figure(figsize=(12, 8))
        set_window_title(self.fig_scene, "3D Sensor Rig and Live LiDAR Scan")
        self.ax_scene = self.fig_scene.add_subplot(111, projection="3d")
        ax = self.ax_scene
        ax.set_title("3D Sensor Rig and Live LiDAR Scan", fontsize=15, fontweight="bold")

        draw_ground(ax, y0=-1.0)

        for obj in self.objects:
            if isinstance(obj, Sphere):
                draw_sphere(ax, obj.center, obj.radius, color="orange", alpha=0.55)
                ax.text(obj.center[0], obj.center[1] + obj.radius + 0.15, obj.center[2], obj.name)
            elif isinstance(obj, Box):
                c = 0.5 * (obj.min_corner + obj.max_corner)
                s = obj.max_corner - obj.min_corner
                draw_box(ax, c, s, color="tan", alpha=0.70)
                ax.text(c[0], c[1] + s[1]/2 + 0.1, c[2], obj.name)

        draw_box(ax, self.rig.camera_center, self.rig.camera_size, color="crimson", alpha=0.95)
        draw_box(ax, self.rig.lidar_center, self.rig.lidar_size, color="royalblue", alpha=0.95)
        ax.text(*self.rig.camera_center, "Camera")
        ax.text(*self.rig.lidar_center, "LiDAR")

        ax.scatter([], [], [], c="crimson", s=60, label="Camera module")
        ax.scatter([], [], [], c="royalblue", s=60, label="LiDAR module")
        ax.scatter([], [], [], c="orange", s=60, label="Sphere targets")
        ax.scatter([], [], [], c="tan", s=60, label="Box targets")
        ax.scatter([], [], [], c="green", s=60, label="Accumulated hit points")
        ax.plot([], [], [], color="deepskyblue", alpha=0.7, label="Current LiDAR rays")

        self.scene_hits = ax.scatter([], [], [], s=6, c="green", alpha=0.65)

        ax.set_xlabel("X (horizontal)")
        ax.set_ylabel("Y (vertical)")
        ax.set_zlabel("Z (forward depth)")
        ax.set_xlim(-4, 4)
        ax.set_ylim(-1.2, 3.0)
        ax.set_zlim(0, 14)
        ax.view_init(elev=20, azim=-65)
        set_axes_equal_3d(ax)
        ax.legend(loc="upper left")

        ax.text2D(
            0.02, 0.02,
            "Explanation:\n"
            "- Blue module: LiDAR hardware\n"
            "- Red module: camera hardware\n"
            "- Blue transient lines: currently emitted rays\n"
            "- Green points: accumulated ray-object intersections",
            transform=ax.transAxes,
            fontsize=9,
            bbox=dict(facecolor="white", alpha=0.8)
        )

    def _build_measurement_window(self):
        self.fig_meas, self.ax_meas = plt.subplots(1, 3, figsize=(17, 5))
        set_window_title(self.fig_meas, "Live Reconstruction of Depth, TOF, and Intensity Maps")
        self.fig_meas.suptitle("Live Reconstruction of Depth, TOF, and Intensity Maps", fontsize=15, fontweight="bold")

        self.im_depth = self.ax_meas[0].imshow(self.depth, cmap="inferno", vmin=0, vmax=15)
        self.ax_meas[0].set_title("Depth Map")
        self.ax_meas[0].set_xlabel("Pixel u")
        self.ax_meas[0].set_ylabel("Pixel v")
        self.ax_meas[0].text(0.02, -0.22, "Measured depth in meters.", transform=self.ax_meas[0].transAxes, fontsize=9)
        self.fig_meas.colorbar(self.im_depth, ax=self.ax_meas[0], label="meters")

        self.im_tof = self.ax_meas[1].imshow(self.tof * 1e9, cmap="plasma", vmin=0, vmax=110)
        self.ax_meas[1].set_title("Measured Time-of-Flight Map")
        self.ax_meas[1].set_xlabel("Pixel u")
        self.ax_meas[1].set_ylabel("Pixel v")
        self.ax_meas[1].text(0.02, -0.22, "Round-trip light travel time in ns.", transform=self.ax_meas[1].transAxes, fontsize=9)
        self.fig_meas.colorbar(self.im_tof, ax=self.ax_meas[1], label="ns")

        self.im_int = self.ax_meas[2].imshow(self.intensity, cmap="gray", vmin=0, vmax=1)
        self.ax_meas[2].set_title("Return Intensity / Reflectivity Map")
        self.ax_meas[2].set_xlabel("Pixel u")
        self.ax_meas[2].set_ylabel("Pixel v")
        self.ax_meas[2].text(0.02, -0.22, "Received signal strength.", transform=self.ax_meas[2].transAxes, fontsize=9)
        self.fig_meas.colorbar(self.im_int, ax=self.ax_meas[2], label="relative intensity")

    def _build_pointcloud_window(self):
        self.fig_pc = plt.figure(figsize=(11, 8))
        set_window_title(self.fig_pc, "Live Formation of the LiDAR Point Cloud")
        self.ax_pc = self.fig_pc.add_subplot(111, projection="3d")
        ax = self.ax_pc
        ax.set_title("Live Formation of the LiDAR Point Cloud", fontsize=15, fontweight="bold")

        draw_ground(ax, y0=-1.0)
        draw_box(ax, self.rig.camera_center, self.rig.camera_size, color="crimson", alpha=0.9)
        draw_box(ax, self.rig.lidar_center, self.rig.lidar_size, color="royalblue", alpha=0.9)

        self.pc_scatter = ax.scatter([], [], [], s=4, c=[], cmap="viridis", alpha=0.85, label="Point cloud")
        ax.scatter([], [], [], c="crimson", s=60, label="Camera module")
        ax.scatter([], [], [], c="royalblue", s=60, label="LiDAR module")
        ax.scatter([], [], [], c="lightgray", s=60, label="Ground plane")

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        ax.set_xlim(-4, 4)
        ax.set_ylim(-1.2, 3.0)
        ax.set_zlim(0, 14)
        ax.view_init(elev=20, azim=-65)
        set_axes_equal_3d(ax)
        ax.legend(loc="upper left")

        ax.text2D(
            0.02, 0.02,
            "Explanation:\n"
            "- Each visible point is a successful LiDAR return.\n"
            "- As scanning progresses, the 3D structure becomes denser.",
            transform=ax.transAxes,
            fontsize=9,
            bbox=dict(facecolor="white", alpha=0.8)
        )

    def _build_tof_window(self):
        self.fig_tof, ax = plt.subplots(figsize=(12, 4))
        set_window_title(self.fig_tof, "Time-of-Flight Principle for Depth Estimation")
        ax.set_title("Time-of-Flight Principle for Depth Estimation", fontsize=15, fontweight="bold")
        ax.set_xlim(0, 12)
        ax.set_ylim(-1.6, 1.6)
        ax.axis("off")

        ax.plot([1, 11], [0, 0], color="black", linewidth=1.3, label="Time axis")
        ax.plot([2, 2], [0, 1.0], color="blue", linewidth=4, label="Emitted pulse")
        ax.plot([8, 8], [0, 1.0], color="green", linewidth=4, label="Received pulse")
        ax.annotate("", xy=(2, -0.5), xytext=(8, -0.5),
                    arrowprops=dict(arrowstyle="<->", linewidth=1.5))
        self.tof_text = ax.text(5, -0.8, r"$t \approx 0.00\,\mathrm{ns}$", ha="center")
        self.depth_text = ax.text(5, -1.2, r"$d=\frac{c\,t}{2}\approx 0.00\,\mathrm{m}$", ha="center", fontsize=14)
        ax.legend(loc="upper left")

        ax.text(
            0.02, 0.02,
            "Explanation:\n"
            "- Blue marker: pulse emission\n"
            "- Green marker: return detection\n"
            "- The separation in time determines distance",
            transform=ax.transAxes,
            fontsize=9,
            bbox=dict(facecolor="white", alpha=0.8)
        )

    def _build_hist_window(self):
        self.fig_hist, self.ax_hist = plt.subplots(figsize=(10, 5))
        set_window_title(self.fig_hist, "Live Distribution of Measured Return Times")
        self.ax_hist.set_title("Live Distribution of Measured Return Times", fontsize=15, fontweight="bold")
        self.ax_hist.set_xlabel("Time-of-flight [ns]")
        self.ax_hist.set_ylabel("Number of pixels")
        self.ax_hist.grid(True, alpha=0.3)
        self.ax_hist.text(
            0.02, 0.95,
            "Explanation:\nThe histogram grows as more pixels are scanned.",
            transform=self.ax_hist.transAxes,
            fontsize=9,
            va="top",
            bbox=dict(facecolor="white", alpha=0.8)
        )

    def _build_pipeline_window(self):
        self.fig_pipe, ax = plt.subplots(figsize=(15, 4))
        set_window_title(self.fig_pipe, "Software Processing Pipeline from Photon Return to 3D Geometry")
        ax.set_title("Software Processing Pipeline from Photon Return to 3D Geometry", fontsize=15, fontweight="bold")
        ax.set_xlim(0, 15)
        ax.set_ylim(0, 3)
        ax.axis("off")

        steps = [
            (0.5, 1.0, 2.0, 1.0, "1. Emit pulse"),
            (2.9, 1.0, 2.0, 1.0, "2. Scan scene"),
            (5.3, 1.0, 2.0, 1.0, "3. Detect return"),
            (7.7, 1.0, 2.0, 1.0, "4. Compute TOF"),
            (10.1, 1.0, 2.0, 1.0, "5. Build maps"),
            (12.5, 1.0, 2.0, 1.0, "6. Reconstruct 3D"),
        ]
        for x, y, w, h, txt in steps:
            ax.add_patch(Rectangle((x, y), w, h, facecolor="#e8f1ff", edgecolor="black"))
            ax.text(x+w/2, y+h/2, txt, ha="center", va="center")

        for i in range(len(steps)-1):
            x1 = steps[i][0] + steps[i][2]
            x2 = steps[i+1][0]
            arr = FancyArrowPatch((x1, 1.5), (x2, 1.5), arrowstyle="->", mutation_scale=14, linewidth=1.4)
            ax.add_patch(arr)

        ax.text(
            0.02, 0.15,
            "Explanation: This diagram summarizes the processing chain used during the live animation.",
            transform=ax.transAxes,
            fontsize=10
        )


    def _connect_close_events(self):
        figs = [
            self.fig_control, self.fig_hw, self.fig_scene, self.fig_meas,
            self.fig_pc, self.fig_tof, self.fig_hist, self.fig_pipe
        ]
        for fig in figs:
            fig.canvas.mpl_connect("close_event", self._on_any_window_closed)

    def _on_any_window_closed(self, event):
        self.running = False
        try:
            if self.timer is not None:
                self.timer.stop()
        except Exception:
            pass

    def _create_timer(self):
        self.timer = self.fig_control.canvas.new_timer(interval=80)
        self.timer.add_callback(self.update_animation)

    def _reset_views(self):
        self.scan_index = 0
        self._init_buffers()

        # clear scene dynamic rays
        for line in getattr(self, "current_ray_lines", []):
            try:
                line.remove()
            except Exception:
                pass
        self.current_ray_lines = []

        # clear hit points
        self.scene_hits._offsets3d = ([], [], [])

        # clear maps
        self.im_depth.set_data(self.depth)
        self.im_tof.set_data(self.tof * 1e9)
        self.im_int.set_data(np.nan_to_num(self.intensity, nan=0.0))

        # clear point cloud
        self.pc_scatter._offsets3d = ([], [], [])
        self.pc_scatter.set_array(np.array([]))

        # clear histogram
        self.ax_hist.cla()
        self.ax_hist.set_title("Live Distribution of Measured Return Times", fontsize=15, fontweight="bold")
        self.ax_hist.set_xlabel("Time-of-flight [ns]")
        self.ax_hist.set_ylabel("Number of pixels")
        self.ax_hist.grid(True, alpha=0.3)
        self.ax_hist.text(
            0.02, 0.95,
            "Explanation:\nThe histogram grows as more pixels are scanned.",
            transform=self.ax_hist.transAxes,
            fontsize=9,
            va="top",
            bbox=dict(facecolor="white", alpha=0.8)
        )

        self.tof_text.set_text(r"$t \approx 0.00\,\mathrm{ns}$")
        self.depth_text.set_text(r"$d=\frac{c\,t}{2}\approx 0.00\,\mathrm{m}$")

        self._redraw_all()

    def _redraw_all(self):
        for fig in [
            self.fig_control, self.fig_hw, self.fig_scene, self.fig_meas,
            self.fig_pc, self.fig_tof, self.fig_hist, self.fig_pipe
        ]:
            try:
                fig.canvas.draw_idle()
            except Exception:
                pass

    def start(self, event=None):
        if self.running:
            return
        self.running = True
        self.status_text.set_text("Status: Live scan running...")
        self.timer.start()

    def restart(self, event=None):
        self.running = False
        self.timer.stop()
        np.random.seed(None)
        self.status_text.set_text("Status: Restarting live scan...")
        self._reset_views()
        self.running = True
        self.status_text.set_text("Status: Live scan restarted.")
        self.timer.start()

    def exit_all(self, event=None):
        self.running = False
        try:
            self.timer.stop()
        except Exception:
            pass
        plt.close("all")

    def update_animation(self):
        if not self.running:
            return

        if self.scan_index >= self.lidar.num_rays:
            self.running = False
            self.status_text.set_text("Status: Scan completed.")
            return

        # remove previous transient ray lines
        for line in getattr(self, "current_ray_lines", []):
            try:
                line.remove()
            except Exception:
                pass
        self.current_ray_lines = []

        batch_hits = []

        for _ in range(self.rays_per_tick):
            if self.scan_index >= self.lidar.num_rays:
                break

            v, u = self.lidar.scan_order[self.scan_index]
            res = self.lidar.trace_single_ray(u, v, self.objects)

            self.depth[v, u] = res["depth"]
            self.tof[v, u] = res["tof"]
            self.intensity[v, u] = res["intensity"]

            line, = self.ax_scene.plot(
                [res["ray_o"][0], res["end"][0]],
                [res["ray_o"][1], res["end"][1]],
                [res["ray_o"][2], res["end"][2]],
                color="deepskyblue" if res["hit"] else "gray",
                alpha=0.7,
                linewidth=1.2
            )
            self.current_ray_lines.append(line)

            if res["hit"]:
                self.points.append(res["point"])
                batch_hits.append(res["point"])
                self.tof_values_ns.append(res["tof"] * 1e9)

                # update example TOF text from latest valid sample
                self.tof_text.set_text(rf"$t \approx {res['tof'] * 1e9:.2f}\,\mathrm{{ns}}$")
                self.depth_text.set_text(rf"$d=\frac{{c\,t}}{{2}}\approx {res['depth']:.2f}\,\mathrm{{m}}$")

            self.scan_index += 1

        # update accumulated hit points in 3D scene
        if len(self.points) > 0:
            pts = np.array(self.points)
            self.scene_hits._offsets3d = (pts[:, 0], pts[:, 1], pts[:, 2])

            # update point cloud
            self.pc_scatter._offsets3d = (pts[:, 0], pts[:, 1], pts[:, 2])
            self.pc_scatter.set_array(pts[:, 2])

        # update images
        self.im_depth.set_data(self.depth)
        self.im_tof.set_data(self.tof * 1e9)
        self.im_int.set_data(np.nan_to_num(self.intensity, nan=0.0))

        # update histogram
        self.ax_hist.cla()
        self.ax_hist.set_title("Live Distribution of Measured Return Times", fontsize=15, fontweight="bold")
        self.ax_hist.set_xlabel("Time-of-flight [ns]")
        self.ax_hist.set_ylabel("Number of pixels")
        self.ax_hist.grid(True, alpha=0.3)
        if len(self.tof_values_ns) > 0:
            self.ax_hist.hist(self.tof_values_ns, bins=40, color="mediumpurple", edgecolor="black",
                              alpha=0.8, label="TOF samples")
            self.ax_hist.legend(loc="upper right")
        self.ax_hist.text(
            0.02, 0.95,
            f"Explanation:\nScanned pixels: {self.scan_index}/{self.lidar.num_rays}",
            transform=self.ax_hist.transAxes,
            fontsize=9,
            va="top",
            bbox=dict(facecolor="white", alpha=0.8)
        )

        self.status_text.set_text(
            f"Status: Live scan running...\n"
            f"Progress: {self.scan_index}/{self.lidar.num_rays} rays"
        )

        self._redraw_all()


# ============================================================
# Main
# ============================================================

def main():
    app = LiveLidarApp()
    plt.show()


if __name__ == "__main__":
    main()
