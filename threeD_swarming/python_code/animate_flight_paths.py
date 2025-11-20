import os
from glob import glob
from datetime import datetime

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.animation import FFMpegWriter, FuncAnimation


def animate_flight_paths(speed_up=1.0, data=None):
    """
    Render and export a swarm flight video.

    Parameters
    ----------
    speed_up : float
        Playback speed factor (2 => twice as fast as real time).
    data : dict
        Either:
          - {"positions": (n,3,steps) array, "time": (steps,) array}
        OR
          - {"csv_dir": directory with drone_*.csv files}
        Optional keys:
          - "goal": (3,) goal vector
          - "obstacles": list of obstacle dicts
          - "obs_style": dict with keys "color", "alpha", "edgecolor"
          - "output_file": output video path (.mp4)
    """
    if data is None:
        data = {}

    # Validate speed_up
    if not (isinstance(speed_up, (int, float)) and speed_up > 0):
        raise ValueError("speed_up must be a positive real scalar.")

    # ------------------------------------------------------------
    # Load positions/time
    # ------------------------------------------------------------
    if "positions" in data and "time" in data:
        positions = np.asarray(data["positions"], dtype=float)
        time_vec = np.asarray(data["time"], dtype=float).ravel()

        if positions.ndim != 3 or positions.shape[1] != 3:
            raise ValueError("positions must be an (n, 3, steps) array.")
        if time_vec.ndim != 1:
            raise ValueError("time must be a 1D vector.")
        if positions.shape[2] != time_vec.size:
            raise ValueError(
                "positions must be n x 3 x steps with steps matching len(time)."
            )

        n = positions.shape[0]
        steps = time_vec.size
    else:
        # Load from CSV directory
        csv_dir = data.get("csv_dir", os.path.join(os.getcwd(), "swarm_output"))
        positions, time_vec = _load_swarm_csv(csv_dir)
        n = positions.shape[0]
        steps = time_vec.size

    # ------------------------------------------------------------
    # Optional goal
    # ------------------------------------------------------------
    if "goal" in data and data["goal"] is not None:
        goal = np.asarray(data["goal"], dtype=float).ravel()
        if goal.size != 3:
            raise ValueError("goal must be a 3-element vector.")
    else:
        goal = None

    # ------------------------------------------------------------
    # Optional obstacles + style
    # ------------------------------------------------------------
    obstacles = data.get("obstacles", []) or []
    style = {
        "color": "b",
        "alpha": 0.35,
        "edgecolor": "none",
    }
    obs_style = data.get("obs_style", None)
    if obs_style is not None:
        for k, v in obs_style.items():
            style[k] = v

    # ------------------------------------------------------------
    # Output path
    # ------------------------------------------------------------
    if "output_file" in data and data["output_file"]:
        output_path = data["output_file"]
    else:
        has_obs = bool(obstacles)
        tag = "obs" if has_obs else "no_obs"
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        output_path = os.path.join(
            os.getcwd(), f"swarm_animation_{tag}_{ts}.mp4"
        )

    # ------------------------------------------------------------
    # Frame pacing
    # ------------------------------------------------------------
    dt = np.median(np.diff(time_vec))
    frame_rate_desired = speed_up / dt
    max_fps = 60.0
    if frame_rate_desired <= max_fps:
        frame_stride = 1
        frame_rate = frame_rate_desired
    else:
        frame_stride = int(np.ceil(frame_rate_desired / max_fps))
        frame_rate = frame_rate_desired / frame_stride

    frame_indices = np.arange(0, steps, frame_stride, dtype=int)

    # ------------------------------------------------------------
    # Data ranges
    # ------------------------------------------------------------
    xdata = positions[:, 0, :]  # (n, steps)
    ydata = positions[:, 1, :]
    zdata = positions[:, 2, :]

    xrange = np.array([np.min(xdata), np.max(xdata)], dtype=float)
    yrange = np.array([np.min(ydata), np.max(ydata)], dtype=float)
    zrange = np.array([np.min(zdata), np.max(zdata)], dtype=float)

    # Expand ranges to include obstacles
    if obstacles:
        ox, oy, oz = _obstacle_bounds(obstacles)
        xrange = np.array([min(xrange[0], ox[0]), max(xrange[1], ox[1])])
        yrange = np.array([min(yrange[0], oy[0]), max(yrange[1], oy[1])])
        zrange = np.array([min(zrange[0], oz[0]), max(zrange[1], oz[1])])

    margin = 0.05
    xrange = _expand_range(xrange, margin)
    yrange = _expand_range(yrange, margin)
    zrange = _expand_range(zrange, margin)

    # ------------------------------------------------------------
    # Figure setup
    # ------------------------------------------------------------
    colors = plt.cm.tab10(np.linspace(0, 1, max(n, 10)))[:n]  # similar to lines(n)

    fig = plt.figure("Swarm Animation", figsize=(8, 6))
    fig.patch.set_facecolor("white")
    ax = fig.add_subplot(111, projection="3d")
    ax.grid(True)
    ax.set_box_aspect(
        [
            xrange[1] - xrange[0],
            yrange[1] - yrange[0],
            zrange[1] - zrange[0],
        ]
    )  # axis equal-ish
    ax.view_init(elev=25, azim=35)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_xlim(xrange)
    ax.set_ylim(yrange)
    ax.set_zlim(zrange)
    ax.set_title("Swarm trajectories")

    # Floor (simple plane at z_min)
    Xplane, Yplane = np.meshgrid(
        [xrange[0], xrange[1]], [yrange[0], yrange[1]]
    )
    Zplane = np.full_like(Xplane, zrange[0])
    ax.plot_surface(
        Xplane,
        Yplane,
        Zplane,
        facecolor=(0.85, 0.85, 0.85),
        edgecolor="none",
        alpha=0.6,
    )

    # Obstacles (draw once; static)
    if obstacles:
        _draw_obstacles(ax, obstacles, style)

    # Paths and markers
    traj = []
    markers = []
    for ii in range(n):
        line, = ax.plot(
            xdata[ii, 0:1],
            ydata[ii, 0:1],
            zdata[ii, 0:1],
            color=colors[ii],
        )
        marker, = ax.plot(
            [xdata[ii, 0]],
            [ydata[ii, 0]],
            [zdata[ii, 0]],
            "o",
            markerfacecolor=colors[ii],
            markeredgecolor=colors[ii],
        )
        traj.append(line)
        markers.append(marker)

    if goal is not None:
        ax.plot(
            [goal[0]],
            [goal[1]],
            [goal[2]],
            "s",
            color="k",
            markerfacecolor="y",
            markersize=8,
        )

    time_text = ax.text2D(
        0.02, 0.95, "", transform=ax.transAxes, fontsize=12
    )

    # ------------------------------------------------------------
    # Video writer
    # ------------------------------------------------------------
    writer = FFMpegWriter(fps=frame_rate)
    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    with writer.saving(fig, output_path, dpi=100):
        for k in frame_indices:
            for ii in range(n):
                traj[ii].set_data(xdata[ii, : k + 1], ydata[ii, : k + 1])
                traj[ii].set_3d_properties(zdata[ii, : k + 1])

                markers[ii].set_data([xdata[ii, k]], [ydata[ii, k]])
                markers[ii].set_3d_properties([zdata[ii, k]])

            time_text.set_text(f"t = {time_vec[k]:.2f} s")
            plt.draw()
            writer.grab_frame()

    print(
        f"Saved animation to {output_path} "
        f"(frames: {len(frame_indices)}, fps: {frame_rate:.2f})"
    )


# =====================================================================
# Helper functions (Python equivalents of MATLAB subfunctions)
# =====================================================================

def _draw_obstacles(ax, obstacles, style):
    col = style.get("color", "b")
    alp = style.get("alpha", 0.35)
    edg = style.get("edgecolor", "none")

    for obs in obstacles:
        otype = obs.get("type", "cylinder").lower()
        if otype == "cylinder":
            cx, cy = obs["xy"]
            R = obs["radius"]
            z0 = obs["zmin"]
            z1 = obs["zmax"]

            th = np.linspace(0, 2 * np.pi, 72)
            z_vals = np.array([z0, z1])
            TH, ZZ = np.meshgrid(th, z_vals)
            XX = cx + R * np.cos(TH)
            YY = cy + R * np.sin(TH)

            # Lateral surface
            ax.plot_surface(
                XX,
                YY,
                ZZ,
                facecolor=col,
                edgecolor=edg if edg != "none" else "none",
                alpha=alp,
            )

            # Caps
            xc = cx + R * np.cos(th)
            yc = cy + R * np.sin(th)
            ax.add_collection3d(
                Poly3DCollection(
                    [list(zip(xc, yc, np.full_like(xc, z0)))],
                    facecolors=col,
                    edgecolors=edg if edg != "none" else "none",
                    alpha=alp,
                )
            )
            ax.add_collection3d(
                Poly3DCollection(
                    [list(zip(xc, yc, np.full_like(xc, z1)))],
                    facecolors=col,
                    edgecolors=edg if edg != "none" else "none",
                    alpha=alp,
                )
            )

        elif otype == "wall":
            frame = _wall_geometry(obs)
            verts = _wall_vertices(frame)
            faces = np.array(
                [
                    [0, 1, 2, 3],
                    [4, 5, 6, 7],
                    [0, 1, 5, 4],
                    [1, 2, 6, 5],
                    [2, 3, 7, 6],
                    [3, 0, 4, 7],
                ]
            )
            poly3d = [
                [verts[idx] for idx in face] for face in faces
            ]
            ax.add_collection3d(
                Poly3DCollection(
                    poly3d,
                    facecolors=col,
                    edgecolors=edg if edg != "none" else "none",
                    alpha=alp,
                )
            )

        # Extend with spheres/boxes if needed


def _obstacle_bounds(obstacles):
    """
    Compute bounding box [min,max] for obstacles.
    Returns (xr, yr, zr) each as [min, max].
    """
    xmin = np.inf
    xmax = -np.inf
    ymin = np.inf
    ymax = -np.inf
    zmin = np.inf
    zmax = -np.inf

    for obs in obstacles:
        otype = obs.get("type", "cylinder").lower()
        if otype == "cylinder":
            cx, cy = obs["xy"]
            R = obs["radius"]
            xmin = min(xmin, cx - R)
            xmax = max(xmax, cx + R)
            ymin = min(ymin, cy - R)
            ymax = max(ymax, cy + R)
            zmin = min(zmin, obs["zmin"])
            zmax = max(zmax, obs["zmax"])

        elif otype == "wall":
            frame = _wall_geometry(obs)
            pts = _wall_vertices(frame)
            xmin = min(xmin, np.min(pts[:, 0]))
            xmax = max(xmax, np.max(pts[:, 0]))
            ymin = min(ymin, np.min(pts[:, 1]))
            ymax = max(ymax, np.max(pts[:, 1]))
            zmin = min(zmin, np.min(pts[:, 2]))
            zmax = max(zmax, np.max(pts[:, 2]))

    if np.isinf(xmin):
        # no obstacles
        xr = np.array([0.0, 0.0])
        yr = np.array([0.0, 0.0])
        zr = np.array([0.0, 0.0])
    else:
        xr = np.array([xmin, xmax], dtype=float)
        yr = np.array([ymin, ymax], dtype=float)
        zr = np.array([zmin, zmax], dtype=float)
    return xr, yr, zr


def _load_swarm_csv(csv_dir):
    """
    Load swarm CSV files (drone_*.csv) into (positions, time_vec).

    positions: (n, 3, steps)
    time_vec:  (steps,)
    """
    pattern = os.path.join(csv_dir, "drone_*.csv")
    files = sorted(glob(pattern))
    if not files:
        raise FileNotFoundError(f"No drone_*.csv files found in {csv_dir!r}")

    positions = None
    time_vec = None
    for i, fpath in enumerate(files):
        tbl = pd.read_csv(fpath)
        t_s = tbl["t_s"].to_numpy()
        x = tbl["x_m"].to_numpy()
        y = tbl["y_m"].to_numpy()
        z = tbl["z_m"].to_numpy()

        if i == 0:
            time_vec = t_s
            steps = time_vec.size
            positions = np.zeros((len(files), 3, steps), dtype=float)
        else:
            if tbl.shape[0] != steps or np.any(np.abs(t_s - time_vec) > 1e-6):
                raise ValueError(
                    f"Time vector mismatch in file {os.path.basename(fpath)}"
                )

        positions[i, 0, :] = x
        positions[i, 1, :] = y
        positions[i, 2, :] = z

    return positions, time_vec


def _expand_range(rng_in, margin):
    """
    Expand a [min, max] range by a given fractional margin.
    """
    rng_in = np.asarray(rng_in, dtype=float)
    span = rng_in[1] - rng_in[0]
    if span <= 0:
        span = max(abs(rng_in[0]), abs(rng_in[1]))
        if span == 0:
            span = 1.0
        return np.array(
            [rng_in[0] - 0.5 * span, rng_in[0] + 0.5 * span], dtype=float
        )
    else:
        pad = span * margin
        return np.array(
            [rng_in[0] - pad, rng_in[1] + pad],
            dtype=float,
        )


def _wall_geometry(obs):
    """
    Derive orthonormal frame and dimensions for a rectangular wall obstacle.

    obs keys expected:
      center, normal, width, height, (optional) thickness, up
    """
    center = np.asarray(obs["center"], dtype=float).ravel()
    n = np.asarray(obs["normal"], dtype=float).ravel()
    if np.linalg.norm(n) < 1e-8:
        raise ValueError("Wall obstacle normal must be non-zero.")
    normal = n / np.linalg.norm(n)

    if "up" in obs and obs["up"] is not None and len(obs["up"]) != 0:
        up_vec = np.asarray(obs["up"], dtype=float).ravel()
    else:
        up_vec = _pick_orthogonal_vec(normal)

    # remove normal component
    up_vec = up_vec - np.dot(up_vec, normal) * normal
    if np.linalg.norm(up_vec) < 1e-8:
        up_vec = _pick_orthogonal_vec(normal)
    up_vec = up_vec / max(np.linalg.norm(up_vec), 1e-9)

    u = np.cross(normal, up_vec)
    if np.linalg.norm(u) < 1e-8:
        up_vec = _pick_orthogonal_vec(normal)
        u = np.cross(normal, up_vec)
    u = u / max(np.linalg.norm(u), 1e-9)

    v = np.cross(normal, u)
    v = v / max(np.linalg.norm(v), 1e-9)

    half_w = obs["width"] / 2.0
    half_h = obs["height"] / 2.0
    thickness = obs.get("thickness", 1.0)
    half_t = thickness / 2.0

    return {
        "center": center,
        "normal": normal,
        "u": u,
        "v": v,
        "half_w": half_w,
        "half_h": half_h,
        "half_t": half_t,
    }


def _pick_orthogonal_vec(n_hat):
    """
    Return a vector orthogonal (or close) to n_hat for basis construction.
    """
    n_hat = np.asarray(n_hat, dtype=float).ravel()
    idx = np.argmin(np.abs(n_hat))
    basis = np.zeros(3)
    basis[idx] = 1.0
    v = np.cross(n_hat, basis)
    if np.linalg.norm(v) < 1e-8:
        basis = np.zeros(3)
        basis[(idx + 1) % 3] = 1.0
        v = np.cross(n_hat, basis)
    return v


def _wall_vertices(frame):
    """
    Compute the 8 vertices of a rectangular wall prism in world coordinates.
    frame is dict from _wall_geometry.
    """
    half_w = frame["half_w"]
    half_h = frame["half_h"]
    half_t = frame["half_t"]
    u = frame["u"]
    v = frame["v"]
    n = frame["normal"]
    center = frame["center"]

    locals_pts = np.array(
        [
            [-half_w, -half_h, -half_t],
            [half_w, -half_h, -half_t],
            [half_w, half_h, -half_t],
            [-half_w, half_h, -half_t],
            [-half_w, -half_h, half_t],
            [half_w, -half_h, half_t],
            [half_w, half_h, half_t],
            [-half_w, half_h, half_t],
        ]
    )

    verts = np.zeros((8, 3), dtype=float)
    for i in range(8):
        lx, ly, lz = locals_pts[i]
        verts[i] = center + lx * u + ly * v + lz * n
    return verts


def animate_flight_paths_interactive(speed_up=1.0, data=None, repeat=True):
    """
    Display an interactive 3D swarm animation that you can rotate, pan, and zoom.
    
    Parameters
    ----------
    speed_up : float
        Playback speed factor (2 => twice as fast as real time).
    data : dict
        Either:
          - {"positions": (n,3,steps) array, "time": (steps,) array}
        OR
          - {"csv_dir": directory with drone_*.csv files}
        Optional keys:
          - "goal": (3,) goal vector
          - "obstacles": list of obstacle dicts
          - "obs_style": dict with keys "color", "alpha", "edgecolor"
    repeat : bool
        Whether to loop the animation (default: True)
        
    Usage:
        The animation will display in an interactive matplotlib window.
        - Click and drag to rotate the view
        - Right-click and drag to pan
        - Scroll to zoom in/out
        - Close the window to end
    """
    if data is None:
        data = {}

    # Validate speed_up
    if not (isinstance(speed_up, (int, float)) and speed_up > 0):
        raise ValueError("speed_up must be a positive real scalar.")

    # ------------------------------------------------------------
    # Load positions/time
    # ------------------------------------------------------------
    if "positions" in data and "time" in data:
        positions = np.asarray(data["positions"], dtype=float)
        time_vec = np.asarray(data["time"], dtype=float).ravel()

        if positions.ndim != 3 or positions.shape[1] != 3:
            raise ValueError("positions must be an (n, 3, steps) array.")
        if time_vec.ndim != 1:
            raise ValueError("time must be a 1D vector.")
        if positions.shape[2] != time_vec.size:
            raise ValueError(
                "positions must be n x 3 x steps with steps matching len(time)."
            )

        n = positions.shape[0]
        steps = time_vec.size
    else:
        # Load from CSV directory
        csv_dir = data.get("csv_dir", os.path.join(os.getcwd(), "swarm_output"))
        positions, time_vec = _load_swarm_csv(csv_dir)
        n = positions.shape[0]
        steps = time_vec.size

    # ------------------------------------------------------------
    # Optional goal
    # ------------------------------------------------------------
    if "goal" in data and data["goal"] is not None:
        goal = np.asarray(data["goal"], dtype=float).ravel()
        if goal.size != 3:
            raise ValueError("goal must be a 3-element vector.")
    else:
        goal = None

    # ------------------------------------------------------------
    # Optional obstacles + style
    # ------------------------------------------------------------
    obstacles = data.get("obstacles", []) or []
    style = {
        "color": "b",
        "alpha": 0.35,
        "edgecolor": "none",
    }
    obs_style = data.get("obs_style", None)
    if obs_style is not None:
        for k, v in obs_style.items():
            style[k] = v

    # ------------------------------------------------------------
    # Frame pacing
    # ------------------------------------------------------------
    dt = np.median(np.diff(time_vec))
    frame_rate_desired = speed_up / dt
    max_fps = 60.0
    if frame_rate_desired <= max_fps:
        frame_stride = 1
        frame_rate = frame_rate_desired
    else:
        frame_stride = int(np.ceil(frame_rate_desired / max_fps))
        frame_rate = frame_rate_desired / frame_stride

    frame_indices = np.arange(0, steps, frame_stride, dtype=int)
    interval = 1000.0 / frame_rate  # milliseconds per frame

    # ------------------------------------------------------------
    # Data ranges
    # ------------------------------------------------------------
    xdata = positions[:, 0, :]  # (n, steps)
    ydata = positions[:, 1, :]
    zdata = positions[:, 2, :]

    xrange = np.array([np.min(xdata), np.max(xdata)], dtype=float)
    yrange = np.array([np.min(ydata), np.max(ydata)], dtype=float)
    zrange = np.array([np.min(zdata), np.max(zdata)], dtype=float)

    # Expand ranges to include obstacles
    if obstacles:
        ox, oy, oz = _obstacle_bounds(obstacles)
        xrange = np.array([min(xrange[0], ox[0]), max(xrange[1], ox[1])])
        yrange = np.array([min(yrange[0], oy[0]), max(yrange[1], oy[1])])
        zrange = np.array([min(zrange[0], oz[0]), max(zrange[1], oz[1])])

    margin = 0.05
    xrange = _expand_range(xrange, margin)
    yrange = _expand_range(yrange, margin)
    zrange = _expand_range(zrange, margin)

    # ------------------------------------------------------------
    # Figure setup
    # ------------------------------------------------------------
    colors = plt.cm.tab10(np.linspace(0, 1, max(n, 10)))[:n]

    fig = plt.figure("Interactive Swarm Animation", figsize=(10, 8))
    fig.patch.set_facecolor("white")
    ax = fig.add_subplot(111, projection="3d")
    ax.grid(True)
    ax.set_box_aspect(
        [
            xrange[1] - xrange[0],
            yrange[1] - yrange[0],
            zrange[1] - zrange[0],
        ]
    )
    ax.view_init(elev=25, azim=35)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_zlabel("z [m]")
    ax.set_xlim(xrange)
    ax.set_ylim(yrange)
    ax.set_zlim(zrange)
    ax.set_title("Swarm trajectories (Interactive - Click & Drag to Rotate)")

    # Floor
    Xplane, Yplane = np.meshgrid(
        [xrange[0], xrange[1]], [yrange[0], yrange[1]]
    )
    Zplane = np.full_like(Xplane, zrange[0])
    ax.plot_surface(
        Xplane,
        Yplane,
        Zplane,
        facecolor=(0.85, 0.85, 0.85),
        edgecolor="none",
        alpha=0.6,
    )

    # Obstacles (static)
    if obstacles:
        _draw_obstacles(ax, obstacles, style)

    # Paths and markers
    traj = []
    markers = []
    for ii in range(n):
        line, = ax.plot(
            xdata[ii, 0:1],
            ydata[ii, 0:1],
            zdata[ii, 0:1],
            color=colors[ii],
        )
        marker, = ax.plot(
            [xdata[ii, 0]],
            [ydata[ii, 0]],
            [zdata[ii, 0]],
            "o",
            markerfacecolor=colors[ii],
            markeredgecolor=colors[ii],
            markersize=8,
        )
        traj.append(line)
        markers.append(marker)

    if goal is not None:
        ax.plot(
            [goal[0]],
            [goal[1]],
            [goal[2]],
            "s",
            color="k",
            markerfacecolor="y",
            markersize=10,
            label="Goal",
        )

    time_text = ax.text2D(
        0.02, 0.95, "", transform=ax.transAxes, fontsize=12, weight='bold'
    )

    # ------------------------------------------------------------
    # Animation update function
    # ------------------------------------------------------------
    def update(frame_num):
        k = frame_indices[frame_num]
        
        for ii in range(n):
            traj[ii].set_data(xdata[ii, : k + 1], ydata[ii, : k + 1])
            traj[ii].set_3d_properties(zdata[ii, : k + 1])

            markers[ii].set_data([xdata[ii, k]], [ydata[ii, k]])
            markers[ii].set_3d_properties([zdata[ii, k]])

        time_text.set_text(f"t = {time_vec[k]:.2f} s")
        return traj + markers + [time_text]

    # ------------------------------------------------------------
    # Create and display animation
    # ------------------------------------------------------------
    anim = FuncAnimation(
        fig,
        update,
        frames=len(frame_indices),
        interval=interval,
        blit=False,
        repeat=repeat
    )
    
    print("\n" + "="*60)
    print("Interactive Animation Controls:")
    print("  - Click & drag: Rotate view")
    print("  - Right-click & drag: Pan")
    print("  - Scroll wheel: Zoom in/out")
    print("  - Close window to exit")
    print("="*60 + "\n")
    
    plt.show()
    
    return anim
