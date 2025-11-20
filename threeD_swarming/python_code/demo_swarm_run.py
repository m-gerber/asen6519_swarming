import os
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (needed for 3D projection)

# Import obstacle_repulsion function
from obstacle_repulsion import obstacle_repulsion


def demo_swarm_run(
    n=None,
    Ls=None,
    goal=None,
    T=None,
    dt=None,
    P0=None,
    V0=None,
    obstacles=None,
    obs_params=None,
    outdir=None,
    make_plot=True,
):
    """
    Minimal 3D swarm simulator with spacing, heading, altitude, goal tracking,
    and obstacle avoidance via short-range repulsive fields.

    Parameters roughly mirror the MATLAB version.
    CSVs are written to ./swarm_output by default, and the function
    returns (P, V, t) for convenience.

    Returns
    -------
    P : (n, 3, steps) ndarray
        Positions over time.
    V : (n, 3, steps) ndarray
        Velocities over time.
    t : (steps,) ndarray
        Time vector.
    """
    # -----------------------
    # Default parameters
    # -----------------------
    if n is None:
        n = 5
    if Ls is None:
        Ls = 15.0
    if goal is None:
        goal = np.array([200.0, 50.0, 40.0], dtype=float)
    else:
        goal = np.asarray(goal, dtype=float).ravel()
    if T is None:
        T = 60.0
    if dt is None:
        dt = 0.05

    # -----------------------
    # Initial states (n x 3)
    # -----------------------
    if P0 is None:
        side = int(np.ceil(np.sqrt(n)))
        spacing = 10.0
        xs = np.arange(0, side * spacing, spacing)
        ys = np.arange(0, side * spacing, spacing)
        xx, yy = np.meshgrid(xs, ys)
        xy = np.column_stack((xx.ravel(), yy.ravel()))
        P0 = np.column_stack((xy[:n, :], 20.0 * np.ones((n, 1))))
    else:
        P0 = np.asarray(P0, dtype=float)

    if V0 is None:
        V0 = np.tile(np.array([4.0, 0.0, 0.0]), (n, 1))
    else:
        V0 = np.asarray(V0, dtype=float)

    # -----------------------
    # Obstacles and parameters
    # -----------------------
    if obstacles is None:
        obstacles = []

    # Normalize obstacle definitions (assume list of dicts)
    norm_obstacles = []
    for kk, obs in enumerate(obstacles, start=1):
        if "type" not in obs or obs["type"] is None:
            obs["type"] = "cylinder"

        otype = obs["type"].lower()
        if otype == "cylinder":
            need = ["xy", "radius", "zmin", "zmax"]
            for f in need:
                if f not in obs:
                    raise ValueError(f'Cylinder obstacle {kk} missing field "{f}".')

            if obs["radius"] <= 0:
                raise ValueError(f"Cylinder obstacle {kk} must have positive radius.")

        elif otype == "wall":
            need = ["center", "normal", "width", "height"]
            for f in need:
                if f not in obs:
                    raise ValueError(f'Wall obstacle {kk} missing field "{f}".')

            nvec = np.asarray(obs["normal"], dtype=float).ravel()
            if np.linalg.norm(nvec) < 1e-8:
                raise ValueError(f"Wall obstacle {kk} normal must be non-zero.")
            nvec = nvec / np.linalg.norm(nvec)
            obs["normal"] = nvec

            if obs["width"] <= 0 or obs["height"] <= 0:
                raise ValueError(
                    f"Wall obstacle {kk} must have positive width and height."
                )

            if "thickness" not in obs or obs["thickness"] is None:
                obs["thickness"] = 2.0
            if obs["thickness"] <= 0:
                raise ValueError(f"Wall obstacle {kk} must have positive thickness.")

            if "up" not in obs or obs["up"] is None:
                obs["up"] = []
            else:
                obs["up"] = np.asarray(obs["up"], dtype=float).ravel()

        else:
            raise ValueError(f'Unsupported obstacle type "{obs["type"]}" for obstacle {kk}.')

        norm_obstacles.append(obs)

    obstacles = norm_obstacles

    # Normalize obs_params
    default_obs_params = {"k_o": 40.0, "d_safe": 12.0, "fmax": 10.0}
    if obs_params is None:
        obs_params = default_obs_params.copy()
    else:
        # ensure dict
        if not isinstance(obs_params, dict):
            # you can adapt this if you use objects
            obs_params = dict(obs_params)
        for key, val in default_obs_params.items():
            if key not in obs_params or obs_params[key] is None:
                obs_params[key] = val

    # Safety checks
    assert P0.shape == (n, 3), "P0 must be n-by-3"
    assert V0.shape == (n, 3), "V0 must be n-by-3"

    # -----------------------
    # Controller parameters
    # -----------------------
    k_g = 0.02      # goal attraction
    k_s = 0.8       # separation-length regulation
    k_a = 0.6       # heading alignment
    k_h = 0.5       # altitude tracking
    c_d = 0.25      # linear damping on velocity
    goal_tol = 5.0  # considered "at goal" within this radius [m]

    amax = 6.0      # accel limit [m/s^2]
    vmax = 15.0     # speed limit [m/s]
    eps_d = 1e-3    # small distance epsilon

    # Altitude target: fixed or initial mean
    h_star = goal[2]

    # -----------------------
    # Preallocate logs
    # -----------------------
    steps = int(T / dt) + 1
    P = np.zeros((n, 3, steps), dtype=float)
    V = np.zeros((n, 3, steps), dtype=float)

    P[:, :, 0] = P0
    V[:, :, 0] = V0

    g = goal.copy()
    settle_steps_required = 20
    settle_counter = 0

    # -----------------------
    # Nested derivatives function (RK4 system)
    # -----------------------
    def swarm_derivs(p, v):
        # p, v are (n, 3)
        # Goal term
        a_goal = k_g * (g - p)  # broadcast (3,) - (n,3)

        # Alignment term
        vbar = np.mean(v, axis=0)
        a_align = k_a * (vbar - v)

        # Altitude term
        a_alt = np.column_stack(
            (np.zeros((n, 2)), k_h * (h_star - p[:, 2]))
        )

        # Obstacle repulsion
        if len(obstacles) == 0 or obs_params["d_safe"] <= 0:
            a_obs = np.zeros((n, 3), dtype=float)
        else:
            a_obs = obstacle_repulsion(p, obstacles, obs_params)

        # Separation-length regulation
        a_sep = np.zeros((n, 3), dtype=float)
        for ii in range(n):
            acc = np.zeros(3, dtype=float)
            for jj in range(n):
                if jj == ii:
                    continue
                rij = p[jj, :] - p[ii, :]
                dij = np.linalg.norm(rij)
                if dij < eps_d:
                    continue
                rhat = rij / dij
                acc += (dij - Ls) * rhat
            a_sep[ii, :] = (k_s / (n - 1)) * acc

        a = a_goal + a_align + a_sep + a_alt + a_obs - c_d * v

        # Acceleration saturation
        anorm = np.linalg.norm(a, axis=1)
        scl = np.minimum(1.0, amax / np.maximum(anorm, 1e-9))
        a = a * scl[:, None]

        dp = v
        dv = a
        return dp, dv

    # -----------------------
    # Main loop (RK4)
    # -----------------------
    for k in range(steps - 1):
        p = P[:, :, k]
        v = V[:, :, k]

        dp1, dv1 = swarm_derivs(p, v)
        dp2, dv2 = swarm_derivs(p + 0.5 * dt * dp1, v + 0.5 * dt * dv1)
        dp3, dv3 = swarm_derivs(p + 0.5 * dt * dp2, v + 0.5 * dt * dv2)
        dp4, dv4 = swarm_derivs(p + dt * dp3, v + dt * dv3)

        p_next = p + (dt / 6.0) * (dp1 + 2 * dp2 + 2 * dp3 + dp4)
        v_next = v + (dt / 6.0) * (dv1 + 2 * dv2 + 2 * dv3 + dv4)

        # Speed limit
        spd = np.linalg.norm(v_next, axis=1)
        scale = np.minimum(1.0, vmax / np.maximum(spd, 1e-9))
        v_next = v_next * scale[:, None]

        P[:, :, k + 1] = p_next
        V[:, :, k + 1] = v_next

        dist_next = np.linalg.norm(p_next - g, axis=1)
        if np.all(dist_next <= goal_tol):
            settle_counter += 1
        else:
            settle_counter = 0

        if settle_counter >= settle_steps_required:
            # Truncate logs
            P = P[:, :, : (k + 2)]
            V = V[:, :, : (k + 2)]
            steps = k + 2
            break

    # -----------------------
    # Goal diagnostics
    # -----------------------
    final_pos = P[:, :, -1]
    dist_to_goal = np.linalg.norm(final_pos - g, axis=1)
    max_dist = float(np.max(dist_to_goal))
    min_dist = float(np.min(dist_to_goal))
    mean_dist = float(np.mean(dist_to_goal))
    reached_goal = max_dist <= goal_tol

    if reached_goal:
        print(
            "All drones reached the goal within "
            f"{goal_tol:.2f} m. Min/Mean/Max distances: "
            f"{min_dist:.2f} / {mean_dist:.2f} / {max_dist:.2f} m"
        )
    else:
        print(
            "Goal not reached: Min/Mean/Max distances to goal = "
            f"{min_dist:.2f} / {mean_dist:.2f} / {max_dist:.2f} m "
            f"(threshold {goal_tol:.2f} m)"
        )

    # -----------------------
    # Export logs
    # -----------------------
    if outdir is None:
        outdir = os.path.join(os.getcwd(), "swarm_output")
    os.makedirs(outdir, exist_ok=True)

    t = np.arange(steps) * dt

    # Per-drone files
    for i in range(n):
        M = np.column_stack(
            (
                t,
                P[i, 0, :],
                P[i, 1, :],
                P[i, 2, :],
                V[i, 0, :],
                V[i, 1, :],
                V[i, 2, :],
            )
        )
        cols = ["t_s", "x_m", "y_m", "z_m", "vx_ms", "vy_ms", "vz_ms"]
        tbl = pd.DataFrame(M, columns=cols)
        fname = os.path.join(outdir, f"drone_{i+1:02d}.csv")
        tbl.to_csv(fname, index=False)

    # Combined file
    Pflat = P.transpose(2, 0, 1).reshape(-1, 3)  # (steps, n, 3) -> (n*steps, 3)
    Vflat = V.transpose(2, 0, 1).reshape(-1, 3)
    idcol = np.repeat(np.arange(1, n + 1), steps)
    tcol = np.tile(t, n)

    All = np.column_stack((idcol, tcol, Pflat, Vflat))
    cols_all = ["id", "t_s", "x_m", "y_m", "z_m", "vx_ms", "vy_ms", "vz_ms"]
    tblAll = pd.DataFrame(All, columns=cols_all)
    tblAll.to_csv(os.path.join(outdir, "swarm_all.csv"), index=False)

    print(f"Exported {n} per-drone files and combined file to {outdir}")

    # -----------------------
    # Quick plot
    # -----------------------
    if make_plot:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        ax.set_zlabel("z [m]")
        ax.grid(True)

        for i in range(n):
            ax.plot(P[i, 0, :], P[i, 1, :], P[i, 2, :])

        ax.scatter(g[0], g[1], g[2], c="y", edgecolors="k", s=50)  # goal

        # Draw cylinder obstacles (approximate)
        for obs in obstacles:
            if obs["type"].lower() == "cylinder":
                cx, cy = obs["xy"]
                R = obs["radius"]
                z0 = obs["zmin"]
                z1 = obs["zmax"]

                th = np.linspace(0, 2 * np.pi, 60)
                z_vals = np.array([z0, z1])
                TH, ZZ = np.meshgrid(th, z_vals)
                XX = cx + R * np.cos(TH)
                YY = cy + R * np.sin(TH)

                ax.plot_surface(XX, YY, ZZ, alpha=0.35)

                # Caps (as circles)
                xc = cx + R * np.cos(th)
                yc = cy + R * np.sin(th)
                ax.plot(xc, yc, z0, alpha=0.35)
                ax.plot(xc, yc, z1, alpha=0.35)

        ax.set_title("Swarm trajectories")
        ax.view_init(elev=25, azim=35)
        plt.tight_layout()

    return P, V, t
