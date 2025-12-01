import os
import json
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import commands
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (needed for 3D projection)
from dataclasses import dataclass
from typing import Any, Dict, Tuple, Optional, List

# Import obstacle_repulsion function
from obstacle_repulsion import obstacle_repulsion

# Import swarm manager
from swarm_manager import SwarmManager, Drone, DroneMode


@dataclass
class SwarmControlParams:
    """
    Tunable controller parameters for the swarm dynamics.
    """
    k_g: float = 0.02     # goal attraction
    k_s: float = 0.8      # separation-length regulation
    k_a: float = 0.6      # heading alignment
    k_h: float = 0.5      # altitude tracking
    c_d: float = 0.25     # linear damping
    goal_tol: float = 5.0 # distance to consider "at goal" [m]
    amax: float = 6.0     # accel limit [m/s^2]
    vmax: float = 15.0    # speed limit [m/s]
    eps_d: float = 1e-3   # small distance epsilon
    h_star: float = 0.0   # altitude target [m]


class SwarmState:
    """
    Minimal container for the swarm state at a given time.
    """
    def __init__(self, P0: np.ndarray, V0: np.ndarray, t0: float = 0.0):
        P0 = np.asarray(P0, dtype=float)
        V0 = np.asarray(V0, dtype=float)
        if P0.shape[1] != 3 or V0.shape[1] != 3:
            raise ValueError("SwarmState expects P0 and V0 with shape (n, 3).")
        if P0.shape != V0.shape:
            raise ValueError("P0 and V0 must have the same shape.")

        self.P = P0.copy()   # (n, 3)
        self.V = V0.copy()   # (n, 3)
        self.t = float(t0)   # scalar time [s]


def compute_swarm_derivs(
    p: np.ndarray,
    v: np.ndarray,
    goal: np.ndarray,
    Ls: float,
    obstacles: list,
    obs_params: Dict[str, Any],
    params: SwarmControlParams,
    swarm_manager: Optional[SwarmManager] = None,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Compute time derivatives dp/dt and dv/dt for the swarm state.
    
    Enhanced to support multi-group control and individual drone modes.

    Parameters
    ----------
    p : (n, 3)
        Positions.
    v : (n, 3)
        Velocities.
    goal : (3,)
        Goal position (used as fallback for main group).
    Ls : float
        Desired separation distance (used as fallback).
    obstacles : list
        List of obstacle dicts.
    obs_params : dict
        Obstacle parameters (k_o, d_safe, fmax).
    params : SwarmControlParams
        Controller parameters.
    swarm_manager : SwarmManager, optional
        Multi-group swarm manager.

    Returns
    -------
    dp, dv : (n, 3), (n, 3)
        Derivatives for positions and velocities.
    """
    n = p.shape[0]
    
    # Update swarm manager state
    if swarm_manager is not None:
        swarm_manager.set_state_arrays(p, v)
    
    # Initialize acceleration array
    a = np.zeros((n, 3), dtype=float)
    
    # Process each drone based on its mode
    for i in range(n):
        if swarm_manager is None:
            # Legacy mode: all drones in single swarm
            a[i] = compute_single_drone_swarm_accel(
                i, p, v, goal, Ls, obstacles, obs_params, params
            )
        else:
            drone = swarm_manager.drones.get(i)
            if drone is None:
                continue
            
            if drone.mode == DroneMode.SWARM:
                # Swarm mode: compute group-based swarm forces
                a[i] = compute_group_swarm_accel(
                    i, drone, swarm_manager, p, v, obstacles, obs_params, params
                )
            
            elif drone.mode == DroneMode.NAVIGATE:
                # Independent navigation to waypoints
                a[i] = compute_waypoint_navigation_accel(
                    drone, obstacles, obs_params, params
                )
            
            elif drone.mode == DroneMode.LOITER:
                # Hover at loiter position
                a[i] = compute_loiter_accel(drone, params)
            
            elif drone.mode == DroneMode.REJOIN:
                # Blend between independent and swarm control
                a_independent = compute_waypoint_navigation_accel(
                    drone, obstacles, obs_params, params
                )
                a_swarm = compute_group_swarm_accel(
                    i, drone, swarm_manager, p, v, obstacles, obs_params, params
                )
                
                blend = drone.rejoin_blend_factor
                a[i] = (1.0 - blend) * a_independent + blend * a_swarm

            elif drone.mode == DroneMode.LANDED:
                # Treat landed drones like indefinite loiter/position hold
                a[i] = compute_loiter_accel(drone, params)

    dp = v
    dv = a
    return dp, dv


def compute_single_drone_swarm_accel(
    i: int,
    p: np.ndarray,
    v: np.ndarray,
    goal: np.ndarray,
    Ls: float,
    obstacles: list,
    obs_params: Dict[str, Any],
    params: SwarmControlParams,
) -> np.ndarray:
    """
    Compute acceleration for a single drone using standard swarm laws.
    (Original implementation for backward compatibility)
    """
    n = p.shape[0]
    
    # Goal term
    a_goal = params.k_g * (goal - p[i])
    
    # Alignment term
    vbar = np.mean(v, axis=0)
    a_align = params.k_a * (vbar - v[i])
    
    # Altitude term
    a_alt = np.array([0.0, 0.0, params.k_h * (params.h_star - p[i, 2])])
    
    # Obstacle repulsion (compute for all, then extract for drone i)
    if len(obstacles) == 0 or obs_params["d_safe"] <= 0:
        a_obs = np.zeros(3)
    else:
        a_obs_all = obstacle_repulsion(p, obstacles, obs_params)
        a_obs = a_obs_all[i]
    
    # Separation-length regulation
    a_sep = np.zeros(3, dtype=float)
    for j in range(n):
        if j == i:
            continue
        rij = p[j, :] - p[i, :]
        dij = np.linalg.norm(rij)
        if dij < params.eps_d:
            continue
        rhat = rij / dij
        a_sep += (dij - Ls) * rhat
    a_sep = (params.k_s / max(n - 1, 1)) * a_sep
    
    # Combine and add damping
    a = a_goal + a_align + a_sep + a_alt + a_obs - params.c_d * v[i]
    
    # Acceleration saturation
    anorm = np.linalg.norm(a)
    if anorm > params.amax:
        a = a * (params.amax / anorm)
    
    return a


def compute_group_swarm_accel(
    i: int,
    drone: Drone,
    swarm_manager: SwarmManager,
    p: np.ndarray,
    v: np.ndarray,
    obstacles: list,
    obs_params: Dict[str, Any],
    params: SwarmControlParams,
) -> np.ndarray:
    """
    Compute acceleration for drone in swarm mode with group-based coordination.
    """
    group = swarm_manager.groups.get(drone.group_id)
    if group is None:
        # Fallback to independent control
        return np.zeros(3)
    if group.goal_hold_active:
        # Group has reached its destination; keep everyone planted
        return compute_loiter_accel(drone, params)
    
    # Get group members
    group_drone_ids = [did for did in group.drone_ids if did in swarm_manager.drones]
    
    # Goal term (group-specific)
    a_goal = params.k_g * (group.goal - drone.position)
    
    # Alignment and separation only within group
    if len(group_drone_ids) > 1:
        # Compute mean velocity of group
        group_velocities = np.array([swarm_manager.drones[did].velocity 
                                     for did in group_drone_ids])
        vbar = np.mean(group_velocities, axis=0)
        a_align = params.k_a * (vbar - drone.velocity)
        
        # Separation regulation within group
        a_sep = np.zeros(3, dtype=float)
        for j in group_drone_ids:
            if j == i:
                continue
            other_drone = swarm_manager.drones[j]
            rij = other_drone.position - drone.position
            dij = np.linalg.norm(rij)
            if dij < params.eps_d:
                continue
            rhat = rij / dij
            a_sep += (dij - group.Ls) * rhat
        a_sep = (params.k_s / max(len(group_drone_ids) - 1, 1)) * a_sep
    else:
        a_align = np.zeros(3)
        a_sep = np.zeros(3)
    
    # Altitude term
    a_alt = np.array([0.0, 0.0, params.k_h * (params.h_star - drone.position[2])])
    
    # Obstacle repulsion
    if len(obstacles) == 0 or obs_params["d_safe"] <= 0:
        a_obs = np.zeros(3)
    else:
        a_obs_all = obstacle_repulsion(p, obstacles, obs_params)
        a_obs = a_obs_all[i]
    
    # Combine and add damping
    a = a_goal + a_align + a_sep + a_alt + a_obs - params.c_d * drone.velocity
    
    # Acceleration saturation
    anorm = np.linalg.norm(a)
    if anorm > params.amax:
        a = a * (params.amax / anorm)
    
    return a


def compute_waypoint_navigation_accel(
    drone: Drone,
    obstacles: list,
    obs_params: Dict[str, Any],
    params: SwarmControlParams,
) -> np.ndarray:
    """
    Compute acceleration for independent waypoint navigation.
    """
    # Determine current target waypoint
    if not drone.waypoints or drone.current_waypoint_idx >= len(drone.waypoints):
        # No waypoints or all complete - just hold position with damping
        return -params.c_d * drone.velocity
    
    target_wp = drone.waypoints[drone.current_waypoint_idx]
    
    # Goal attraction to waypoint
    a_goal = params.k_g * 2.0 * (target_wp - drone.position)  # Higher gain for navigation
    
    # Altitude tracking
    a_alt = np.array([0.0, 0.0, params.k_h * (target_wp[2] - drone.position[2])])
    
    # Obstacle repulsion (compute for single drone)
    if len(obstacles) == 0 or obs_params["d_safe"] <= 0:
        a_obs = np.zeros(3)
    else:
        p_single = drone.position.reshape(1, 3)
        a_obs_single = obstacle_repulsion(p_single, obstacles, obs_params)
        a_obs = a_obs_single[0]
    
    # Strong damping for independent flight
    a_damp = -params.c_d * 1.5 * drone.velocity
    
    # Combine
    a = a_goal + a_alt + a_obs + a_damp
    
    # Acceleration saturation
    anorm = np.linalg.norm(a)
    if anorm > params.amax:
        a = a * (params.amax / anorm)
    
    return a


def compute_loiter_accel(
    drone: Drone,
    params: SwarmControlParams,
) -> np.ndarray:
    """
    Compute acceleration for loiter/hover mode.
    High position-hold gain and strong damping.
    """
    if drone.loiter_position is None:
        loiter_pos = drone.position
    else:
        loiter_pos = drone.loiter_position
    
    # Strong position hold
    k_hover = 0.5  # Position hold gain
    a_hold = k_hover * (loiter_pos - drone.position)
    
    # Very strong damping to minimize drift
    a_damp = -params.c_d * 3.0 * drone.velocity
    
    # Combine
    a = a_hold + a_damp
    
    # Acceleration saturation
    anorm = np.linalg.norm(a)
    if anorm > params.amax:
        a = a * (params.amax / anorm)
    
    return a


def step_swarm(
    state: SwarmState,
    dt: float,
    goal: np.ndarray,
    Ls: float,
    obstacles: list,
    obs_params: Dict[str, Any],
    params: SwarmControlParams,
    swarm_manager: Optional[SwarmManager] = None,
) -> SwarmState:
    """
    Advance the swarm by a single time step using RK4.

    This does not allocate new arrays; it updates the given state in-place
    and returns it for convenience.

    Parameters
    ----------
    state : SwarmState
        Current swarm state (positions, velocities, time).
    dt : float
        Time step [s].
    goal, Ls, obstacles, obs_params, params
        As in compute_swarm_derivs.
    swarm_manager : SwarmManager, optional
        Multi-group swarm manager.

    Returns
    -------
    state : SwarmState
        Updated state (P, V, t).
    """
    p = state.P
    v = state.V

    dp1, dv1 = compute_swarm_derivs(p, v, goal, Ls, obstacles, obs_params, params, swarm_manager)
    dp2, dv2 = compute_swarm_derivs(
        p + 0.5 * dt * dp1, v + 0.5 * dt * dv1, goal, Ls, obstacles, obs_params, params, swarm_manager
    )
    dp3, dv3 = compute_swarm_derivs(
        p + 0.5 * dt * dp2, v + 0.5 * dt * dv2, goal, Ls, obstacles, obs_params, params, swarm_manager
    )
    dp4, dv4 = compute_swarm_derivs(
        p + dt * dp3, v + dt * dv3, goal, Ls, obstacles, obs_params, params, swarm_manager
    )

    p_next = p + (dt / 6.0) * (dp1 + 2 * dp2 + 2 * dp3 + dp4)
    v_next = v + (dt / 6.0) * (dv1 + 2 * dv2 + 2 * dv3 + dv4)

    # Enforce speed limit
    spd = np.linalg.norm(v_next, axis=1)
    scale = np.minimum(1.0, params.vmax / np.maximum(spd, 1e-9))
    v_next = v_next * scale[:, None]

    state.P = p_next
    state.V = v_next
    state.t += dt
    if swarm_manager is not None:
        swarm_manager.set_state_arrays(state.P, state.V)
    return state


def _write_command_file(command_path: str, command_dict: Dict[str, Any]) -> None:
    """Persist command JSON atomically so the simulator can consume it."""
    if not command_path:
        raise ValueError("command_file_path must be provided to write commands.")

    os.makedirs(os.path.dirname(command_path), exist_ok=True)
    tmp_path = command_path + ".tmp"
    with open(tmp_path, "w", encoding="utf-8") as tmp_file:
        json.dump(command_dict, tmp_file, indent=2)
    os.replace(tmp_path, command_path)


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
    log_every_n_steps=1,
    command_file_path=None,
    command_check_every_n_steps=None,
    scheduled_command_events: Optional[List[Dict[str, Any]]] = None,
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
    # Logging stride
    # -----------------------
    if log_every_n_steps is None:
        log_every_n_steps = 1
    else:
        log_every_n_steps = int(log_every_n_steps)

    if log_every_n_steps <= 0:
        raise ValueError("log_every_n_steps must be a positive integer.")

    # -----------------------
    # Command polling setup
    # -----------------------
    if command_file_path is None:
        command_file_path = os.path.join(os.getcwd(), "commands", "command.json")
    if command_check_every_n_steps is None:
        command_check_every_n_steps = 0
    command_check_every_n_steps = int(command_check_every_n_steps)
    if command_check_every_n_steps < 0:
        raise ValueError("command_check_every_n_steps must be >= 0.")
    last_command_id = 0

    # Normalize scheduled command events (if provided)
    normalized_events: List[Dict[str, Any]] = []
    if scheduled_command_events:
        if command_file_path is None:
            raise ValueError(
                "scheduled_command_events provided but no command_file_path configured."
            )
        for raw_event in scheduled_command_events:
            if not raw_event or "command" not in raw_event:
                continue
            try:
                trigger_time = float(raw_event.get("trigger_time_s", 0.0))
            except (TypeError, ValueError):
                continue
            normalized_events.append(
                {
                    "trigger_time_s": trigger_time,
                    "command": raw_event["command"],
                }
            )
        normalized_events.sort(key=lambda evt: evt["trigger_time_s"])
    next_scheduled_event_idx = 0

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
    params = SwarmControlParams(
        k_g=0.02,
        k_s=0.8,
        k_a=0.6,
        k_h=0.5,
        c_d=0.25,
        goal_tol=5.0,
        amax=6.0,
        vmax=15.0,
        eps_d=1e-3,
        h_star=goal[2],   # altitude target from goal z
    )

    # Goal vector used throughout simulation
    g = np.asarray(goal, dtype=float).ravel().copy()

    # -----------------------
    # Initialize SwarmManager for multi-group control
    # -----------------------
    swarm_manager = SwarmManager(n, P0, V0, g, Ls)

    # -----------------------
    # Preallocate logs
    # -----------------------
    steps = int(T / dt) + 1
    P = np.zeros((n, 3, steps), dtype=float)
    V = np.zeros((n, 3, steps), dtype=float)

    P[:, :, 0] = P0
    V[:, :, 0] = V0

    settle_steps_required = 20
    settle_counter = 0

    # -----------------------
    # Main loop (RK4 via step_swarm)
    # -----------------------
    state = SwarmState(P0, V0, t0=0.0)
    P[:, :, 0] = state.P
    V[:, :, 0] = state.V

    for k in range(steps - 1):
        # Update loiter timers
        if swarm_manager is not None:
            swarm_manager.update_loiter_state(state.t)
            swarm_manager.check_waypoint_progress()
        
        # Advance one time step
        state = step_swarm(
            state=state,
            dt=dt,
            goal=g,
            Ls=Ls,
            obstacles=obstacles,
            obs_params=obs_params,
            params=params,
            swarm_manager=swarm_manager,
        )

        if swarm_manager is not None:
            swarm_manager.check_group_goal_completion(params.goal_tol)

        # ------------------------------------------------
        # Inject scripted commands tied to simulation time
        # ------------------------------------------------
        while (
            next_scheduled_event_idx < len(normalized_events)
            and state.t >= normalized_events[next_scheduled_event_idx]["trigger_time_s"]
        ):
            event = normalized_events[next_scheduled_event_idx]
            _write_command_file(command_file_path, event["command"])
            cmd = event["command"]
            print(
                f"[Sim t={state.t:6.1f}s] Injected scheduled command "
                f"id={cmd.get('command_id')} type={cmd.get('command_type')}"
            )
            next_scheduled_event_idx += 1

        # ------------------------------------------------
        # Optional: check for new high-level command
        # ------------------------------------------------
        if command_check_every_n_steps > 0 and (k % command_check_every_n_steps) == 0:
            cmd, last_command_id = commands.check_for_new_command(
                command_file_path, last_command_id
            )
            if cmd is not None:
                g, params, obstacles = commands.apply_command_to_sim(
                    cmd, g, params, obstacles, swarm_manager, state.t
                )

        P[:, :, k + 1] = state.P
        V[:, :, k + 1] = state.V

        dist_next = np.linalg.norm(state.P - g, axis=1)
        if np.all(dist_next <= params.goal_tol):
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
    reached_goal = max_dist <= params.goal_tol

    if reached_goal:
        print(
            "All drones reached the goal within "
            f"{params.goal_tol:.2f} m. Min/Mean/Max distances: "
            f"{min_dist:.2f} / {mean_dist:.2f} / {max_dist:.2f} m"
        )
    else:
        print(
            "Goal not reached: Min/Mean/Max distances to goal = "
            f"{min_dist:.2f} / {mean_dist:.2f} / {max_dist:.2f} m "
            f"(threshold {params.goal_tol:.2f} m)"
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
    # Tracking logs (downsampled by log_every_n_steps)
    # -----------------------
    tracking_dir = os.path.join(os.getcwd(), "tracking_logs")
    os.makedirs(tracking_dir, exist_ok=True)

    # Choose indices to log: 0, log_every_n_steps, 2*log_every_n_steps, ...
    indices = np.arange(0, steps, log_every_n_steps, dtype=int)
    if indices[-1] != steps - 1:
        # Ensure we always log the final step as well
        indices = np.append(indices, steps - 1)

    t_sel = t[indices]
    steps_sel = t_sel.size

    # Subsample positions and velocities
    P_sel = P[:, :, indices]  # (n, 3, steps_sel)
    V_sel = V[:, :, indices]  # (n, 3, steps_sel)

    # Flatten as in swarm_all.csv: one row per (drone, time)
    Pflat = P_sel.transpose(2, 0, 1).reshape(-1, 3)
    Vflat = V_sel.transpose(2, 0, 1).reshape(-1, 3)
    idcol = np.repeat(np.arange(1, n + 1), steps_sel)
    tcol = np.tile(t_sel, n)

    Track = np.column_stack((idcol, tcol, Pflat, Vflat))
    cols_track = ["id", "t_s", "x_m", "y_m", "z_m", "vx_ms", "vy_ms", "vz_ms"]

    tblTrack = pd.DataFrame(Track, columns=cols_track)
    tracking_path = os.path.join(tracking_dir, "swarm_tracking.csv")
    tblTrack.to_csv(tracking_path, index=False)

    print(
        f"Tracking log written to {tracking_path} "
        f"(log_every_n_steps = {log_every_n_steps}, "
        f"samples per drone = {steps_sel})"
    )

    # -----------------------
    # Summary telemetry log
    # -----------------------
    # We reuse the same indices used for swarm_tracking.csv
    # (indices, t_sel, steps_sel already defined above).

    summary_rows = []

    # Precompute goal as a flat array
    goal_vec = np.asarray(g, dtype=float).ravel()
    if goal_vec.size != 3:
        raise ValueError("Goal vector 'g' must have length 3 for summary metrics.")

    for idx_step, k_idx in enumerate(indices):
        # Positions and velocities at this time step
        Pk = P[:, :, k_idx]  # (n, 3)
        Vk = V[:, :, k_idx]  # (n, 3)

        # Pairwise separation distances between drones
        # Compute full distance matrix and then take upper triangle
        diff = Pk[:, None, :] - Pk[None, :, :]  # (n, n, 3)
        D = np.linalg.norm(diff, axis=2)        # (n, n)
        iu = np.triu_indices(n, k=1)
        pair_dists = D[iu]
        if pair_dists.size > 0:
            min_sep = float(np.min(pair_dists))
            max_sep = float(np.max(pair_dists))
            mean_sep = float(np.mean(pair_dists))
        else:
            # Degenerate case (n < 2) – no pairwise distances
            min_sep = float("nan")
            max_sep = float("nan")
            mean_sep = float("nan")

        # Distances to goal for each drone
        d_goal = np.linalg.norm(Pk - goal_vec[None, :], axis=1)  # (n,)
        min_d_goal = float(np.min(d_goal))
        max_d_goal = float(np.max(d_goal))
        mean_d_goal = float(np.mean(d_goal))

        # Speeds
        speed = np.linalg.norm(Vk, axis=1)  # (n,)
        mean_speed = float(np.mean(speed))
        max_speed = float(np.max(speed))

        summary_rows.append(
            {
                "t_s": float(t[k_idx]),
                "step_idx": int(k_idx),
                "n_drones": int(n),
                "min_separation_m": min_sep,
                "max_separation_m": max_sep,
                "mean_separation_m": mean_sep,
                "min_distance_to_goal_m": min_d_goal,
                "mean_distance_to_goal_m": mean_d_goal,
                "max_distance_to_goal_m": max_d_goal,
                "mean_speed_ms": mean_speed,
                "max_speed_ms": max_speed,
            }
        )

    tblSummary = pd.DataFrame(summary_rows)
    summary_path = os.path.join(tracking_dir, "swarm_summary.csv")
    tblSummary.to_csv(summary_path, index=False)

    print(
        f"Summary log written to {summary_path} "
        f"(log_every_n_steps = {log_every_n_steps}, "
        f"samples = {steps_sel})"
    )

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
