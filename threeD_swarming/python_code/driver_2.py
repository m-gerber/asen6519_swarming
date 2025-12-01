import os
import yaml
import numpy as np

# Import your previously defined functions
from demo_swarm_run import demo_swarm_run
from obstacle_repulsion import obstacle_repulsion
from animate_flight_paths import animate_flight_paths, animate_flight_paths_interactive


def load_initial_config(config_path="initial_config.yaml"):
    import os

    if not os.path.exists(config_path):
        raise FileNotFoundError(
            f"Configuration file '{config_path}' not found in {os.getcwd()}"
        )

    with open(config_path, "r") as f:
        cfg = yaml.safe_load(f)

    required_keys = [
        "n", "Ls_m", "goal_m",
        "initial_positions_m", "initial_velocities_mps",
        "T_s", "dt_s", "run_w_obstacles",
        "log_every_n_steps",
        "command_file_path", "command_check_every_n_steps",
    ]

    missing = [k for k in required_keys if k not in cfg]
    if missing:
        raise KeyError(f"Missing required config keys: {missing}")

    n = int(cfg["n"])
    Ls = float(cfg["Ls_m"])
    goal = np.asarray(cfg["goal_m"], dtype=float).ravel()

    P0 = np.asarray(cfg["initial_positions_m"], dtype=float)
    V0 = np.asarray(cfg["initial_velocities_mps"], dtype=float)

    if P0.shape != (n, 3):
        raise ValueError(f"initial_positions_m must be {n}x3; got {P0.shape}")
    if V0.shape != (n, 3):
        raise ValueError(f"initial_velocities_mps must be {n}x3; got {V0.shape}")

    T = float(cfg["T_s"])
    dt = float(cfg["dt_s"])
    run_w_obstacles = bool(cfg["run_w_obstacles"])

    log_every_n_steps = int(cfg["log_every_n_steps"])
    if log_every_n_steps <= 0:
        raise ValueError("log_every_n_steps must be a positive integer.")

    command_file_path = str(cfg["command_file_path"])
    command_check_every_n_steps = int(cfg["command_check_every_n_steps"])
    if command_check_every_n_steps < 0:
        raise ValueError("command_check_every_n_steps must be >= 0.")

    # Optional hazards configuration
    raw_hazards = cfg.get("hazards", [])
    if raw_hazards is None:
        raw_hazards = []

    if not isinstance(raw_hazards, list):
        raise ValueError("hazards (if provided) must be a list of hazard definitions.")

    hazards = []
    for h in raw_hazards:
        if not isinstance(h, dict):
            continue
        h_type = str(h.get("type", "")).lower()
        if h_type != "sphere":
            # For now, only support spherical hazards
            continue

        center = h.get("center_m", None)
        radius = h.get("radius_m", None)
        if center is None or radius is None:
            continue

        try:
            center_arr = [float(center[0]), float(center[1]), float(center[2])]
            radius_val = float(radius)
        except Exception:
            continue

        hazards.append(
            {
                "name": h.get("name", "hazard"),
                "type": "sphere",
                "center_m": center_arr,
                "radius_m": radius_val,
            }
        )

    # Optional reroute goal for hazard avoidance
    reroute_goal_cfg = cfg.get("reroute_goal_m", None)
    reroute_goal = None
    if reroute_goal_cfg is not None:
        try:
            reroute_goal = [
                float(reroute_goal_cfg[0]),
                float(reroute_goal_cfg[1]),
                float(reroute_goal_cfg[2]),
            ]
        except Exception:
            reroute_goal = None

    return {
        "n": n, "Ls": Ls, "goal": goal, "P0": P0, "V0": V0,
        "T": T, "dt": dt, "run_w_obstacles": run_w_obstacles,
        "log_every_n_steps": log_every_n_steps,
        "command_file_path": command_file_path,
        "command_check_every_n_steps": command_check_every_n_steps,
        "hazards": hazards,
        "reroute_goal_m": reroute_goal,
    }


def main():
    cfg = load_initial_config("../inputs/initial_config.yaml")

    n = cfg["n"]
    Ls = cfg["Ls"]
    goal = cfg["goal"]
    T = cfg["T"]
    dt = cfg["dt"]
    P0 = cfg["P0"]
    V0 = cfg["V0"]
    run_w_obstacles = cfg["run_w_obstacles"]
    log_every_n_steps = cfg["log_every_n_steps"]
    command_file_path = cfg["command_file_path"]
    command_check_every_n_steps = cfg["command_check_every_n_steps"]

    use_interactive = True    # Set to True for interactive animation, False for video export

    # ------------------------------------------------------------
    # Obstacles
    # ------------------------------------------------------------
    obstacles = []

    # Cylinder centered on the straight path (45° line)
    obstacles.append(
        {
            "type": "cylinder",
            "xy": [100.0, 100.0],
            "radius": 6.0,
            "zmin": 0.0,
            "zmax": 120.0,
        }
    )

    # Planar wall (rectangular panel) to stress-test avoidance logic
    obstacles.append(
        {
            "type": "wall",
            "center": [140.0, 120.0, 40.0],  # [m]
            "normal": [-1.0, 0.0, 0.0],      # outward normal
            "width": 45.0,                   # [m]
            "height": 60.0,                  # [m]
            "up": [0.0, 0.0, 1.0],           # keep panel upright
            "thickness": 8.0,                # finite thickness
            "d_safe": 25.0,                  # obstacle-specific safety radius
            "fmax": 15.0,                    # obstacle-specific accel cap
        }
    )

    # Global obstacle parameters
    obs_params = {
        "k_o": 40.0,    # repulsion gain [m/s^2]
        "d_safe": 18.0, # radius of influence [m]
        "fmax": 10.0,   # max obstacle avoidance accel [m/s^2]
    }

    # ------------------------------------------------------------
    # Run simulation + animation
    # ------------------------------------------------------------
    if run_w_obstacles:
        demo_swarm_run(
            n=n,
            Ls=Ls,
            goal=goal,
            T=T,
            dt=dt,
            P0=P0,
            V0=V0,
            obstacles=obstacles,
            obs_params=obs_params,
            log_every_n_steps=log_every_n_steps,
            command_file_path=command_file_path,
            command_check_every_n_steps=command_check_every_n_steps,
        )

        data = {
            "csv_dir": os.path.join(os.getcwd(), "swarm_output"),
            "goal": goal,
            "obstacles": obstacles,
            "obs_style": {
                "color": "b",
                "alpha": 0.35,
                "edgecolor": "none",
            },
        }
        
        if use_interactive:
            # Interactive animation - rotate, pan, zoom while it plays
            animate_flight_paths_interactive(speed_up=2.0, data=data, repeat=True)
        else:
            # Export video file
            animate_flight_paths(speed_up=2.0, data=data)

    else:
        demo_swarm_run(
            n=n,
            Ls=Ls,
            goal=goal,
            T=T,
            dt=dt,
            P0=P0,
            V0=V0,
            obstacles=[],     # no obstacles
            obs_params=None,  # or {}
            log_every_n_steps=log_every_n_steps,
            command_file_path=command_file_path,
            command_check_every_n_steps=command_check_every_n_steps,
        )
        data = {
            "csv_dir": os.path.join(os.getcwd(), "swarm_output"),
            "goal": goal,
        }
        
        if use_interactive:
            # Interactive animation - rotate, pan, zoom while it plays
            animate_flight_paths_interactive(speed_up=2.0, data=data, repeat=True)
        else:
            # Export video file
            animate_flight_paths(speed_up=2.0, data=data)


if __name__ == "__main__":
    main()
