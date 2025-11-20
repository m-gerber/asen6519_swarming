import os
import numpy as np

# Import your previously defined functions
from demo_swarm_run import demo_swarm_run
from obstacle_repulsion import obstacle_repulsion
from animate_flight_paths import animate_flight_paths, animate_flight_paths_interactive


def main():
    n = 5                     # number of drones (>=3)
    Ls = 3.0                  # desired mean separation [m]
    goal = np.array([200.0, 200.0, 50.0])  # final goal position [m]
    T = 45.0                  # total sim time [s]
    dt = 0.05                 # time step [s]

    run_w_obstacles = True
    use_interactive = True    # Set to True for interactive animation, False for video export

    # ------------------------------------------------------------
    # Initial states (n x 3 each)
    # ------------------------------------------------------------
    P0 = np.array(
        [
            [4.0,   0.0, 20.0],
            [-4.0,  0.0, 20.0],
            [8.0,   0.0, 20.0],
            [-8.0,  0.0, 20.0],
            [12.0,  0.0, 20.0],
        ],
        dtype=float,
    )
    V0 = np.tile(np.array([4.0, 0.0, 0.0]), (n, 1))  # velocities [m/s]

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
