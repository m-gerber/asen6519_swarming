import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider
from typing import Dict
import json

# ----------------------------
# Helper functions
# ----------------------------
def get_params() -> Dict:
    """Return all simulation parameters in a dictionary."""
    return {
        "N_AGENTS": 15,
        "DT": 0.1,
        "NEIGHBOR_RADIUS": 3.0,
        "MIN_DIST": 2.0,
        "VELOCITY_DAMPING": 0.95,
        # Leader
        "LEADER_INIT_POS": np.array([0.0, 0.0]),
        "LEADER_INIT_VEL": np.array([0.5, 0.0]),
        # Swarm weights (modifiable in real time)
        "W_ALIGNMENT": 0.05,
        "W_COHESION": 0.05,
        "W_SEPARATION": 0.5,
        "W_LEADER_AVOID": 0.5,
        "W_LEADER_FOLLOW": 0.3
    }

def load_json_params(path="params.json"):
    try:
        with open(path, "r") as f:
            return json.load(f)
    except Exception:
        return {}

def save_json_params(params, path="params.json"):
    """Save selected parameters to JSON without removing other keys like WAYPOINTS."""
    try:
        # Load existing data
        with open(path, "r") as f:
            existing = json.load(f)
    except Exception:
        existing = {}

    # Update only the swarm parameters
    for key in ["W_ALIGNMENT", "W_COHESION", "W_SEPARATION"]:
        existing[key] = params[key]

    # Write back
    with open(path, "w") as f:
        json.dump(existing, f, indent=4)

# ----------------------------
# Agent update logic
# ----------------------------
def update_agents(positions, velocities, leader_pos, leader_vel, params):
    new_positions = np.copy(positions)
    new_velocities = np.copy(velocities)
    n = params["N_AGENTS"]
    neighbor_radius = params["NEIGHBOR_RADIUS"]
    min_dist = params["MIN_DIST"]

    for i in range(n):
        diffs = positions - positions[i]
        distances = np.linalg.norm(diffs, axis=1)
        neighbors = (distances < neighbor_radius) & (distances > 0)

        # Cohesion
        cohesion_vec = np.zeros(2)
        if np.any(neighbors):
            center_of_mass = np.mean(positions[neighbors], axis=0)
            cohesion_vec = (center_of_mass - positions[i]) * params["W_COHESION"]

        # Alignment
        alignment_vec = np.zeros(2)
        if np.any(neighbors):
            mean_velocity = np.mean(velocities[neighbors], axis=0)
            alignment_vec = (mean_velocity - velocities[i]) * params["W_ALIGNMENT"]

        # Separation
        separation_vec = np.zeros(2)
        too_close = (distances < min_dist) & (distances > 0)
        if np.any(too_close):
            separation_vec = np.sum(-diffs[too_close] / (distances[too_close][:, np.newaxis] ** 2), axis=0)
            separation_vec *= params["W_SEPARATION"]

        # --- Virtual leader influence (SAFE NORMALIZATION) ---
        to_leader = leader_pos - positions[i]
        if np.linalg.norm(to_leader) > 0:

            # Avoid division by zero if leader stops
            speed = np.linalg.norm(leader_vel)
            if speed > 1e-6:
                direction = leader_vel / speed
            else:
                # Use last direction or a default
                if not hasattr(update_agents, "last_dir"):
                    update_agents.last_dir = np.array([1.0, 0.0])
                direction = update_agents.last_dir

            update_agents.last_dir = direction  # store for next frame

            behind_pos = leader_pos - direction * 2.0
            leader_follow_vec = (behind_pos - positions[i]) * params["W_LEADER_FOLLOW"]

        else:
            leader_follow_vec = np.zeros(2)

        acceleration = cohesion_vec + alignment_vec + separation_vec + leader_follow_vec
        new_velocities[i] += acceleration * params["DT"]
        new_velocities[i] *= params["VELOCITY_DAMPING"]
        new_positions[i]  += new_velocities[i] * params["DT"]

    return new_positions, new_velocities

# ----------------------------
# Leader path (virtual)
# ----------------------------
def straight_line_leader(pos, vel, dt, t=0.0):
    # Just keep moving back and forth
    new_pos = pos + vel * dt
    if np.abs(new_pos[0]) > 25:
        vel[0] *= -1
    return new_pos, vel

def waypoint_leader_live(params, speed=1.0):
    """
    Creates a leader update function that reads WAYPOINTS from params.json
    every frame and moves through them, stopping at the final waypoint.
    """
    idx = 0  # current waypoint index
    waypoints = []

    # Helper to detect changes in waypoint lists
    def waypoints_changed(old, new):
        if len(old) != len(new):
            return True
        for o, n in zip(old, new):
            if not np.array_equal(o, n):
                return True
        return False

    def update(pos, vel, dt, t):
        nonlocal idx, waypoints

        # Reload JSON params every frame
        external = load_json_params()

        # Check for updated waypoints
        if "WAYPOINTS" in external:
            new_wp = [np.array(wp, dtype=float) for wp in external["WAYPOINTS"]]

            # If waypoint list changed → reset progress
            if waypoints_changed(waypoints, new_wp):
                waypoints = new_wp
                idx = 0

        # If no waypoints exist → stay still
        if len(waypoints) == 0:
            return pos, np.zeros_like(vel)

        # If finished all waypoints → stop at final
        if idx >= len(waypoints):
            return waypoints[-1], np.zeros_like(vel)

        target = waypoints[idx]
        direction = target - pos
        dist = np.linalg.norm(direction)

        # Arrived at current waypoint → move to next
        if dist < 0.5:
            idx += 1
            if idx >= len(waypoints):
                return target, np.zeros_like(vel)
            target = waypoints[idx]
            direction = target - pos
            dist = np.linalg.norm(direction)

        # Normalize direction
        if dist > 1e-6:
            direction = direction / dist

        new_vel = direction * speed
        new_pos = pos + new_vel * dt
        return new_pos, new_vel

    return update


# ----------------------------
# Simulation
# ----------------------------
def run_simulation(leader_update_func, params):
    np.random.seed(7)
    leader_pos = params["LEADER_INIT_POS"].copy()
    leader_vel = params["LEADER_INIT_VEL"].copy()

    behind_distance, spread_x, spread_y = 2.5, 2.5, 5.0
    positions = np.zeros((params["N_AGENTS"], 2))
    positions[:, 0] = leader_pos[0] - behind_distance + np.random.uniform(-spread_x, spread_x, params["N_AGENTS"])
    positions[:, 1] = leader_pos[1] + np.random.uniform(-spread_y / 2, spread_y / 2, params["N_AGENTS"])
    velocities = np.zeros((params["N_AGENTS"], 2))

    fig, ax = plt.subplots(figsize=(12, 6))
    plt.subplots_adjust(left=0.1, bottom=0.35)
    ax.set_xlim(-25, 25)
    ax.set_ylim(-10, 10)
    ax.set_title("Continuous Swarming Simulation (Virtual Leader)")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_aspect('equal')
    ax.grid(True)
    agent_dots, = ax.plot([], [], 'bo', label="Agents")
    ax.legend()

    # --- Sliders for live parameter tuning ---
    axcolor = 'lightgoldenrodyellow'
    ax_align = plt.axes([0.1, 0.25, 0.8, 0.03], facecolor=axcolor)
    ax_cohes = plt.axes([0.1, 0.20, 0.8, 0.03], facecolor=axcolor)
    ax_sep = plt.axes([0.1, 0.15, 0.8, 0.03], facecolor=axcolor)

    s_align = Slider(ax_align, 'Alignment', 0.0, 2.0, valinit=params["W_ALIGNMENT"])
    s_cohes = Slider(ax_cohes, 'Cohesion', 0.0, 2.0, valinit=params["W_COHESION"])
    s_sep = Slider(ax_sep, 'Separation', 0.0, 5.0, valinit=params["W_SEPARATION"])

    def update_sliders(val):
        params["W_ALIGNMENT"] = s_align.val
        params["W_COHESION"] = s_cohes.val
        params["W_SEPARATION"] = s_sep.val
        save_json_params(params)

    s_align.on_changed(update_sliders)
    s_cohes.on_changed(update_sliders)
    s_sep.on_changed(update_sliders)

    def init():
        agent_dots.set_data([], [])
        return agent_dots,

    # Infinite simulation — continues until user closes window or interrupts
    frame_count = [0]
    def update(_):
        nonlocal positions, velocities, leader_pos, leader_vel
        frame_count[0] += 1
        t = frame_count[0] * params["DT"]

        # Load external updates from JSON (if modified by another script)
        external = load_json_params()
        for key in ["W_ALIGNMENT", "W_COHESION", "W_SEPARATION"]:
            if key in external:
                params[key] = external[key]
                # also reflect updates in sliders
                if key == "W_ALIGNMENT":
                    s_align.set_val(external[key])
                elif key == "W_COHESION":
                    s_cohes.set_val(external[key])
                elif key == "W_SEPARATION":
                    s_sep.set_val(external[key])

        leader_pos, leader_vel = leader_update_func(leader_pos, leader_vel, params["DT"], t)
        positions, velocities = update_agents(positions, velocities, leader_pos, leader_vel, params)
        agent_dots.set_data(positions[:, 0], positions[:, 1])

        # --- CAMERA FOLLOWING ---
        swarm_center = positions.mean(axis=0)

        # smoothing helps reduce jitter
        if not hasattr(update, "cam"):
            update.cam = swarm_center.copy()
        update.cam = 0.9 * update.cam + 0.1 * swarm_center

        cam_x, cam_y = update.cam

        if not np.isfinite(cam_x):
            cam_x = 0
        if not np.isfinite(cam_y):
            cam_y = 0

        view_width  = 25   # half-width of view window
        view_height = 10   # half-height

        ax.set_xlim(cam_x - view_width, cam_x + view_width)
        ax.set_ylim(cam_y - view_height, cam_y + view_height)

        return agent_dots,

    ani = FuncAnimation(fig, update, init_func=init, interval=30, blit=False)
    plt.show()

# ----------------------------
# Main
# ----------------------------
def main():
    params = get_params()
    leader_func = waypoint_leader_live(params, speed=1.0)
    run_simulation(leader_func, params)
    # run_simulation(straight_line_leader, params)

if __name__ == "__main__":
    main()