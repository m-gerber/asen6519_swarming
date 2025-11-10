import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.widgets import Slider
from typing import Callable, Tuple, Dict

import json, time

# ----------------------------
# Helper functions
# ----------------------------
def get_params() -> Dict:
    """Return all simulation parameters in a dictionary."""
    return {
        "N_AGENTS": 15,
        "DT": 0.1,
        "STEPS": 1_000,
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

        # Leader influence (still present, just not drawn)
        to_leader = leader_pos - positions[i]
        if np.linalg.norm(to_leader) > 0:
            behind_pos = leader_pos - (leader_vel / np.linalg.norm(leader_vel)) * 2.0
            leader_follow_vec = (behind_pos - positions[i]) * params["W_LEADER_FOLLOW"]
        else:
            leader_follow_vec = np.zeros(2)

        acceleration = cohesion_vec + alignment_vec + separation_vec + leader_follow_vec
        new_velocities[i] += acceleration * params["DT"]
        new_velocities[i] *= params["VELOCITY_DAMPING"]
        new_positions[i] += new_velocities[i] * params["DT"]

    return new_positions, new_velocities


# ----------------------------
# Leader path (virtual)
# ----------------------------
def straight_line_leader(pos, vel, dt, t=0.0, total_time=40.0):
    halfway = total_time / 2.0
    if t < halfway:
        new_pos = pos + vel * dt
    else:
        new_pos = pos - vel * dt
    return new_pos, vel


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
    ax.set_xlim(-10, 25)
    ax.set_ylim(-7.5, 7.5)
    ax.set_title("Swarming Simulation (Virtual Leader)")
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

    s_align = Slider(ax_align, 'Alignment', 0.0, 0.2, valinit=params["W_ALIGNMENT"])
    s_cohes = Slider(ax_cohes, 'Cohesion', 0.0, 0.2, valinit=params["W_COHESION"])
    s_sep = Slider(ax_sep, 'Separation', 0.0, 1.0, valinit=params["W_SEPARATION"])

    # --- Update sliders dynamically ---
    def update_sliders(val):
        params["W_ALIGNMENT"] = s_align.val
        params["W_COHESION"] = s_cohes.val
        params["W_SEPARATION"] = s_sep.val

    s_align.on_changed(update_sliders)
    s_cohes.on_changed(update_sliders)
    s_sep.on_changed(update_sliders)

    def init():
        agent_dots.set_data([], [])
        return agent_dots,

    
    def update(frame):
        external = load_json_params()
        for key in ["W_ALIGNMENT", "W_COHESION", "W_SEPARATION"]:
            if key in external:
                params[key] = external[key]
            nonlocal positions, velocities, leader_pos, leader_vel
        t = frame * params["DT"]
        leader_pos, leader_vel = leader_update_func(leader_pos, leader_vel, params["DT"], t, params["STEPS"] * params["DT"])
        positions, velocities = update_agents(positions, velocities, leader_pos, leader_vel, params)
        agent_dots.set_data(positions[:, 0], positions[:, 1])
        return agent_dots,

    ani = FuncAnimation(fig, update, frames=params["STEPS"], init_func=init, interval=30, blit=True, repeat=False)
    plt.show()


# ----------------------------
# Main
# ----------------------------
def main():
    params = get_params()
    run_simulation(straight_line_leader, params)

if __name__ == "__main__":
    main()
