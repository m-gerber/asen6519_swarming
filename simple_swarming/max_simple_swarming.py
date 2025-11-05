import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from typing import Callable, Tuple, Dict

# ----------------------------
# Helper functions
# ----------------------------
def get_params() -> Dict:
    """Return all simulation parameters in a dictionary."""
    return {
        # Agents
        "N_AGENTS": 15,
        "DT": 0.1,
        "STEPS": 400,
        "NEIGHBOR_RADIUS": 3.0,
        "MIN_DIST": 2.0,
        "VELOCITY_DAMPING": 0.95,

        # Leader
        "LEADER_INIT_POS": np.array([0.0, 0.0]),
        "LEADER_INIT_VEL": np.array([0.5, 0.0]),

        # Swarm behavior weights
        "W_ALIGNMENT": 0.05,
        "W_COHESION": 0.05,
        "W_SEPARATION": 0.5,
        "W_LEADER_AVOID": 0.5,
        "W_LEADER_FOLLOW": 0.3
    }


def limit_vector(v: np.ndarray, max_val: float = 1.0) -> np.ndarray:
    """Limit the magnitude of a vector."""
    norm = np.linalg.norm(v)
    return v if norm < max_val else v / norm * max_val


def update_agents(
    positions: np.ndarray,
    velocities: np.ndarray,
    leader_pos: np.ndarray,
    leader_vel: np.ndarray,
    params: Dict
) -> Tuple[np.ndarray, np.ndarray]:
    """Update positions and velocities of agents based on swarm rules."""
    new_positions = np.copy(positions)
    new_velocities = np.copy(velocities)

    N_AGENTS = params["N_AGENTS"]
    neighbor_radius = params["NEIGHBOR_RADIUS"]
    min_dist = params["MIN_DIST"]

    for i in range(N_AGENTS):
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

        # Leader interactions
        to_leader = leader_pos - positions[i]
        dist_to_leader = np.linalg.norm(to_leader)
        leader_avoid_vec = np.zeros(2)
        if dist_to_leader < 1.0:
            leader_avoid_vec = -to_leader / dist_to_leader * (1.0 - dist_to_leader) * params["W_LEADER_AVOID"]

        leader_follow_vec = np.zeros(2)
        if np.linalg.norm(leader_vel) > 0:
            behind_pos = leader_pos - (leader_vel / np.linalg.norm(leader_vel)) * 2.0
            leader_follow_vec = (behind_pos - positions[i]) * params["W_LEADER_FOLLOW"]

        # Combine influences
        acceleration = cohesion_vec + alignment_vec + separation_vec + leader_avoid_vec + leader_follow_vec

        # Update velocities and positions
        new_velocities[i] += acceleration * params["DT"]
        new_velocities[i] *= params["VELOCITY_DAMPING"]
        new_positions[i] += new_velocities[i] * params["DT"]

    return new_positions, new_velocities


# ----------------------------
# Leader paths
# ----------------------------
def straight_line_leader(pos: np.ndarray, vel: np.ndarray, dt: float) -> Tuple[np.ndarray, np.ndarray]:
    return pos + vel * dt, vel

def sinusoidal_leader(pos: np.ndarray, vel: np.ndarray, dt: float, amplitude: float = 2.0, frequency: float = 0.5) -> Tuple[np.ndarray, np.ndarray]:
    new_pos = pos + vel * dt
    new_pos[1] = amplitude * np.sin(frequency * new_pos[0])
    return new_pos, vel


# ----------------------------
# Simulation and animation
# ----------------------------
def run_simulation(leader_update_func: Callable[[np.ndarray, np.ndarray, float], Tuple[np.ndarray, np.ndarray]], params: Dict):
    # Initialize positions and velocities
    np.random.seed(7)
    leader_pos = params["LEADER_INIT_POS"].copy()
    leader_vel = params["LEADER_INIT_VEL"].copy()
    # Distance behind the leader
    behind_distance = 2.5  # 1 unit behind
    spread_x = 2.5          # small x spread
    spread_y = 5.0          # small y spread

    positions = np.zeros((params["N_AGENTS"], 2))
    positions[:, 0] = leader_pos[0] - behind_distance + np.random.uniform(-spread_x, spread_x, params["N_AGENTS"])
    positions[:, 1] = leader_pos[1] + np.random.uniform(-spread_y / 2, spread_y / 2, params["N_AGENTS"])

    velocities = np.zeros((params["N_AGENTS"], 2))

    # Set up plot
    fig, ax = plt.subplots(figsize=(12, 6))
    ax.set_xlim(-10, 25)
    ax.set_ylim(-7.5, 7.5)
    ax.set_title("Swarming Simulation (Alignment + Cohesion + Separation)")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_aspect('equal')
    ax.grid(True)

    agent_dots, = ax.plot([], [], 'bo', label="Agents")
    leader_dot, = ax.plot([], [], 'ro', label="Leader")
    ax.legend()

    def init():
        agent_dots.set_data([], [])
        leader_dot.set_data([], [])
        return agent_dots, leader_dot

    def update(frame):
        nonlocal positions, velocities, leader_pos, leader_vel

        # Update leader
        leader_pos, leader_vel = leader_update_func(leader_pos, leader_vel, params["DT"])

        # Update swarm
        positions, velocities = update_agents(positions, velocities, leader_pos, leader_vel, params)

        # Update plot
        agent_dots.set_data(positions[:, 0], positions[:, 1])
        leader_dot.set_data([leader_pos[0]], [leader_pos[1]])

        return agent_dots, leader_dot

    ani = FuncAnimation(fig, update, frames=params["STEPS"], init_func=init,
                        interval=30, blit=True, repeat=False)
    plt.show()


# ----------------------------
# Main function
# ----------------------------
def main():
    params = get_params()
    # Choose the leader path:
    leader_func = straight_line_leader
    leader_func = sinusoidal_leader
    run_simulation(leader_func, params)


if __name__ == "__main__":
    main()