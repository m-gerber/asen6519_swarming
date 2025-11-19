import numpy as np
import json
from typing import Dict, List, Tuple

# ----------------------------
# Helper functions
# ----------------------------
def load_json_params(path="params.json") -> Dict:
    try:
        with open(path, "r") as f:
            return json.load(f)
    except Exception:
        return {}

# ----------------------------
# Swarm update function
# ----------------------------
def update_swarm(positions: np.ndarray, velocities: np.ndarray,
                 leader_pos: np.ndarray, leader_vel: np.ndarray,
                 params_path="params.json") -> Tuple[np.ndarray, np.ndarray]:
    """
    Computes new velocities and positions for all agents.
    
    Args:
        positions: Nx2 array of current positions.
        velocities: Nx2 array of current velocities.
        leader_pos: 2D array.
        leader_vel: 2D array.
        params_path: path to JSON params.
    
    Returns:
        new_positions, new_velocities: updated arrays
    """
    # Load params
    params = load_json_params(params_path)

    # Fallback if keys missing
    W_ALIGNMENT = params.get("W_ALIGNMENT", 0.05)
    W_COHESION = params.get("W_COHESION", 0.05)
    W_SEPARATION = params.get("W_SEPARATION", 0.5)
    NEIGHBOR_RADIUS = params.get("NEIGHBOR_RADIUS", 3.0)
    MIN_DIST = params.get("MIN_DIST", 2.0)
    VELOCITY_DAMPING = params.get("VELOCITY_DAMPING", 0.95)
    N_AGENTS = positions.shape[0]

    new_positions = np.copy(positions)
    new_velocities = np.copy(velocities)

    for i in range(N_AGENTS):
        diffs = positions - positions[i]
        distances = np.linalg.norm(diffs, axis=1)
        neighbors = (distances < NEIGHBOR_RADIUS) & (distances > 0)

        # Cohesion
        cohesion_vec = np.zeros(2)
        if np.any(neighbors):
            center_of_mass = np.mean(positions[neighbors], axis=0)
            cohesion_vec = (center_of_mass - positions[i]) * W_COHESION

        # Alignment
        alignment_vec = np.zeros(2)
        if np.any(neighbors):
            mean_velocity = np.mean(velocities[neighbors], axis=0)
            alignment_vec = (mean_velocity - velocities[i]) * W_ALIGNMENT

        # Separation
        separation_vec = np.zeros(2)
        too_close = (distances < MIN_DIST) & (distances > 0)
        if np.any(too_close):
            separation_vec = np.sum(-diffs[too_close] / (distances[too_close][:, np.newaxis] ** 2), axis=0)
            separation_vec *= W_SEPARATION

        # Leader following (simple)
        leader_follow_vec = (leader_pos - positions[i]) * 0.3

        # Update velocity
        acceleration = cohesion_vec + alignment_vec + separation_vec + leader_follow_vec
        new_velocities[i] += acceleration * 0.1  # assuming DT=0.1
        new_velocities[i] *= VELOCITY_DAMPING
        new_positions[i] += new_velocities[i] * 0.1

    return new_positions, new_velocities
