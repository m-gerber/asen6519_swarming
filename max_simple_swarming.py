import numpy as np
import matplotlib.pyplot as plt


def square_formation(num_agents, spacing=1.0, lead_pos=(0, 0)):
    """
    Compute positions for agents forming a square grid behind a lead agent.

    Args:
        num_agents (int): number of agents (including the lead)
        spacing (float): distance between rows/columns
        lead_pos (tuple): (x, y) position of the lead agent

    Returns:
        np.ndarray: positions of agents as [[x1, y1], [x2, y2], ...]
    """
    n_per_row = int(np.ceil(np.sqrt(num_agents-1)))  # number of agents per row
    positions = [(0,0)]

    # Start placing agents behind the lead (negative y-direction)
    for i in range(num_agents-1):
        row = i // n_per_row
        col = i % n_per_row

        # Center the formation horizontally
        x = (col - (n_per_row - 1) / 2) * spacing
        y = -(row + 1) * spacing
        positions.append((lead_pos[0] + x, lead_pos[1] + y))

    return np.array(positions)


def run_swarming_simulation(num_agents: int, plot: bool = False) -> None:
    """
    Propogate swarm positions.

    Args:
        num_agents (int): number of agents (excluding the lead)
        plot (bool): plot graphics for the swarm if True

    Returns:
        None
    """
    agent_positions = square_formation(num_agents, spacing=1.0, lead_pos=(0, 0))

    if plot:
        plt.figure()
        for i, pos in enumerate(agent_positions):
            plt.plot(pos[0], pos[1], 'bo')
        plt.plot(0, 0, 'ro')
        plt.text(0, 0, 'Lead Agent', fontsize=10, ha='right', color='r')
        plt.title('Agent Positions in Square Formation')
        plt.xlabel('X Position')
        plt.ylabel('Y Position')
        plt.axis('equal')
        plt.grid(True)
        plt.show()

    return 0


if __name__ == "__main__":
    run_swarming_simulation(num_agents=5, plot=False)