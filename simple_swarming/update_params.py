import json

def update_swarm_mode(command: str, path: str = "params.json"):
    """
    Updates swarm behavior OR appends/clears leader waypoints in params.json.
    
    Commands:
        - Swarm modes: "tight", "dispersed", "relaxed", "aggressive", "chaotic"
        - Add waypoint: "waypoint X Y"
        - Clear waypoints: "clear"
    """
    command = command.lower().strip()

    # -------------------------------
    # 1. Handle waypoint append
    # -------------------------------
    if command.startswith("waypoint"):
        parts = command.split()
        if len(parts) != 3:
            print("Waypoint command format:  waypoint X Y")
            return
        
        try:
            x = float(parts[1])
            y = float(parts[2])
        except ValueError:
            print("Invalid waypoint coordinates.")
            return

        try:
            # Load existing file
            with open(path, "r") as f:
                params = json.load(f)
        except:
            params = {}

        # Ensure WAYPOINTS list exists
        if "WAYPOINTS" not in params:
            params["WAYPOINTS"] = []

        # Append new waypoint
        params["WAYPOINTS"].append([x, y])
        print(f"Appending waypoint: [{x}, {y}]")

        # Write back to file
        with open(path, "w") as f:
            json.dump(params, f, indent=4)
        print(f"Waypoint saved to {path}")
        return

    # -------------------------------
    # 2. Handle clearing waypoints
    # -------------------------------
    if command == "clear":
        try:
            with open(path, "r") as f:
                params = json.load(f)
        except:
            params = {}

        params["WAYPOINTS"] = []
        with open(path, "w") as f:
            json.dump(params, f, indent=4)
        print("All waypoints cleared.")
        return

    # -------------------------------
    # 3. Handle swarm mode updates
    # -------------------------------
    mode_map = {
        "tight":       {"W_ALIGNMENT": 0.1,  "W_COHESION": 0.15, "W_SEPARATION": 0.1},
        "dispersed":   {"W_ALIGNMENT": 0.05, "W_COHESION": 0.02, "W_SEPARATION": 0.8},
        "relaxed":     {"W_ALIGNMENT": 0.05, "W_COHESION": 0.05, "W_SEPARATION": 0.3},
        "aggressive":  {"W_ALIGNMENT": 0.15, "W_COHESION": 0.2,  "W_SEPARATION": 0.05},
        "chaotic":     {"W_ALIGNMENT": 0.01, "W_COHESION": 0.01, "W_SEPARATION": 1.0}
    }

    if command in mode_map:
        new_params = mode_map[command]
        print(f"Setting swarm mode to '{command}': {new_params}")
    else:
        print(f"Unknown command '{command}'. No changes made.")
        return

    # Load full existing params
    try:
        with open(path, "r") as f:
            params = json.load(f)
    except:
        params = {}

    # Update only swarm mode keys, preserve others (like WAYPOINTS)
    params.update(new_params)

    # Save
    with open(path, "w") as f:
        json.dump(params, f, indent=4)
    print(f"Parameters updated and saved to {path}")


# -------------------------------
# Example usage
# -------------------------------
if __name__ == "__main__":
    user_input = input("Enter command (swarm mode, waypoint X Y, or clear): ")
    update_swarm_mode(user_input)
