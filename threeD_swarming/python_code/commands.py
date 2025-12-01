import json
import os
from typing import Any, Dict, Optional, Tuple

import numpy as np


def load_command(path: str) -> Optional[Dict[str, Any]]:
    """
    Load a JSON command from disk.

    Returns
    -------
    dict or None
        Parsed command dictionary, or None if the file does not exist
        or cannot be parsed into a dict.
    """
    if not os.path.exists(path):
        return None

    try:
        with open(path, "r") as f:
            data = json.load(f)
    except Exception:
        # If the file is being written / corrupted / not valid JSON, ignore for now
        return None

    if not isinstance(data, dict):
        return None

    return data


def check_for_new_command(
    path: str,
    last_command_id: int,
) -> Tuple[Optional[Dict[str, Any]], int]:
    """
    Check for a new command in the given JSON file.

    Parameters
    ----------
    path : str
        Path to the JSON command file.
    last_command_id : int
        ID of the last command that was applied.

    Returns
    -------
    (cmd, new_last_command_id) : (dict or None, int)
        cmd is the new command dict if a new command was found and passes
        validation, otherwise None. new_last_command_id is updated accordingly.
    """
    cmd = load_command(path)
    if cmd is None:
        return None, last_command_id

    try:
        cmd_id = int(cmd.get("command_id", 0))
    except Exception:
        return None, last_command_id

    if cmd_id <= last_command_id:
        # Old or duplicate command
        return None, last_command_id

    return cmd, cmd_id


def apply_command_to_sim(
    cmd: Dict[str, Any],
    goal: np.ndarray,
    params: Any,
    obstacles: Any,
    swarm_manager: Any = None,
    current_time: float = 0.0,
) -> tuple[np.ndarray, Any, Any]:
    """
    Apply a high-level command to the current simulation configuration.

    Supported command types:
    
    1. 'update_goal': Change swarm destination
        {
          "command_id": 1,
          "command_type": "update_goal",
          "new_goal_m": [x, y, z],
          "reason": "...",
          "timestamp": "..."
        }
    
    2. 'split_drones': Split drones into new group
        {
          "command_id": 2,
          "command_type": "split_drones",
          "drone_ids": [0, 1],
          "new_group_id": "alpha",
          "new_goal_m": [x, y, z],
          "Ls": 15.0  (optional)
        }
    
    3. 'assign_waypoints': Assign waypoint sequence to drone
        {
          "command_id": 3,
          "command_type": "assign_waypoints",
          "drone_id": 2,
          "waypoints": [[x1, y1, z1], [x2, y2, z2], ...],
          "mode": "navigate"  (or "swarm")
        }
    
    4. 'loiter': Command drones to hover
        {
          "command_id": 4,
          "command_type": "loiter",
          "drone_ids": [0, 1],
          "duration_s": 30.0,
          "position": [x, y, z]  (optional)
        }
    
    5. 'rejoin_swarm': Rejoin drones to target group
        {
          "command_id": 5,
          "command_type": "rejoin_swarm",
          "drone_ids": [0, 1],
          "target_group_id": "main",
          "rendezvous_point": [x, y, z]  (optional)
        }
    
    6. 'add_obstacle': Add obstacle mid-flight
        {
          "command_id": 6,
          "command_type": "add_obstacle",
          "obstacle": {
            "type": "cylinder",
            "xy": [100.0, 150.0],
            "radius": 10.0,
            "zmin": 0.0,
            "zmax": 80.0
          }
        }
    
    7. 'update_group_goal': Change goal for specific group
        {
          "command_id": 7,
          "command_type": "update_group_goal",
          "group_id": "alpha",
          "new_goal_m": [x, y, z]
        }

    Parameters
    ----------
    cmd : dict
        Parsed command dict.
    goal : np.ndarray
        Current goal position (3,) for main swarm.
    params : Any
        Controller parameters object (e.g. SwarmControlParams).
    obstacles : list
        Current obstacles structure.
    swarm_manager : SwarmManager, optional
        Swarm manager instance for multi-group operations.
    current_time : float
        Current simulation time.

    Returns
    -------
    (goal, params, obstacles)
        Updated versions after applying the command.
    """
    cmd_type = str(cmd.get("command_type", "")).lower()

    if cmd_type == "update_goal":
        if "new_goal_m" not in cmd:
            return goal, params, obstacles

        new_goal = np.asarray(cmd["new_goal_m"], dtype=float).ravel()
        if new_goal.size != 3:
            # Invalid shape; ignore
            return goal, params, obstacles

        # Update main group goal
        if swarm_manager is not None and "main" in swarm_manager.groups:
            swarm_manager.set_group_goal("main", new_goal.copy())
        
        return new_goal, params, obstacles
    
    elif cmd_type == "split_drones":
        if swarm_manager is None:
            return goal, params, obstacles
        
        drone_ids = cmd.get("drone_ids", [])
        new_group_id = cmd.get("new_group_id", "")
        new_goal_m = cmd.get("new_goal_m", None)
        Ls = cmd.get("Ls", None)
        
        if not drone_ids or not new_group_id or new_goal_m is None:
            return goal, params, obstacles
        
        new_goal = np.asarray(new_goal_m, dtype=float).ravel()
        if new_goal.size != 3:
            return goal, params, obstacles
        
        try:
            swarm_manager.split_drones(drone_ids, new_group_id, new_goal, Ls)
            print(f"Split drones {drone_ids} into group '{new_group_id}' heading to {new_goal}")
        except Exception as e:
            print(f"Failed to split drones: {e}")
        
        return goal, params, obstacles
    
    elif cmd_type == "assign_waypoints":
        if swarm_manager is None:
            return goal, params, obstacles
        
        drone_id = cmd.get("drone_id", None)
        waypoints = cmd.get("waypoints", [])
        mode_str = cmd.get("mode", "navigate").lower()
        
        if drone_id is None or not waypoints:
            return goal, params, obstacles
        
        # Import DroneMode here to avoid circular import
        from swarm_manager import DroneMode
        mode = DroneMode.NAVIGATE if mode_str == "navigate" else DroneMode.SWARM
        
        try:
            swarm_manager.assign_waypoints(drone_id, waypoints, mode)
            print(f"Assigned {len(waypoints)} waypoints to drone {drone_id} in {mode_str} mode")
        except Exception as e:
            print(f"Failed to assign waypoints: {e}")
        
        return goal, params, obstacles
    
    elif cmd_type == "loiter":
        if swarm_manager is None:
            return goal, params, obstacles
        
        drone_ids = cmd.get("drone_ids", [])
        duration_s = cmd.get("duration_s", 0.0)
        position = cmd.get("position", None)
        
        if not drone_ids or duration_s <= 0:
            return goal, params, obstacles
        
        loiter_pos = np.asarray(position, dtype=float).ravel() if position is not None else None
        
        try:
            swarm_manager.set_loiter(drone_ids, duration_s, loiter_pos, current_time)
            print(f"Drones {drone_ids} entering loiter mode for {duration_s}s")
        except Exception as e:
            print(f"Failed to set loiter: {e}")
        
        return goal, params, obstacles
    
    elif cmd_type == "rejoin_swarm":
        if swarm_manager is None:
            return goal, params, obstacles
        
        drone_ids = cmd.get("drone_ids", [])
        target_group_id = cmd.get("target_group_id", "main")
        rendezvous_point = cmd.get("rendezvous_point", None)
        
        if not drone_ids:
            return goal, params, obstacles
        
        rdv = np.asarray(rendezvous_point, dtype=float).ravel() if rendezvous_point is not None else None
        
        try:
            swarm_manager.initiate_rejoin(drone_ids, target_group_id, rdv)
            print(f"Drones {drone_ids} rejoining group '{target_group_id}'")
        except Exception as e:
            print(f"Failed to initiate rejoin: {e}")
        
        return goal, params, obstacles
    
    elif cmd_type == "add_obstacle":
        obstacle = cmd.get("obstacle", None)
        if obstacle is None or not isinstance(obstacle, dict):
            return goal, params, obstacles
        
        # Validate obstacle structure
        obs_type = obstacle.get("type", "").lower()
        if obs_type == "cylinder":
            required = ["xy", "radius", "zmin", "zmax"]
            if all(k in obstacle for k in required):
                obstacles.append(obstacle)
                print(f"Added cylinder obstacle at {obstacle['xy']}")
        elif obs_type == "wall":
            required = ["center", "normal", "width", "height"]
            if all(k in obstacle for k in required):
                obstacles.append(obstacle)
                print(f"Added wall obstacle at {obstacle['center']}")
        
        return goal, params, obstacles
    
    elif cmd_type == "update_group_goal":
        if swarm_manager is None:
            return goal, params, obstacles
        
        group_id = cmd.get("group_id", "")
        new_goal_m = cmd.get("new_goal_m", None)
        
        if not group_id or new_goal_m is None:
            return goal, params, obstacles
        
        new_goal = np.asarray(new_goal_m, dtype=float).ravel()
        if new_goal.size != 3:
            return goal, params, obstacles
        
        if group_id in swarm_manager.groups:
            swarm_manager.set_group_goal(group_id, new_goal.copy())
            print(f"Updated goal for group '{group_id}' to {new_goal}")
            
            # If it's main group, also update the main goal
            if group_id == "main":
                goal = new_goal
        
        return goal, params, obstacles

    # Unknown / unsupported command type – ignore
    return goal, params, obstacles
