"""
Enhanced swarm management with individual drone control, multi-group coordination,
and dynamic behavior modes for LLM-driven commands.
"""

import numpy as np
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Any
from enum import Enum


class DroneMode(Enum):
    """Operating modes for individual drones."""
    SWARM = "swarm"              # Follow swarm control laws with group
    NAVIGATE = "navigate"        # Independent waypoint navigation
    LOITER = "loiter"           # Hover at current/specified position
    REJOIN = "rejoin"           # Transitioning back to swarm


@dataclass
class Drone:
    """Individual drone state and configuration."""
    id: int
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    mode: DroneMode = DroneMode.SWARM
    group_id: str = "main"
    waypoints: List[np.ndarray] = field(default_factory=list)
    current_waypoint_idx: int = 0
    waypoint_tolerance: float = 5.0
    
    # Loiter state
    loiter_position: Optional[np.ndarray] = None
    loiter_start_time: Optional[float] = None
    loiter_duration: float = 0.0
    
    # Rejoin state
    rejoin_target_group: Optional[str] = None
    rejoin_rendezvous: Optional[np.ndarray] = None
    rejoin_blend_factor: float = 0.0  # 0=independent, 1=full swarm


@dataclass
class DroneGroup:
    """Collection of drones operating as a coordinated unit."""
    group_id: str
    drone_ids: List[int]
    goal: np.ndarray = field(default_factory=lambda: np.zeros(3))
    Ls: float = 15.0  # Desired separation distance
    
    def get_positions(self, drones: Dict[int, Drone]) -> np.ndarray:
        """Get positions of all drones in this group."""
        positions = []
        for drone_id in self.drone_ids:
            if drone_id in drones:
                positions.append(drones[drone_id].position)
        return np.array(positions) if positions else np.zeros((0, 3))
    
    def get_velocities(self, drones: Dict[int, Drone]) -> np.ndarray:
        """Get velocities of all drones in this group."""
        velocities = []
        for drone_id in self.drone_ids:
            if drone_id in drones:
                velocities.append(drones[drone_id].velocity)
        return np.array(velocities) if velocities else np.zeros((0, 3))


class SwarmManager:
    """
    Manages multiple drone groups with individual control capabilities.
    Enables dynamic splitting, rejoining, and mode transitions.
    """
    
    def __init__(self, n_drones: int, P0: np.ndarray, V0: np.ndarray, 
                 goal: np.ndarray, Ls: float = 15.0):
        """
        Initialize swarm with all drones in main group.
        
        Parameters
        ----------
        n_drones : int
            Number of drones
        P0 : (n, 3) ndarray
            Initial positions
        V0 : (n, 3) ndarray
            Initial velocities
        goal : (3,) ndarray
            Initial goal position
        Ls : float
            Desired separation distance
        """
        self.drones: Dict[int, Drone] = {}
        self.groups: Dict[str, DroneGroup] = {}
        
        # Initialize all drones in main group
        for i in range(n_drones):
            self.drones[i] = Drone(
                id=i,
                position=P0[i].copy(),
                velocity=V0[i].copy(),
                mode=DroneMode.SWARM,
                group_id="main"
            )
        
        self.groups["main"] = DroneGroup(
            group_id="main",
            drone_ids=list(range(n_drones)),
            goal=goal.copy(),
            Ls=Ls
        )
    
    def split_drones(self, drone_ids: List[int], new_group_id: str, 
                     new_goal: np.ndarray, Ls: Optional[float] = None):
        """
        Split specified drones into a new group.
        
        Parameters
        ----------
        drone_ids : list of int
            Drone IDs to split off
        new_group_id : str
            Name for new group
        new_goal : (3,) ndarray
            Goal for new group
        Ls : float, optional
            Separation distance for new group
        """
        if new_group_id in self.groups:
            raise ValueError(f"Group '{new_group_id}' already exists")
        
        # Remove drones from their current groups
        for drone_id in drone_ids:
            if drone_id not in self.drones:
                continue
            
            drone = self.drones[drone_id]
            old_group = self.groups.get(drone.group_id)
            if old_group and drone_id in old_group.drone_ids:
                old_group.drone_ids.remove(drone_id)
            
            drone.group_id = new_group_id
        
        # Create new group
        if Ls is None:
            Ls = self.groups["main"].Ls
        
        self.groups[new_group_id] = DroneGroup(
            group_id=new_group_id,
            drone_ids=drone_ids,
            goal=new_goal.copy(),
            Ls=Ls
        )
    
    def assign_waypoints(self, drone_id: int, waypoints: List[np.ndarray], 
                        mode: DroneMode = DroneMode.NAVIGATE):
        """
        Assign waypoint sequence to a drone.
        
        Parameters
        ----------
        drone_id : int
            Target drone ID
        waypoints : list of (3,) ndarrays
            Waypoint positions
        mode : DroneMode
            Navigation mode (NAVIGATE for independent, SWARM for formation)
        """
        if drone_id not in self.drones:
            raise ValueError(f"Drone {drone_id} not found")
        
        drone = self.drones[drone_id]
        drone.waypoints = [np.array(wp) for wp in waypoints]
        drone.current_waypoint_idx = 0
        drone.mode = mode
    
    def set_loiter(self, drone_ids: List[int], duration: float, 
                   position: Optional[np.ndarray] = None, current_time: float = 0.0):
        """
        Command drones to loiter/hover.
        
        Parameters
        ----------
        drone_ids : list of int
            Drones to put in loiter mode
        duration : float
            Loiter duration in seconds
        position : (3,) ndarray, optional
            Loiter position (uses current position if None)
        current_time : float
            Current simulation time
        """
        for drone_id in drone_ids:
            if drone_id not in self.drones:
                continue
            
            drone = self.drones[drone_id]
            drone.mode = DroneMode.LOITER
            drone.loiter_start_time = current_time
            drone.loiter_duration = duration
            drone.loiter_position = position.copy() if position is not None else drone.position.copy()
    
    def initiate_rejoin(self, drone_ids: List[int], target_group_id: str,
                       rendezvous_point: Optional[np.ndarray] = None):
        """
        Begin rejoin sequence for drones to merge back into target group.
        
        Parameters
        ----------
        drone_ids : list of int
            Drones to rejoin
        target_group_id : str
            Target group to join
        rendezvous_point : (3,) ndarray, optional
            Meeting point (uses target group goal if None)
        """
        if target_group_id not in self.groups:
            raise ValueError(f"Target group '{target_group_id}' not found")
        
        target_group = self.groups[target_group_id]
        
        for drone_id in drone_ids:
            if drone_id not in self.drones:
                continue
            
            drone = self.drones[drone_id]
            drone.mode = DroneMode.REJOIN
            drone.rejoin_target_group = target_group_id
            drone.rejoin_rendezvous = (rendezvous_point.copy() 
                                       if rendezvous_point is not None 
                                       else target_group.goal.copy())
            drone.rejoin_blend_factor = 0.0
            # Navigate toward rendezvous using waypoint guidance
            drone.waypoints = [drone.rejoin_rendezvous.copy()]
            drone.current_waypoint_idx = 0
    
    def update_rejoin_progress(self, drone_id: int, blend_factor: float):
        """Update blend factor for rejoining drone (0=independent, 1=swarm)."""
        if drone_id in self.drones:
            self.drones[drone_id].rejoin_blend_factor = np.clip(blend_factor, 0.0, 1.0)
    
    def complete_rejoin(self, drone_id: int):
        """Complete rejoin sequence and merge drone back into target group."""
        if drone_id not in self.drones:
            return
        
        drone = self.drones[drone_id]
        if drone.mode != DroneMode.REJOIN or drone.rejoin_target_group is None:
            return
        
        target_group_id = drone.rejoin_target_group
        
        # Remove from current group
        if drone.group_id in self.groups:
            old_group = self.groups[drone.group_id]
            if drone_id in old_group.drone_ids:
                old_group.drone_ids.remove(drone_id)
        
        # Add to target group
        drone.group_id = target_group_id
        drone.mode = DroneMode.SWARM
        drone.rejoin_target_group = None
        drone.rejoin_rendezvous = None
        drone.rejoin_blend_factor = 0.0
        
        if target_group_id in self.groups and drone_id not in self.groups[target_group_id].drone_ids:
            self.groups[target_group_id].drone_ids.append(drone_id)
    
    def update_loiter_state(self, current_time: float):
        """Check loiter timers and transition drones back to navigation."""
        for drone in self.drones.values():
            if drone.mode == DroneMode.LOITER and drone.loiter_start_time is not None:
                elapsed = current_time - drone.loiter_start_time
                if elapsed >= drone.loiter_duration:
                    # Loiter complete - transition to appropriate mode
                    drone.mode = DroneMode.SWARM  # Default back to swarm
                    drone.loiter_start_time = None
                    drone.loiter_position = None
    
    def check_waypoint_progress(self):
        """Check if drones have reached waypoints and advance to next."""
        for drone in self.drones.values():
            if drone.mode not in [DroneMode.NAVIGATE, DroneMode.SWARM, DroneMode.REJOIN] or not drone.waypoints:
                continue
            
            if drone.current_waypoint_idx >= len(drone.waypoints):
                continue
            
            current_wp = drone.waypoints[drone.current_waypoint_idx]
            dist = np.linalg.norm(drone.position - current_wp)
            
            if dist <= drone.waypoint_tolerance:
                drone.current_waypoint_idx += 1
                if drone.mode == DroneMode.REJOIN and drone.current_waypoint_idx >= len(drone.waypoints):
                    self.complete_rejoin(drone.id)
    
    def get_state_arrays(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get current positions and velocities as arrays.
        
        Returns
        -------
        P : (n, 3) ndarray
            Positions
        V : (n, 3) ndarray
            Velocities
        """
        n = len(self.drones)
        P = np.zeros((n, 3))
        V = np.zeros((n, 3))
        
        for i in range(n):
            if i in self.drones:
                P[i] = self.drones[i].position
                V[i] = self.drones[i].velocity
        
        return P, V
    
    def set_state_arrays(self, P: np.ndarray, V: np.ndarray):
        """Update drone states from position and velocity arrays."""
        for i in range(min(len(self.drones), P.shape[0])):
            if i in self.drones:
                self.drones[i].position = P[i].copy()
                self.drones[i].velocity = V[i].copy()
