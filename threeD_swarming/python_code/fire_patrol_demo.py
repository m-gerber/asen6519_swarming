"""
Fire Patrol Demonstration Script

Orchestrates the fire patrol scenario with timed LLM command injection:
1. Launch 5-drone swarm from Golden, CO toward Boulder, CO
2. At midpoint, trigger fire investigation command
3. Two drones break off to investigate reported fire
4. Breakoff drones encounter obstacle near fire site
5. Investigation drones loiter at fire location for sensor scan
6. After investigation, drones rejoin main swarm
7. All drones continue to Boulder landing zone
"""

import os
import yaml
import numpy as np

# Import simulation components
from demo_swarm_run import demo_swarm_run
from llm_interface import (
    create_fire_patrol_command,
    create_loiter_command,
    create_rejoin_command,
    LLMCommandGenerator
)
from animate_flight_paths import animate_flight_paths, animate_flight_paths_interactive


class FirePatrolOrchestrator:
    """
    Orchestrates fire patrol mission with timed command injection.
    """
    
    def __init__(self, config_path: str = "../inputs/fire_patrol_config.yaml"):
        """
        Initialize orchestrator with configuration.
        
        Parameters
        ----------
        config_path : str
            Path to fire patrol configuration YAML
        """
        self.config_path = config_path
        self.base_dir = os.path.dirname(os.path.abspath(__file__))
        self.config = self._load_config()
        self.command_sequence = []
        self.command_schedule = []
        # Resolve command file path once so threads & simulator share identical target
        raw_command_path = self.config.get('command_file_path', 'commands/fire_patrol_command.json')
        self.command_file_path = self._resolve_path(raw_command_path)
        
        # Initialize LLM interface
        self.llm_generator = LLMCommandGenerator()
        self._setup_llm_context()

    def _resolve_path(self, path_str: str) -> str:
        """Resolve relative paths against the python_code directory."""
        if path_str is None:
            return None
        if os.path.isabs(path_str):
            return path_str
        return os.path.normpath(os.path.join(self.base_dir, path_str))
    
    def _load_config(self) -> dict:
        """Load configuration from YAML file."""
        with open(self.config_path, 'r') as f:
            config = yaml.safe_load(f)
        return config
    
    def _setup_llm_context(self):
        """Set up LLM with scenario context."""
        mission_params = self.config.get('mission_params', {})
        
        context = {
            "mission_type": "fire_patrol",
            "mission_description": "Fire patrol along Rocky Mountain front range from Golden to Boulder, CO",
            "n_drones": self.config['n'],
            "start_location": {
                "name": "Golden, CO",
                "coordinates": [0, 0, 100]
            },
            "end_location": {
                "name": "Boulder, CO",
                "coordinates": self.config['goal_m']
            },
            "fire_location": {
                "name": "Reported fire site",
                "coordinates": self.config.get('fire_location_m', [10000, 3000, 50])
            },
            "available_drones": list(range(self.config['n'])),
            "investigation_team": mission_params.get('investigation_drone_ids', [0, 1]),
            "flight_speed_ms": 15.0,
            "altitude_m": 100.0
        }
        
        self.llm_generator.set_scenario_context(context)
    
    def prepare_command_sequence(self, use_llm: bool = False):
        """
        Prepare sequence of commands for the mission.
        
        Parameters
        ----------
        use_llm : bool
            If True, generate commands using LLM. If False, use pre-scripted commands.
        """
        mission_params = self.config.get('mission_params', {})

        # Ensure no stale command survives from previous runs
        self._clear_command_file()
        fire_location = self.config.get('fire_location_m', [10000, 3000, 50])
        investigation_drones = mission_params.get('investigation_drone_ids', [0, 1])
        loiter_duration = mission_params.get('investigation_loiter_duration_s', 3.0)
        rejoin_point = mission_params.get('rejoin_rendezvous_m', [15000, 0, 100])
        
        if use_llm and self.llm_generator.available:
            print("Generating commands using LLM...")
            
            # Command 1: Break off investigation team
            cmd1 = self.llm_generator.generate_command(
                f"Break off drones {investigation_drones} to investigate possible fire at coordinates "
                f"{fire_location[0]}, {fire_location[1]}, {fire_location[2]}",
                next_command_id=1
            )
            
            # Command 2: Loiter for investigation
            cmd2 = self.llm_generator.generate_command(
                f"Command investigation drones {investigation_drones} to hover at fire location "
                f"for {loiter_duration} seconds to scan for fire",
                next_command_id=2
            )
            
            # Command 3: Rejoin main swarm
            cmd3 = self.llm_generator.generate_command(
                f"After investigation complete, command drones {investigation_drones} to rejoin "
                f"main swarm at rendezvous point {rejoin_point}",
                next_command_id=3
            )
            
            self.command_sequence = [cmd1, cmd2, cmd3]
        
        else:
            print("Using pre-scripted commands...")
            
            # Pre-scripted commands for demo reliability
            cmd1 = create_fire_patrol_command(
                fire_location=fire_location,
                drone_ids=investigation_drones,
                command_id=1
            )
            
            cmd2 = create_loiter_command(
                drone_ids=investigation_drones,
                duration=loiter_duration,
                command_id=2
            )
            
            cmd3 = create_rejoin_command(
                drone_ids=investigation_drones,
                rendezvous=rejoin_point,
                command_id=3
            )
            
            self.command_sequence = [cmd1, cmd2, cmd3]
        
        print(f"Prepared {len(self.command_sequence)} commands for mission")
        for i, cmd in enumerate(self.command_sequence):
            print(f"  Command {i+1}: {cmd['command_type']} - {cmd.get('reason', 'N/A')}")
        self.command_schedule = self._build_command_schedule()

    def _clear_command_file(self):
        """Remove any leftover command so each mission starts clean."""
        if not self.command_file_path:
            return
        try:
            if os.path.exists(self.command_file_path):
                os.remove(self.command_file_path)
        except OSError as exc:
            print(f"Warning: unable to clear command file {self.command_file_path}: {exc}")

    def _build_command_schedule(self) -> list[dict]:
        """Construct timeline of scripted commands keyed to simulation time."""
        mission_params = self.config.get('mission_params', {})
        fire_alert_time = mission_params.get('fire_alert_time_s', 667.0)
        loiter_duration = mission_params.get('investigation_loiter_duration_s', 3.0)
        travel_time = mission_params.get('travel_time_to_fire_s', 60.0)
        rejoin_buffer = mission_params.get('rejoin_buffer_s', 5.0)

        timings = [
            fire_alert_time,
            fire_alert_time + travel_time,
            fire_alert_time + travel_time + loiter_duration + rejoin_buffer,
        ]

        schedule = []
        for trigger, cmd in zip(timings, self.command_sequence):
            schedule.append({
                "trigger_time_s": trigger,
                "command": cmd,
            })
        return schedule
    
    def run_simulation(self, use_interactive_viz: bool = True, 
                      inject_commands: bool = True):
        """
        Run the fire patrol simulation.
        
        Parameters
        ----------
        use_interactive_viz : bool
            If True, use interactive 3D viewer. If False, export video.
        inject_commands : bool
            If True, inject commands during simulation. If False, run without commands.
        """
        print("\n" + "="*60)
        print("FIRE PATROL MISSION: GOLDEN TO BOULDER")
        print("="*60)
        print(f"Mission: {self.config['n']} drones on fire watch patrol")
        print(f"Route: Golden, CO → Boulder, CO ({self.config['goal_m'][0]/1000:.1f} km)")
        print(f"Altitude: {self.config['goal_m'][2]} m AGL")
        print(f"Expected duration: ~{self.config['goal_m'][0]/15.0:.0f}s at 15 m/s")
        print("="*60 + "\n")
        
        # Setup obstacles including fire area obstacle
        obstacles = []
        
        # Add fire area obstacle from config
        fire_obstacle = self.config.get('fire_area_obstacle')
        if fire_obstacle:
            obstacles.append(fire_obstacle)
            print(f"Obstacle added near fire location: {fire_obstacle['type']} at {fire_obstacle['xy']}")
        
        # Obstacle parameters
        obs_params = {
            "k_o": 40.0,
            "d_safe": 150.0,  # Larger safety distance for demonstration
            "fmax": 10.0
        }
        
        # Print schedule for visibility
        if inject_commands and self.command_schedule:
            print(f"{'='*60}")
            print("COMMAND INJECTION SCHEDULE (SIM-TIMED)")
            print(f"{'='*60}")
            for event in self.command_schedule:
                cmd = event['command']
                trigger = event['trigger_time_s']
                print(f"  T+{trigger:6.1f}s: {cmd['command_type']} - {cmd.get('reason', 'N/A')}")
            print(f"{'='*60}\n")
        
        # Run simulation
        print("Starting simulation...\n")
        
        try:
            P, V, t = demo_swarm_run(
                n=self.config['n'],
                Ls=self.config['Ls_m'],
                goal=np.array(self.config['goal_m']),
                T=self.config['T_s'],
                dt=self.config['dt_s'],
                P0=np.array(self.config['initial_positions_m']),
                V0=np.array(self.config['initial_velocities_mps']),
                obstacles=obstacles,
                obs_params=obs_params,
                log_every_n_steps=self.config.get('log_every_n_steps', 10),
                command_file_path=self.command_file_path,
                command_check_every_n_steps=self.config.get('command_check_every_n_steps', 50),
                make_plot=False,  # Disable quick plot, use animation instead
                scheduled_command_events=self.command_schedule if inject_commands else None,
            )
            
            print("\nSimulation complete!")
            print(f"Total time simulated: {t[-1]:.1f}s")
            print(f"Final drone positions written to swarm_output/")
            
        except KeyboardInterrupt:
            print("\n\nSimulation interrupted by user")
            return
        except Exception as e:
            print(f"\n\nSimulation error: {e}")
            import traceback
            traceback.print_exc()
            return
        
        # Determine animation speed so playback <= 30 seconds (with floor for usability)
        sim_duration = float(t[-1]) if len(t) > 0 else self.config['T_s']
        target_runtime = 30.0
        min_speed = 3.0
        speed_up = max(min_speed, sim_duration / target_runtime)

        # Animate results
        print("\n" + "="*60)
        print("GENERATING VISUALIZATION")
        print("="*60 + "\n")
        print(f"Simulation duration {sim_duration:.1f}s → animation speed-up {speed_up:.1f}x (<= {target_runtime:.0f}s video)\n")
        
        data = {
            "csv_dir": os.path.join(os.getcwd(), "swarm_output"),
            "goal": np.array(self.config['goal_m']),
            "obstacles": obstacles,
            "obs_style": {
                "color": "red",
                "alpha": 0.4,
                "edgecolor": "darkred",
            },
        }
        
        try:
            if use_interactive_viz:
                print("Launching interactive 3D viewer...")
                print("  - Use mouse to rotate, pan, zoom")
                print("  - Close window when done\n")
                animate_flight_paths_interactive(speed_up=speed_up, data=data, repeat=True)
            else:
                print("Exporting animation video...")
                animate_flight_paths(speed_up=speed_up, data=data)
                print("Video exported to swarm_output/")
        
        except Exception as e:
            print(f"Visualization error: {e}")
            print("Trajectory data still available in swarm_output/")


def main():
    """Main entry point for fire patrol demo."""
    import argparse
    
    parser = argparse.ArgumentParser(description="Fire Patrol Demonstration")
    parser.add_argument('--config', type=str, 
                       default='../inputs/fire_patrol_config.yaml',
                       help='Path to configuration YAML file')
    parser.add_argument('--use-llm', action='store_true',
                       help='Use LLM to generate commands (requires OpenAI API key)')
    parser.add_argument('--no-commands', action='store_true',
                       help='Run simulation without command injection')
    parser.add_argument('--export-video', action='store_true',
                       help='Export video instead of interactive visualization')
    
    args = parser.parse_args()
    
    # Create orchestrator
    orchestrator = FirePatrolOrchestrator(config_path=args.config)
    
    # Prepare command sequence
    orchestrator.prepare_command_sequence(use_llm=args.use_llm)
    
    # Run simulation
    orchestrator.run_simulation(
        use_interactive_viz=not args.export_video,
        inject_commands=not args.no_commands
    )
    
    print("\n" + "="*60)
    print("MISSION COMPLETE")
    print("="*60)


if __name__ == "__main__":
    main()
