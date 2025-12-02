# Fire Patrol Scenario - LLM-Driven Swarm Control

## Overview

This implementation extends the 3D drone swarm simulation to support **dynamic mid-flight control** with **LLM-driven semantic reasoning**. The code demonstrates how a Large Language Model can be integrated into a physics-based drone swarm to provide real-time, semantically-reasoned instructions for complex missions.

### Fire Patrol Scenario

**Mission Brief:**
- **Start:** Golden, CO (takeoff point)
- **End:** Boulder, CO (landing zone)  
- **Distance:** ~20 km along Rocky Mountain front range
- **Drones:** 5 drones in V-formation
- **Altitude:** 100m AGL
- **Speed:** 15 m/s cruise

**Mission Timeline:**
1. **T+0s:** Five drones depart Golden in formation, flying north toward Boulder
2. **T+667s (midpoint):** Jefferson County Fire Dept reports possible fire 3km east of flight path
3. **LLM Command:** Break off drones 0 and 1 to investigate fire location
4. **Obstacle Avoidance:** Investigation team encounters obstacle near fire site (demonstrates dynamic avoidance)
5. **T+727s:** Investigation drones arrive and loiter for 3 seconds (fire detection scan)
6. **T+730s:** Investigation complete, drones commanded to rejoin main swarm
7. **T+~900s:** All five drones converge on Boulder landing zone

## Key Features Implemented

### 1. Multi-Group Swarm Management (`swarm_manager.py`)
- **Individual drone tracking** with unique IDs, modes, and states
- **Dynamic group splitting** - drones can break off into subgroups with independent goals
- **Multiple operating modes:**
  - `SWARM`: Coordinated group behavior with separation/alignment
  - `NAVIGATE`: Independent waypoint following
  - `LOITER`: Hover at position for specified duration
  - `REJOIN`: Smooth transition back to swarm formation
- **Per-group control laws** - separation and alignment only within groups

### 2. Enhanced Command System (`commands.py`)
New command types supported:

```json
{
  "command_type": "split_drones",
  "drone_ids": [0, 1],
  "new_group_id": "fire_investigation",
  "new_goal_m": [10000, 3000, 50]
}
```

- `split_drones` - Split drones into new group with goal
- `loiter` - Hover at location for duration
- `rejoin_swarm` - Merge drones back to target group
- `assign_waypoints` - Individual waypoint sequences
- `add_obstacle` - Dynamic obstacle injection
- `update_group_goal` - Change goal for specific group

### 3. LLM Interface (`llm_interface.py`)
- **Natural language → JSON translation** using OpenAI API
- **Scenario context awareness** for mission-appropriate commands
- **Fallback mode** when LLM unavailable (rule-based parsing)
- **Pre-scripted helpers** for reliable demonstration

### 4. Multi-Mode Control Laws (`demo_swarm_run.py`)
Enhanced physics simulation with:
- **Group-based swarm forces** (separation/alignment within groups only)
- **Independent navigation control** for waypoint following
- **Loiter/hover mode** with high damping and position-hold
- **Rejoin blending** - smooth transition between independent and swarm control
- **Obstacle avoidance** integrated across all modes

## File Structure

```
threeD_swarming/
├── inputs/
│   ├── fire_patrol_config.yaml       # Fire patrol mission configuration
│   └── initial_config.yaml           # Original demo configuration
│
├── python_code/
│   ├── fire_patrol_demo.py           # Main demo orchestrator
│   ├── swarm_manager.py              # Multi-group drone management (NEW)
│   ├── llm_interface.py              # LLM command generation (NEW)
│   ├── commands.py                   # Extended command system (MODIFIED)
│   ├── demo_swarm_run.py             # Enhanced simulation core (MODIFIED)
│   ├── driver_2.py                   # Driver script (backward compatible)
│   ├── obstacle_repulsion.py         # Obstacle avoidance physics
│   ├── animate_flight_paths.py       # Visualization
│   └── supervisor.py                 # External monitoring
│
└── README_FIRE_PATROL.md             # This file
```

## Installation

### Prerequisites

```bash
# Core dependencies
pip install numpy pandas matplotlib pyyaml

# Optional: For LLM command generation
pip install openai
export OPENAI_API_KEY="your-api-key-here"
```

### Setup

```bash
cd threeD_swarming/python_code
```

## Usage

### Quick Start - Fire Patrol Demo

Run the complete fire patrol scenario with pre-scripted commands:

```bash
python fire_patrol_demo.py
```

This will:
1. Launch 5 drones from Golden toward Boulder
2. Inject commands at timed intervals (split, loiter, rejoin)
3. Display interactive 3D visualization of flight paths

### Advanced Usage

**Use LLM for command generation:**
```bash
python fire_patrol_demo.py --use-llm
```
Requires `OPENAI_API_KEY` environment variable set.

**Export video instead of interactive view:**
```bash
python fire_patrol_demo.py --export-video
```

**Run without command injection (baseline swarm):**
```bash
python fire_patrol_demo.py --no-commands
```

**Custom configuration:**
```bash
python fire_patrol_demo.py --config path/to/custom_config.yaml
```

### Command Line Options

```
--config PATH          Path to configuration YAML (default: ../inputs/fire_patrol_config.yaml)
--use-llm             Generate commands using LLM instead of pre-scripted
--no-commands         Run simulation without mid-flight command injection
--export-video        Export MP4 video instead of interactive viewer
```

## Configuration

Edit `inputs/fire_patrol_config.yaml` to customize:

```yaml
# Mission parameters
n: 5                              # Number of drones
goal_m: [20000.0, 0.0, 100.0]    # Boulder destination
fire_location_m: [10000.0, 3000.0, 50.0]  # Reported fire

mission_params:
  fire_alert_time_s: 667.0                    # When to trigger investigation
  investigation_drone_ids: [0, 1]             # Which drones to break off
  investigation_loiter_duration_s: 3.0        # Hover duration at fire
  rejoin_rendezvous_m: [15000.0, 0.0, 100.0] # Where to rejoin swarm
```

## LLM Integration

### How It Works

1. **Context Setup:** Mission parameters, drone states, and capabilities are encoded
2. **Natural Language Input:** Human operator or automated system provides instruction
3. **LLM Translation:** GPT-4 converts instruction to structured JSON command
4. **Validation:** Command schema checked before execution
5. **Simulation Application:** Command written to file, picked up by simulation loop

### Example LLM Interaction

**Input:**
```
"Break off two drones to investigate possible fire at coordinates 10000, 3000, 50. 
Have them hover for 3 seconds then rejoin the main swarm."
```

**LLM Output:**
```json
{
  "command_type": "split_drones",
  "drone_ids": [0, 1],
  "new_group_id": "fire_investigation",
  "new_goal_m": [10000, 3000, 50],
  "reason": "Jefferson County Fire Department reported possible fire. Dispatching investigation team."
}
```

### Adding Custom LLM Commands

```python
from llm_interface import LLMCommandGenerator

# Initialize with scenario context
generator = LLMCommandGenerator()
generator.set_scenario_context({
    "mission_type": "fire_patrol",
    "n_drones": 5,
    "available_drones": [0, 1, 2, 3, 4]
})

# Generate command from natural language
instruction = "Send drone 3 to scan the perimeter at 500m radius"
command = generator.generate_command(instruction, next_command_id=5)

# Write to file for simulation pickup
generator.write_command_to_file(command, "commands/command.json")
```

## Architecture Details

### Control Flow Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                    Fire Patrol Demo                          │
│  ┌─────────────────────────────────────────────────────┐    │
│  │  Orchestrator                                        │    │
│  │  - Load config                                       │    │
│  │  - Prepare command sequence (LLM or scripted)       │    │
│  │  - Launch simulation thread                         │    │
│  │  - Launch command injection thread                  │    │
│  └──────────┬──────────────────────────────┬───────────┘    │
│             │                                │                │
│             ▼                                ▼                │
│  ┌──────────────────────┐       ┌───────────────────────┐   │
│  │  Simulation Loop     │       │  Command Injector     │   │
│  │  (demo_swarm_run)    │       │  (timed thread)       │   │
│  │                      │       │                       │   │
│  │  Every step:         │       │  At trigger times:    │   │
│  │  - Update loiter     │◄──────┤  - Write command JSON │   │
│  │  - Check waypoints   │       │  - Log injection      │   │
│  │  - Poll commands     │       └───────────────────────┘   │
│  │  - Compute forces    │                                    │
│  │  - RK4 integration   │                                    │
│  │  - Log state         │                                    │
│  └──────────┬───────────┘                                    │
│             │                                                 │
│             ▼                                                 │
│  ┌──────────────────────────────────────────────────────┐   │
│  │  SwarmManager                                         │   │
│  │  - Track individual drone states                     │   │
│  │  - Manage groups (main, fire_investigation)          │   │
│  │  - Handle mode transitions (SWARM↔NAVIGATE↔LOITER)   │   │
│  │  - Coordinate rejoin blending                        │   │
│  └──────────┬───────────────────────────────────────────┘   │
│             │                                                 │
│             ▼                                                 │
│  ┌──────────────────────────────────────────────────────┐   │
│  │  Control Law Computation                              │   │
│  │  For each drone:                                      │   │
│  │    if mode == SWARM:                                  │   │
│  │      - Group-based separation/alignment               │   │
│  │      - Goal attraction to group goal                  │   │
│  │    elif mode == NAVIGATE:                             │   │
│  │      - Waypoint attraction                            │   │
│  │      - Independent control                            │   │
│  │    elif mode == LOITER:                               │   │
│  │      - Position hold + high damping                   │   │
│  │    elif mode == REJOIN:                               │   │
│  │      - Blend independent + swarm (blend_factor)       │   │
│  │    Always: obstacle avoidance                         │   │
│  └───────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
```

### State Machine (Per Drone)

```
    ┌─────────┐
    │  SWARM  │────────────┐
    └────┬────┘            │
         │                 │ split_drones
         │ assign_waypoint │
         ▼                 │
   ┌──────────┐            │
   │ NAVIGATE │◄───────────┘
   └────┬─────┘
        │ loiter command
        ▼
   ┌────────┐
   │ LOITER │
   └────┬───┘
        │ duration expires
        │ OR rejoin_swarm command
        ▼
   ┌────────┐   blend_factor: 0.0 → 1.0
   │ REJOIN │───────────────────────────► Back to SWARM
   └────────┘   (smooth transition)
```

## Output Files

After running simulation:

```
swarm_output/
├── drone_01.csv          # Individual drone trajectory
├── drone_02.csv
├── ...
├── swarm_all.csv         # Combined trajectories
└── swarm_animation.mp4   # Video (if --export-video used)

tracking_logs/
├── swarm_tracking.csv    # Downsampled state history
└── swarm_summary.csv     # Aggregate metrics (separation, distance to goal, etc.)
```

## Validation & Testing

### Test Individual Features

**Test multi-group splitting:**
```python
from swarm_manager import SwarmManager
import numpy as np

# Create manager with 5 drones
manager = SwarmManager(5, P0, V0, goal=[20000, 0, 100], Ls=8.0)

# Split off two drones
manager.split_drones([0, 1], "alpha", new_goal=np.array([10000, 3000, 50]))

# Verify groups
print(f"Main group: {manager.groups['main'].drone_ids}")
print(f"Alpha group: {manager.groups['alpha'].drone_ids}")
```

**Test command generation:**
```python
from llm_interface import create_fire_patrol_command

cmd = create_fire_patrol_command(
    fire_location=[10000, 3000, 50],
    drone_ids=[0, 1],
    command_id=1
)
print(json.dumps(cmd, indent=2))
```

### Run Test Cases

```bash
# Test 1: Baseline swarm (no commands)
python fire_patrol_demo.py --no-commands

# Test 2: Command injection with pre-scripted commands
python fire_patrol_demo.py

# Test 3: LLM-generated commands (requires API key)
python fire_patrol_demo.py --use-llm
```

## Troubleshooting

### "ModuleNotFoundError: No module named 'swarm_manager'"
Ensure you're running from the `python_code` directory:
```bash
cd threeD_swarming/python_code
python fire_patrol_demo.py
```

### "openai package not installed"
LLM features are optional. Either:
- Install: `pip install openai`
- Or use pre-scripted mode (default)

### Visualization window not showing
Try disabling interactive mode:
```bash
python fire_patrol_demo.py --export-video
```
