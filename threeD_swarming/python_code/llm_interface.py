"""
LLM Interface for Semantic Command Generation.

This module provides integration with OpenAI's API to translate natural language
instructions into structured swarm commands. Can be used for real-time command
generation or demonstration scenarios.
"""

import json
import os
from datetime import datetime
from typing import Dict, Any, Optional, List
import numpy as np


class LLMCommandGenerator:
    """
    Generates structured swarm commands from natural language using LLM.
    """
    
    def __init__(self, api_key: Optional[str] = None, model: str = "gpt-4"):
        """
        Initialize LLM command generator.
        
        Parameters
        ----------
        api_key : str, optional
            OpenAI API key. If None, reads from OPENAI_API_KEY environment variable.
        model : str
            Model to use (gpt-4, gpt-3.5-turbo, etc.)
        """
        self.api_key = api_key or os.getenv("OPENAI_API_KEY")
        self.model = model
        self.scenario_context = {}
        self.command_history = []
        
        # Try to import openai, but don't fail if not available
        try:
            import openai
            self.openai = openai
            if self.api_key:
                self.openai.api_key = self.api_key
            self.available = True
        except ImportError:
            print("Warning: openai package not installed. LLM features disabled.")
            print("Install with: pip install openai")
            self.available = False
    
    def set_scenario_context(self, context: Dict[str, Any]):
        """
        Set scenario context for LLM to understand mission parameters.
        
        Parameters
        ----------
        context : dict
            Scenario information including:
            - mission_type: str
            - n_drones: int
            - start_location: dict with name and coordinates
            - end_location: dict with name and coordinates
            - current_positions: list of drone positions
            - available_drones: list of drone IDs
            - obstacles: list of obstacle definitions
        """
        self.scenario_context = context
    
    def generate_command(self, natural_language_instruction: str,
                        next_command_id: int = 1) -> Optional[Dict[str, Any]]:
        """
        Generate structured command from natural language instruction.
        
        Parameters
        ----------
        natural_language_instruction : str
            Natural language instruction (e.g., "Send two drones to investigate fire at coordinates X, Y")
        next_command_id : int
            ID for the next command
        
        Returns
        -------
        dict or None
            Structured command dictionary ready for simulation
        """
        if not self.available:
            print("LLM not available. Using fallback command generation.")
            return self._fallback_command_generation(natural_language_instruction, next_command_id)
        
        # Build prompt with scenario context
        prompt = self._build_command_generation_prompt(natural_language_instruction)
        
        try:
            response = self.openai.ChatCompletion.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": self._get_system_prompt()},
                    {"role": "user", "content": prompt}
                ],
                temperature=0.1,  # Low temperature for consistent structured output
                max_tokens=500
            )
            
            # Parse LLM response
            content = response.choices[0].message.content.strip()
            
            # Extract JSON from response (handle markdown code blocks)
            if "```json" in content:
                content = content.split("```json")[1].split("```")[0].strip()
            elif "```" in content:
                content = content.split("```")[1].split("```")[0].strip()
            
            command = json.loads(content)
            command["command_id"] = next_command_id
            command["timestamp"] = datetime.utcnow().isoformat() + "Z"
            command["source"] = "llm"
            
            # Validate command structure
            if self._validate_command(command):
                self.command_history.append(command)
                return command
            else:
                print(f"Generated command failed validation: {command}")
                return None
        
        except Exception as e:
            print(f"Error generating command from LLM: {e}")
            return self._fallback_command_generation(natural_language_instruction, next_command_id)
    
    def _get_system_prompt(self) -> str:
        """Get system prompt defining LLM's role and capabilities."""
        return """You are an AI assistant for a drone swarm control system. Your task is to translate 
natural language instructions into structured JSON commands that the swarm simulation can execute.

Available command types:
1. split_drones: Split drones from main swarm to new group
2. assign_waypoints: Assign waypoint sequence to specific drone
3. loiter: Command drones to hover at location for duration
4. rejoin_swarm: Command drones to rejoin target group
5. update_goal: Change goal for entire swarm or specific group
6. add_obstacle: Add new obstacle during flight

Always respond with valid JSON matching the exact schema for the command type.
Consider the scenario context and current swarm state when generating commands.
Be precise with coordinates and drone IDs."""
    
    def _build_command_generation_prompt(self, instruction: str) -> str:
        """Build detailed prompt with scenario context."""
        context_str = json.dumps(self.scenario_context, indent=2)
        
        schemas = """
Command Schemas:

split_drones:
{
  "command_type": "split_drones",
  "drone_ids": [0, 1],
  "new_group_id": "alpha",
  "new_goal_m": [x, y, z],
  "reason": "explanation"
}

loiter:
{
  "command_type": "loiter",
  "drone_ids": [0, 1],
  "duration_s": 30.0,
  "position": [x, y, z] (optional),
  "reason": "explanation"
}

rejoin_swarm:
{
  "command_type": "rejoin_swarm",
  "drone_ids": [0, 1],
  "target_group_id": "main",
  "rendezvous_point": [x, y, z] (optional),
  "reason": "explanation"
}

update_goal:
{
  "command_type": "update_goal",
  "new_goal_m": [x, y, z],
  "reason": "explanation"
}
"""
        
        prompt = f"""Scenario Context:
{context_str}

Instruction: {instruction}

Available Command Schemas:
{schemas}

Generate the appropriate JSON command(s) for this instruction. 
Respond with ONLY the JSON command, no additional text."""
        
        return prompt
    
    def _validate_command(self, command: Dict[str, Any]) -> bool:
        """Validate command structure and required fields."""
        if "command_type" not in command:
            return False
        
        cmd_type = command["command_type"]
        
        # Type-specific validation
        if cmd_type == "split_drones":
            return all(k in command for k in ["drone_ids", "new_group_id", "new_goal_m"])
        elif cmd_type == "loiter":
            return all(k in command for k in ["drone_ids", "duration_s"])
        elif cmd_type == "rejoin_swarm":
            return all(k in command for k in ["drone_ids", "target_group_id"])
        elif cmd_type == "update_goal":
            return "new_goal_m" in command
        elif cmd_type == "add_obstacle":
            return "obstacle" in command
        
        return True
    
    def _fallback_command_generation(self, instruction: str, 
                                    next_command_id: int) -> Optional[Dict[str, Any]]:
        """
        Fallback command generation using rule-based parsing when LLM unavailable.
        """
        instruction_lower = instruction.lower()
        
        # Simple pattern matching for common commands
        if "split" in instruction_lower or "break off" in instruction_lower:
            # Extract numbers for drone IDs (simple heuristic)
            import re
            numbers = re.findall(r'\d+', instruction)
            if len(numbers) >= 2:
                return {
                    "command_id": next_command_id,
                    "command_type": "split_drones",
                    "drone_ids": [0, 1],  # Default to first two drones
                    "new_group_id": "breakoff",
                    "new_goal_m": [0, 0, 50],  # Placeholder
                    "reason": instruction,
                    "timestamp": datetime.utcnow().isoformat() + "Z",
                    "source": "fallback"
                }
        
        elif "loiter" in instruction_lower or "hover" in instruction_lower:
            return {
                "command_id": next_command_id,
                "command_type": "loiter",
                "drone_ids": [0, 1],
                "duration_s": 30.0,
                "reason": instruction,
                "timestamp": datetime.utcnow().isoformat() + "Z",
                "source": "fallback"
            }
        
        elif "rejoin" in instruction_lower or "return" in instruction_lower:
            return {
                "command_id": next_command_id,
                "command_type": "rejoin_swarm",
                "drone_ids": [0, 1],
                "target_group_id": "main",
                "reason": instruction,
                "timestamp": datetime.utcnow().isoformat() + "Z",
                "source": "fallback"
            }
        
        print(f"Could not parse instruction: {instruction}")
        return None
    
    def write_command_to_file(self, command: Dict[str, Any], 
                             filepath: str = "commands/command.json"):
        """
        Write command to JSON file for simulation to pick up.
        
        Parameters
        ----------
        command : dict
            Command dictionary
        filepath : str
            Path to command file
        """
        os.makedirs(os.path.dirname(filepath), exist_ok=True)
        
        with open(filepath, 'w') as f:
            json.dump(command, f, indent=2)
        
        print(f"Command written to {filepath}")
        print(f"Command type: {command.get('command_type')}")
        print(f"Reason: {command.get('reason', 'N/A')}")


def create_fire_patrol_command(fire_location: List[float], 
                               drone_ids: List[int] = [0, 1],
                               command_id: int = 1) -> Dict[str, Any]:
    """
    Create pre-scripted fire patrol command for demonstration.
    
    Parameters
    ----------
    fire_location : list of float
        [x, y, z] coordinates of reported fire
    drone_ids : list of int
        Drones to send to investigate
    command_id : int
        Command ID
    
    Returns
    -------
    dict
        Split command to send drones to fire location
    """
    return {
        "command_id": command_id,
        "command_type": "split_drones",
        "drone_ids": drone_ids,
        "new_group_id": "fire_investigation",
        "new_goal_m": fire_location,
        "reason": "Jefferson County Fire Department reported possible fire. Dispatching investigation team.",
        "timestamp": datetime.utcnow().isoformat() + "Z",
        "source": "fire_patrol_scenario"
    }


def create_loiter_command(drone_ids: List[int], 
                         duration: float = 3.0,
                         command_id: int = 2) -> Dict[str, Any]:
    """
    Create loiter command for fire investigation.
    
    Parameters
    ----------
    drone_ids : list of int
        Drones to loiter
    duration : float
        Loiter duration in seconds
    command_id : int
        Command ID
    
    Returns
    -------
    dict
        Loiter command
    """
    return {
        "command_id": command_id,
        "command_type": "loiter",
        "drone_ids": drone_ids,
        "duration_s": duration,
        "reason": "Hovering to allow fire detection equipment to scan area.",
        "timestamp": datetime.utcnow().isoformat() + "Z",
        "source": "fire_patrol_scenario"
    }


def create_rejoin_command(drone_ids: List[int],
                         rendezvous: Optional[List[float]] = None,
                         command_id: int = 3) -> Dict[str, Any]:
    """
    Create rejoin command after investigation complete.
    
    Parameters
    ----------
    drone_ids : list of int
        Drones to rejoin main swarm
    rendezvous : list of float, optional
        Rendezvous point [x, y, z]
    command_id : int
        Command ID
    
    Returns
    -------
    dict
        Rejoin command
    """
    cmd = {
        "command_id": command_id,
        "command_type": "rejoin_swarm",
        "drone_ids": drone_ids,
        "target_group_id": "main",
        "reason": "Fire investigation complete. Rejoining main swarm for transit to Boulder.",
        "timestamp": datetime.utcnow().isoformat() + "Z",
        "source": "fire_patrol_scenario"
    }
    
    if rendezvous is not None:
        cmd["rendezvous_point"] = rendezvous
    
    return cmd


if __name__ == "__main__":
    # Demo usage
    generator = LLMCommandGenerator()
    
    # Set scenario context
    generator.set_scenario_context({
        "mission_type": "fire_patrol",
        "n_drones": 5,
        "start_location": {"name": "Golden, CO", "coordinates": [0, 0, 50]},
        "end_location": {"name": "Boulder, CO", "coordinates": [20000, 0, 50]},
        "available_drones": [0, 1, 2, 3, 4]
    })
    
    # Test command generation
    instruction = "Break off two drones to investigate possible fire at coordinates 10000, 2000, 50"
    command = generator.generate_command(instruction, next_command_id=1)
    
    if command:
        print("Generated command:")
        print(json.dumps(command, indent=2))
        
        # Optionally write to file
        # generator.write_command_to_file(command)
