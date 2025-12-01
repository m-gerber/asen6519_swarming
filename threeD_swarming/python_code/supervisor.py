import os
import time
import json
from datetime import datetime

import numpy as np
import pandas as pd
import yaml

from commands import check_for_new_command, load_command  # reuse logic if helpful


# ----------------------------------------------------------------------
# Configuration for the supervisor
# ----------------------------------------------------------------------

# Path to the YAML config (same file used by driver_2.py)
CONFIG_PATH = os.path.join(os.getcwd(), "..", "inputs", "initial_config.yaml")

# Path to the tracking log produced by the simulator
TRACKING_FILE_PATH = os.path.join(os.getcwd(), "tracking_logs", "swarm_tracking.csv")

# Path to the JSON command file the simulator is polling
COMMAND_FILE_PATH = os.path.join(os.getcwd(), "commands", "command.json")

# How often (in seconds) the supervisor checks the tracking log
POLL_INTERVAL_SEC = 1.0

# These will be set by load_scenario_from_config()
HAZARD_CENTER_M = None
HAZARD_RADIUS_M = None
NEW_GOAL_M = None


# ----------------------------------------------------------------------
# Helper functions
# ----------------------------------------------------------------------

def load_scenario_from_config(
    config_path: str = CONFIG_PATH,
) -> tuple[np.ndarray, float, np.ndarray]:
    """
    Load hazard and reroute-goal scenario data from the YAML configuration.

    Returns
    -------
    (hazard_center, hazard_radius, reroute_goal)
        hazard_center : np.ndarray shape (3,)
        hazard_radius : float
        reroute_goal  : np.ndarray shape (3,)
    Raises
    ------
    RuntimeError if no suitable hazard or reroute goal is found.
    """
    if not os.path.exists(config_path):
        raise RuntimeError(f"Config file not found: {config_path}")

    with open(config_path, "r") as f:
        cfg = yaml.safe_load(f) or {}

    raw_hazards = cfg.get("hazards", [])
    if raw_hazards is None:
        raw_hazards = []

    chosen_center = None
    chosen_radius = None

    # Pick the first spherical hazard, if any
    if isinstance(raw_hazards, list):
        for h in raw_hazards:
            if not isinstance(h, dict):
                continue
            h_type = str(h.get("type", "")).lower()
            if h_type != "sphere":
                continue
            center = h.get("center_m")
            radius = h.get("radius_m")
            if center is None or radius is None:
                continue
            try:
                center_arr = np.array(
                    [float(center[0]), float(center[1]), float(center[2])],
                    dtype=float,
                )
                radius_val = float(radius)
            except Exception:
                continue

            chosen_center = center_arr
            chosen_radius = radius_val
            break

    if chosen_center is None or chosen_radius is None:
        raise RuntimeError(
            "No valid spherical hazard found in 'hazards' section of config."
        )

    reroute_cfg = cfg.get("reroute_goal_m", None)
    if reroute_cfg is None:
        raise RuntimeError("Config missing 'reroute_goal_m' for supervisor rerouting.")

    try:
        reroute_goal = np.array(
            [float(reroute_cfg[0]), float(reroute_cfg[1]), float(reroute_cfg[2])],
            dtype=float,
        )
    except Exception as exc:
        raise RuntimeError(f"Invalid 'reroute_goal_m' in config: {exc}") from exc

    return chosen_center, chosen_radius, reroute_goal

def get_last_command_id(path: str) -> int:
    """
    Read the last command_id from the command JSON file, if it exists.
    Returns 0 if the file is missing or invalid.
    """
    if not os.path.exists(path):
        return 0

    try:
        with open(path, "r") as f:
            data = json.load(f)
        return int(data.get("command_id", 0))
    except Exception:
        return 0


def write_command(path: str, command: dict) -> None:
    """
    Atomically write a command JSON file.

    This writes to a temporary file and then renames it to avoid
    partial writes being seen by the simulator.
    """
    os.makedirs(os.path.dirname(path), exist_ok=True)

    tmp_path = path + ".tmp"
    with open(tmp_path, "w") as f:
        json.dump(command, f, indent=2)

    os.replace(tmp_path, path)


def compute_min_distance_to_hazard(df_latest: pd.DataFrame) -> float:
    """
    Given the rows for the latest time step from swarm_tracking.csv,
    compute the minimum distance from any drone to the hazard center.
    """
    if df_latest.empty:
        return np.inf

    positions = df_latest[["x_m", "y_m", "z_m"]].to_numpy(dtype=float)
    deltas = positions - HAZARD_CENTER_M[None, :]
    dists = np.linalg.norm(deltas, axis=1)

    return float(np.min(dists))


def read_latest_step(tracking_path: str) -> pd.DataFrame:
    """
    Load the tracking log and return a DataFrame corresponding to the
    most recent time step (max t_s).
    """
    try:
        df = pd.read_csv(tracking_path)
    except Exception:
        # File might not be ready or partially written; treat as no data yet
        return pd.DataFrame()

    if df.empty or "t_s" not in df.columns:
        return pd.DataFrame()

    t_max = df["t_s"].max()
    df_latest = df[df["t_s"] == t_max].copy()
    return df_latest


def build_update_goal_command(command_id: int, new_goal_m: np.ndarray, reason: str) -> dict:
    """
    Build an 'update_goal' command conforming to the expected JSON schema.
    """
    return {
        "command_id": int(command_id),
        "command_type": "update_goal",
        "new_goal_m": [float(new_goal_m[0]), float(new_goal_m[1]), float(new_goal_m[2])],
        "reason": reason,
        "timestamp": datetime.utcnow().isoformat() + "Z",
    }


# ----------------------------------------------------------------------
# Main supervisor logic
# ----------------------------------------------------------------------

def main():
    print("=== Swarm supervisor started ===")
    print(f"Watching tracking file: {TRACKING_FILE_PATH}")
    print(f"Writing commands to:     {COMMAND_FILE_PATH}")
    print(f"Using scenario config:   {CONFIG_PATH}")

    global HAZARD_CENTER_M, HAZARD_RADIUS_M, NEW_GOAL_M
    HAZARD_CENTER_M, HAZARD_RADIUS_M, NEW_GOAL_M = load_scenario_from_config()

    print(f"Hazard center: {HAZARD_CENTER_M}, radius: {HAZARD_RADIUS_M} m")
    print(f"Reroute goal:  {NEW_GOAL_M}")

    # Keep track if we've already issued a hazard-avoidance command
    hazard_command_issued = False

    # Get the current last command ID so we append cleanly
    last_command_id = get_last_command_id(COMMAND_FILE_PATH)
    print(f"Initial last_command_id detected: {last_command_id}")

    while True:
        if not os.path.exists(TRACKING_FILE_PATH):
            # Tracking file not yet created; wait and try again
            time.sleep(POLL_INTERVAL_SEC)
            continue

        df_latest = read_latest_step(TRACKING_FILE_PATH)
        if df_latest.empty:
            time.sleep(POLL_INTERVAL_SEC)
            continue

        t_latest = df_latest["t_s"].iloc[0]
        min_dist = compute_min_distance_to_hazard(df_latest)

        print(
            f"[Supervisor] t = {t_latest:.2f} s, "
            f"min distance to hazard = {min_dist:.2f} m"
        )

        # If we've already issued a hazard command once, just monitor
        if hazard_command_issued:
            time.sleep(POLL_INTERVAL_SEC)
            continue

        # Trigger condition: any drone inside the hazard radius
        if min_dist <= HAZARD_RADIUS_M:
            print(
                "[Supervisor] Hazard violation detected! "
                "Issuing update_goal command to redirect the swarm."
            )

            last_command_id = get_last_command_id(COMMAND_FILE_PATH)
            new_id = last_command_id + 1

            cmd = build_update_goal_command(
                command_id=new_id,
                new_goal_m=NEW_GOAL_M,
                reason=(
                    f"Redirecting swarm away from hazard centered at "
                    f"{HAZARD_CENTER_M.tolist()} with radius {HAZARD_RADIUS_M} m."
                ),
            )

            write_command(COMMAND_FILE_PATH, cmd)
            print(f"[Supervisor] Wrote command_id={new_id} to {COMMAND_FILE_PATH}")

            hazard_command_issued = True

        time.sleep(POLL_INTERVAL_SEC)


if __name__ == "__main__":
    main()
