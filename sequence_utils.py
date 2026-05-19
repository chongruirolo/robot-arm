"""Shared helpers for saving and loading position sequences from CSV."""

import csv
import os

SEQ_DIR = "sequences"


def save_positions(positions: list[dict], path: str):
    os.makedirs(SEQ_DIR, exist_ok=True)
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["J1", "J2", "J3", "J4", "J5", "J6", "gripper_mm"])
        for pos in positions:
            writer.writerow(pos["joints_deg"] + [pos["gripper_mm"]])
    print(f"Saved {len(positions)} position(s) to {path}")


def load_positions(path: str) -> list[dict]:
    if not os.path.exists(path):
        raise FileNotFoundError(f"No positions file found at '{path}'. "
                                "Run with 'record' first.")
    positions = []
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            positions.append({
                "joints_deg": [float(row[f"J{i}"]) for i in range(1, 7)],
                "gripper_mm": float(row["gripper_mm"]),
            })
    if not positions:
        raise ValueError(f"'{path}' is empty — nothing to replay.")
    print(f"Loaded {len(positions)} position(s) from {path}")
    return positions


def seq_path(name: str) -> str:
    """Resolve a bare name, name.csv, or full path to sequences/<name>.csv."""
    if os.sep not in name and not name.endswith(".csv"):
        name += ".csv"
    return name if os.sep in name else os.path.join(SEQ_DIR, name)
