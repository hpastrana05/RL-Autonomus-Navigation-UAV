import json
from pathlib import Path

def load_scenario(path):
    with Path(path).open(encoding="utf-8") as file:
        scenario = json.load(file)

    required_fields = {"name", "start", "end", "obstacles"}
    missing_fields = required_fields - scenario.keys()

    if missing_fields:
        raise ValueError(f"Scenario is missing: {missing_fields}")

    return scenario