"""
uav_simulation package
======================

This package provides a lightweight simulation environment for testing
adaptive multi‑UAV coordination strategies.  It includes implementations
of A* and D*‑Lite path planners, a simple threat simulator that produces
dynamic no‑fly zones, a UAV class representing individual agents on a
grid, a coordinator to orchestrate multiple UAVs and handle goal
swapping and collision avoidance, and a Matplotlib visualiser.
"""

from .path_planner import AStarPlanner, DStarLitePlanner
from .uav import UAV
from .coordinator import Coordinator
from .threat_simulator import ThreatSimulator
from .visualizer import Visualizer

__all__ = [
    "AStarPlanner",
    "DStarLitePlanner",
    "UAV",
    "Coordinator",
    "ThreatSimulator",
    "Visualizer",
]
