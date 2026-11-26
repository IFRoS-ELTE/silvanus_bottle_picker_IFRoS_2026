"""Tree parameters. All declared on the ROS node before the tree is built."""

from __future__ import annotations

from dataclasses import dataclass, field, fields
from typing import List

DEFAULT_SURVEY_DEG = [0.0, 0.0, -16.46, 0.0, -42.62, 0.0]
DEFAULT_NEAR_DEG = [-6.34, -16.30, -24.53, 1.46, 27.77, -9.76]


@dataclass
class Config:
    tick_hz: float = 10.0
    map_frame: str = 'map'
    robot_frame: str = 'base_footprint'

    # --- Bottles (input: bottle_localizer) ---
    detections_topic: str = '/bottles/detections_3d'
    min_confidence: float = 0.0          # drop detections below this
    assoc_radius: float = 0.30           # m: same bottle if closer than this
    forget_after: float = 3.0            # s unseen -> track dropped
    min_hits: int = 3                    # sightings before a bottle counts as "stable"
    selection: str = 'confidence'        # 'confidence' | 'nearest'
    max_bottle_distance: float = 2.5     # m: farther bottles are not chosen
    blacklist_radius: float = 0.40       # m: around a bottle already tried
    retry_after: float = 60.0            # s: after a failure, when it may be retried

    # --- Approach (Nav2) ---
    nav_action: str = 'navigate_to_pose'
    standoff_distance: float = 0.55      # m: final stop in front of the bottle
    refine: bool = True                  # two steps: approach, then refine
    approach_distance: float = 1.2       # m: intermediate stop of the first step
    already_there_extra: float = 0.10    # m: within standoff+this, do not drive
    check_map_bounds: bool = True
    map_margin: float = 0.15

    # --- Exploration (explore_lite) ---
    use_explore: bool = True
    explore_resume_topic: str = '/explore/resume'

    # --- Arm (MoveIt) ---
    use_arm: bool = True
    arm_group: str = 'xarm6'
    survey_joints_deg: List[float] = field(default_factory=lambda: list(DEFAULT_SURVEY_DEG))
    near_joints_deg: List[float] = field(default_factory=lambda: list(DEFAULT_NEAR_DEG))
    velocity_scaling: float = 0.1
    planning_time: float = 5.0
    planning_attempts: int = 5

    # --- Pick ---
    # 'action': call /pick_bottle (pick_bottle with server:=true) - fully automatic
    # 'wait':   manual pick, then publish a Bool on pick_result_topic
    # 'skip':   no pick (navigation tests)
    pick_mode: str = 'action'
    pick_action: str = '/pick_bottle'
    pick_max_attempts: int = 0           # 0 = pick_bottle's server_max_attempts
    pick_bottle_timeout: float = 0.0     # 0 = pick_bottle's server_bottle_timeout
    pick_result_topic: str = '/bt/pick_result'
    pick_timeout: float = 0.0            # 'wait' mode only; s, 0 = no limit

    @classmethod
    def from_node(cls, node) -> 'Config':
        cfg = cls()
        for f in fields(cls):
            default = getattr(cfg, f.name)
            node.declare_parameter(f.name, default)
            value = node.get_parameter(f.name).value
            if isinstance(default, float):
                value = float(value)
            elif isinstance(default, bool):
                value = bool(value)
            elif isinstance(default, int):
                value = int(value)
            elif isinstance(default, list):
                value = [float(v) for v in value]
            setattr(cfg, f.name, value)
        return cfg
