"""Data types for Autoware MCP Bridge."""

from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class VehicleState:
    """Current vehicle state (analogous to UR5ROSNode.RobotState)."""

    # Pose in map frame
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0

    # Kinematics
    velocity_mps: float = 0.0
    lateral_velocity_mps: float = 0.0
    heading_rate_rps: float = 0.0
    steering_tire_angle_rad: float = 0.0

    # Velocity limit
    current_max_velocity_mps: float = 0.0
    current_max_velocity_sender: str = ""

    # Vehicle status
    gear: int = 0  # GearReport values
    control_mode: int = 0  # ControlModeReport values
    turn_indicators: int = 0
    hazard_lights: int = 0

    # Operation mode
    operation_mode: str = "UNKNOWN"
    is_autoware_control_enabled: bool = False
    is_in_transition: bool = False

    # System
    is_engaged: bool = False
    mrm_state: str = "NORMAL"
    mrm_behavior: str = "NONE"
    gate_mode: str = "AUTO"

    # Connection
    is_connected: bool = False
    timestamp: float = 0.0


GEAR_MAP = {0: "NONE", 1: "NEUTRAL", 2: "DRIVE", 20: "REVERSE", 22: "PARK", 23: "LOW"}
GEAR_REVERSE_MAP = {v: k for k, v in GEAR_MAP.items()}


@dataclass
class GoalCandidate:
    """A candidate goal pose on a lane centerline."""

    lanelet_id: int = 0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw_rad: float = 0.0
    lateral_dist_m: float = 0.0


@dataclass
class DetectedObject:
    """Simplified detected object for MCP response."""

    object_id: str = ""
    classification: str = "UNKNOWN"
    probability: float = 0.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    distance_m: float = 0.0
    length: float = 0.0
    width: float = 0.0
    height: float = 0.0


@dataclass
class TrafficSignalElement:
    """A single traffic signal element."""

    color: str = "UNKNOWN"
    shape: str = "CIRCLE"
    status: str = "UNKNOWN"
    confidence: float = 0.0


@dataclass
class TrafficSignal:
    """A traffic signal group."""

    signal_id: int = 0
    elements: list[TrafficSignalElement] = field(default_factory=list)
