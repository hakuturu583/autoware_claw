"""Autoware ROS 2 node for MCP bridge (analogous to UR5ROSNode in rosclaw)."""

from __future__ import annotations

import math
import time
from threading import Lock
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from autoware_control_msgs.msg import Control
from autoware_vehicle_msgs.msg import (
    GearCommand,
    GearReport,
    HazardLightsCommand,
    SteeringReport,
    TurnIndicatorsCommand,
    VelocityReport,
    ControlModeReport,
    Engage,
)
from autoware_perception_msgs.msg import PredictedObjects, TrafficLightGroupArray
from autoware_planning_msgs.msg import LaneletRoute
from autoware_adapi_v1_msgs.msg import OperationModeState
from nav_msgs.msg import Odometry
from tier4_control_msgs.msg import GateMode
from tier4_external_api_msgs.msg import Heartbeat

from autoware_claw.topic_adapters import make_control_msg
from autoware_claw.types import (
    VehicleState,
    DetectedObject,
    TrafficSignal,
    TrafficSignalElement,
    GEAR_MAP,
    GEAR_REVERSE_MAP,
)

# QoS for sensor-like topics
SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=1,
)

RELIABLE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    depth=1,
)


class AutowareROSNode(Node):
    """ROS 2 node that manages subscriptions/publishers for the MCP bridge.

    Follows the same pattern as rosclaw's UR5ROSNode:
    - Caches latest state from subscriptions
    - Provides methods to send commands via publishers
    - Maintains a heartbeat for vehicle_cmd_gate safety
    """

    def __init__(self) -> None:
        super().__init__("autoware_claw")

        self._lock = Lock()
        self._state = VehicleState()
        self._predicted_objects: list[DetectedObject] = []
        self._traffic_signals: list[TrafficSignal] = []
        self._route_state: str = "UNSET"
        self._heartbeat_active = False

        self._setup_subscribers()
        self._setup_publishers()

    # ──────────────────────────────────────────────
    # Subscribers
    # ──────────────────────────────────────────────

    def _setup_subscribers(self) -> None:
        self.create_subscription(
            Odometry, "/localization/kinematic_state", self._on_kinematic_state, SENSOR_QOS
        )
        self.create_subscription(
            VelocityReport, "/vehicle/status/velocity_status", self._on_velocity, SENSOR_QOS
        )
        self.create_subscription(
            SteeringReport, "/vehicle/status/steering_status", self._on_steering, SENSOR_QOS
        )
        self.create_subscription(
            GearReport, "/vehicle/status/gear_status", self._on_gear, SENSOR_QOS
        )
        self.create_subscription(
            ControlModeReport, "/vehicle/status/control_mode", self._on_control_mode, SENSOR_QOS
        )
        self.create_subscription(
            OperationModeState, "/api/operation_mode/state", self._on_operation_mode, RELIABLE_QOS
        )
        self.create_subscription(
            PredictedObjects,
            "/perception/object_recognition/objects",
            self._on_objects,
            SENSOR_QOS,
        )
        self.create_subscription(
            TrafficLightGroupArray,
            "/perception/traffic_light_recognition/traffic_signals",
            self._on_traffic_signals,
            SENSOR_QOS,
        )
        self.create_subscription(
            GateMode, "/control/current_gate_mode", self._on_gate_mode, RELIABLE_QOS
        )
        self.create_subscription(
            Engage, "/api/autoware/get/engage", self._on_engage, RELIABLE_QOS
        )

    # ──────────────────────────────────────────────
    # Publishers
    # ──────────────────────────────────────────────

    def _setup_publishers(self) -> None:
        self._pub_control = self.create_publisher(
            Control, "/external/selected/control_cmd", RELIABLE_QOS
        )
        self._pub_gear = self.create_publisher(
            GearCommand, "/external/selected/gear_cmd", RELIABLE_QOS
        )
        self._pub_turn_indicators = self.create_publisher(
            TurnIndicatorsCommand, "/external/selected/turn_indicators_cmd", RELIABLE_QOS
        )
        self._pub_hazard_lights = self.create_publisher(
            HazardLightsCommand, "/external/selected/hazard_lights_cmd", RELIABLE_QOS
        )
        self._pub_heartbeat = self.create_publisher(
            Heartbeat, "/external/selected/heartbeat", RELIABLE_QOS
        )
        self._pub_gate_mode = self.create_publisher(
            GateMode, "/control/gate_mode_cmd", RELIABLE_QOS
        )
        self._pub_engage = self.create_publisher(Engage, "/autoware/engage", RELIABLE_QOS)

    # ──────────────────────────────────────────────
    # Heartbeat
    # ──────────────────────────────────────────────

    def start_heartbeat(self, rate_hz: float = 20.0) -> None:
        """Start publishing heartbeat at the given rate."""
        self._heartbeat_active = True
        period = 1.0 / rate_hz
        self._heartbeat_timer = self.create_timer(period, self._heartbeat_cb)
        self.get_logger().info(f"Heartbeat started at {rate_hz} Hz")

    def stop_heartbeat(self) -> None:
        """Stop heartbeat (triggers vehicle_cmd_gate emergency stop)."""
        self._heartbeat_active = False
        if hasattr(self, "_heartbeat_timer"):
            self._heartbeat_timer.cancel()

    def _heartbeat_cb(self) -> None:
        if self._heartbeat_active:
            msg = Heartbeat()
            msg.stamp = self.get_clock().now().to_msg()
            self._pub_heartbeat.publish(msg)

    # ──────────────────────────────────────────────
    # Subscription callbacks
    # ──────────────────────────────────────────────

    def _on_kinematic_state(self, msg: Odometry) -> None:
        with self._lock:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            self._state.x = p.x
            self._state.y = p.y
            self._state.z = p.z
            # Extract yaw from quaternion
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            self._state.yaw = math.atan2(siny_cosp, cosy_cosp)
            self._state.is_connected = True
            self._state.timestamp = time.time()

    def _on_velocity(self, msg: VelocityReport) -> None:
        with self._lock:
            self._state.velocity_mps = msg.longitudinal_velocity
            self._state.lateral_velocity_mps = msg.lateral_velocity
            self._state.heading_rate_rps = msg.heading_rate

    def _on_steering(self, msg: SteeringReport) -> None:
        with self._lock:
            self._state.steering_tire_angle_rad = msg.steering_tire_angle

    def _on_gear(self, msg: GearReport) -> None:
        with self._lock:
            self._state.gear = msg.report

    def _on_control_mode(self, msg: ControlModeReport) -> None:
        with self._lock:
            self._state.control_mode = msg.mode

    def _on_operation_mode(self, msg: OperationModeState) -> None:
        with self._lock:
            mode_map = {
                0: "UNKNOWN",
                1: "AUTONOMOUS",
                2: "LOCAL",
                3: "REMOTE",
                4: "STOP",
            }
            self._state.operation_mode = mode_map.get(msg.mode, "UNKNOWN")
            self._state.is_autoware_control_enabled = msg.is_autoware_control_enabled
            self._state.is_in_transition = msg.is_in_transition

    def _on_objects(self, msg: PredictedObjects) -> None:
        classification_map = {
            0: "UNKNOWN", 1: "CAR", 2: "TRUCK", 3: "BUS",
            4: "TRAILER", 5: "MOTORCYCLE", 6: "BICYCLE", 7: "PEDESTRIAN",
        }
        objects = []
        for obj in msg.objects:
            p = obj.kinematics.initial_pose_with_covariance.pose.position
            tw = obj.kinematics.initial_twist_with_covariance.twist.linear
            # Distance from ego (0,0 in base_link, but we use map frame approx)
            dist = math.sqrt(p.x * p.x + p.y * p.y)
            cls_label = "UNKNOWN"
            cls_prob = 0.0
            if obj.classification:
                cls_label = classification_map.get(obj.classification[0].label, "UNKNOWN")
                cls_prob = obj.classification[0].probability
            shape = obj.shape
            objects.append(
                DetectedObject(
                    object_id=str(uuid.UUID(bytes=bytes(obj.object_id.uuid))),
                    classification=cls_label,
                    probability=cls_prob,
                    x=p.x, y=p.y, z=p.z,
                    vx=tw.x, vy=tw.y,
                    distance_m=dist,
                    length=shape.dimensions.x,
                    width=shape.dimensions.y,
                    height=shape.dimensions.z,
                )
            )
        with self._lock:
            self._predicted_objects = objects

    def _on_traffic_signals(self, msg: TrafficLightGroupArray) -> None:
        color_map = {0: "UNKNOWN", 1: "RED", 2: "AMBER", 3: "GREEN", 4: "WHITE"}
        shape_map = {
            0: "UNKNOWN", 1: "CIRCLE", 2: "LEFT_ARROW", 3: "RIGHT_ARROW",
            4: "UP_ARROW", 5: "DOWN_ARROW", 6: "DOWN_LEFT_ARROW", 7: "DOWN_RIGHT_ARROW",
            8: "CROSS",
        }
        status_map = {0: "UNKNOWN", 1: "SOLID_ON", 2: "SOLID_OFF", 3: "FLASHING"}
        signals = []
        for group in msg.traffic_light_groups:
            elements = []
            for e in group.elements:
                elements.append(
                    TrafficSignalElement(
                        color=color_map.get(e.color, "UNKNOWN"),
                        shape=shape_map.get(e.shape, "UNKNOWN"),
                        status=status_map.get(e.status, "UNKNOWN"),
                        confidence=e.confidence,
                    )
                )
            signals.append(TrafficSignal(signal_id=group.traffic_light_group_id, elements=elements))
        with self._lock:
            self._traffic_signals = signals

    def _on_gate_mode(self, msg: GateMode) -> None:
        with self._lock:
            self._state.gate_mode = "EXTERNAL" if msg.data == 1 else "AUTO"

    def _on_engage(self, msg: Engage) -> None:
        with self._lock:
            self._state.is_engaged = msg.engage

    # ──────────────────────────────────────────────
    # State getters (thread-safe)
    # ──────────────────────────────────────────────

    def get_vehicle_state(self) -> VehicleState:
        with self._lock:
            return VehicleState(**vars(self._state))

    def get_predicted_objects(self) -> list[DetectedObject]:
        with self._lock:
            return list(self._predicted_objects)

    def get_traffic_signals(self) -> list[TrafficSignal]:
        with self._lock:
            return list(self._traffic_signals)

    # ──────────────────────────────────────────────
    # Command methods (analogous to UR5ROSNode.execute_*)
    # ──────────────────────────────────────────────

    def send_control(
        self,
        steering_rad: float,
        velocity_mps: float,
        acceleration_mps2: float = 0.0,
    ) -> None:
        stamp = self.get_clock().now().to_msg()
        msg = make_control_msg(steering_rad, velocity_mps, acceleration_mps2, stamp)
        self._pub_control.publish(msg)

    def send_gear(self, gear_str: str) -> bool:
        gear_val = GEAR_REVERSE_MAP.get(gear_str.upper())
        if gear_val is None:
            self.get_logger().error(f"Unknown gear: {gear_str}")
            return False
        msg = GearCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.command = gear_val
        self._pub_gear.publish(msg)
        return True

    def send_turn_indicators(self, command: str) -> bool:
        cmd_map = {"DISABLE": 1, "LEFT": 2, "RIGHT": 3}
        val = cmd_map.get(command.upper())
        if val is None:
            self.get_logger().error(f"Unknown turn indicator command: {command}")
            return False
        msg = TurnIndicatorsCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.command = val
        self._pub_turn_indicators.publish(msg)
        return True

    def send_hazard_lights(self, command: str) -> bool:
        cmd_map = {"DISABLE": 1, "ENABLE": 2}
        val = cmd_map.get(command.upper())
        if val is None:
            self.get_logger().error(f"Unknown hazard lights command: {command}")
            return False
        msg = HazardLightsCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.command = val
        self._pub_hazard_lights.publish(msg)
        return True

    def set_gate_mode(self, mode: str) -> bool:
        mode_map = {"AUTO": 0, "EXTERNAL": 1}
        val = mode_map.get(mode.upper())
        if val is None:
            return False
        msg = GateMode()
        msg.data = val
        self._pub_gate_mode.publish(msg)
        return True

    def set_engage(self, engage: bool) -> None:
        msg = Engage()
        msg.engage = engage
        self._pub_engage.publish(msg)

    def emergency_stop(self) -> None:
        """Send zero-velocity control and stop heartbeat."""
        self.send_control(steering_rad=0.0, velocity_mps=0.0, acceleration_mps2=-2.5)
        self.stop_heartbeat()
        self.get_logger().warn("Emergency stop triggered — heartbeat stopped")
