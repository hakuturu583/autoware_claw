#!/usr/bin/env python3
"""Autoware MCP Server — bridges LLM agents to Autoware via MCP protocol.

Follows the ur5_server.py pattern from ROSClaw: a Server instance registers
tool handlers that read/write state via the AutowareROSNode.
"""

from __future__ import annotations

import asyncio
import json
import signal
import threading
from dataclasses import asdict

import rclpy
from rclpy.executors import MultiThreadedExecutor

from mcp.server import Server
from mcp.types import Tool, TextContent

from autoware_claw.autoware_ros_node import AutowareROSNode
from autoware_claw.coordinate_resolver import CoordinateResolver
from autoware_claw.types import GEAR_MAP


class AutowareMCPServer:
    """MCP server exposing Autoware HMI tools to LLM agents."""

    def __init__(
        self,
        ros_node: AutowareROSNode,
        ollama_url: str = "http://127.0.0.1:11435",
        ollama_model: str = "gemma4",
        nemoclaw_port: int = 18789,
        nemoclaw_token: str = "",
    ) -> None:
        self._node = ros_node
        self._server = Server("rosclaw-autoware-mcp")
        self._coord_resolver: CoordinateResolver | None = None
        self._ollama_url = ollama_url
        self._ollama_model = ollama_model
        self._nemoclaw_port = nemoclaw_port
        self._nemoclaw_ws_url = f"ws://127.0.0.1:{nemoclaw_port}"
        self._nemoclaw_token = nemoclaw_token
        self._register_tools()

    def init_coordinate_resolver(
        self, map_path: str, lat: float, lon: float, alt: float = 0.0
    ) -> bool:
        """Initialize lanelet2 map for coordinate resolution."""
        if not map_path:
            self._node.get_logger().warn("No map_path configured — coordinate resolver disabled")
            return False
        try:
            self._coord_resolver = CoordinateResolver(map_path, lat, lon, alt)
            self._node.get_logger().info(f"Coordinate resolver loaded: {map_path}")
            return True
        except Exception as e:
            self._node.get_logger().error(f"Failed to load lanelet2 map: {e}")
            return False

    # ──────────────────────────────────────────────
    # Tool registration
    # ──────────────────────────────────────────────

    def _get_tools_list(self) -> list[Tool]:
        """Return the list of available MCP tools (used by both MCP and chat endpoints)."""
        return self._tools

    def _register_tools(self) -> None:
        server = self._server

        self._tools = [
                # Display tools
                Tool(
                    name="autoware_get_vehicle_state",
                    description="Get current vehicle state: position, velocity, steering, gear, and system status.",
                    inputSchema={"type": "object", "properties": {}},
                ),
                Tool(
                    name="autoware_get_operation_mode",
                    description="Get current operation mode (AUTONOMOUS/LOCAL/REMOTE/STOP) and transition state.",
                    inputSchema={"type": "object", "properties": {}},
                ),
                Tool(
                    name="autoware_get_surrounding_objects",
                    description="Get detected objects around the vehicle (cars, pedestrians, cyclists, etc.).",
                    inputSchema={
                        "type": "object",
                        "properties": {
                            "max_distance_m": {
                                "type": "number",
                                "description": "Maximum distance filter in meters (default: no filter).",
                            }
                        },
                    },
                ),
                Tool(
                    name="autoware_get_traffic_signals",
                    description="Get current traffic signal states (color, shape, status).",
                    inputSchema={"type": "object", "properties": {}},
                ),
                Tool(
                    name="autoware_get_diagnostics",
                    description="Get system diagnostics: engage state, MRM state, gate mode, control mode.",
                    inputSchema={"type": "object", "properties": {}},
                ),
                # Coordinate resolution tools
                Tool(
                    name="autoware_resolve_goal",
                    description="Convert lat/lon to lane-aligned goal candidates. Returns poses on lane centerlines near the specified GPS coordinate.",
                    inputSchema={
                        "type": "object",
                        "properties": {
                            "lat": {"type": "number", "description": "Latitude in degrees."},
                            "lon": {"type": "number", "description": "Longitude in degrees."},
                            "search_radius": {
                                "type": "number",
                                "description": "Search radius in meters (default: 50).",
                            },
                        },
                        "required": ["lat", "lon"],
                    },
                ),
                Tool(
                    name="autoware_get_lane_info",
                    description="Get lane information near a map-frame coordinate (lanelet ID, length, subtype, speed limit).",
                    inputSchema={
                        "type": "object",
                        "properties": {
                            "x": {"type": "number", "description": "X coordinate in map frame."},
                            "y": {"type": "number", "description": "Y coordinate in map frame."},
                            "search_radius": {
                                "type": "number",
                                "description": "Search radius in meters (default: 10).",
                            },
                        },
                        "required": ["x", "y"],
                    },
                ),
                # Command tools
                Tool(
                    name="autoware_set_goal",
                    description="Set a navigation goal from map-frame coordinates (use autoware_resolve_goal first to get candidates).",
                    inputSchema={
                        "type": "object",
                        "properties": {
                            "x": {"type": "number", "description": "X in map frame."},
                            "y": {"type": "number", "description": "Y in map frame."},
                            "z": {"type": "number", "description": "Z in map frame (default: 0)."},
                            "yaw_rad": {"type": "number", "description": "Yaw in radians."},
                        },
                        "required": ["x", "y", "yaw_rad"],
                    },
                ),
                Tool(
                    name="autoware_engage",
                    description="Enable or disable Autoware autonomous control (engage/disengage).",
                    inputSchema={
                        "type": "object",
                        "properties": {
                            "engage": {"type": "boolean", "description": "True to engage, False to disengage."},
                        },
                        "required": ["engage"],
                    },
                ),
                Tool(
                    name="autoware_emergency_stop",
                    description="Trigger emergency stop: sends zero velocity and stops the heartbeat.",
                    inputSchema={"type": "object", "properties": {}},
                ),
                Tool(
                    name="autoware_set_turn_indicators",
                    description="Set turn indicators: LEFT, RIGHT, or DISABLE.",
                    inputSchema={
                        "type": "object",
                        "properties": {
                            "command": {
                                "type": "string",
                                "enum": ["LEFT", "RIGHT", "DISABLE"],
                                "description": "Turn indicator command.",
                            },
                        },
                        "required": ["command"],
                    },
                ),
                Tool(
                    name="autoware_set_hazard_lights",
                    description="Set hazard lights: ENABLE or DISABLE.",
                    inputSchema={
                        "type": "object",
                        "properties": {
                            "command": {
                                "type": "string",
                                "enum": ["ENABLE", "DISABLE"],
                                "description": "Hazard lights command.",
                            },
                        },
                        "required": ["command"],
                    },
                ),
                Tool(
                    name="autoware_set_gate_mode",
                    description="Switch vehicle_cmd_gate mode: AUTO (Autoware planning) or EXTERNAL (MCP control).",
                    inputSchema={
                        "type": "object",
                        "properties": {
                            "mode": {
                                "type": "string",
                                "enum": ["AUTO", "EXTERNAL"],
                                "description": "Gate mode.",
                            },
                        },
                        "required": ["mode"],
                    },
                ),
                Tool(
                    name="autoware_send_control",
                    description="Send direct vehicle control command (steering, velocity, acceleration). Requires EXTERNAL gate mode.",
                    inputSchema={
                        "type": "object",
                        "properties": {
                            "steering_rad": {"type": "number", "description": "Steering tire angle in radians."},
                            "velocity_mps": {"type": "number", "description": "Target velocity in m/s."},
                            "acceleration_mps2": {
                                "type": "number",
                                "description": "Acceleration in m/s^2 (default: 0).",
                            },
                        },
                        "required": ["steering_rad", "velocity_mps"],
                    },
                ),
                Tool(
                    name="autoware_send_gear",
                    description="Send gear command: DRIVE, REVERSE, PARK, NEUTRAL, LOW.",
                    inputSchema={
                        "type": "object",
                        "properties": {
                            "gear": {
                                "type": "string",
                                "enum": ["DRIVE", "REVERSE", "PARK", "NEUTRAL", "LOW"],
                                "description": "Gear command.",
                            },
                        },
                        "required": ["gear"],
                    },
                ),
                Tool(
                    name="autoware_start_heartbeat",
                    description="Start the heartbeat publisher (required for vehicle_cmd_gate to accept external commands).",
                    inputSchema={"type": "object", "properties": {}},
                ),
                Tool(
                    name="autoware_stop_heartbeat",
                    description="Stop the heartbeat publisher (vehicle_cmd_gate will trigger emergency stop).",
                    inputSchema={"type": "object", "properties": {}},
                ),
            ]

        @server.list_tools()
        async def list_tools() -> list[Tool]:
            return self._tools

        @server.call_tool()
        async def call_tool(name: str, arguments: dict) -> list[TextContent]:
            handler = self._tool_handlers.get(name)
            if handler is None:
                return [TextContent(type="text", text=f"Unknown tool: {name}")]
            try:
                result = handler(arguments)
                text = json.dumps(result, ensure_ascii=False, default=str)
                return [TextContent(type="text", text=text)]
            except Exception as e:
                return [TextContent(type="text", text=json.dumps({"error": str(e)}))]

    # ──────────────────────────────────────────────
    # Tool handlers
    # ──────────────────────────────────────────────

    @property
    def _tool_handlers(self) -> dict:
        return {
            "autoware_get_vehicle_state": self._handle_get_vehicle_state,
            "autoware_get_operation_mode": self._handle_get_operation_mode,
            "autoware_get_surrounding_objects": self._handle_get_surrounding_objects,
            "autoware_get_traffic_signals": self._handle_get_traffic_signals,
            "autoware_get_diagnostics": self._handle_get_diagnostics,
            "autoware_resolve_goal": self._handle_resolve_goal,
            "autoware_get_lane_info": self._handle_get_lane_info,
            "autoware_set_goal": self._handle_set_goal,
            "autoware_engage": self._handle_engage,
            "autoware_emergency_stop": self._handle_emergency_stop,
            "autoware_set_turn_indicators": self._handle_set_turn_indicators,
            "autoware_set_hazard_lights": self._handle_set_hazard_lights,
            "autoware_set_gate_mode": self._handle_set_gate_mode,
            "autoware_send_control": self._handle_send_control,
            "autoware_send_gear": self._handle_send_gear,
            "autoware_start_heartbeat": self._handle_start_heartbeat,
            "autoware_stop_heartbeat": self._handle_stop_heartbeat,
        }

    # --- Display tools ---

    def _handle_get_vehicle_state(self, args: dict) -> dict:
        state = self._node.get_vehicle_state()
        d = asdict(state)
        d["gear_name"] = GEAR_MAP.get(state.gear, "UNKNOWN")
        return d

    def _handle_get_operation_mode(self, args: dict) -> dict:
        state = self._node.get_vehicle_state()
        return {
            "operation_mode": state.operation_mode,
            "is_autoware_control_enabled": state.is_autoware_control_enabled,
            "is_in_transition": state.is_in_transition,
        }

    def _handle_get_surrounding_objects(self, args: dict) -> dict:
        objects = self._node.get_predicted_objects()
        max_dist = args.get("max_distance_m")
        if max_dist is not None:
            objects = [o for o in objects if o.distance_m <= max_dist]
        return {
            "count": len(objects),
            "objects": [asdict(o) for o in objects],
        }

    def _handle_get_traffic_signals(self, args: dict) -> dict:
        signals = self._node.get_traffic_signals()
        return {
            "count": len(signals),
            "signals": [asdict(s) for s in signals],
        }

    def _handle_get_diagnostics(self, args: dict) -> dict:
        state = self._node.get_vehicle_state()
        return {
            "is_connected": state.is_connected,
            "is_engaged": state.is_engaged,
            "gate_mode": state.gate_mode,
            "control_mode": state.control_mode,
            "mrm_state": state.mrm_state,
            "mrm_behavior": state.mrm_behavior,
            "operation_mode": state.operation_mode,
            "gear": state.gear,
            "gear_name": GEAR_MAP.get(state.gear, "UNKNOWN"),
        }

    # --- Coordinate resolution tools ---

    def _handle_resolve_goal(self, args: dict) -> dict:
        if self._coord_resolver is None:
            return {"error": "Coordinate resolver not initialized (no map loaded)"}
        lat = args["lat"]
        lon = args["lon"]
        radius = args.get("search_radius", 50.0)
        candidates = self._coord_resolver.resolve_goal(lat, lon, search_radius=radius)
        return {
            "candidates": [asdict(c) for c in candidates],
        }

    def _handle_get_lane_info(self, args: dict) -> dict:
        if self._coord_resolver is None:
            return {"error": "Coordinate resolver not initialized (no map loaded)"}
        x = args["x"]
        y = args["y"]
        radius = args.get("search_radius", 10.0)
        lanes = self._coord_resolver.get_lane_info(x, y, search_radius=radius)
        return {"lanes": lanes}

    # --- Command tools ---

    def _handle_set_goal(self, args: dict) -> dict:
        # Publish goal pose to /planning/mission_planning/goal
        # This uses the ADAPI service via topic
        from geometry_msgs.msg import PoseStamped
        import math

        x = args["x"]
        y = args["y"]
        z = args.get("z", 0.0)
        yaw = args["yaw_rad"]

        msg = PoseStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        # yaw to quaternion
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)

        if not hasattr(self._node, "_pub_goal"):
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
            qos = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                depth=1,
            )
            self._node._pub_goal = self._node.create_publisher(
                PoseStamped, "/planning/mission_planning/goal", qos
            )
        self._node._pub_goal.publish(msg)
        return {"status": "ok", "x": x, "y": y, "z": z, "yaw_rad": yaw}

    def _handle_engage(self, args: dict) -> dict:
        engage = args["engage"]
        self._node.set_engage(engage)
        return {"status": "ok", "engaged": engage}

    def _handle_emergency_stop(self, args: dict) -> dict:
        self._node.emergency_stop()
        return {"status": "ok", "message": "Emergency stop triggered"}

    def _handle_set_turn_indicators(self, args: dict) -> dict:
        command = args["command"]
        ok = self._node.send_turn_indicators(command)
        return {"status": "ok" if ok else "error", "command": command}

    def _handle_set_hazard_lights(self, args: dict) -> dict:
        command = args["command"]
        ok = self._node.send_hazard_lights(command)
        return {"status": "ok" if ok else "error", "command": command}

    def _handle_set_gate_mode(self, args: dict) -> dict:
        mode = args["mode"]
        ok = self._node.set_gate_mode(mode)
        return {"status": "ok" if ok else "error", "mode": mode}

    def _handle_send_control(self, args: dict) -> dict:
        steering = args["steering_rad"]
        velocity = args["velocity_mps"]
        accel = args.get("acceleration_mps2", 0.0)
        self._node.send_control(steering, velocity, accel)
        return {
            "status": "ok",
            "steering_rad": steering,
            "velocity_mps": velocity,
            "acceleration_mps2": accel,
        }

    def _handle_send_gear(self, args: dict) -> dict:
        gear = args["gear"]
        ok = self._node.send_gear(gear)
        return {"status": "ok" if ok else "error", "gear": gear}

    def _handle_start_heartbeat(self, args: dict) -> dict:
        self._node.start_heartbeat()
        return {"status": "ok", "message": "Heartbeat started"}

    def _handle_stop_heartbeat(self, args: dict) -> dict:
        self._node.stop_heartbeat()
        return {"status": "ok", "message": "Heartbeat stopped"}

    # ──────────────────────────────────────────────
    # Server lifecycle
    # ──────────────────────────────────────────────

    def _build_vehicle_context(self) -> str:
        """Build a system prompt section with current vehicle state."""
        state = self._node.get_vehicle_state()
        if not state.is_connected:
            return "Vehicle status: NOT CONNECTED (no data available)"
        return (
            f"Vehicle status (live):\n"
            f"  Position: x={state.x:.1f}, y={state.y:.1f}, z={state.z:.1f}\n"
            f"  Orientation: yaw={state.yaw:.3f} rad\n"
            f"  Speed: {state.velocity_mps:.2f} m/s ({state.velocity_mps * 3.6:.1f} km/h)\n"
            f"  Steering: {state.steering_tire_angle_rad:.3f} rad\n"
            f"  Gear: {GEAR_MAP.get(state.gear, 'UNKNOWN')}\n"
            f"  Operation mode: {state.operation_mode}\n"
            f"  Autoware control: {'enabled' if state.is_autoware_control_enabled else 'disabled'}\n"
            f"  Engaged: {'yes' if state.is_engaged else 'no'}\n"
            f"  MRM: {state.mrm_state}/{state.mrm_behavior}\n"
        )

    async def run_sse(self, host: str = "0.0.0.0", port: int = 8765) -> None:
        """Run MCP server with SSE transport."""
        from mcp.server.sse import SseServerTransport
        from starlette.applications import Starlette
        from starlette.routing import Route, Mount
        from starlette.requests import Request
        from starlette.responses import JSONResponse
        import uvicorn
        import httpx

        sse = SseServerTransport("/messages/")

        async def handle_sse(request):
            async with sse.connect_sse(
                request.scope, request.receive, request._send
            ) as (read_stream, write_stream):
                await self._server.run(
                    read_stream, write_stream, self._server.create_initialization_options()
                )

        async def handle_health(request):
            state = self._node.get_vehicle_state()
            return JSONResponse({"status": "ok", "connected": state.is_connected})

        async def _nemoclaw_chat(message: str, timeout_s: float = 120.0) -> str:
            """Send a chat message to NemoClaw via WebSocket and return the response."""
            import websockets

            gateway_url = self._nemoclaw_ws_url
            gateway_token = self._nemoclaw_token

            headers = {"Origin": f"http://127.0.0.1:{self._nemoclaw_port}"}
            async with websockets.connect(gateway_url, additional_headers=headers) as ws:
                # 1. Wait for challenge
                await asyncio.wait_for(ws.recv(), timeout=5.0)

                # 2. Connect with auth
                import uuid as _uuid
                connect_id = str(_uuid.uuid4())
                await ws.send(json.dumps({
                    "type": "req", "id": connect_id,
                    "method": "connect",
                    "params": {
                        "minProtocol": 3, "maxProtocol": 3,
                        "client": {
                            "id": "openclaw-control-ui",
                            "version": "1.0.0",
                            "platform": "linux",
                            "mode": "webchat",
                        },
                        "role": "operator",
                        "scopes": ["operator.read", "operator.write"],
                        "auth": {"token": gateway_token},
                        "caps": ["tool-events"],
                    },
                }))

                # Wait for connect OK
                connected = False
                for _ in range(5):
                    msg = await asyncio.wait_for(ws.recv(), timeout=5.0)
                    data = json.loads(msg)
                    if data.get("type") == "res":
                        if data.get("ok"):
                            connected = True
                            break
                        else:
                            err = data.get("error", {}).get("message", "unknown error")
                            raise ConnectionError(f"NemoClaw connect failed: {err}")

                if not connected:
                    raise ConnectionError("NemoClaw connect: no response")

                # 3. Send chat message
                session_key = str(_uuid.uuid4())
                chat_id = str(_uuid.uuid4())
                await ws.send(json.dumps({
                    "type": "req", "id": chat_id,
                    "method": "chat.send",
                    "params": {
                        "sessionKey": session_key,
                        "message": message,
                        "deliver": True,
                        "idempotencyKey": str(_uuid.uuid4()),
                    },
                }))

                # 4. Collect streamed response until lifecycle/end or chat/final
                full_text = ""
                deadline = asyncio.get_event_loop().time() + timeout_s
                while asyncio.get_event_loop().time() < deadline:
                    remaining = deadline - asyncio.get_event_loop().time()
                    try:
                        msg = await asyncio.wait_for(ws.recv(), timeout=min(remaining, 30.0))
                    except asyncio.TimeoutError:
                        break
                    data = json.loads(msg)
                    if data.get("type") != "event":
                        continue
                    payload = data.get("payload", {})
                    event = data.get("event", "")

                    if event == "agent":
                        stream = payload.get("stream", "")
                        pdata = payload.get("data", {})
                        if stream == "assistant":
                            delta = pdata.get("delta", "")
                            if delta:
                                full_text += delta
                        elif stream == "lifecycle" and pdata.get("phase") == "end":
                            break
                    elif event == "chat":
                        state = payload.get("state", "")
                        if state == "final":
                            # Extract final text from structured content
                            msg_obj = payload.get("message", {})
                            content = msg_obj.get("content", [])
                            if content and isinstance(content, list):
                                for part in content:
                                    if isinstance(part, dict) and part.get("type") == "text":
                                        full_text = part.get("text", full_text)
                            break

                return full_text

        async def handle_chat(request: Request):
            """Chat proxy: forwards messages to NemoClaw agent via WebSocket."""
            try:
                body = await request.json()
            except Exception:
                return JSONResponse({"error": "invalid JSON"}, status_code=400)

            user_message = body.get("message", "").strip()
            if not user_message:
                return JSONResponse({"error": "message is required"}, status_code=400)

            try:
                response_text = await _nemoclaw_chat(user_message)
                return JSONResponse({"response": response_text})
            except ConnectionError as e:
                return JSONResponse({"error": str(e)}, status_code=502)
            except Exception as e:
                self._node.get_logger().error(f"Chat proxy error: {e}")
                return JSONResponse({"error": f"NemoClaw error: {e}"}, status_code=502)

        routes = [
            Route("/health", handle_health),
            Route("/chat", handle_chat, methods=["POST"]),
            Route("/sse", endpoint=handle_sse),
            Mount("/messages/", app=sse.handle_post_message),
        ]
        if hasattr(sse, 'get_streamable_http_app'):
            routes.insert(2, Mount("/sse", app=sse.get_streamable_http_app()))

        app = Starlette(routes=routes)

        self._node.get_logger().info(f"MCP SSE server starting on {host}:{port}")
        config = uvicorn.Config(app, host=host, port=port, log_level="info")
        server = uvicorn.Server(config)
        await server.serve()

    async def run_stdio(self) -> None:
        """Run MCP server with stdio transport."""
        from mcp.server.stdio import stdio_server

        async with stdio_server() as (read_stream, write_stream):
            await self._server.run(
                read_stream, write_stream, self._server.create_initialization_options()
            )


def main():
    """Entry point for autoware_mcp_server node."""
    rclpy.init()
    node = AutowareROSNode()

    # Read parameters
    node.declare_parameter("transport", "sse")
    node.declare_parameter("host", "0.0.0.0")
    node.declare_parameter("port", 8765)
    node.declare_parameter("map_path", "")
    node.declare_parameter("map_origin_lat", 0.0)
    node.declare_parameter("map_origin_lon", 0.0)
    node.declare_parameter("map_origin_alt", 0.0)
    node.declare_parameter("ollama_url", "http://127.0.0.1:11435")
    node.declare_parameter("ollama_model", "gemma4")
    node.declare_parameter("nemoclaw_port", 18789)
    node.declare_parameter("nemoclaw_token", "")

    transport = node.get_parameter("transport").get_parameter_value().string_value
    host = node.get_parameter("host").get_parameter_value().string_value
    port = node.get_parameter("port").get_parameter_value().integer_value
    map_path = node.get_parameter("map_path").get_parameter_value().string_value
    origin_lat = node.get_parameter("map_origin_lat").get_parameter_value().double_value
    origin_lon = node.get_parameter("map_origin_lon").get_parameter_value().double_value
    origin_alt = node.get_parameter("map_origin_alt").get_parameter_value().double_value
    ollama_url = node.get_parameter("ollama_url").get_parameter_value().string_value
    ollama_model = node.get_parameter("ollama_model").get_parameter_value().string_value
    nemoclaw_port = node.get_parameter("nemoclaw_port").get_parameter_value().integer_value
    nemoclaw_token = node.get_parameter("nemoclaw_token").get_parameter_value().string_value

    # Auto-detect NemoClaw token if not provided
    if not nemoclaw_token:
        import subprocess
        try:
            result = subprocess.run(
                ["docker", "exec", "autoware_claw-nemoclaw-1", "python3", "-c",
                 "import json; print(json.load(open('/sandbox/.openclaw/openclaw.json'))"
                 ".get('gateway',{}).get('auth',{}).get('token',''))"],
                capture_output=True, text=True, timeout=5,
            )
            nemoclaw_token = result.stdout.strip()
            if nemoclaw_token:
                node.get_logger().info("NemoClaw gateway token auto-detected")
        except Exception:
            node.get_logger().warn("Could not auto-detect NemoClaw token — chat proxy disabled")

    # Create MCP server
    mcp_server = AutowareMCPServer(
        node,
        ollama_url=ollama_url,
        ollama_model=ollama_model,
        nemoclaw_port=nemoclaw_port,
        nemoclaw_token=nemoclaw_token,
    )
    mcp_server.init_coordinate_resolver(map_path, origin_lat, origin_lon, origin_alt)

    # Start ROS 2 executor in background thread
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    # Start heartbeat
    node.start_heartbeat()

    # Run MCP server (blocks)
    try:
        if transport == "sse":
            asyncio.run(mcp_server.run_sse(host=host, port=port))
        else:
            asyncio.run(mcp_server.run_stdio())
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down...")
    finally:
        node.stop_heartbeat()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
