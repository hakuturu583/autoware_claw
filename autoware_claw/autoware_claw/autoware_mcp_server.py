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
        # OpenAI-compatible API
        openai_api_key: str = "",
        openai_base_url: str = "https://api.openai.com/v1",
        openai_model: str = "gpt-4.1",
        # Google Maps API (Geocoding + Roads)
        google_maps_api_key: str = "",
        # # Ollama (commented out — gemma4 does not support function calling)
        # ollama_url: str = "http://127.0.0.1:11435",
        # ollama_model: str = "gemma4",
        nemoclaw_port: int = 18789,
        nemoclaw_token: str = "",
    ) -> None:
        self._node = ros_node
        self._server = Server("rosclaw-autoware-mcp")
        self._coord_resolver: CoordinateResolver | None = None
        self._openai_api_key = openai_api_key
        self._openai_base_url = openai_base_url.rstrip("/")
        self._openai_model = openai_model
        self._google_maps_api_key = google_maps_api_key
        # # Ollama (commented out)
        # self._ollama_url = ollama_url
        # self._ollama_model = ollama_model
        self._nemoclaw_port = nemoclaw_port
        self._nemoclaw_ws_url = f"ws://127.0.0.1:{nemoclaw_port}"
        self._nemoclaw_token = nemoclaw_token
        self._register_tools()

    def init_coordinate_resolver_from_topic(self) -> None:
        """Register callback to initialize coordinate resolver when vector map arrives."""
        def _on_map(data: bytes) -> None:
            if self._coord_resolver is not None:
                return  # already initialized
            try:
                self._coord_resolver = CoordinateResolver.from_bin_data(data)
                self._node.get_logger().info(
                    "Coordinate resolver initialized from /map/vector_map"
                )
            except Exception as e:
                self._node.get_logger().error(
                    f"Failed to initialize coordinate resolver: {e}"
                )

        # If map already available, use it immediately
        existing = self._node.get_vector_map_bin()
        if existing:
            _on_map(existing)
        # Also register for future updates
        self._node.set_on_vector_map_callback(_on_map)

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
                    name="autoware_set_velocity_limit",
                    description=(
                        "Set the maximum velocity limit for autonomous driving. "
                        "Accepts speed in km/h (converted to m/s internally). "
                        "The vehicle will not exceed this speed."
                    ),
                    inputSchema={
                        "type": "object",
                        "properties": {
                            "velocity_kmh": {
                                "type": "number",
                                "description": "Maximum velocity in km/h (must be > 0).",
                            },
                        },
                        "required": ["velocity_kmh"],
                    },
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
            "autoware_set_velocity_limit": self._handle_set_velocity_limit,
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

    def _handle_set_velocity_limit(self, args: dict) -> dict:
        velocity_kmh = args["velocity_kmh"]
        if velocity_kmh <= 0:
            return {"error": "velocity_kmh must be greater than 0"}
        velocity_mps = velocity_kmh / 3.6
        result = self._node.set_velocity_limit(velocity_mps)
        if not result["success"]:
            return {"error": result["message"]}
        return {
            "status": "ok",
            "velocity_kmh": velocity_kmh,
            "velocity_mps": round(velocity_mps, 3),
        }

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

    # ──────────────────────────────────────────────
    # Local Ollama agent (tool-calling)
    # ──────────────────────────────────────────────

    async def _geocode(self, query: str) -> dict:
        """Geocode a place name to lat/lon using Google Maps Geocoding API,
        then snap to nearest road via Roads API."""
        import httpx

        logger = self._node.get_logger()
        logger.info(f"[geocode] Resolving via Google Maps: {query}")

        if not self._google_maps_api_key:
            return {"error": "GOOGLE_MAPS_API_KEY is not configured"}

        try:
            async with httpx.AsyncClient(timeout=15.0) as client:
                # 1. Geocode: place name -> lat/lon
                geo_resp = await client.get(
                    "https://maps.googleapis.com/maps/api/geocode/json",
                    params={
                        "address": query,
                        "language": "ja",
                        "key": self._google_maps_api_key,
                    },
                )
                geo_resp.raise_for_status()
                geo_data = geo_resp.json()

                if geo_data.get("status") != "OK" or not geo_data.get("results"):
                    return {"error": f"Geocoding failed: {geo_data.get('status', 'no results')}"}

                top = geo_data["results"][0]
                location = top["geometry"]["location"]
                lat = location["lat"]
                lon = location["lng"]
                display_name = top.get("formatted_address", query)
                logger.info(f"[geocode] Geocoded: {display_name} ({lat}, {lon})")

                # 2. Snap to nearest road via Roads API
                roads_resp = await client.get(
                    "https://roads.googleapis.com/v1/nearestRoads",
                    params={
                        "points": f"{lat},{lon}",
                        "key": self._google_maps_api_key,
                    },
                )
                roads_resp.raise_for_status()
                roads_data = roads_resp.json()

                snapped = roads_data.get("snappedPoints", [])
                if snapped:
                    snapped_loc = snapped[0]["location"]
                    snapped_lat = snapped_loc["latitude"]
                    snapped_lon = snapped_loc["longitude"]
                    place_id = snapped[0].get("placeId", "")
                    logger.info(
                        f"[geocode] Snapped to road: ({snapped_lat}, {snapped_lon}) "
                        f"placeId={place_id}"
                    )
                    return {
                        "lat": snapped_lat,
                        "lon": snapped_lon,
                        "original_lat": lat,
                        "original_lon": lon,
                        "snapped_to_road": True,
                        "place_id": place_id,
                        "display_name": display_name,
                    }

                # Roads API returned no snapped points — return raw geocode result
                logger.info("[geocode] No nearby road found, returning raw geocode result")
                return {
                    "lat": lat,
                    "lon": lon,
                    "snapped_to_road": False,
                    "display_name": display_name,
                }
        except Exception as e:
            logger.error(f"[geocode] Google Maps API error: {e}")
            return {"error": f"Google Maps API failed: {e}"}

    def _build_openai_tools(self) -> list[dict]:
        """Build OpenAI function-calling tool definitions."""
        tools = []
        for tool in self._tools:
            tools.append({
                "type": "function",
                "function": {
                    "name": tool.name,
                    "description": tool.description,
                    "parameters": tool.inputSchema,
                },
            })
        tools.append({
            "type": "function",
            "function": {
                "name": "geocode",
                "description": (
                    "Convert a place name or address to latitude and longitude, "
                    "snapped to the nearest road via Google Maps Roads API. "
                    "Use this when the user mentions a destination by name."
                ),
                "parameters": {
                    "type": "object",
                    "properties": {
                        "query": {
                            "type": "string",
                            "description": "Place name or address to geocode.",
                        },
                    },
                    "required": ["query"],
                },
            },
        })
        return tools

    async def _local_agent_chat(self, message: str) -> dict:
        """Handle chat via OpenAI-compatible API with native function calling."""
        import httpx

        logger = self._node.get_logger()

        if not self._openai_api_key:
            raise RuntimeError("OpenAI API key is not configured")

        openai_tools = self._build_openai_tools()

        system_prompt = (
            "You are Autoware CLAW, an autonomous driving assistant controlling a real vehicle.\n"
            "IMPORTANT: You MUST use tools to perform every action. "
            "Never claim to have done something without calling the corresponding tool.\n\n"
            "When the user asks to navigate somewhere:\n"
            "1. Use geocode to get lat/lon from the place name\n"
            "2. Use autoware_resolve_goal to get lane-aligned goal candidates\n"
            "3. Use autoware_set_goal with the first candidate\n"
            "4. Use autoware_engage with engage=true to start autonomous driving\n"
            "5. Report the result to the user\n\n"
            f"{self._build_vehicle_context()}\n"
            "Respond in the same language as the user."
        )

        messages = [
            {"role": "system", "content": system_prompt},
            {"role": "user", "content": message},
        ]
        tool_calls_log = []

        headers = {
            "Content-Type": "application/json",
            "Authorization": f"Bearer {self._openai_api_key}",
        }

        async with httpx.AsyncClient(timeout=120.0) as client:
            for iteration in range(10):
                logger.info(f"[agent] Iteration {iteration + 1}")
                try:
                    resp = await client.post(
                        f"{self._openai_base_url}/chat/completions",
                        headers=headers,
                        json={
                            "model": self._openai_model,
                            "messages": messages,
                            "tools": openai_tools,
                        },
                        timeout=60.0,
                    )
                    resp.raise_for_status()
                    result = resp.json()
                except Exception as e:
                    logger.error(f"[agent] OpenAI API error: {e}")
                    return {"text": f"Agent error: {e}", "tool_calls": tool_calls_log}

                choice = result.get("choices", [{}])[0]
                assistant_msg = choice.get("message", {})
                finish_reason = choice.get("finish_reason", "")
                content = assistant_msg.get("content") or ""
                tool_calls = assistant_msg.get("tool_calls") or []

                if not tool_calls:
                    logger.info(f"[agent] Final response: {content[:100]}")
                    return {"text": content, "tool_calls": tool_calls_log}

                # Append the full assistant message (with tool_calls) to history
                messages.append(assistant_msg)

                for tc in tool_calls:
                    tc_id = tc.get("id", "")
                    func = tc.get("function", {})
                    name = func.get("name", "unknown")
                    raw_args = func.get("arguments", "{}")
                    try:
                        args = json.loads(raw_args) if isinstance(raw_args, str) else raw_args
                    except json.JSONDecodeError:
                        args = {}

                    logger.info(
                        f"[agent] Tool call: {name}"
                        f"({json.dumps(args, ensure_ascii=False)[:200]})"
                    )
                    tool_calls_log.append(name)

                    if name == "geocode":
                        tool_result = await self._geocode(args.get("query", ""))
                    else:
                        handler = self._tool_handlers.get(name)
                        if handler:
                            try:
                                tool_result = handler(args)
                            except Exception as e:
                                tool_result = {"error": str(e)}
                        else:
                            tool_result = {"error": f"Unknown tool: {name}"}

                    result_str = json.dumps(
                        tool_result, ensure_ascii=False, default=str
                    )
                    logger.info(f"[agent] Tool result: {name} -> {result_str[:200]}")
                    messages.append({
                        "role": "tool",
                        "tool_call_id": tc_id,
                        "content": result_str,
                    })

        return {
            "text": "Tool execution limit reached.",
            "tool_calls": tool_calls_log,
        }

    # ──────────────────────────────────────────────
    # HTTP / SSE server
    # ──────────────────────────────────────────────

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

            logger = self._node.get_logger()
            gateway_url = self._nemoclaw_ws_url
            gateway_token = self._nemoclaw_token

            logger.info(f"[chat] Sending to NemoClaw: {message[:80]}")
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
                            logger.info("[chat] WebSocket connected to NemoClaw")
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
                tool_calls = []
                deadline = asyncio.get_event_loop().time() + timeout_s
                while asyncio.get_event_loop().time() < deadline:
                    remaining = deadline - asyncio.get_event_loop().time()
                    try:
                        msg = await asyncio.wait_for(ws.recv(), timeout=min(remaining, 30.0))
                    except asyncio.TimeoutError:
                        logger.warn("[chat] Timeout waiting for NemoClaw response")
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
                        elif stream == "tool-call":
                            tool_name = pdata.get("name", pdata.get("tool", "unknown"))
                            logger.info(f"[chat] Tool call: {tool_name}")
                            tool_calls.append(tool_name)
                        elif stream == "tool-result":
                            tool_name = pdata.get("name", pdata.get("tool", "unknown"))
                            status = pdata.get("status", "")
                            logger.info(f"[chat] Tool result: {tool_name} -> {status}")
                        elif stream == "lifecycle":
                            phase = pdata.get("phase", "")
                            logger.info(f"[chat] Lifecycle: {phase}")
                            if phase == "end":
                                break
                        elif stream == "tool":
                            phase = pdata.get("phase", "")
                            name = pdata.get("name", "?")
                            is_error = pdata.get("isError", False)
                            if phase == "start":
                                logger.info(f"[chat] Tool call: {name}({json.dumps(pdata.get('args', {}), ensure_ascii=False)[:200]})")
                                tool_calls.append(name)
                            elif phase == "end":
                                logger.info(f"[chat] Tool done: {name} error={is_error}")
                        else:
                            pass  # skip item, status, etc.
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
                            logger.info("[chat] Received final response")
                            break
                        else:
                            logger.info(f"[chat] Chat event: state={state}")
                    elif event == "error":
                        err_msg = payload.get("message", str(payload))
                        logger.error(f"[chat] Agent error: {err_msg}")
                    else:
                        logger.info(f"[chat] Event: {event} payload_keys={list(payload.keys())}")

                if tool_calls:
                    logger.info(f"[chat] Tools used: {', '.join(tool_calls)}")
                logger.info(f"[chat] Response: {full_text[:100]}...")
                return {"text": full_text, "tool_calls": tool_calls}

        async def handle_chat(request: Request):
            """Chat handler: uses OpenAI agent with tool calling,
            falls back to NemoClaw proxy only when API key is missing."""
            try:
                body = await request.json()
            except Exception:
                return JSONResponse({"error": "invalid JSON"}, status_code=400)

            user_message = body.get("message", "").strip()
            if not user_message:
                return JSONResponse({"error": "message is required"}, status_code=400)

            # Primary: OpenAI agent (reliable function calling)
            if self._openai_api_key:
                try:
                    self._node.get_logger().info(
                        f"[chat] Using OpenAI agent ({self._openai_model})"
                    )
                    result = await self._local_agent_chat(user_message)
                    return JSONResponse({
                        "response": result["text"],
                        "tool_calls": result["tool_calls"],
                    })
                except Exception as e:
                    self._node.get_logger().error(f"[chat] OpenAI agent error: {e}")
                    return JSONResponse(
                        {"error": f"OpenAI agent error: {e}"}, status_code=502
                    )

            # Fallback: NemoClaw proxy (only when no API key)
            self._node.get_logger().info("[chat] No OpenAI key, using NemoClaw proxy")
            try:
                result = await _nemoclaw_chat(user_message)
                return JSONResponse({
                    "response": result["text"],
                    "tool_calls": result["tool_calls"],
                })
            except ConnectionError as e:
                return JSONResponse({"error": str(e)}, status_code=502)
            except Exception as e:
                self._node.get_logger().error(f"Chat error: {e}")
                return JSONResponse({"error": f"Chat error: {e}"}, status_code=502)

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
    # OpenAI-compatible API
    node.declare_parameter("openai_api_key", "")
    node.declare_parameter("openai_base_url", "https://api.openai.com/v1")
    node.declare_parameter("openai_model", "gpt-4.1")
    # # Ollama (commented out — gemma4 does not support function calling)
    # node.declare_parameter("ollama_url", "http://127.0.0.1:11435")
    # node.declare_parameter("ollama_model", "gemma4")
    node.declare_parameter("nemoclaw_port", 18789)
    node.declare_parameter("nemoclaw_token", "")

    transport = node.get_parameter("transport").get_parameter_value().string_value
    host = node.get_parameter("host").get_parameter_value().string_value
    port = node.get_parameter("port").get_parameter_value().integer_value
    openai_api_key = node.get_parameter("openai_api_key").get_parameter_value().string_value
    openai_base_url = node.get_parameter("openai_base_url").get_parameter_value().string_value
    openai_model = node.get_parameter("openai_model").get_parameter_value().string_value
    # # Ollama (commented out)
    # ollama_url = node.get_parameter("ollama_url").get_parameter_value().string_value
    # ollama_model = node.get_parameter("ollama_model").get_parameter_value().string_value
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

    # Auto-detect API keys from environment if not set via parameter
    import os
    if not openai_api_key:
        openai_api_key = os.environ.get("OPENAI_API_KEY", "")
        if openai_api_key:
            node.get_logger().info("OpenAI API key loaded from OPENAI_API_KEY env var")
        else:
            node.get_logger().warn("No OpenAI API key — local agent chat disabled")

    google_maps_api_key = os.environ.get("GOOGLE_MAPS_API_KEY", "")
    if google_maps_api_key:
        node.get_logger().info("Google Maps API key loaded from GOOGLE_MAPS_API_KEY env var")
    else:
        node.get_logger().warn("No Google Maps API key — geocode disabled")

    # Create MCP server
    mcp_server = AutowareMCPServer(
        node,
        openai_api_key=openai_api_key,
        openai_base_url=openai_base_url,
        openai_model=openai_model,
        google_maps_api_key=google_maps_api_key,
        # # Ollama (commented out)
        # ollama_url=ollama_url,
        # ollama_model=ollama_model,
        nemoclaw_port=nemoclaw_port,
        nemoclaw_token=nemoclaw_token,
    )
    mcp_server.init_coordinate_resolver_from_topic()

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
