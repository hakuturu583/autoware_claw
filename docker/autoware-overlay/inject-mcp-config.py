#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Inject Autoware MCP tool server into openclaw.json.

Run after generate-openclaw-config.py to add the Autoware MCP SSE endpoint
as a tool server that OpenClaw agents can use for vehicle control.

Environment variables:
    AUTOWARE_MCP_URL  URL of the Autoware MCP server (default: http://host.docker.internal:8765)
"""

from __future__ import annotations

import json
import os


def main() -> None:
    mcp_url = os.environ.get("AUTOWARE_MCP_URL", "http://host.docker.internal:8765")
    config_path = os.path.expanduser("~/.openclaw/openclaw.json")

    with open(config_path) as f:
        config = json.load(f)

    # Add MCP server configuration for Autoware
    # OpenClaw's mcpServers key maps server names to their transport config.
    config.setdefault("mcpServers", {})
    config["mcpServers"]["autoware"] = {
        "url": f"{mcp_url}/sse",
        "transport": {
            "type": "sse",
        },
    }

    with open(config_path, "w") as f:
        json.dump(config, f, indent=2)

    print(f"[autoware_claw] Injected Autoware MCP server: {mcp_url}/sse")


if __name__ == "__main__":
    main()
