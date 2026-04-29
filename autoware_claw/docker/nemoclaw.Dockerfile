# Autoware NemoClaw — NVIDIA NemoClaw sandbox with Autoware MCP + Ollama inference
#
# Builds the official NemoClaw sandbox image from source, then layers
# Autoware-specific configuration:
#   - Ollama inference provider (Docker internal network)
#   - Autoware MCP tool server endpoint (host network via host.docker.internal)
#   - Custom network policy allowing the Autoware MCP endpoint
#
# Usage:
#   docker compose build nemoclaw   (from autoware_claw package root)
#   colcon build --packages-select autoware_claw  (automatic via CMake)

# Global ARG — must be before the first FROM to be visible in FROM directives.
ARG BASE_IMAGE=ghcr.io/nvidia/nemoclaw/sandbox-base:latest

# ── Stage 1: Clone NemoClaw source and build TypeScript plugin ──
FROM node:22-slim AS nemoclaw-builder

ENV NPM_CONFIG_AUDIT=false \
    NPM_CONFIG_FUND=false \
    NPM_CONFIG_UPDATE_NOTIFIER=false

RUN apt-get update && apt-get install -y --no-install-recommends git ca-certificates \
    && rm -rf /var/lib/apt/lists/*

ARG NEMOCLAW_VERSION=main
RUN git clone --depth 1 --branch ${NEMOCLAW_VERSION} \
    https://github.com/NVIDIA/NemoClaw.git /opt/nemoclaw-src

# Build NemoClaw TypeScript plugin
WORKDIR /opt/nemoclaw-src/nemoclaw
RUN npm ci && npm run build

# ── Stage 2: Runtime — NemoClaw sandbox-base + Autoware config ──
FROM ${BASE_IMAGE}

# Copy built NemoClaw plugin from builder stage
COPY --from=nemoclaw-builder /opt/nemoclaw-src/nemoclaw/dist/ /opt/nemoclaw/dist/
COPY --from=nemoclaw-builder /opt/nemoclaw-src/nemoclaw/openclaw.plugin.json /opt/nemoclaw/
COPY --from=nemoclaw-builder /opt/nemoclaw-src/nemoclaw/package.json \
     /opt/nemoclaw-src/nemoclaw/package-lock.json /opt/nemoclaw/

# Copy blueprint (base NemoClaw blueprint + our network policy overlay)
COPY --from=nemoclaw-builder /opt/nemoclaw-src/nemoclaw-blueprint/ /opt/nemoclaw-blueprint/

# Copy startup and config generation scripts
COPY --from=nemoclaw-builder /opt/nemoclaw-src/scripts/nemoclaw-start.sh /usr/local/bin/nemoclaw-start
COPY --from=nemoclaw-builder /opt/nemoclaw-src/scripts/lib/sandbox-init.sh /usr/local/lib/nemoclaw/sandbox-init.sh
COPY --from=nemoclaw-builder /opt/nemoclaw-src/scripts/generate-openclaw-config.py /usr/local/lib/nemoclaw/generate-openclaw-config.py
RUN chmod 755 /usr/local/bin/nemoclaw-start /usr/local/lib/nemoclaw/sandbox-init.sh

# Install NemoClaw runtime dependencies
WORKDIR /opt/nemoclaw
RUN npm ci --omit=dev

# Set up blueprint for local resolution
RUN mkdir -p /sandbox/.nemoclaw/blueprints/0.1.0 \
    && cp -r /opt/nemoclaw-blueprint/* /sandbox/.nemoclaw/blueprints/0.1.0/

# ── Autoware-specific: Ollama inference configuration ──
# Ollama exposes an OpenAI-compatible API at /v1/
ARG NEMOCLAW_MODEL=gemma4
ARG NEMOCLAW_PROVIDER_KEY=ollama
ARG NEMOCLAW_PRIMARY_MODEL_REF=gemma4
ARG NEMOCLAW_INFERENCE_BASE_URL=http://ollama:11434/v1
ARG NEMOCLAW_INFERENCE_API=openai-completions
ARG NEMOCLAW_CONTEXT_WINDOW=131072
ARG NEMOCLAW_MAX_TOKENS=4096
ARG NEMOCLAW_REASONING=false
ARG NEMOCLAW_INFERENCE_INPUTS=text
ARG NEMOCLAW_AGENT_TIMEOUT=600
ARG NEMOCLAW_INFERENCE_COMPAT_B64=e30=
ARG NEMOCLAW_DISABLE_DEVICE_AUTH=1
ARG CHAT_UI_URL=http://0.0.0.0:18789
ARG NEMOCLAW_PROXY_HOST=10.200.0.1
ARG NEMOCLAW_PROXY_PORT=3128
ARG NEMOCLAW_WEB_SEARCH_ENABLED=0
ARG NEMOCLAW_MESSAGING_CHANNELS_B64=W10=
ARG NEMOCLAW_MESSAGING_ALLOWED_IDS_B64=e30=
ARG NEMOCLAW_DISCORD_GUILDS_B64=e30=
ARG NEMOCLAW_BUILD_ID=default

ENV NEMOCLAW_MODEL=${NEMOCLAW_MODEL} \
    NEMOCLAW_PROVIDER_KEY=${NEMOCLAW_PROVIDER_KEY} \
    NEMOCLAW_PRIMARY_MODEL_REF=${NEMOCLAW_PRIMARY_MODEL_REF} \
    CHAT_UI_URL=${CHAT_UI_URL} \
    NEMOCLAW_INFERENCE_BASE_URL=${NEMOCLAW_INFERENCE_BASE_URL} \
    NEMOCLAW_INFERENCE_API=${NEMOCLAW_INFERENCE_API} \
    NEMOCLAW_CONTEXT_WINDOW=${NEMOCLAW_CONTEXT_WINDOW} \
    NEMOCLAW_MAX_TOKENS=${NEMOCLAW_MAX_TOKENS} \
    NEMOCLAW_REASONING=${NEMOCLAW_REASONING} \
    NEMOCLAW_INFERENCE_INPUTS=${NEMOCLAW_INFERENCE_INPUTS} \
    NEMOCLAW_AGENT_TIMEOUT=${NEMOCLAW_AGENT_TIMEOUT} \
    NEMOCLAW_INFERENCE_COMPAT_B64=${NEMOCLAW_INFERENCE_COMPAT_B64} \
    NEMOCLAW_MESSAGING_CHANNELS_B64=${NEMOCLAW_MESSAGING_CHANNELS_B64} \
    NEMOCLAW_MESSAGING_ALLOWED_IDS_B64=${NEMOCLAW_MESSAGING_ALLOWED_IDS_B64} \
    NEMOCLAW_DISCORD_GUILDS_B64=${NEMOCLAW_DISCORD_GUILDS_B64} \
    NEMOCLAW_DISABLE_DEVICE_AUTH=${NEMOCLAW_DISABLE_DEVICE_AUTH} \
    NEMOCLAW_PROXY_HOST=${NEMOCLAW_PROXY_HOST} \
    NEMOCLAW_PROXY_PORT=${NEMOCLAW_PROXY_PORT} \
    NEMOCLAW_WEB_SEARCH_ENABLED=${NEMOCLAW_WEB_SEARCH_ENABLED}

# Generate openclaw.json from env vars (NemoClaw's standard mechanism)
WORKDIR /sandbox
USER sandbox
RUN python3 /usr/local/lib/nemoclaw/generate-openclaw-config.py

# ── Autoware-specific: inject MCP server config into openclaw.json ──
# Add Autoware MCP tool server so OpenClaw can access vehicle control tools.
# This runs after generate-openclaw-config.py which creates the base config.
COPY docker/autoware-overlay/inject-mcp-config.py /tmp/inject-mcp-config.py
ARG AUTOWARE_MCP_URL=http://host.docker.internal:8765
RUN AUTOWARE_MCP_URL="${AUTOWARE_MCP_URL}" python3 /tmp/inject-mcp-config.py

# Install NemoClaw plugin into OpenClaw
RUN openclaw doctor --fix > /dev/null 2>&1 || true \
    && openclaw plugins install /opt/nemoclaw > /dev/null 2>&1 || true

# Inject gateway auth token (must be last mutable layer — cache busted by BUILD_ID)
RUN NEMOCLAW_BUILD_ID="${NEMOCLAW_BUILD_ID}" python3 -c "\
import json, os, secrets; \
path = os.path.expanduser('~/.openclaw/openclaw.json'); \
cfg = json.load(open(path)); \
cfg.setdefault('gateway', {}).setdefault('auth', {})['token'] = secrets.token_hex(32); \
json.dump(cfg, open(path, 'w'), indent=2); \
os.chmod(path, 0o600)"

# Lock config (root ownership)
USER root
RUN chown root:root /sandbox/.openclaw \
    && find /sandbox/.openclaw -mindepth 1 -maxdepth 1 -exec chown -h root:root {} + \
    && chmod 755 /sandbox/.openclaw \
    && chmod 444 /sandbox/.openclaw/openclaw.json \
    && sha256sum /sandbox/.openclaw/openclaw.json > /sandbox/.openclaw/.config-hash \
    && chmod 444 /sandbox/.openclaw/.config-hash \
    && chown root:root /sandbox/.openclaw/.config-hash

# Lock .nemoclaw directory (blueprint immutability)
RUN chown root:root /sandbox/.nemoclaw \
    && chmod 1755 /sandbox/.nemoclaw \
    && chown -R root:root /sandbox/.nemoclaw/blueprints \
    && chmod -R 755 /sandbox/.nemoclaw/blueprints \
    && mkdir -p /sandbox/.nemoclaw/state /sandbox/.nemoclaw/migration \
               /sandbox/.nemoclaw/snapshots /sandbox/.nemoclaw/staging \
    && chown sandbox:sandbox /sandbox/.nemoclaw/state /sandbox/.nemoclaw/migration \
               /sandbox/.nemoclaw/snapshots /sandbox/.nemoclaw/staging \
    && touch /sandbox/.nemoclaw/config.json \
    && chown sandbox:sandbox /sandbox/.nemoclaw/config.json

ENTRYPOINT ["/usr/local/bin/nemoclaw-start"]
CMD ["/bin/bash"]
