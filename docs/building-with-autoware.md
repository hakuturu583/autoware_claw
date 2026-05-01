# Building Autoware CLAW with Autoware

This guide explains how to add `autoware_claw` to an existing Autoware workspace and build everything together.

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- Autoware workspace already set up ([Autoware installation guide](https://autowarefoundation.github.io/autoware-documentation/main/installation/))
- Docker with [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
- NVIDIA GPU (for Ollama inference)

### Python Dependencies

Install the MCP and ROSClaw Python packages:

```bash
pip3 install mcp rosclaw uvicorn starlette
```

## Step 1: Clone into the Autoware Workspace

From the root of your Autoware workspace, clone `autoware_claw` into the external packages directory:

```bash
cd ~/workspace/autoware
git clone https://github.com/hakuturu583/autoware_claw.git \
  src/universe/external/autoware_claw
```

The resulting directory structure should look like:

```
~/workspace/autoware/
├── src/
│   ├── core/
│   ├── universe/
│   │   ├── autoware_universe/
│   │   └── external/
│   │       └── autoware_claw/        # <-- cloned here
│   │           ├── autoware_claw/            # MCP server package
│   │           └── autoware_claw_rviz_plugins/  # RViz chat panel
│   └── ...
├── build/
├── install/
└── repositories/
```

## Step 2: Build

### Full Workspace Build

Build the entire Autoware workspace including `autoware_claw`:

```bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

### Build Only autoware_claw

If the rest of Autoware is already built, you can build just `autoware_claw`:

```bash
colcon build --packages-select autoware_claw autoware_claw_rviz_plugins
source install/setup.bash
```

### Build Without Docker (faster)

To skip the NemoClaw/Ollama Docker image build (useful for development):

```bash
colcon build --packages-select autoware_claw autoware_claw_rviz_plugins \
  --cmake-args -DBUILD_DOCKER=OFF
```

### CUDA Architecture

If you encounter `nvcc fatal: Unsupported gpu architecture` errors, specify your GPU's compute capability:

```bash
# RTX 4090 (Ada Lovelace, compute_89)
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=89

# RTX 3090 (Ampere, compute_86)
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CUDA_ARCHITECTURES=86
```

## Step 3: Set Up Environment Variables (Optional)

For NemoClaw web search, set the Brave API key:

```bash
# Add to ~/.bashrc
export BRAVE_API_KEY=your-brave-api-key
```

## Step 4: Launch

### Planning Simulator with Autoware CLAW

```bash
source install/setup.bash

ros2 launch autoware_claw planning_simulator.launch.xml \
  map_path:=$HOME/Downloads/shinjyuku/Shinjuku-Map/map
```

This single command starts:

1. **Autoware planning simulator** — loads the Lanelet2 + point cloud map
2. **Autoware MCP server** — listens on `127.0.0.1:8765`
3. **NemoClaw + Ollama Docker containers** — LLM agent + inference
4. **RViz** — with the Claw chat panel pre-configured

### Without Docker Containers

If you want to use an external LLM agent instead of NemoClaw:

```bash
ros2 launch autoware_claw planning_simulator.launch.xml \
  map_path:=$HOME/Downloads/shinjyuku/Shinjuku-Map/map \
  launch_nemoclaw_docker:=false
```

### Without Perception (lighter)

If perception packages (`autoware_shape_estimation`, etc.) are not built:

```bash
ros2 launch autoware_claw planning_simulator.launch.xml \
  map_path:=$HOME/Downloads/shinjyuku/Shinjuku-Map/map \
  perception/enable_object_recognition:=false \
  perception/enable_detection_failure:=false
```

### MCP Server Standalone

To run only the MCP server against an already-running Autoware:

```bash
ros2 launch autoware_claw mcp_server.launch.xml
```

## Step 5: Verify

### Check MCP Server Health

```bash
curl http://localhost:8765/health
# Expected: {"status": "ok", "connected": true}
```

### Check NemoClaw Dashboard

Open `http://localhost:18789` in your browser.

### Check RViz Chat Panel

The Claw chat panel should appear in RViz. Type a message and the LLM agent will respond using Autoware's vehicle state.

## What Gets Built

| Stage | What Happens |
|-------|-------------|
| `autoware_claw` (CMake) | Installs Python MCP server, config, launch files |
| `autoware_claw` (Docker) | Builds NemoClaw Docker image, pulls gemma4 model (~5 GB, cached in Docker volume) |
| `autoware_claw_rviz_plugins` (CMake) | Builds C++ RViz chat panel plugin |

The first build takes longer due to the Docker image build and gemma4 model download. Subsequent builds are fast because both are cached.

## Troubleshooting

### `package 'autoware_shape_estimation' not found`

Perception packages require CUDA and TensorRT. Either build them or disable perception:

```bash
ros2 launch autoware_claw planning_simulator.launch.xml \
  map_path:=/path/to/map \
  perception/enable_object_recognition:=false \
  perception/enable_detection_failure:=false
```

### `nvcc fatal: Unsupported gpu architecture 'compute_101'`

Your CUDA toolkit does not support the target architecture. Check `CMakeLists.txt` files in perception packages for hardcoded `-gencode` flags and remove unsupported architectures, or pass `-DCMAKE_CUDA_ARCHITECTURES=89` (adjust for your GPU).

### TensorRT version detection fails

If `autoware_tensorrt_common` reports "cuda, tensorrt libraries are not found", the `FindTENSORRT.cmake` module may not support your TensorRT version's header format. See the [tensorrt_cmake_module overlay](https://github.com/ros-perception/tensorrt_cmake_module) for a fix.

### Docker build fails or is very slow

- Ensure Docker daemon is running: `sudo systemctl start docker`
- Skip Docker build during development: `--cmake-args -DBUILD_DOCKER=OFF`
- Pre-pull the gemma4 model:

```bash
docker compose -f src/universe/external/autoware_claw/autoware_claw/docker-compose.yml \
  --profile setup run --rm ollama-pull
```

### `rosclaw` or `mcp` module not found at runtime

Install the Python dependencies:

```bash
pip3 install mcp rosclaw uvicorn starlette
```
