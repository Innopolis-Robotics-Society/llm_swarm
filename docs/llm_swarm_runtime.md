# LLM Swarm Runtime

This is the single runtime note for the LLM swarm demo: external API mode,
local Ollama mode, MCP read-only context, safety boundaries, and validation.

## Current Defaults

Shared LLM defaults live in `iros_llm_orchestrator/config/orchestrator.yaml`
under the ROS wildcard block:

```yaml
"/**":
  ros__parameters:
    llm_mode: "http"
    llm_endpoint: "https://api.groq.com/openai/v1/chat/completions"
    llm_model: "llama-3.3-70b-versatile"
    llm_api_key: ""
    llm_api_key_env: "LLM_API_KEY"
    llm_force_chat: true
    llm_enable_stop: false
    llm_temperature: 0.1
```

Channel 3, `/llm/chat`, uses MCP read-only context by default:

```yaml
llm_chat_server:
  ros__parameters:
    context_provider: "mcp_readonly"
    mcp_enabled: true
```

Never put a real API key in YAML, docs, reports, logs, or tests. Use only:

```bash
export LLM_API_KEY="REPLACE_WITH_FRESH_GROQ_KEY"
```

## Safety Boundary

MCP is observation-only. The model may read bounded context and propose a plan,
but execution still goes through:

```text
LLM observes and proposes.
PlanExecutor -> /llm/command -> Behavior Tree executes.
```

MCP must not expose robot write/control tools. Blocked tools include:

```text
publish_once
publish_for_durations
call_service
set_parameter
delete_parameter
send_action_goal
cancel_action_goal
connect_to_robot
disconnect_from_robot
set_websocket_ip
set_websocket_uri
ping_robot
ping_robots
```

The read allowlist is limited to:

```yaml
mcp_tool_allowlist:
  - "get_topics"
  - "get_topic_type"
  - "get_topic_details"
  - "subscribe_once"
  - "get_nodes"
  - "get_node_details"
  - "get_services"
  - "get_service_type"
  - "get_service_details"
  - "get_actions"
  - "get_action_details"
  - "get_action_status"
```

## Full Demo

Default launch:

```bash
ros2 launch iros_llm_swarm_bringup swarm_full_demo.launch.py
```

The full demo starts rosbridge by default for MCP context:

```text
enable_rosbridge:=true
```

Disable it only for debugging:

```bash
ros2 launch iros_llm_swarm_bringup swarm_full_demo.launch.py \
  enable_rosbridge:=false
```

## External API Mode

```bash
export LLM_API_KEY="REPLACE_WITH_FRESH_GROQ_KEY"
ros2 launch iros_llm_swarm_bringup swarm_full_demo.launch.py llm_backend:=http
```

Optional explicit model/endpoint override:

```bash
ros2 launch iros_llm_swarm_bringup swarm_full_demo.launch.py \
  llm_backend:=http \
  llm_endpoint:=https://api.groq.com/openai/v1/chat/completions \
  llm_model:=llama-3.3-70b-versatile
```

## Local Ollama Mode

```bash
ollama serve
ollama pull qwen2.5:7b
ros2 launch iros_llm_swarm_bringup swarm_full_demo.launch.py \
  llm_backend:=ollama \
  llm_model:=qwen2.5:7b
```

Ollama mode does not require `LLM_API_KEY`.

## Context Fallback Modes

Use local ROS topic context if MCP dependencies are unavailable:

```yaml
context_provider: "ros_readonly"
mcp_enabled: false
```

Use Phase 1 behavior with no runtime context:

```yaml
context_provider: "none"
mcp_enabled: false
```

## MCP Dependencies

MCP default context requires:

- `uvx`
- `ros-mcp`
- MCP Python SDK
- `rosbridge_server`
- something reachable on TCP port `9090`

Manual checks:

```bash
which uvx || true
uvx --version || true
uvx ros-mcp --help || true
ros2 pkg prefix rosbridge_server || true
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

The provider logs explicit diagnostics for missing dependencies, including:

- `uvx not found in PATH. Install uv or set mcp_command to the absolute uvx path.`
- `mcp Python SDK is not installed. Install the MCP Python SDK in the ROS container environment.`
- `rosbridge is not reachable on port 9090. Start rosbridge_server or enable_rosbridge:=true.`

It also logs command path resolution, MCP command, stdio initialization, and
read-only tool-call execution. It does not log API keys or full prompts.

## Docker Runtime Validation

Runtime checks should be done inside the project Docker terminal:

```bash
cd ~/Documents/work/llm_swarm
docker compose exec terminal bash
```

Inside the container:

```bash
cd /ros2_ws || cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

Build:

```bash
colcon build --packages-select \
  iros_llm_swarm_interfaces \
  iros_llm_orchestrator \
  iros_llm_swarm_bringup
```

Targeted tests:

```bash
python3 -m pytest \
  src/iros_llm_orchestrator/test/test_http_client.py \
  src/iros_llm_orchestrator/test/test_chat_context.py \
  src/iros_llm_orchestrator/test/test_chat_server_backend.py
```

Parameter checks after launch:

```bash
ros2 param get /llm_chat_server context_provider
ros2 param get /llm_chat_server mcp_enabled
ros2 param get /llm_chat_server llm_mode
ros2 param get /llm_chat_server llm_model
```

Expected:

```text
context_provider = mcp_readonly
mcp_enabled = true
llm_mode = http
llm_model = llama-3.3-70b-versatile
```

Port check, if `ss` is available:

```bash
ss -ltnp | grep 9090 || true
```

If `ss` is missing, use Python:

```bash
python3 - <<'PY'
import socket
try:
    with socket.create_connection(("127.0.0.1", 9090), timeout=1.0):
        print("port 9090 reachable")
except OSError as exc:
    print(f"port 9090 not reachable: {exc}")
PY
```

## Smoke Checks

State question:

```text
what is the state of the system?
```

Expected:

- no robot movement,
- `plan_executed: false`,
- `source=mcp_readonly` in logs,
- if MCP is unavailable, a concrete dependency warning.

Normal planning command:

```text
cyan go to center, then form a wedge
```

Expected:

- normal plan preview,
- execution only when requested/confirmed,
- execution goes through `PlanExecutor -> /llm/command -> BT`.

Fleet command smoke:

```bash
ros2 run iros_llm_swarm_bt fleet_cmd --scenario simple
```

Expected log shape:

```text
fleet/cmd: robot_ids=...
fleet/cmd: goals=...
fleet/cmd: mode=mapf
MapfPlan IDLE -> RUNNING
```

## Last Known Validation

Host checks:

- `py_compile` for `http_client.py`, `chat_server.py`,
  `mcp_readonly_provider.py`, `orchestrator.launch.py`,
  `swarm_full_demo.launch.py`: passed.
- `compileall iros_llm_orchestrator/iros_llm_orchestrator iros_llm_orchestrator/test`: passed.
- `git diff --check`: passed.
- Host pytest was blocked because `pytest` was not installed.

Docker checks were run inside `docker compose exec terminal bash`:

- `uvx` exists at `/home/fabian/.local/bin/uvx`.
- `uvx --version`: `uvx 0.11.12`.
- `uvx ros-mcp --help`: works.
- `rosbridge_server` package exists.
- Port `9090` was reachable using a Python socket check.
- `colcon build --packages-select iros_llm_swarm_interfaces iros_llm_orchestrator iros_llm_swarm_bringup`: passed.
- Targeted pytest: `18 passed`.

Blocked / remaining:

- Clean full-demo launch was blocked in that Docker image because
  `stage_ros2` was missing.
- Full MCP tool calls did not run because the Docker Python environment was
  missing the MCP Python SDK.
- Existing ROS nodes in Docker can contaminate live graph checks; use a clean
  container/session for final acceptance.
