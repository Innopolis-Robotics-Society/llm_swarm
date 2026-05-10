# Phase 2 MCP Context Integration

Phase 2 adds read-only runtime context to Channel 3, `/llm/chat`.
The LLM receives a small sanitized snapshot before it answers or proposes a
plan. It does not receive MCP tool access.

The safety boundary stays unchanged:

```text
LLM observes and proposes
PlanExecutor -> /llm/command -> Behavior Tree executes
```

## What This Does

- `context_provider: "none"` preserves Phase 1 behavior.
- `context_provider: "ros_readonly"` caches selected local ROS topics:
  `/bt/state`, `/llm/events`, and `/formations/status`.
- `context_provider: "mcp_readonly"` optionally starts a stdio MCP client and
  asks fixed read-only tools for a bounded snapshot.
- Missing topics, missing MCP SDK, missing `uvx`, stopped rosbridge, or stopped
  `ros-mcp` become warnings in the context. `/llm/chat` continues.

The prompt tells the model that the context is read-only, that unknown state
must remain unknown, and that factual state answers should return a no-op
`idle` plan with a `reply_only:` reason.

## What This Does Not Do

Phase 2 does not allow the LLM to:

- publish ROS topics,
- send ROS action goals,
- call arbitrary ROS services,
- set or delete ROS parameters,
- send `/cmd_vel`,
- bypass `PlanExecutor`,
- bypass `/llm/command`,
- bypass the Behavior Tree,
- directly control MAPF, formation, Nav2, or robot motion.

Operator text is never used as an MCP tool name or tool argument.

## Enable Local ROS Read-Only Context

Edit `iros_llm_orchestrator/config/orchestrator.yaml`:

```yaml
llm_chat_server:
  ros__parameters:
    context_provider: "ros_readonly"
```

Ask from the RViz panel:

```text
what are the robots doing right now?
```

Expected behavior:

- The answer can reference cached `/bt/state`, recent `/llm/events`, available
  formation status, and map zone names.
- No robot moves for factual questions.
- Normal commands still produce the existing plan schema.

## Enable MCP Read-Only Context

`ros-mcp-server` uses rosbridge to see ROS. Run rosbridge in the ROS/container
terminal where your workspace is sourced:

```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Then enable MCP context in the orchestrator config:

```yaml
llm_chat_server:
  ros__parameters:
    context_provider: "mcp_readonly"
    mcp_enabled: true
    mcp_transport: "stdio"
    mcp_command: "uvx"
    mcp_args: ["ros-mcp", "--transport=stdio"]
```

The provider uses a fixed read-only allowlist:

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

`ping_robot`/`ping_robots` and write/control tools are blocked. The provider
does not expose arbitrary MCP tools to the LLM.

## Fallback Behavior

If MCP is unavailable, `/llm/chat` still completes. The context snapshot looks
like this shape:

```json
{
  "source": "mcp_readonly",
  "warnings": [
    "MCP server unavailable; continuing without live MCP context: ..."
  ]
}
```

The LLM can answer with limited context or say that live MCP state is
unavailable.

## Manual Checks

Phase 1 regression:

```text
cyan go to center, then form a wedge
```

Expected:

- External or local LLM backend still works.
- A normal plan is produced.
- Execution, when confirmed/enabled, still goes through
  `PlanExecutor -> /llm/command -> BT`.

ROS read-only context:

```text
what are the robots doing right now?
```

Expected:

- The answer uses available read-only state.
- Factual state questions do not execute robot motion.

MCP unavailable fallback:

```yaml
context_provider: "mcp_readonly"
mcp_enabled: true
```

Do not start rosbridge or MCP, then ask a state question. Expected:

- No crash.
- The response notes limited or unavailable live MCP context.
- `/llm/chat` action completes safely.

## References

- `robotmcp/ros-mcp-server`: https://github.com/robotmcp/ros-mcp-server
- MCP server metadata and stdio package entry:
  https://raw.githubusercontent.com/robotmcp/ros-mcp-server/main/server.json
- Programmatic stdio client example:
  https://raw.githubusercontent.com/robotmcp/ros-mcp-server/main/docs/install/clients/custom-client.md
- Rosbridge setup:
  https://raw.githubusercontent.com/robotmcp/ros-mcp-server/main/docs/install/rosbridge.md
- MCP security audit:
  https://arxiv.org/abs/2504.03767
