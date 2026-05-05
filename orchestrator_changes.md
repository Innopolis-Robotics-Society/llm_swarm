# Orchestrator changes — wiring guide

This is the minimum set of edits in **iros_llm_orchestrator** that the panel
needs to be useful. None of the existing channels (1, 2, 3-stdin) break.

## 1. Register the new types in `iros_llm_swarm_interfaces`

Drop the two new files from `interfaces_additions/` into the interfaces
package:

```
iros_llm_swarm_interfaces/
├── action/LlmChat.action       ← new
└── msg/LlmEvent.msg            ← new
```

Then in its `CMakeLists.txt`, extend `rosidl_generate_interfaces(...)`:

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  ...existing entries...
  "action/LlmChat.action"
  "msg/LlmEvent.msg"
  DEPENDENCIES geometry_msgs std_msgs
)
```

Rebuild before continuing:

```bash
colcon build --packages-select iros_llm_swarm_interfaces
```

## 2. Add `/llm/chat` action server (channel 3 over ROS instead of stdin)

The cleanest move is a new node alongside `user_chat_node.py` that
**reuses everything in `common/`** and only swaps the I/O surface.

New file: `iros_llm_orchestrator/iros_llm_orchestrator/chat_server.py`

```python
"""Channel 3 over ROS — /llm/chat action server.

Same pipeline as user_chat_node.py (build_user_prompt → stream → parse_plan
→ PlanExecutor), but I/O goes through a ROS action instead of stdin/stdout.
Reply text is streamed via feedback chunks.
"""

import asyncio, json, threading
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Point

from iros_llm_swarm_interfaces.action import LlmChat, LlmCommand
from rclpy.action import ActionClient

from iros_llm_orchestrator.common.plan_executor import PlanExecutor, parse_plan
from iros_llm_orchestrator.common.user_prompt   import build_user_prompt, load_map_config
from iros_llm_orchestrator.local.ollama_client  import OllamaClient
from iros_llm_orchestrator.user_chat_node       import _parse_response, _postprocess_plan


class ChatServer(Node):
    def __init__(self):
        super().__init__('llm_chat_server')

        self.declare_parameter('llm_endpoint',    'http://localhost:11434/api/chat')
        self.declare_parameter('llm_model',       'qwen2.5:14b')
        self.declare_parameter('llm_max_tokens',  768)
        self.declare_parameter('llm_temperature', 0.1)
        self.declare_parameter('timeout_sec',     30.0)
        self.declare_parameter('map_name',        'cave')

        self._timeout  = float(self.get_parameter('timeout_sec').value)
        self._map_name = self.get_parameter('map_name').value
        self._ollama = OllamaClient(
            endpoint=self.get_parameter('llm_endpoint').value,
            model=self.get_parameter('llm_model').value,
            max_tokens=int(self.get_parameter('llm_max_tokens').value),
            temperature=float(self.get_parameter('llm_temperature').value),
        )
        try:
            self._map_cfg = load_map_config(self._map_name)
        except Exception:
            self._map_cfg = {}

        self._cmd_client = ActionClient(self, LlmCommand, '/llm/command')

        self._loop = asyncio.new_event_loop()
        threading.Thread(target=self._loop.run_forever, daemon=True).start()

        self._action_server = ActionServer(
            self, LlmChat, '/llm/chat',
            execute_callback=self._execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
        )
        self.get_logger().info('LlmChatServer ready on /llm/chat')

    def _execute(self, goal_handle):
        fut = asyncio.run_coroutine_threadsafe(
            self._execute_async(goal_handle), self._loop)
        return fut.result()

    async def _execute_async(self, goal_handle):
        req = goal_handle.request
        result = LlmChat.Result()

        # ---- 1. Build messages and stream reply ----
        messages = build_user_prompt(req.user_message, history=[], map_name=self._map_name)
        self._publish_fb(goal_handle, stage='thinking')

        full_raw = ''
        try:
            async for chunk in self._ollama.stream(messages):
                full_raw += chunk
                # NOTE: this streams the full JSON, not just the reply text.
                # For MVP we forward chunks as-is. v2 should mimic the
                # _stream_command state machine in user_chat_node.py to
                # only forward the "reply" field.
                self._publish_fb(goal_handle, stage='streaming', chunk=chunk)
        except Exception as exc:
            return self._fail(goal_handle, result, f'LLM error: {exc}')

        # ---- 2. Parse ----
        try:
            reply, plan = _parse_response(full_raw)
        except ValueError as exc:
            return self._fail(goal_handle, result, f'parse error: {exc}')

        plan = _postprocess_plan(plan, self._map_cfg)
        plan_json = json.dumps(plan, ensure_ascii=False)
        result.final_reply = reply
        result.plan_json   = plan_json

        self._publish_fb(goal_handle, stage='parsed', detail=plan_json)

        # ---- 3. Optional execute ----
        if req.execute_after_planning:
            self._publish_fb(goal_handle, stage='executing')
            executor = PlanExecutor(send_fn=self._send_leaf, log_fn=lambda _: None)
            ok = await executor.run(plan)
            result.plan_executed = ok
            if not ok:
                return self._fail(goal_handle, result, 'plan execution failed', got_plan=True)

        self._publish_fb(goal_handle, stage='done')
        result.success = True
        result.info    = ''
        goal_handle.succeed()
        return result

    async def _send_leaf(self, command: dict) -> bool:
        if not self._cmd_client.wait_for_server(timeout_sec=3.0):
            return False
        goal = LlmCommand.Goal()
        goal.mode         = command.get('type') or command.get('mode', '')
        goal.reason       = command.get('reason', '')
        goal.robot_ids    = [int(r) for r in command.get('robot_ids', [])]
        goal.goals        = [Point(x=float(g[0]), y=float(g[1]), z=0.0)
                             for g in command.get('goals', [])]
        goal.formation_id = command.get('formation_id', '')
        goal.leader_ns    = command.get('leader_ns', '')
        goal.follower_ns  = list(command.get('follower_ns', []))
        goal.offsets_x    = [float(o) for o in command.get('offsets_x', [])]
        goal.offsets_y    = [float(o) for o in command.get('offsets_y', [])]

        handle = await self._cmd_client.send_goal_async(goal)
        if not handle.accepted:
            return False
        result = await handle.get_result_async()
        return result.result.success

    def _publish_fb(self, gh, *, stage='', chunk='', detail=''):
        fb = LlmChat.Feedback()
        fb.stage = stage; fb.chunk = chunk; fb.detail = detail
        gh.publish_feedback(fb)

    def _fail(self, gh, result, info, got_plan=False):
        self._publish_fb(gh, stage='error', detail=info)
        result.success = False
        result.info    = info
        if not got_plan:
            result.plan_json = ''
        gh.succeed()         # action did execute, just unsuccessfully
        return result


def main(args=None):
    rclpy.init(args=args)
    node = ChatServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Add to `setup.py` `entry_points`:

```python
'console_scripts': [
    'decision_server  = iros_llm_orchestrator.decision_server:main',
    'passive_observer = iros_llm_orchestrator.passive_observer:main',
    'user_chat        = iros_llm_orchestrator.user_chat_node:main',
    'chat_server      = iros_llm_orchestrator.chat_server:main',   # ← new
],
```

Add to `launch/orchestrator.launch.py`:

```python
Node(
    package='iros_llm_orchestrator',
    executable='chat_server',
    name='llm_chat_server',
    parameters=[config],     # reuses orchestrator.yaml — see (3) below
    output='screen',
),
```

And a section in `config/orchestrator.yaml`:

```yaml
llm_chat_server:
  ros__parameters:
    llm_endpoint: "http://localhost:11434/api/chat"
    llm_model:    "qwen2.5:14b"
    llm_max_tokens:  768
    llm_temperature: 0.1
    timeout_sec:     30.0
    map_name:        "cave"
```

> **Known limitation in this sketch.** The streaming forwards the *raw* JSON
> chunks (not just the `"reply"` field). The panel will see the JSON until
> the model finishes. For the MVP that's acceptable — you can post-filter
> in the panel or polish later by porting the `_stream_command` state
> machine from `user_chat_node.py` into `ChatServer._execute_async`.

## 3. Publish `/llm/events`

Add to **both** `decision_server.py` and `passive_observer.py`:

```python
# in __init__ (after super().__init__)
from iros_llm_swarm_interfaces.msg import LlmEvent
self._event_pub = self.create_publisher(LlmEvent, '/llm/events', 10)

# in decision_server._execute_async, right after self._logger_ds.log(...):
ev = LlmEvent()
ev.stamp_ms = int(self.get_clock().now().nanoseconds / 1e6)
ev.channel  = LlmEvent.CHANNEL_DECISION
ev.trigger  = f'[{req.level}] {req.event}'
ev.output   = decision
ev.reason   = reason
self._event_pub.publish(ev)
```

For `passive_observer._think_and_command`, after the `_logger_ds.log(...)` call:

```python
ev = LlmEvent()
ev.stamp_ms = int(self.get_clock().now().nanoseconds / 1e6)
ev.channel  = LlmEvent.CHANNEL_OBSERVER
ev.trigger  = f'{trigger.active_action}: {trigger.last_error}'
ev.output   = command.get('mode', 'idle')
ev.reason   = command.get('reason', '')
self._event_pub.publish(ev)
```

`chat_server.py` should also publish on each successful turn:

```python
ev = LlmEvent()
ev.stamp_ms = int(self.get_clock().now().nanoseconds / 1e6)
ev.channel  = LlmEvent.CHANNEL_USER
ev.trigger  = req.user_message
ev.output   = plan_json
ev.reason   = reply
self._event_pub.publish(ev)
```

The MVP panel doesn't subscribe to this topic yet (the **Events** tab is a
stub). It just needs to be flowing so v2 has something to render.
