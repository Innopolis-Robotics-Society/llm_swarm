iros_llm_orchestrator
=====================

Three-channel LLM advisory layer for the ``iros_llm_swarm`` stack. The
orchestrator does **not** drive motors directly; it produces decisions
and plan JSON, which are then dispatched through the existing BT and
``/swarm/set_goals`` primitives. Every LLM call is logged to a JSONL
dataset for SFT collection.

Channels
--------

The orchestrator runs three independent channels that share the BT
command bus but never collide:

* **Channel 1 — reactive (always on).** ``decision_server`` exposes the
  ``/llm/decision`` action. ``MapfPlan`` and ``SetFormation`` BT nodes
  call it directly when they hit a WARN or ERROR state. The action
  returns one of ``wait | abort | replan``. Lowest latency, single-shot
  per BT event.
* **Channel 2 — proactive (opt-in).** ``passive_observer`` subscribes to
  ``/bt/state``, decides on its own whether to intervene, and pushes an
  ``LlmCommand`` to ``/llm/command`` (received by ``LlmCommandReceiver``
  in the BT runner). Cooldown-gated; checks the ``llm_thinking`` flag on
  ``/bt/state`` to skip when channel 1 is busy. Disabled by default —
  enable with ``enable_passive_observer:=true``.
* **Channel 3 — operator chat.** ``chat_server`` exposes the
  ``/llm/chat`` action. The RViz panel sends free-form text; the server
  streams the reply back chunk-by-chunk, parses an executable plan JSON,
  and dispatches its leaves through ``/llm/execute_plan`` →
  ``execute_server``. A remediation loop (default two retries) re-prompts
  the LLM with refreshed runtime context if execution fails.

Nodes
-----

.. list-table::
   :header-rows: 1
   :widths: 22 28 50

   * - Executable
     - Class
     - Role
   * - ``decision_server``
     - ``LlmDecisionServer``
     - Channel 1 — ``/llm/decision`` action server.
   * - ``passive_observer``
     - ``PassiveObserver``
     - Channel 2 — watches ``/bt/state``, calls ``/llm/command``.
   * - ``chat_server``
     - ``ChatServer``
     - Channel 3 — ``/llm/chat`` action server, plan execution,
       remediation, MCP context.
   * - ``execute_server``
     - ``ExecuteServer``
     - Plan replay via ``/llm/execute_plan`` (operator-confirmed).
   * - ``user_chat``
     - ``UserChatNode``
     - CLI chatbot, mirrors ``chat_server`` for offline testing.

ROS interfaces
--------------

**Publishers:**

* ``/llm/events`` (``iros_llm_swarm_interfaces/LlmEvent``, reliable,
  depth 10) — one event per LLM call, consumed by the RViz Events tab.

**Subscribers:**

* ``/bt/state`` (``iros_llm_swarm_interfaces/BTState``, best-effort) —
  consumed by ``passive_observer`` and ``chat_server`` (for in-session
  BT-event injection into conversation history).

**Action servers:**

* ``/llm/decision`` (``LlmDecision``) — channel 1.
* ``/llm/chat`` (``LlmChat``) — channel 3, streams chunks via feedback.
* ``/llm/execute_plan`` (``LlmExecutePlan``) — operator-confirmed plan
  replay.

**Action clients:**

* ``/llm/command`` (``LlmCommand``) — used by ``passive_observer`` and
  by ``chat_server`` (when executing a plan leaf).

**Service clients (chat_server, via ``BTLeafSender``):**

* ``/obstacles/{add_circle, add_rectangle, add_door, remove, list}``
* ``/doors/{open, close}``

LLM backend selection
---------------------

The backend is selected by the ``llm_mode`` parameter (or the
``llm_backend`` launch argument) and produced by ``llm_factory``:

.. list-table::
   :header-rows: 1
   :widths: 20 80

   * - Mode
     - Backend
   * - ``mock``
     - In-process keyword heuristic. No network. Used by tests and
       offline development.
   * - ``ollama``
     - Local Ollama instance over ``/api/chat`` (streaming).
   * - ``http``
     - OpenAI-compatible ``/v1/chat/completions`` (default for the
       hosted setup; the orchestrator can be pointed at any compatible
       endpoint).
   * - ``local``
     - In-process HuggingFace Transformers inference (``transformers``
       client).

API keys are pulled from the env var named by ``llm_api_key_env``
(default ``LLM_API_KEY``); the parameter ``llm_api_key`` is a fallback
override and **must not** be hardcoded into a YAML file.

Key parameters
--------------

Defaults live in ``config/orchestrator.yaml``. Selected highlights:

.. list-table::
   :header-rows: 1
   :widths: 28 12 60

   * - Parameter
     - Default
     - Purpose
   * - ``llm_mode``
     - ``http``
     - Backend selector — see table above.
   * - ``llm_temperature``
     - ``0.1`` (chat) / ``0.2-0.3`` (decision)
     - Sampling temperature.
   * - ``timeout_sec``
     - ``45`` (decision/observer) / ``60`` (chat)
     - Per-call timeout. On expiry, channel 1 falls back to
       ``default_on_error`` (``wait``); other channels surface an error.
   * - ``cooldown_sec``
     - ``10.0``
     - Channel 2 minimum interval between calls — prevents trigger
       storms on rapid WARN/ERROR bursts.
   * - ``max_remediation_attempts``
     - ``2``
     - Channel 3 retries on plan-execution failure before escalating to
       operator.
   * - ``context_provider``
     - ``mcp_readonly``
     - One of ``none | ros_readonly | mcp_readonly``. The MCP provider
       spawns ``uvx ros-mcp --transport=stdio``.
   * - ``mcp_tool_allowlist``
     - read-only set
     - Allowlist constrains MCP to ``get_topics``, ``get_topic_type``,
       ``subscribe_once``, ``get_nodes``, ``get_services``,
       ``get_actions`` and similar. **No execute / write tools** ever
       reach the LLM.
   * - ``formation_tolerance_m``
     - ``0.5``
     - Distance within which followers are considered "at position" for
       chat-driven formation auto-staging.
   * - ``dataset_path``
     - ``~/.ros/llm_decisions`` / ``~/.ros/llm_commands``
     - Per-channel JSONL dataset directories (one file per UTC day).

Remediation loop and MCP context
--------------------------------

When channel 3 fails to execute a plan, ``chat_server``:

1. Captures the failed leaf and its error metadata via ``BTLeafSender``.
2. Calls ``_get_targeted_runtime_context(failure)`` — the MCP provider
   refreshes only the tools relevant to the failure (e.g. formation
   issues re-subscribe to ``/formations/status``).
3. Re-prompts the LLM with the refreshed snapshot up to
   ``max_remediation_attempts`` times.
4. On exhaustion, surfaces ``success=True, plan_executed=False`` and
   leaves the help-escalation hook in place for the operator UI.

Datasets
--------

Every call is appended atomically to the appropriate JSONL file:

* Channel 1 → ``~/.ros/llm_decisions/decisions_YYYYMMDD.jsonl``
* Channel 2 → ``~/.ros/llm_commands/decisions_YYYYMMDD.jsonl``

Schema: ``{timestamp, level, event, log_buffer, decision, reason}``
(channel 2 uses the command mode in the ``decision`` field).

Tests
-----

Unit tests live under ``test/`` and cover decision parsing, command
parsing, prompt construction, the channel-1 / 2 / 3 server lifecycles,
the remediation loop, the plan executor, the HTTP client, and the MCP
context provider. Run with::

    colcon test --packages-select iros_llm_orchestrator
    colcon test-result --verbose

Launch
------

``launch/orchestrator.launch.py`` brings up all five nodes plus an
optional ``rosbridge_server``. Arguments mirror the chat-channel
parameters (``llm_backend``, ``llm_endpoint``, ``llm_model``,
``enable_passive_observer``, ``enable_rosbridge``, ``scenario``,
``scenarios_file``). The full demo (``swarm_full_demo.launch.py`` from
``iros_llm_swarm_bringup``) includes this launch file.
