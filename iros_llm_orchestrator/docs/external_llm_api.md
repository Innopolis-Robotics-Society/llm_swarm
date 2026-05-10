# External LLM API

Phase 1 connects the existing orchestrator to an OpenAI-compatible HTTP API.
The main path is `/llm/chat`, used by the RViz operator chat, but the same
backend configuration is also available to `/llm/decision` and the passive
observer.

The LLM still produces structured JSON only. Execution remains routed through
`PlanExecutor`, `/llm/command`, and the Behavior Tree; it does not send
low-level robot velocity commands.

## Configuration

Set the provider key in the environment:

```bash
export LLM_API_KEY="your_key_here"
```

Do not put provider keys in YAML, logs, docs, reports, or tests.

Default examples in `config/orchestrator.yaml` use:

- Groq: `https://api.groq.com/openai/v1/chat/completions`
- OpenRouter: `https://openrouter.ai/api/v1/chat/completions`
- Local vLLM: `http://localhost:8000/v1/chat/completions`

For local offline demos without an API key, switch `llm_mode` back to `ollama`
or `mock` in the relevant node parameters.

## Build And Launch

```bash
colcon build --packages-select iros_llm_orchestrator
source install/setup.bash
ros2 launch iros_llm_swarm_bringup swarm_full_demo.launch.py
```

To launch only the orchestrator:

```bash
ros2 launch iros_llm_orchestrator orchestrator.launch.py
```

## `/llm/chat` Preview Smoke Test

Use preview mode first so no robots move:

```bash
ros2 action send_goal /llm/chat iros_llm_swarm_interfaces/action/LlmChat \
  "{user_message: 'cyan go to center, then form a wedge', execute_after_planning: false}"
```

Expected result:

- the goal is accepted,
- the action reaches `thinking`, then `parsed` or `done`,
- `success` is true,
- `final_reply` is non-empty,
- `plan_json` is valid JSON,
- no robot motion starts because `execute_after_planning` is false.
