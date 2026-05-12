# LlmEvent.msg
#
# Published by the orchestrator on /llm/events. All three LLM channels
# emit here so panels and dataset loggers can see the full picture without
# per-channel introspection.
#
# Channel codes
uint8 CHANNEL_DECISION = 1     # /llm/decision (BT → wait/abort/replan)
uint8 CHANNEL_OBSERVER = 2     # PassiveObserver → /llm/command
uint8 CHANNEL_USER     = 3     # User chat / RViz panel

int64  stamp_ms

uint8  channel                 # one of CHANNEL_*

# What caused this event:
#   channel 1: BT WARN/INFO event text (+ short log_buffer summary)
#   channel 2: BTState summary line that triggered the observer
#   channel 3: raw user message
string trigger

# What the LLM produced:
#   channel 1: "wait" / "abort" / "replan"
#   channel 2: command JSON or short mode label ("idle" / "mapf" / "formation")
#   channel 3: plan JSON
string output

# LLM-provided justification (the "reason" field of the response).
string reason
