---
name: Project rules
description: Hard rules from CLAUDE.md used as reviewer ammunition
type: project
---

## Hard project rules (from CLAUDE.md)

- No allocation in `update()`, `read()`, `write()`, or any >100 Hz timer callback.
- No `time.sleep` / `std::this_thread::sleep_for` in callbacks.
- QoS must be justified; the four defaults (sensor/cmd/status/latched) are presumed.
- Lifecycle nodes for hardware-owning nodes; plain nodes for everything else.
- `MutuallyExclusive` callback groups by default; `Reentrant` requires comment proving thread-safety.
- Every parameter has a descriptor with type and range.
- C++: no `new`/`delete`, RAII, `noexcept` destructors.
- Python: type hints, no bare `except`, numpy-vectorise anything >16 elements.
- 20-robot ceiling — CycloneDDS with `cyclonedds_swarm.xml` mandatory.
- AMCL intentionally disabled — do not recommend re-enabling.
- LNS2 package (`iros_llm_swarm_mapf_lns`) is planned for full refactor — flag what's broken but note the refactor context.

**Why:** These come directly from the project's CLAUDE.md and represent the team's explicit quality bar.
**How to apply:** Use these as the primary benchmark when flagging issues. A violation of these rules is always worth flagging regardless of whether it appears in a diff or a full-repo audit.
