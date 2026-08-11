# Runbook — recording a swarm session

You run one command, drive the swarm through the RViz chat panel, then press
Ctrl+C. Everything needed for the analysis is recorded automatically into a
single folder. You do not need to start anything else or configure anything.

---

## 1. Start the container

From the repository root on the host:

```bash
docker compose up -d terminal      # use terminal-cpu if the machine has no NVIDIA GPU
docker compose exec terminal bash
```

Everything below runs **inside** that shell.

## 2. Build once

```bash
cd /home/fabian/ros2_ws
colcon build --packages-select iros_llm_swarm_interfaces iros_llm_swarm_mapf_lns
colcon build
```

The first line must come first — the other packages depend on the message
types it generates.

## 3. Check the LLM is reachable

```bash
curl -s http://localhost:11434/api/tags | head -c 300
```

You should see a JSON list of models. If this is empty or errors, start Ollama
on the **host** first; the container reaches it over the host network.

## 4. Record a session

```bash
bash /home/fabian/ros2_ws/src/scripts/record_session.sh
```

Options, none of them required:

```bash
bash .../record_session.sh --planner pbs           # default is lns
bash .../record_session.sh --scenario amongus      # default is amongus
bash .../record_session.sh --note "door closure"   # free-text label for the run
```

RViz opens. Wait until the robots have appeared and stopped settling — roughly
30–60 seconds. Bring-up is staggered on purpose; the fleet is not ready the
instant the window appears.

## 5. Drive the swarm

Use the **Chat** tab of the LLM panel in RViz. Type ordinary English commands
and wait for each to finish before sending the next.

A good session exercises several kinds of command. Suggested run:

1. `send cyan to cafeteria`
2. `move yellow and green to navigation at the same time`
3. `make a line formation from magenta led by robot 4`
4. `send the magenta line to storage`
5. `disband the magenta formation and send all four to reactor`
6. `all robots go home`

Wait for the reply and for the robots to actually stop moving between steps.
Some commands take a minute or more — the system plans, executes, verifies and
may repair. That is expected and is exactly what we are measuring.

If something looks stuck, wait at least two minutes before intervening. If it
is still stuck, note it in your report and move on to the next command.

## 6. Stop

Press **Ctrl+C once** in the terminal where you started the script, and wait.

It will print a summary and the session folder path. **Do not press Ctrl+C
repeatedly** — the recorder needs a few seconds to close the bag properly, and
killing it early makes the recording unreadable.

A healthy summary looks like:

```
Session: /home/fabian/swarm_sessions/20260728_143512
  bag finalised          : yes
  operator commands logged: 6
```

If it says `bag finalised : NO` or `operator commands logged: 0`, the run is
not usable — please say so when you report back.

## 7. Send the folder back

The whole folder, e.g. `~/swarm_sessions/20260728_143512/`. Typical size is
a few hundred MB.

```bash
cd ~/swarm_sessions
tar czf 20260728_143512.tar.gz 20260728_143512/
```

Please also say, in your own words:

- which commands you typed and roughly when
- anything that looked wrong (robots stuck, collisions, nonsense replies)
- anything you did that the runbook does not mention

That free-text note matters. The logs record what the system did, not what it
looked like to you, and the two disagreeing is itself a result.

---

## What gets recorded

| in the folder | what it is |
| --- | --- |
| `bag/` | task state, formation status, BT state, LLM events, TF, odometry |
| `llm_chat/*.jsonl` | one record per operator command: plan, guard firings, LLM calls, repairs |
| `llm_decisions/*.jsonl` | channel 1 (behaviour-tree escalations) |
| `llm_commands/*.jsonl` | channel 2, only if the proactive observer was enabled |
| `session.json` | git commit, model, arguments, timings |
| `launch.log` | full console output |

Laser scans and costmaps are deliberately **not** recorded — at 20 robots they
would add gigabytes per run and nothing in the analysis reads them.

## If it goes wrong

**RViz never opens / no robots appear.** Check `launch.log` in the session
folder for the first `[ERROR]`. Most often the workspace was not rebuilt after
a pull.

**The chat panel replies but nothing moves.** The plan was generated but
execution failed. This is a legitimate result — keep going, it is recorded.

**Everything is very slow.** Check the model actually fits in VRAM
(`nvidia-smi`). A model spilling to CPU makes each reply take minutes.

**You need to abandon a run.** Ctrl+C and start again. Delete the abandoned
session folder or mark it in the note; a half-run is worse than no run if we
cannot tell which is which.
