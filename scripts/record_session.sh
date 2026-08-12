#!/bin/bash
# record_session.sh — run the full demo and record everything needed to score
# it later. One command in, one self-contained folder out.
#
#   bash src/scripts/record_session.sh                 # defaults (lns, amongus)
#   bash src/scripts/record_session.sh --planner pbs
#   bash src/scripts/record_session.sh --note "door closure test"
#
# Stop with Ctrl+C. The bag is closed cleanly and the folder is summarised.
#
# Produces ~/swarm_sessions/<timestamp>/
#   bag/                 rosbag2 of the topics listed in TOPICS below
#   llm_chat/*.jsonl     channel 3, one record per operator command
#   llm_decisions/*.jsonl  channel 1
#   llm_commands/*.jsonl   channel 2 (only if the observer is enabled)
#   session.json         what was run: git commit, model, args, timings
#   launch.log           full stdout of the stack
#
# Deliberately NOT recorded: /robot_*/scan and the costmaps. At 20 robots
# those dominate the bag (gigabytes per run) and nothing in E3/E5 reads them.
# Add them here if a future experiment needs them, and expect the size.

set -u

WS=/home/fabian/ros2_ws
SRC="$WS/src"
PLANNER=lns
SCENARIO=amongus
NUM_ROBOTS=20
NOTE=""
PROFILE=full
EXTRA=()

# E3 ablation factors. Empty = use whatever orchestrator.yaml ships; the
# resolved value is written into session.json either way, so a folder of bags
# is never ambiguous about which configuration produced it.
REMEDIATION=""
REPAIR=""
SUPERVISION=""
TOOL_CALLING=""
STRUCTURED_OUTPUT=""
MODEL_OVERRIDE=""
# Empty means "whatever orchestrator.yaml ships", which is local Ollama. A
# hosted campaign passes the OpenRouter URL here; the endpoint must be known to
# resolve_llm_endpoint() in orchestrator.launch.py or the stack refuses to
# start.
LLM_ENDPOINT=""

while [ $# -gt 0 ]; do
  case "$1" in
    --planner)     PLANNER="$2";     shift 2 ;;
    --scenario)    SCENARIO="$2";    shift 2 ;;
    --num-robots)  NUM_ROBOTS="$2";  shift 2 ;;
    --note)        NOTE="$2";        shift 2 ;;
    --profile)     PROFILE="$2";     shift 2 ;;
    --remediation)        REMEDIATION="$2";       shift 2 ;;
    --repair)             REPAIR="$2";            shift 2 ;;
    --supervision)        SUPERVISION="$2";       shift 2 ;;
    --tool-calling)       TOOL_CALLING="$2";      shift 2 ;;
    --structured-output)  STRUCTURED_OUTPUT="$2"; shift 2 ;;
    --model)              MODEL_OVERRIDE="$2";    shift 2 ;;
    --llm-endpoint)       LLM_ENDPOINT="$2";      shift 2 ;;
    -h|--help)     sed -n '2,25p' "$0"; exit 0 ;;
    *)             EXTRA+=("$1");    shift ;;
  esac
done

STAMP=$(date +%Y%m%d_%H%M%S)
SESSION="$HOME/swarm_sessions/$STAMP"
mkdir -p "$SESSION"

# Topic set depends on the experiment. Recording everything for every run is
# wasteful: /tf plus twenty odometries are ~63% of the messages, and E5 needs
# none of them — the per-follower error is already computed by the monitor.
#
#   full  E3 and general sessions. Mission state, decisions, all poses.
#   e5    formation tracking only. ~35 MB/hour instead of ~800 MB/hour.
BAG_FLAGS=()
case "$PROFILE" in
  e5)
    TOPICS=(
      /formations/status      # the measurement itself, 10 Hz
      /formations/config      # which offsets were in force
      /robot_0/odom           # leader only: achieved speed + route segmentation
      /clock
    )
    ;;
  full)
    TOPICS=(
      /tasks/state
      /tasks/markers
      /formations/status
      /formations/config
      /bt/state
      /llm/events
      /llm/command
      /tf
      /tf_static
      /clock
      # Action feedback carries per-robot arrival / deviation / stall counts.
      # Naming a hidden topic explicitly is NOT enough: rosbag2 filters the
      # whole /_action/ family out unless --include-hidden-topics is passed,
      # and it says so only as a WARN in the recorder log, so the bag looks
      # fine until the analysis finds the topic missing. The flag is added
      # below for this profile. The action RESULT (planning time, expansions,
      # replans) travels over a service and can never be bagged — use
      # `test_send_goals --json-out` for those.
      /swarm/set_goals/_action/feedback
      /swarm/set_goals/_action/status
    )
    BAG_FLAGS+=(--include-hidden-topics)
    ;;
  *)
    echo "FATAL: unknown --profile '$PROFILE' (expected 'full' or 'e5')"; exit 1 ;;
esac

if [ "$PROFILE" = "full" ]; then
  for i in $(seq 0 $((NUM_ROBOTS - 1))); do
    TOPICS+=("/robot_$i/odom")
  done
fi

# ROS setup scripts read unset variables (AMENT_TRACE_SETUP_FILES and friends),
# so `set -u` has to stand down for the duration of the sourcing.
set +u
source /opt/ros/humble/setup.bash
# stage_ros2 is built into a separate workspace by the Dockerfile and is only
# on the path via ~/.bashrc, which a non-interactive shell never reads. Source
# it explicitly or the simulator is "package not found" under this script while
# working fine in an interactive terminal.
[ -f "$HOME/extras_ws/install/setup.bash" ] && source "$HOME/extras_ws/install/setup.bash"
source "$WS/install/setup.bash"
source "$SRC/scripts/setup_swarm_env.sh" >/dev/null 2>&1 || true
set -u

if ! ros2 pkg prefix stage_ros2 >/dev/null 2>&1; then
  echo "FATAL: stage_ros2 not found. The simulator cannot start."
  echo "  Expected it in ~/extras_ws (built by the Dockerfile)."
  echo "  Rebuild the image, or build it manually:"
  echo "    cd ~/extras_ws && colcon build --symlink-install"
  exit 1
fi

# ── provenance ────────────────────────────────────────────────────────────
GIT_COMMIT=$(git -C "$SRC" rev-parse HEAD 2>/dev/null || echo unknown)
GIT_DIRTY=$(git -C "$SRC" status --porcelain 2>/dev/null | head -c 1)
# Strip the trailing comment first, then the surrounding quotes — doing it the
# other way round eats the whole value, because the opening quote is the first
# character after the colon.
MODEL=$(grep -m1 '^[[:space:]]*llm_model:' "$SRC/iros_llm_orchestrator/config/orchestrator.yaml" 2>/dev/null \
        | sed 's/.*llm_model:[[:space:]]*//; s/[[:space:]]*#.*//; s/^"//; s/"$//' | xargs)

YAML="$SRC/iros_llm_orchestrator/config/orchestrator.yaml"
python3 - "$SESSION" "$GIT_COMMIT" "$GIT_DIRTY" "$MODEL" "$PLANNER" "$SCENARIO" \
         "$NUM_ROBOTS" "$NOTE" "$YAML" \
         "$REMEDIATION" "$REPAIR" "$SUPERVISION" "$TOOL_CALLING" \
         "$STRUCTURED_OUTPUT" "$MODEL_OVERRIDE" "$LLM_ENDPOINT" <<'PROV'
import json, os, re, sys, datetime
(d, commit, dirty, model, planner, scenario, n, note, yaml_path,
 remediation, repair, supervision, tool_calling, structured, model_override,
 llm_endpoint) = sys.argv[1:17]

# Resolve every ablation factor to the value that will ACTUALLY be in force:
# the CLI override when given, otherwise whatever orchestrator.yaml ships.
# Recording only the override would leave "" meaning "unknown" in the analysis,
# which is exactly the ambiguity this file exists to remove.
try:
    yaml_text = open(yaml_path, encoding='utf-8').read()
except OSError:
    yaml_text = ''


def from_yaml(key):
    m = re.search(r'^\s*' + re.escape(key) + r'\s*:\s*([^\s#]+)', yaml_text, re.M)
    if not m:
        return None
    v = m.group(1).strip().strip('"\'')
    if v.lower() in ('true', 'false'):
        return v.lower() == 'true'
    return v


def resolve(override, key):
    if override:
        return {'value': override.lower() == 'true', 'source': 'cli'}
    return {'value': from_yaml(key), 'source': 'yaml'}


factors = {
    'remediation_enabled': resolve(remediation, 'remediation_enabled'),
    'llm_repair_enabled': resolve(repair, 'llm_repair_enabled'),
    'llm_mission_supervision_enabled': resolve(
        supervision, 'llm_mission_supervision_enabled'),
    'tool_calling_enabled': resolve(tool_calling, 'tool_calling_enabled'),
    'structured_output_enabled': resolve(structured, 'structured_output_enabled'),
}
effective_model = ({'value': model_override, 'source': 'cli'} if model_override
                   else {'value': model, 'source': 'yaml'})
effective_endpoint = ({'value': llm_endpoint, 'source': 'cli'} if llm_endpoint
                      else {'value': from_yaml('llm_endpoint'),
                            'source': 'yaml'})

# Routing lives in environment variables read by web/http_client.py, so it
# leaves no trace anywhere else. Without this block a hosted run cannot say
# which provider served it, and provider decides quantisation, latency and
# whether tool calling works at all. Recorded even when empty: "the router
# chose per request" is itself the fact worth knowing.
routing = {
    'openrouter_provider': os.environ.get('OPENROUTER_PROVIDER', ''),
    'openrouter_allow_fallbacks': os.environ.get(
        'OPENROUTER_ALLOW_FALLBACKS', ''),
    'llm_call_delay_sec': os.environ.get('LLM_CALL_DELAY_SEC', ''),
    'api_key_present': bool(os.environ.get('LLM_API_KEY', '')),
}

json.dump({
    'started_at': datetime.datetime.now(datetime.timezone.utc).isoformat(),
    'git_commit': commit,
    'git_dirty': bool(dirty),
    'planner': planner,
    'scenario': scenario,
    'num_robots': int(n),
    'note': note,
    'llm_model': effective_model,
    'llm_endpoint': effective_endpoint,
    'llm_routing': routing,
    'ablation_factors': factors,
}, open(d + '/session.json', 'w'), indent=2)

print(' factors: ' + ', '.join(
    k.replace('_enabled', '') + '=' + str(v['value'])
    for k, v in sorted(factors.items())))
print(' model  : {} ({})'.format(
    effective_model['value'], effective_model['source']))
print(' endpoint: {} ({})'.format(
    effective_endpoint['value'], effective_endpoint['source']))
_hosted = '/chat/completions' in str(effective_endpoint['value'] or '')
if _hosted:
    print(' routing : provider={} fallbacks={} delay={} key={}'.format(
        routing['openrouter_provider'] or '(не прибит — роутер сам)',
        routing['openrouter_allow_fallbacks'] or '(по умолчанию)',
        routing['llm_call_delay_sec'] or '(без паузы)',
        'есть' if routing['api_key_present'] else 'НЕТ'))
    if not routing['api_key_present']:
        print(' WARNING: LLM_API_KEY пуст, а точка входа сетевая — '
              'вызовы модели упадут на авторизации')
    if not routing['openrouter_provider'] and 'openrouter' in str(
            effective_endpoint['value'] or ''):
        print(' WARNING: провайдер не прибит — между прогонами кампании '
              'молча меняются квантизация, задержка и поддержка инструментов '
              '(paper/E3_spec.md §4.6)')
PROV

echo "=============================================================="
echo " Recording session: $SESSION"
echo " planner=$PLANNER scenario=$SCENARIO robots=$NUM_ROBOTS"
[ -n "$NOTE" ] && echo " note: $NOTE"
echo " Stop with Ctrl+C when the mission is finished."
echo "=============================================================="

# ── recorder ──────────────────────────────────────────────────────────────
# Started first so bring-up itself is captured. Missing topics are fine:
# rosbag2 subscribes as they appear.
ros2 bag record -o "$SESSION/bag" \
  "${BAG_FLAGS[@]+"${BAG_FLAGS[@]}"}" "${TOPICS[@]}" \
  > "$SESSION/bag_record.log" 2>&1 &
BAG_PID=$!

cleanup() {
  echo
  echo "Stopping recorder…"
  # SIGINT, not SIGKILL: rosbag2 must finalise metadata.yaml or the bag is
  # unreadable. Give it time to flush before falling back.
  kill -INT "$BAG_PID" 2>/dev/null
  for _ in $(seq 1 20); do kill -0 "$BAG_PID" 2>/dev/null || break; sleep 0.5; done
  kill -0 "$BAG_PID" 2>/dev/null && kill -9 "$BAG_PID" 2>/dev/null

  python3 - "$SESSION" "${TOPICS[@]}" <<'PY'
import json, os, sys, datetime
d = sys.argv[1]
requested = sys.argv[2:]
p = f'{d}/session.json'
meta = json.load(open(p)) if os.path.exists(p) else {}
meta['finished_at'] = datetime.datetime.now(datetime.timezone.utc).isoformat()
bag_ok = os.path.exists(f'{d}/bag/metadata.yaml')
meta['bag_finalised'] = bag_ok
missions = 0
chat = f'{d}/llm_chat'
if os.path.isdir(chat):
    for f in os.listdir(chat):
        if f.endswith('.jsonl'):
            missions += sum(1 for _ in open(os.path.join(chat, f)))
meta['operator_commands_logged'] = missions

# Did the stack actually come up? A launch that dies on a bad argument leaves a
# finalised but empty bag, which looks healthy in every other check — this is
# the signal that distinguishes "recorded nothing" from "recorded a quiet run".
db = f'{d}/bag/bag_0.db3'
msgs = 0
# A requested topic that recorded nothing is the other silent failure: rosbag2
# drops whole families (anything under /_action/) without --include-hidden-topics
# and says so only as a WARN in bag_record.log, so the run looks healthy right
# up to the analysis. Name the empties here instead.
empty = []
# Channel 2 is opt-in (enable_passive_observer), so its topic is legitimately
# silent in every default run. Warning about it each time would train the
# operator to ignore this whole block, which is the one thing it must not do.
expected_empty = {'/llm/command'}
if os.path.exists(db):
    try:
        import sqlite3
        con = sqlite3.connect(db)
        msgs = con.execute('SELECT COUNT(*) FROM messages').fetchone()[0]
        counts = dict(con.execute(
            'SELECT t.name, COUNT(m.id) FROM topics t '
            'LEFT JOIN messages m ON m.topic_id = t.id GROUP BY t.id'))
        con.close()
        empty = [t for t in requested
                 if counts.get(t, 0) == 0 and t not in expected_empty]
    except Exception:
        msgs = -1
meta['bag_messages'] = msgs
meta['topics_recorded_empty'] = empty
json.dump(meta, open(p, 'w'), indent=2)

launch_log = f'{d}/launch.log'
launch_lines = 0
if os.path.exists(launch_log):
    launch_lines = sum(1 for _ in open(launch_log, errors='replace'))

print()
print('=' * 62)
print(f'Session: {d}')
print(f"  bag finalised          : {'yes' if bag_ok else 'NO — bag may be unreadable'}")
print(f'  messages recorded       : {msgs}')
print(f'  operator commands logged: {missions}')
if empty:
    print(f'  topics that recorded NOTHING ({len(empty)}):')
    for t in empty:
        print(f'      {t}')
if msgs <= 0 or launch_lines < 5:
    print()
    print('  *** RUN FAILED — the stack did not start. ***')
    print(f'  launch.log is {launch_lines} line(s); check its first line for the')
    print('  reason. This folder has no usable data; delete it and retry.')
elif missions == 0:
    print('  WARNING: no channel-3 records. Did you type anything into the')
    print('           RViz chat panel? Without commands there is nothing to score.')
print('=' * 62)
print('Send this whole folder back for analysis.')
PY
}
trap cleanup EXIT INT TERM

# ros2 launch rejects `name:=` with an empty value ("malformed launch
# argument"), so unset factors must be omitted entirely rather than passed
# blank. Build the argument list instead of interpolating.
LAUNCH_ARGS=(
  "planner:=$PLANNER"
  "scenario:=$SCENARIO"
  "num_robots:=$NUM_ROBOTS"
  "session_dir:=$SESSION"
)
[ -n "$REMEDIATION" ]       && LAUNCH_ARGS+=("remediation_enabled:=$REMEDIATION")
[ -n "$REPAIR" ]            && LAUNCH_ARGS+=("llm_repair_enabled:=$REPAIR")
[ -n "$SUPERVISION" ]       && LAUNCH_ARGS+=("llm_mission_supervision_enabled:=$SUPERVISION")
[ -n "$TOOL_CALLING" ]      && LAUNCH_ARGS+=("tool_calling_enabled:=$TOOL_CALLING")
[ -n "$STRUCTURED_OUTPUT" ] && LAUNCH_ARGS+=("structured_output_enabled:=$STRUCTURED_OUTPUT")
[ -n "$MODEL_OVERRIDE" ]    && LAUNCH_ARGS+=("llm_model:=$MODEL_OVERRIDE")
[ -n "$LLM_ENDPOINT" ]      && LAUNCH_ARGS+=("llm_endpoint:=$LLM_ENDPOINT")

ros2 launch iros_llm_swarm_bringup swarm_full_demo.launch.py \
  "${LAUNCH_ARGS[@]}" \
  "${EXTRA[@]+"${EXTRA[@]}"}" \
  2>&1 | tee "$SESSION/launch.log"
