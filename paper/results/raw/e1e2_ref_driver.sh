#!/bin/bash
# E1/E2 hosted reference sweep — four models through OpenRouter.
#
# Runs INSIDE the dev container. Offline from ROS: python3 + outbound HTTPS.
#
#   docker compose exec -e LLM_API_KEY=sk-or-... terminal bash \
#     /home/fabian/ros2_ws/src/paper/results/raw/e1e2_ref_driver.sh
#
# The key is read from the environment and is never written to disk, to the
# sweep log, or to any output file. Do not pass it as a CLI argument — it would
# land in the process table.
#
# WHAT THIS IS FOR
# E1 measured five locally-hosted open-weight models (3.4-24B) and E2 found
# that qwen3.5-9b collapses under PLAN_RESPONSE_SCHEMA. Both results have a
# hole a reviewer will find:
#
#   * E1's "roughly one command in three fails regardless of which model is
#     deployed" generalises from a 3.4-24B band. Nothing here says what a much
#     larger model does.
#   * E2's collapse was observed on ONE model under ONE runtime (Ollama +
#     llama.cpp GBNF) at one quantisation. If it does not reproduce elsewhere,
#     the mechanism claim is about our build, not about the model.
#
# The four models answer those separately:
#
#   qwen3.5-9b        same weights as the collapsed local model, different
#                     runtime and quantisation -> does the collapse reproduce?
#   qwen3.5-397b-a17b same family, 44x bigger -> scale with tuning held fixed.
#                     A smoke test showed it alphabetises keys AND plans
#                     correctly, which already narrows the E2 mechanism claim.
#   nemotron-3-ultra  open-weight 550B: the ceiling of the SAME category the
#                     paper is about (open weights), not a different one.
#   gemini-2.5-flash  proprietary contrast.
#
# NOT COMPARABLE TO ANYTHING ELSE: these runs use --llm-max-tokens 8192 while
# E1/E2 used 2048. Hosted reasoning routes spend the whole 2048 on thinking and
# return empty content, so 2048 would manufacture failures unrelated to
# planning. The `budget_control` cell below quantifies what the larger budget
# is worth on its own; report it next to any hosted-vs-local comparison.

set -u

BENCH_DIR=/home/fabian/ros2_ws/src/iros_llm_orchestrator
OUT=/home/fabian/ros2_ws/src/paper/results/raw
SWEEP="$OUT/e1e2_ref_sweep.log"

BASELINE_FP=42d660fe681eaa3857c239bbe8d8e97043ae15c07ef8fe088a202e49048c37ed
ENDPOINT=https://openrouter.ai/api/v1/chat/completions

REPEAT=5
MAX_TOKENS=8192
NUM_CTX=32768
TEMPERATURE=0.1
TIMEOUT=240
MAP=amongus

# Abort before starting a run that would leave less than this on the key. A run
# that dies at case 180 of 220 wastes everything spent on it.
BUDGET_FLOOR=0.20

MODELS=(
  "qwen/qwen3.5-9b"
  "qwen/qwen3.5-397b-a17b"
  "nvidia/nemotron-3-ultra-550b-a55b"
  "google/gemini-2.5-flash"
)

mkdir -p "$OUT"
log() { echo "=== [$(date -u +%FT%TZ)] $* ===" | tee -a "$SWEEP"; }
slug() { echo "$1" | tr '/.' '__'; }

cd "$BENCH_DIR" || { echo "FATAL: $BENCH_DIR missing"; exit 1; }

# ─────────────────────────────────────────────────────────── preflight ──
log "E1/E2 HOSTED REFERENCE PREFLIGHT"

if [ -z "${LLM_API_KEY:-}" ]; then
  log "FATAL: LLM_API_KEY is not set"
  echo "  docker compose exec -e LLM_API_KEY=sk-or-... terminal bash $0" | tee -a "$SWEEP"
  exit 1
fi

# Same gate as every other sweep: the reference is compared against E1 and E2,
# so the prompts must still be byte-identical to what those ran.
FP=$(python3 benchmark_ch3.py --map "$MAP" --grounding full --print-prompt-hash 2>&1)
if [ "$FP" != "$BASELINE_FP" ]; then
  log "FATAL: prompt fingerprint drift"
  echo "  expected $BASELINE_FP" | tee -a "$SWEEP"
  echo "  actual   $FP"          | tee -a "$SWEEP"
  exit 1
fi
log "PREFLIGHT ok: prompt fingerprint matches ($BASELINE_FP)"

balance() {
  curl -s --max-time 20 -H "Authorization: Bearer $LLM_API_KEY" \
    https://openrouter.ai/api/v1/key \
  | python3 -c "import json,sys; print('%.4f' % (json.load(sys.stdin)['data']['limit_remaining'] or 0))" \
    2>/dev/null || echo "ERR"
}

BAL=$(balance)
if [ "$BAL" = "ERR" ]; then
  log "FATAL: key rejected or /api/v1/key unreachable"
  exit 1
fi
log "PREFLIGHT ok: key accepted, remaining \$$BAL"

# A model id that 404s only shows up as a failed run 200 calls later.
MISSING=$(curl -s --max-time 30 https://openrouter.ai/api/v1/models \
  | python3 -c "
import json,sys
have={m['id'] for m in json.load(sys.stdin)['data']}
want=['qwen/qwen3.5-9b','qwen/qwen3.5-397b-a17b',
      'nvidia/nemotron-3-ultra-550b-a55b','google/gemini-2.5-flash']
print(' '.join(w for w in want if w not in have))")
if [ -n "$MISSING" ]; then
  log "FATAL: model id(s) not offered: $MISSING"
  exit 1
fi
log "PREFLIGHT ok: all four model ids present"

# ────────────────────────────────────────────────────────────── sweep ──
run_one() {
  local model="$1" arm="$2" mt="$3"; shift 3
  local extra=("$@")
  local json="$OUT/${arm}_$(slug "$model").json"
  local log_f="$OUT/${arm}_$(slug "$model").log"

  if [ -s "$json" ]; then
    log "SKIP $arm $model (output exists)"
    return 0
  fi

  local bal; bal=$(balance)
  if [ "$bal" = "ERR" ]; then log "ABORT: key stopped answering"; return 1; fi
  if python3 -c "import sys; sys.exit(0 if float('$bal') < $BUDGET_FLOOR else 1)"; then
    log "ABORT $arm $model: only \$$bal left, floor is \$$BUDGET_FLOOR"
    return 1
  fi

  log "START $arm $model (max_tokens=$mt, 44 cases x $REPEAT, \$$bal left)"
  local t0=$SECONDS
  python3 benchmark_ch3.py \
    --map "$MAP" \
    --llm-mode http \
    --llm-endpoint "$ENDPOINT" \
    --llm-model "$model" \
    --llm-max-tokens "$mt" \
    --llm-num-ctx "$NUM_CTX" \
    --llm-temperature "$TEMPERATURE" \
    --timeout "$TIMEOUT" \
    --repeat "$REPEAT" \
    --no-color \
    "${extra[@]+"${extra[@]}"}" \
    --json "$json" \
    > "$log_f" 2>&1
  local rc=$? el=$((SECONDS - t0))

  if [ -s "$json" ]; then
    python3 - "$json" "$el" <<'PY' | tee -a "$SWEEP"
import json, statistics, sys
d = json.load(open(sys.argv[1]))
el = [r['elapsed_sec'] for r in d['results']]
med = statistics.median(el) if el else 0.0
print(f"    n={len(el)} passed={d['passed']}/{d['total']} "
      f"median={med:.2f}s wall={int(sys.argv[2])}s "
      f"schema={d.get('schema_constrained')}")
if med < 0.5:
    print("    WARNING: median latency < 0.5 s -- the endpoint probably "
          "rejected every call (check the model id and the key)")
PY
  else
    log "  no output written; tail of $log_f:"
    tail -3 "$log_f" | sed 's/^/    /' | tee -a "$SWEEP"
  fi
  log "DONE $arm $model (exit $rc, ${el}s, \$$(balance) left)"
}

# Order matters: cheapest and most load-bearing first, so a budget that runs
# out early still leaves the reproducibility check done.
for M in "${MODELS[@]}"; do
  run_one "$M" e1ref "$MAX_TOKENS"                    || break
  run_one "$M" e2ref "$MAX_TOKENS" --schema-constrained || break
done

# Token-budget control: the only cell that isolates what raising 2048 -> 8192
# buys, on the cheapest model. Without it, every hosted-vs-local number is open
# to "you just gave the hosted model four times the room".
run_one "qwen/qwen3.5-9b" e1ref_mt2048 2048

log "E1/E2 HOSTED REFERENCE COMPLETE (\$$(balance) left)"
ls -la "$OUT"/e1ref_*.json "$OUT"/e2ref_*.json 2>/dev/null
