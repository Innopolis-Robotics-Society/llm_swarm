#!/bin/bash
# E2 follow-up — schema ladder on qwen3.5-9b.
#
# Runs INSIDE the dev container. Offline: python3 + a reachable Ollama, no ROS.
#
#   docker compose exec terminal bash \
#     /home/fabian/ros2_ws/src/paper/results/raw/e2_ladder_driver.sh
#
# Question: is qwen3.5-9b's collapse under PLAN_RESPONSE_SCHEMA caused by the
# recursive $ref, by the oneOf construct, or by the availability of a cheap
# `idle` exit? The ladder separates them:
#
#   S1  six-branch recursive oneOf   — the shipped schema (already run, reused)
#   S2  four-branch oneOf, no $ref   — S1 vs S2 isolates recursion
#   S3  single branch, no oneOf      — S2 vs S3 isolates the oneOf construct
#   S1n S1 minus the idle branch     — removes the cheapest valid exit
#
# SCORING CONSTRAINT (the thing that can silently invalidate this):
# S2 cannot express a plan whose correct answer is a sequence or parallel, and
# each S3 run can express only its own node type. Scoring those over all 44
# cases would measure "the grammar forbids the right answer", not "the grammar
# breaks the model". So each variant runs ONLY on the cases it can express, and
# build_e2_ladder.py compares against S1 and the schema-off baseline recomputed
# on that same subset. Comparisons against the full-44 numbers in Sec. 6.2 are
# NOT valid and must never appear in the writeup.
#
# Resumable: an existing, non-empty output JSON causes that run to be skipped.

set -u

BENCH_DIR=/home/fabian/ros2_ws/src/iros_llm_orchestrator
OUT=/home/fabian/ros2_ws/src/paper/results/raw
SWEEP="$OUT/e2_ladder_sweep.log"

BASELINE_FP=42d660fe681eaa3857c239bbe8d8e97043ae15c07ef8fe088a202e49048c37ed
MODEL="qwen3.5-9b:latest"

REPEAT=5
MAX_TOKENS=2048
NUM_CTX=32768
TEMPERATURE=0.1
TIMEOUT=180
MAP=amongus

# Expressible subsets, derived from each case's expected top-level node type
# (recovered from its validator by paper/results/analyze_e2_nodes.py).
MAPF_CASES="mapf_basic_01 mapf_basic_02 mapf_basic_03 mapf_basic_04 mapf_basic_05 \
mapf_spread_01 mapf_spread_02 mapf_explicit_01 mapf_explicit_02 \
formation_move_01 formation_move_02 edge_03 edge_04"
IDLE_CASES="idle_01 idle_02 idle_03 escalate_01 escalate_02 escalate_03 escalate_04 \
edge_02 edge_06"
DISBAND_CASES="disband_01"
S2_CASES="$MAPF_CASES $IDLE_CASES $DISBAND_CASES"      # 13 + 9 + 1 = 23

mkdir -p "$OUT"
log() { echo "=== [$(date -u +%FT%TZ)] $* ===" | tee -a "$SWEEP"; }

cd "$BENCH_DIR" || { echo "FATAL: $BENCH_DIR missing"; exit 1; }

# ─────────────────────────────────────────────────────────── preflight ──
log "E2 LADDER PREFLIGHT"

# Same gate as the E2 sweep: the ladder is compared against S1 and the E1b
# baseline, so the prompts must still be byte-identical to what those ran.
FP=$(python3 benchmark_ch3.py --map "$MAP" --grounding full --print-prompt-hash 2>&1)
if [ "$FP" != "$BASELINE_FP" ]; then
  log "FATAL: prompt fingerprint drift"
  echo "  expected $BASELINE_FP" | tee -a "$SWEEP"
  echo "  actual   $FP"          | tee -a "$SWEEP"
  exit 1
fi
log "PREFLIGHT ok: prompt fingerprint matches ($BASELINE_FP)"

if ! curl -s http://localhost:11434/api/tags | grep -q "\"$MODEL\""; then
  log "FATAL: model tag '$MODEL' not present in ollama /api/tags"
  exit 1
fi
log "PREFLIGHT ok: model tag present"

# The s2/s3 variants are derived from PLAN_RESPONSE_SCHEMA at import time and
# benchmark_ch3.py asserts the derivation reproduces it exactly. Import here so
# a broken derivation aborts before burning ~300 model calls.
python3 -c "import benchmark_ch3" || { log "FATAL: schema derivation assertion failed"; exit 1; }
log "PREFLIGHT ok: schema-ladder derivation reproduces PLAN_RESPONSE_SCHEMA"

# ────────────────────────────────────────────────────────────── sweep ──
run_one() {
  local label="$1" variant="$2"; shift 2
  local cases=("$@")
  local json="$OUT/e2_ladder_${label}.json"
  local log_f="$OUT/e2_ladder_${label}.log"

  if [ -s "$json" ]; then
    log "SKIP $label (output exists)"
    return 0
  fi

  log "START $label (variant=$variant, ${#cases[@]} cases x $REPEAT)"
  python3 benchmark_ch3.py \
    --map "$MAP" \
    --llm-mode ollama \
    --llm-model "$MODEL" \
    --llm-max-tokens "$MAX_TOKENS" \
    --llm-num-ctx "$NUM_CTX" \
    --llm-temperature "$TEMPERATURE" \
    --timeout "$TIMEOUT" \
    --repeat "$REPEAT" \
    --no-color \
    --schema-constrained \
    --schema-variant "$variant" \
    --tests "${cases[@]}" \
    --json "$json" \
    > "$log_f" 2>&1
  local rc=$?

  if [ -s "$json" ]; then
    python3 - "$json" <<'PY' | tee -a "$SWEEP"
import json, sys, statistics
d = json.load(open(sys.argv[1]))
el = [r['elapsed_sec'] for r in d['results']]
med = statistics.median(el) if el else 0.0
print(f"    n={len(el)} passed={d['passed']}/{d['total']} "
      f"median={med:.2f}s variant={d.get('schema_variant')} "
      f"branches={d.get('schema_branches')}")
if med < 0.5:
    print("    WARNING: median latency < 0.5 s -- backend probably rejected "
          "every call (check the model tag and endpoint)")
PY
  fi
  log "DONE $label (exit $rc)"
}

# shellcheck disable=SC2086
run_one s2          s2         $S2_CASES
# shellcheck disable=SC2086
run_one s3_mapf     s3_mapf    $MAPF_CASES
# shellcheck disable=SC2086
run_one s3_idle     s3_idle    $IDLE_CASES
# shellcheck disable=SC2086
run_one s3_disband  s3_disband $DISBAND_CASES
# S1 with the cheapest exit removed. Scored on the mapf subset only: the
# question is whether the mapf branch becomes reachable once idle is gone, or
# whether the collapse merely relocates to the next-simplest branch.
# shellcheck disable=SC2086
run_one s1_noidle   s1_noidle  $MAPF_CASES

log "E2 LADDER COMPLETE"
ls -la "$OUT"/e2_ladder_*.json 2>/dev/null
