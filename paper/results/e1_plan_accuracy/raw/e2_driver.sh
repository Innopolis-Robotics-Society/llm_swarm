#!/bin/bash
# E2 — grounding and constraint ablation sweep driver.
#
# Runs INSIDE the dev container. Offline benchmark: no ROS stack, no simulator,
# no Stage. Only python3 + a reachable Ollama on the host.
#
#   docker compose up -d terminal
#   docker compose exec terminal bash /home/fabian/ros2_ws/src/paper/results/raw/e2_driver.sh
#
# Six new runs = 3 conditions x 2 models. The baseline arm (schema off,
# grounding full) is NOT re-run: it is the existing E1b data. The preflight
# below proves that reuse is legitimate by checking the prompt fingerprint.
#
# Resumable: an existing, non-empty output JSON causes that run to be skipped.

set -u

BENCH_DIR=/home/fabian/ros2_ws/src/iros_llm_orchestrator
OUT=/home/fabian/ros2_ws/src/paper/results/raw
SWEEP="$OUT/e2_sweep.log"

# Fingerprint of all 44 fully-built prompts at grounding=full, captured from
# the pre-ablation code. E1b's baseline is only reusable as E2's "off" arm if
# this still matches -- i.e. if the ablation changed nothing at its defaults.
BASELINE_FP=42d660fe681eaa3857c239bbe8d8e97043ae15c07ef8fe088a202e49048c37ed

MODELS=("qwen3.5:4b" "qwen3.5-9b:latest")

# E1b protocol, with one deliberate change: --timeout 90 -> 180.
# E1b recorded 0/220 timeouts per model at 90 s, so a higher ceiling could not
# have altered its numbers. Constrained decoding is measurably slower (a cold
# constrained call was observed at 57 s), and leaving the ceiling at 90 risks
# manufacturing timeout-failures in the constrained arm only -- which would
# read as "constrained decoding is worse" when it is really "the clock ran
# out". Raising it removes that confound without invalidating comparability.
REPEAT=5
MAX_TOKENS=2048
NUM_CTX=32768
TEMPERATURE=0.1
TIMEOUT=180
MAP=amongus

mkdir -p "$OUT"

log() { echo "=== [$(date -u +%FT%TZ)] $* ===" | tee -a "$SWEEP"; }

# Turn an Ollama tag into the filename form used since E1b:
#   qwen3.5:4b        -> qwen3.5-4b
#   qwen3.5-9b:latest -> qwen3.5-9b
slug() { echo "$1" | sed 's/:latest$//; s/:/-/g'; }

# ─────────────────────────────────────────────────────────── preflight ──
log "E2 PREFLIGHT"

cd "$BENCH_DIR" || { echo "FATAL: $BENCH_DIR missing"; exit 1; }

# 1. Baseline reuse check. If the default prompt drifted, E1b is no longer a
#    valid "off" arm and must be re-run instead of referenced.
FP=$(python3 benchmark_ch3.py --map "$MAP" --grounding full --print-prompt-hash 2>&1)
if [ "$FP" != "$BASELINE_FP" ]; then
  log "FATAL: prompt fingerprint drift"
  echo "  expected $BASELINE_FP" | tee -a "$SWEEP"
  echo "  actual   $FP"          | tee -a "$SWEEP"
  echo "  The default prompt changed, so the E1b baseline can NOT be reused" | tee -a "$SWEEP"
  echo "  as E2's schema-off / grounding-full arm. Either revert the change" | tee -a "$SWEEP"
  echo "  or re-run the baseline for both models and update the manifest."   | tee -a "$SWEEP"
  exit 1
fi
log "PREFLIGHT ok: prompt fingerprint matches E1b baseline ($BASELINE_FP)"

# 2. Model tags must exist. A wrong tag returns HTTP 404 instantly and silently
#    produces a full run of ~0.0 s failures -- this burned a slot during E1b.
TAGS=$(curl -s http://localhost:11434/api/tags)
for m in "${MODELS[@]}"; do
  if ! echo "$TAGS" | grep -q "\"$m\""; then
    log "FATAL: model tag '$m' not present in ollama /api/tags"
    echo "  available:" | tee -a "$SWEEP"
    echo "$TAGS" | python3 -c "import json,sys;[print('   ',x['name']) for x in json.load(sys.stdin)['models']]" | tee -a "$SWEEP"
    exit 1
  fi
done
log "PREFLIGHT ok: both model tags present"

# ────────────────────────────────────────────────────────────── sweep ──
# Conditions are (label, extra-args). Grounding arms keep schema OFF and the
# schema arm keeps grounding FULL: one factor at a time, never crossed.
run_one() {
  local model="$1" label="$2"; shift 2
  local extra=("$@")
  local mslug json log_f
  mslug=$(slug "$model")
  json="$OUT/e2_${label}_${mslug}.json"
  log_f="$OUT/e2_${label}_${mslug}.log"

  if [ -s "$json" ]; then
    log "SKIP $label / $model (output exists: $(basename "$json"))"
    return 0
  fi

  log "START $label / $model"
  python3 benchmark_ch3.py \
    --map "$MAP" \
    --llm-mode ollama \
    --llm-model "$model" \
    --llm-max-tokens "$MAX_TOKENS" \
    --llm-num-ctx "$NUM_CTX" \
    --llm-temperature "$TEMPERATURE" \
    --timeout "$TIMEOUT" \
    --repeat "$REPEAT" \
    --no-color \
    --json "$json" \
    "${extra[@]}" \
    > "$log_f" 2>&1
  local rc=$?

  # Latency sanity: a run whose calls all took ~0 s means the backend rejected
  # every request (bad tag / dead endpoint), not that the model is fast.
  if [ -s "$json" ]; then
    python3 - "$json" <<'PY' | tee -a "$SWEEP"
import json, sys, statistics
d = json.load(open(sys.argv[1]))
el = [r['elapsed_sec'] for r in d['results']]
med = statistics.median(el) if el else 0.0
print(f"    n={len(el)} passed={d['passed']}/{d['total']} "
      f"median={med:.2f}s min={min(el):.2f}s max={max(el):.2f}s "
      f"fp={d.get('prompt_fingerprint','?')[:16]}")
if med < 0.5:
    print("    WARNING: median latency < 0.5 s -- backend probably rejected "
          "every call (check the model tag and endpoint)")
PY
  fi
  log "DONE $label / $model (exit $rc)"
}

# Grouped by model so each stays warm in VRAM across its three conditions
# rather than thrashing 3.4 GB / 10 GB in and out between every run.
for model in "${MODELS[@]}"; do
  log "MODEL $model"
  run_one "$model" schema_on                 --schema-constrained
  run_one "$model" grounding_locations_only  --grounding locations_only
  run_one "$model" grounding_coords_only     --grounding coords_only
done

log "E2 SWEEP COMPLETE"
echo
echo "Outputs in $OUT:"
ls -la "$OUT"/e2_*.json 2>/dev/null
