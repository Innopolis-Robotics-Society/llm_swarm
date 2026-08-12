#!/bin/bash
# E1/E2 hosted reference — ONE model per process, for running the four models
# in parallel on separate keys.
#
#   docker compose exec -e LLM_API_KEY=sk-or-... -e REF_MODEL=google/gemini-2.5-flash \
#     terminal bash .../e1e2_ref_worker.sh
#
# Same contract as e1e2_ref_driver.sh (same output names, same resumability, so
# the two can be mixed freely), with two differences that matter:
#
# 1. ONE model, taken from REF_MODEL. Parallelism is by model on purpose: each
#    model is served by a different upstream (Venice, StreamLake, DeepInfra,
#    Google), so four streams do not contend. Splitting a single model's two
#    arms across processes would put two streams on one provider, which is
#    exactly where a 429 would come from.
#
# 2. Budget is read from the ACCOUNT, not the key. Keys issued without a spend
#    limit report limit_remaining=null, which the original driver read as $0 and
#    refused to start on. Parallel workers share one credit pool anyway, so the
#    account figure is the only one that means anything here.
#
# ARMS selects which cells to run: "e1", "e2", "mt2048", or any space-separated
# subset. Default runs e1 and e2.

set -u

BENCH_DIR=/home/fabian/ros2_ws/src/iros_llm_orchestrator
OUT=/home/fabian/ros2_ws/src/paper/results/raw

BASELINE_FP=42d660fe681eaa3857c239bbe8d8e97043ae15c07ef8fe088a202e49048c37ed
ENDPOINT=https://openrouter.ai/api/v1/chat/completions

REPEAT=5
MAX_TOKENS=8192
NUM_CTX=32768
TEMPERATURE=0.1
TIMEOUT=240
MAP=amongus
BUDGET_FLOOR=0.40      # shared pool, so leave more room than a single stream needs

MODEL="${REF_MODEL:-}"
ARMS="${ARMS:-e1 e2}"

[ -n "$MODEL" ]              || { echo "FATAL: REF_MODEL не задан"; exit 1; }
[ -n "${LLM_API_KEY:-}" ]    || { echo "FATAL: LLM_API_KEY не задан"; exit 1; }

SLUG=$(echo "$MODEL" | tr '/.' '__')
SWEEP="$OUT/e1e2_ref_${SLUG}.sweep.log"
mkdir -p "$OUT"
log() { echo "=== [$(date -u +%FT%TZ)] [$SLUG] $* ===" | tee -a "$SWEEP"; }

cd "$BENCH_DIR" || { echo "FATAL: $BENCH_DIR missing"; exit 1; }

FP=$(python3 benchmark_ch3.py --map "$MAP" --grounding full --print-prompt-hash 2>&1)
if [ "$FP" != "$BASELINE_FP" ]; then
  log "FATAL: prompt fingerprint drift (got $FP)"
  exit 1
fi
log "preflight ok: fingerprint matches"

# Account-level, because these keys have no individual cap.
balance() {
  curl -s --max-time 20 -H "Authorization: Bearer $LLM_API_KEY" \
    https://openrouter.ai/api/v1/credits \
  | python3 -c "
import json,sys
d=json.load(sys.stdin)['data']
print('%.4f' % (float(d['total_credits']) - float(d['total_usage'])))" 2>/dev/null || echo "ERR"
}

BAL=$(balance)
[ "$BAL" = "ERR" ] && { log "FATAL: key rejected or /credits unreachable"; exit 1; }
log "preflight ok: account has \$$BAL"

run_arm() {
  local arm="$1" mt="$2"; shift 2
  local extra=("$@")
  local json="$OUT/${arm}_${SLUG}.json"
  local log_f="$OUT/${arm}_${SLUG}.log"

  if [ -s "$json" ]; then log "SKIP $arm (output exists)"; return 0; fi

  local bal; bal=$(balance)
  [ "$bal" = "ERR" ] && { log "ABORT $arm: /credits stopped answering"; return 1; }
  if python3 -c "import sys; sys.exit(0 if float('$bal') < $BUDGET_FLOOR else 1)"; then
    log "ABORT $arm: shared pool down to \$$bal, floor is \$$BUDGET_FLOOR"
    return 1
  fi

  log "START $arm (max_tokens=$mt, 44 x $REPEAT, pool \$$bal)"
  local t0=$SECONDS
  python3 benchmark_ch3.py \
    --map "$MAP" --llm-mode http --llm-endpoint "$ENDPOINT" --llm-model "$MODEL" \
    --llm-max-tokens "$mt" --llm-num-ctx "$NUM_CTX" --llm-temperature "$TEMPERATURE" \
    --timeout "$TIMEOUT" --repeat "$REPEAT" --no-color \
    "${extra[@]+"${extra[@]}"}" --json "$json" > "$log_f" 2>&1
  local rc=$? el=$((SECONDS - t0))

  if [ -s "$json" ]; then
    python3 - "$json" "$el" <<'PY' | tee -a "$SWEEP"
import json, statistics, sys
d = json.load(open(sys.argv[1]))
el = [r['elapsed_sec'] for r in d['results']]
med = statistics.median(el) if el else 0.0
print(f"    n={len(el)} passed={d['passed']}/{d['total']} "
      f"median={med:.2f}s wall={int(sys.argv[2])}s schema={d.get('schema_constrained')}")
errs = [r for r in d['results'] if (r.get('parse_error') or '').strip()]
if errs:
    kinds = {}
    for r in errs:
        k = (r['parse_error'] or '')[:60]
        kinds[k] = kinds.get(k, 0) + 1
    print(f"    *** {len(errs)}/{len(d['results'])} вызовов без разбираемого ответа "
          f"-- они посчитаны ПРОВАЛАМИ ***")
    for k, n in sorted(kinds.items(), key=lambda kv: -kv[1])[:4]:
        print(f"      {n:4d}x {k}")
    print("      Транспортные ошибки чинит repair_transport_errors.py")
PY
  else
    log "  выхода нет; хвост $log_f:"; tail -3 "$log_f" | sed 's/^/    /' | tee -a "$SWEEP"
  fi
  log "DONE $arm (exit $rc, ${el}s, pool \$$(balance))"
}

for a in $ARMS; do
  case "$a" in
    e1)     run_arm e1ref        "$MAX_TOKENS" || break ;;
    e2)     run_arm e2ref        "$MAX_TOKENS" --schema-constrained || break ;;
    mt2048) run_arm e1ref_mt2048 2048 || break ;;
    *)      log "неизвестное плечо '$a', пропускаю" ;;
  esac
done

log "WORKER COMPLETE (pool \$$(balance))"
