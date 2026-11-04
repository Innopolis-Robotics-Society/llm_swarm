#!/usr/bin/env bash
# Does the provider's constrained-decoding engine explain the 9b schema drop?
#
# THE PROBLEM THIS ANSWERS
# e2ref_qwen_qwen3_5-9b.json is now served by two providers: 27 cases by
# whoever the router picked during the original sweep, and 17 -- re-run after
# OpenRouter 403s -- pinned to SiliconFlow. Those two subsets score wildly
# differently under the schema: 85.2% vs 17.6% pass@1.
#
# Read naively that says multi-step plans collapse under constrained decoding,
# which is a claim the paper would lean on. But the split is perfectly
# confounded with the provider split, and unlike quantisation this confound is
# not incidental: providers implement structured outputs with different
# grammar engines (vLLM's outlines, xgrammar, llguidance), and the quality of
# that engine on a recursive $ref is precisely the thing being measured. A
# provider whose engine truncates the `steps` array would produce this exact
# signature -- "no mapf step", "robot_ids=[]", "no goals".
#
# THE TEST
# Cross the two case sets with three providers, one repeat each. If the drop
# follows the CASES across providers, it is a property of multi-step plans
# under the schema and the finding stands. If it follows SILICONFLOW across
# case sets, the repaired subset has to be re-run elsewhere before any of it
# is reported.
#
# 18 calls, roughly $0.03.
#
# Usage:  paper/results/raw/provider_decoder_control.sh
set -uo pipefail
cd "$(dirname "$0")/../../.." || exit 1
if [ -z "${API_KEY:-}" ] && [ -f .env ]; then set -a; . ./.env; set +a; fi

SRC=/home/fabian/ros2_ws/src
OUT=paper/results/raw/control_decoder
mkdir -p "${OUT}"

# Three that scored 5/5 in the original (router-chosen) sweep against three
# that scored badly in the SiliconFlow re-run. parallel_01 vs parallel_02 is
# the tightest pair available: same plan shape, same difficulty, different
# provider history -- if the shape were the cause, parallel_01 should have
# collapsed too, and it did not. formation_complex_01 is likewise multi-step
# and clean, so the clean set is not just the easy cases.
CLEAN="parallel_01 formation_complex_01 escalate_01"
REPAIRED="sequence_01 parallel_02 formation_create_01"

for prov in SiliconFlow Venice Parasail; do
  for set_name in clean repaired; do
    case ${set_name} in
      clean)    tests=${CLEAN} ;;
      repaired) tests=${REPAIRED} ;;
    esac
    dest="${OUT}/9b_sch_${prov}_${set_name}.json"
    echo "== ${prov} / ${set_name}: ${tests}"
    docker compose exec -T \
      -e LLM_API_KEY="${API_KEY}" \
      -e OPENROUTER_PROVIDER="${prov}" \
      -e LLM_CALL_DELAY_SEC=1 \
      terminal bash -lc "cd ${SRC}/iros_llm_orchestrator && python3 benchmark_ch3.py \
        --map amongus --llm-mode http \
        --llm-endpoint https://openrouter.ai/api/v1/chat/completions \
        --llm-model qwen/qwen3.5-9b \
        --llm-max-tokens 8192 --llm-num-ctx 32768 --llm-temperature 0.1 \
        --timeout 240 --repeat 1 --no-color --grounding full \
        --schema-constrained --schema-variant s1 \
        --tests ${tests} --json /home/fabian/ros2_ws/src/${dest}" >/dev/null 2>&1
  done
done

echo
python3 - <<'PYEOF'
import glob, json, os
rows = {}
for f in sorted(glob.glob('paper/results/raw/control_decoder/9b_sch_*.json')):
    _, _, prov, sets = os.path.basename(f)[:-5].split('_', 3)
    d = json.load(open(f, encoding='utf-8'))
    ok = sum(1 for r in d['results'] if r['passed'])
    err = sum(1 for r in d['results'] if (r.get('parse_error') or '').strip())
    rows.setdefault(prov, {})[sets] = '%d/%d%s' % (
        ok, len(d['results']), ' (+%d транспорт)' % err if err else '')
print('%-14s %-22s %s' % ('провайдер', 'исходно чистые 3', 'починенные 3'))
for prov, r in rows.items():
    print('%-14s %-22s %s' % (prov, r.get('clean', '—'), r.get('repaired', '—')))
print('\nследует за кейсами -> вывод про многошаговые планы устоял')
print('следует за SiliconFlow -> починенную подвыборку надо перегнать')
PYEOF
