#!/usr/bin/env bash
# Close the reference cells left incomplete by OpenRouter 403s.
#
# WHY
# Four of the eight hosted reference cells carry cases that never reached a
# model: OpenRouter answers "Access denied by security policy." in under a
# second, before generation. Scored as-is they read as planning failures, which
# would understate three of the four models. repair_transport_errors.py re-runs
# such cases in full; this script drives it cell by cell.
#
# WHAT IS PINNED AND WHY
# Each cell gets one provider for the whole repair (`OPENROUTER_PROVIDER`, which
# sets provider.order with allow_fallbacks=false) so the replacement data is
# attributable rather than sprayed across whatever the router picked that
# minute. The pin is chosen by probe_provider_block.py immediately before the
# run, because availability moves: DeepInfra answered 429 for qwen3.5-9b at the
# same minute Venice, SiliconFlow and Parasail all answered fine.
#
# This does mean a repaired cell can mix providers -- the 26 surviving nemotron
# cases were served by whoever the router chose, the 18 repaired ones by the
# pin. Accepted deliberately: the comparison in the paper is across models and
# across the schema switch, and provider identity is not expected to correlate
# with either. The manifest records the pin so the choice stays visible.
#
# Together is excluded everywhere the schema is on -- it rejects the recursive
# $ref outright ("tool schema contains a circular reference"), as do Anthropic
# and OpenAI. That is a finding, not a routing problem.
#
# LLM_CALL_DELAY_SEC paces the harness, which otherwise fires the next request
# the instant the last one lands. The 403 is edge-side and burst-triggered --
# it has been observed on /api/v1/key, an endpoint with no provider behind it
# at all -- so pacing, not provider choice, is the actual mitigation.
#
# Usage (the repo-root .env supplies API_KEY; it is read here, so no sourcing
# dance in the caller -- `set +a; . ./.env` leaves API_KEY unexported and a
# child script never sees it):
#     paper/results/raw/close_blocked_cells.sh            # every pending cell
#     paper/results/raw/close_blocked_cells.sh nemo_e1    # one cell
set -uo pipefail

cd "$(dirname "$0")/../../.." || exit 1
if [ -z "${API_KEY:-}" ] && [ -f .env ]; then
  set -a; . ./.env; set +a
fi

SRC=/home/fabian/ros2_ws/src
RAW=paper/results/raw
DELAY=${LLM_CALL_DELAY_SEC:-1}
ATTEMPTS=${MAX_ATTEMPTS:-2}

# cell            provider      file
#
# 397b: Chutes, DeepInfra, Parasail and DigitalOcean all served the full
# 9k-token schema-constrained prompt when probed. Chutes is fp8 and among the
# cheapest of those.
#
# nemotron: DeepInfra is not a preference, it is the only option. Of the four
# providers carrying this model, BaseTen and Venice do not implement structured
# outputs at all -- pinning either with response_format set makes OpenRouter
# answer 404 "No endpoints found", because it filters the pool down to
# providers that can honour the request and finds none. Together implements
# structured outputs but rejects this particular schema ("tool schema contains
# a circular reference"), the same recursive-$ref refusal Anthropic and OpenAI
# give. That leaves DeepInfra to serve the schema arm, so the no-schema arm is
# pinned there too: the contrast this cell exists to measure is schema on vs
# off, and splitting it across two providers would confound exactly that.
CELLS=(
  "b397_e1        Chutes        ${RAW}/e1ref_qwen_qwen3_5-397b-a17b.json"
  "b397_sch       Chutes        ${RAW}/e2ref_qwen_qwen3_5-397b-a17b.json"
  "nemo_e1        DeepInfra     ${RAW}/e1ref_nvidia_nemotron-3-ultra-550b-a55b.json"
  "nemo_sch       DeepInfra     ${RAW}/e2ref_nvidia_nemotron-3-ultra-550b-a55b.json"
)

if [ -z "${API_KEY:-}" ]; then
  echo "FATAL: API_KEY не задан (set +a; . ./.env)" >&2; exit 1
fi

# Wait for a pinned provider to stop answering 429. DeepInfra rate-limits
# nemotron upstream for stretches at a time; the repair script's 20 s backoff is
# built for a blip, not for that, and firing a 220-call run into an upstream
# limit would burn the cell rather than fill it.
wait_for_provider() {
  local model=$1 prov=$2 tries=${3:-40} i=1 code
  while [ ${i} -le ${tries} ]; do
    code=$(curl -s -o /dev/null -w '%{http_code}' -X POST \
      https://openrouter.ai/api/v1/chat/completions \
      -H "Authorization: Bearer ${API_KEY}" -H 'Content-Type: application/json' \
      -d "{\"model\":\"${model}\",\"messages\":[{\"role\":\"user\",\"content\":\"hi\"}],\"max_tokens\":1,\"provider\":{\"order\":[\"${prov}\"],\"allow_fallbacks\":false}}")
    if [ "${code}" = "200" ]; then
      echo "   ${prov} доступен (попытка ${i})"; return 0
    fi
    echo "   $(date '+%H:%M:%S') ${prov} отдаёт ${code}, жду 60s (${i}/${tries})"
    sleep 60; i=$((i + 1))
  done
  echo "   ${prov} так и не освободился — ячейка пропущена"; return 1
}

want=${1:-}
for row in "${CELLS[@]}"; do
  read -r cell prov file <<<"${row}"
  [ -n "${want}" ] && [ "${cell}" != "${want}" ] && continue
  if [ ! -f "${file}" ]; then
    echo "== ${cell}: нет файла ${file}, пропускаю"; continue
  fi
  echo
  echo "===================================================================="
  echo "== ${cell}  провайдер=${prov}  пауза=${DELAY}s  попыток=${ATTEMPTS}"
  echo "== $(date '+%H:%M:%S')  ${file}"
  echo "===================================================================="
  model=$(python3 -c "import json,sys;print(json.load(open(sys.argv[1]))['llm_model'])" "${file}")
  wait_for_provider "${model}" "${prov}" || continue
  docker compose exec -T \
    -e LLM_API_KEY="${API_KEY}" \
    -e OPENROUTER_PROVIDER="${prov}" \
    -e LLM_CALL_DELAY_SEC="${DELAY}" \
    -e MAX_ATTEMPTS="${ATTEMPTS}" \
    terminal bash -lc "cd ${SRC} && python3 ${RAW}/repair_transport_errors.py ${file}"
  echo "== ${cell} завершён в $(date '+%H:%M:%S')"
done
