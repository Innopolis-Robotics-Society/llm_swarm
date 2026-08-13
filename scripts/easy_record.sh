#!/bin/bash
# easy_record.sh — one flag per E3 grid cell. Everything else is baked in.
#
#   bash src/scripts/easy_record.sh --B
#   bash src/scripts/easy_record.sh --M2d-cut
#   bash src/scripts/easy_record.sh --list
#
# WHY THIS EXISTS
# The campaign is 61 runs across 13 cells whose commands differ by one or two
# flags. Assembling those by hand 61 times is the single most likely way to
# lose a run: cells B, B-M1, B-M3 and M4 launch with byte-identical flags and
# differ ONLY by the mission the operator types, so a wrong --cell is invisible
# afterwards -- the factors, model and planner all match. Here the cell name
# picks the flags, so the two cannot disagree.
#
# WHAT IT ADDS BEYOND SHORTER TYPING
#   * repeat numbering is counted from the sessions already on disk, so nobody
#     has to remember whether this is rep 3 or rep 4;
#   * the mission text is printed for copy-paste, because it must go into the
#     RViz panel byte-identical and retyping it makes runs incomparable;
#   * cells that need a second terminal (M2d door closure, no-LLM driver) print
#     exactly what to run there and when.
#
# It is a wrapper, not a replacement: every flag still goes through
# record_session.sh, and session.json is written by that script alone.
set -u

# Derived from where this script lives, not hardcoded to the container path.
# record_session.sh may hardcode /home/fabian/ros2_ws because it must source the
# workspace, but this wrapper only needs to find its sibling and the sessions
# directory -- and deriving them means `--dry-run` and the repeat counter also
# work from the host, which is the only way to check the wiring without bringing
# up a 20-robot stack.
SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REC="$SRC/scripts/record_session.sh"
SESSIONS_ROOT="${SWARM_SESSIONS_DIR:-$SRC/paper/results/sessions}"

# ── campaign constants ────────────────────────────────────────────────────
# Pinned for the whole campaign. Changing any of these mid-run invalidates the
# comparison between cells; see paper/E3_spec.md §4.2 and §4.6.
ENDPOINT=https://openrouter.ai/api/v1/chat/completions
MODEL=qwen/qwen3.5-397b-a17b
MODEL_SMALL=qwen/qwen3.5-9b        # cell model-9b only
PROVIDER="${OPENROUTER_PROVIDER:-Chutes}"

# Base configuration: every guard on, tool-calling path.
BASE=(--remediation true --repair true --supervision true
      --tool-calling true --structured-output false)

# ── mission texts ─────────────────────────────────────────────────────────
# Copied verbatim from paper/E3_spec.md §5. Operator pastes these into the RViz
# chat panel. Do not reword: any edit makes runs incomparable.
M1_TEXT='Send two orange robots to the electrical task at (-6.31, -3.66) and two orange robots to the communications task at (7.84, -15.76).'
M2_TEXT='Send two magenta robots to the med task at (-6.84, 1.37). Send two other magenta robots to carry the security cargo: first to the pickup at (-15.99, 3.65), then to the dropoff at (12.29, -5.06). Send two orange robots to the electrical task at (-6.31, -3.66) and two orange robots to the communications task at (7.84, -15.76). Send two yellow robots to the O2 task at (12.13, 2.05).'
M3_TEXT='Complete all declared tasks.'
M4_TEXT='Form a column of four cyan robots led by robot_0 in the cafeteria at (2.7, 10.1), then move the column to storage at (0.6, -10.9), then disband it.'

CASES="B no-remediation no-repair no-supervision constrained model-9b no-LLM \
M2d-detour M2d-cut PBS M4 B-M1 B-M3"

usage() {
  cat <<'EOF'
easy_record.sh — запуск одной ячейки сетки E3

  bash src/scripts/easy_record.sh --<ЯЧЕЙКА> [--rep N]
  bash src/scripts/easy_record.sh --list

Ячейки (13, всего 61 прогон):

  --B               опорная точка: всё включено, миссия M2          5 прогонов
  --no-remediation  без перепромпта после отказа исполнения         5
  --no-repair       без постпроверки и починки результата           5
  --no-supervision  без цикла выполнить→проверить→продолжить        5
  --constrained     схема вместо инструментов                       5
  --model-9b        та же семья, в 44 раза меньше параметров        5
  --no-LLM          потолок субстрата, план без модели              5
  --M2d-detour      дверь ломает маршрут, обход есть                5
  --M2d-cut         дверь делает цель недостижимой                  5
  --PBS             другой планировщик                              5
  --M4              строй: сбор, проход, роспуск                    5
  --B-M1            нижний край сложности                           3
  --B-M3            верхний край сложности                          3

Номер повтора считается сам по уже записанным сессиям этой ячейки.
Переопределяется через --rep N.
EOF
}

CASE=""
REP=""
DRY=0
while [ $# -gt 0 ]; do
  case "$1" in
    --list|-l|-h|--help) usage; exit 0 ;;
    --rep) REP="$2"; shift 2 ;;
    --dry-run) DRY=1; shift ;;
    --*)
      CANDIDATE="${1#--}"
      # shellcheck disable=SC2076
      if [[ " $CASES " == *" $CANDIDATE "* ]]; then
        CASE="$CANDIDATE"; shift
      else
        echo "FATAL: неизвестная ячейка '$1'"; echo; usage; exit 1
      fi ;;
    *) echo "FATAL: лишний аргумент '$1'"; exit 1 ;;
  esac
done

if [ -z "$CASE" ]; then
  echo "FATAL: не указана ячейка"; echo; usage; exit 1
fi

# ── repeat numbering ──────────────────────────────────────────────────────
# Counted from session.json files already on disk rather than kept in a
# counter file: the folders are the state, so this stays correct after a
# machine change, a discarded run, or someone else recording part of the cell.
if [ -z "$REP" ]; then
  REP=$(python3 - "$SESSIONS_ROOT" "$CASE" <<'PY'
import glob, json, os, sys
root, cell = sys.argv[1], sys.argv[2]
n = 0
for p in glob.glob(os.path.join(root, '*', 'session.json')):
    try:
        if json.load(open(p, encoding='utf-8')).get('cell') == cell:
            n += 1
    except Exception:
        pass
print(n + 1)
PY
)
fi

# ── per-cell configuration ────────────────────────────────────────────────
ARGS=("${BASE[@]}")
MISSION=M2
MISSION_TEXT="$M2_TEXT"
EXTRA_MODEL=""
SECOND_WINDOW=""

case "$CASE" in
  B) ;;
  no-remediation) ARGS=(--remediation false --repair true --supervision true
                        --tool-calling true --structured-output false) ;;
  no-repair)      ARGS=(--remediation true --repair false --supervision true
                        --tool-calling true --structured-output false) ;;
  no-supervision) ARGS=(--remediation true --repair true --supervision false
                        --tool-calling true --structured-output false) ;;
  constrained)    ARGS=(--remediation true --repair true --supervision true
                        --tool-calling false --structured-output true) ;;
  model-9b)
    EXTRA_MODEL="$MODEL_SMALL"
    SECOND_WINDOW="ВНИМАНИЕ: Chutes не раздаёт девятку. Провайдер должен быть
  SiliconFlow, DeepInfra, Venice или Together — и НЕ Parasail, он для неё
  не поддерживает инструменты. Сейчас прибит: $PROVIDER" ;;
  PBS)            ARGS+=(--planner pbs) ;;
  M4)             MISSION=M4; MISSION_TEXT="$M4_TEXT" ;;
  B-M1)           MISSION=M1; MISSION_TEXT="$M1_TEXT" ;;
  B-M3)           MISSION=M3; MISSION_TEXT="$M3_TEXT" ;;
  M2d-detour|M2d-cut)
    if [ "$CASE" = "M2d-detour" ]; then DOOR=lower_engine_west
    else DOOR=cafeteria_north; fi
    SECOND_WINDOW="РОВНО через 60 с после отправки команды, во втором окне:

  docker compose exec terminal bash
  source /home/fabian/ros2_ws/install/setup.bash
  source /home/fabian/ros2_ws/src/scripts/setup_swarm_env.sh
  ros2 service call /doors/close \\
    iros_llm_swarm_interfaces/srv/CloseDoor \"{door_id: '$DOOR'}\"

Ответ сервиса записать: без него «модель справилась» неотличимо от
«дверь не закрылась». Прогон негоден, если роботы сломанного плеча
успели пройти дверь до закрытия." ;;
  no-LLM)
    MISSION_TEXT=""
    SECOND_WINDOW="В панель НИЧЕГО не печатать. Когда стек поднялся, во втором окне:

  docker compose exec terminal bash
  source /home/fabian/ros2_ws/install/setup.bash
  source /home/fabian/ros2_ws/src/scripts/setup_swarm_env.sh
  ros2 run iros_llm_swarm_bt e3_scripted_driver

В финальном блоке будет 'WARNING: no channel-3 records' — здесь это
правильно, миссию вёл скрипт. Для всех остальных ячеек это брак." ;;
esac

# ── operator briefing ─────────────────────────────────────────────────────
echo "=============================================================="
echo " ЯЧЕЙКА: $CASE      миссия: $MISSION      повтор: $REP"
echo " модель: ${EXTRA_MODEL:-$MODEL}   провайдер: $PROVIDER"
echo "=============================================================="
if [ -n "$MISSION_TEXT" ]; then
  echo
  echo " Когда все 20 роботов поднимутся и Nav2 станет активен,"
  echo " вставить В ПАНЕЛЬ RViz ровно этот текст:"
  echo
  echo "--------------------------------------------------------------"
  echo "$MISSION_TEXT"
  echo "--------------------------------------------------------------"
fi
if [ -n "$SECOND_WINDOW" ]; then
  echo
  echo "$SECOND_WINDOW"
fi
echo
echo " Останов — Ctrl+C. Записи: $SESSIONS_ROOT"
echo "=============================================================="
echo

MODEL_ARGS=(--model "${EXTRA_MODEL:-$MODEL}")
FULL=(bash "$REC"
      --llm-endpoint "$ENDPOINT"
      "${MODEL_ARGS[@]}"
      "${ARGS[@]}"
      --cell "$CASE" --mission "$MISSION" --rep "$REP"
      --note "provider=$PROVIDER")

if [ "$DRY" = 1 ]; then
  echo " СУХОЙ ПРОГОН — стек не поднимается. Команда была бы:"
  echo
  printf '   %q' "${FULL[@]}"; echo
  exit 0
fi

exec "${FULL[@]}"
