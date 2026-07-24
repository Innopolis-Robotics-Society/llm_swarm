#!/bin/bash
source /home/fabian/ros2_ws/install/setup.bash
source /home/fabian/extras_ws/install/setup.bash
source /home/fabian/ros2_ws/src/scripts/setup_swarm_env.sh 2>/dev/null
cd /home/fabian/ros2_ws
OUT=/home/fabian/ros2_ws/src/paper/results/raw
SWEEP="$OUT/e4_sweep.log"

wait_ready() {
  local log="$1" tries=0
  while [ $tries -lt 60 ]; do
    grep -q "MAPF stack ready" "$log" 2>/dev/null && return 0
    sleep 1
    tries=$((tries+1))
  done
  return 1
}

teardown() {
  local pid="$1"
  kill -INT "$pid" 2>/dev/null
  sleep 6
  pkill -9 -f "[s]tage_ros2" 2>/dev/null
  pkill -9 -f "[m]apf_planner_node" 2>/dev/null
  pkill -9 -f "[m]apf_lns2" 2>/dev/null
  pkill -9 -f "[c]ontroller_server" 2>/dev/null
  pkill -9 -f "[b]ehavior_server" 2>/dev/null
  pkill -9 -f "[p]bs_motion_controller" 2>/dev/null
  pkill -9 -f "[l]ns_motion_controller" 2>/dev/null
  pkill -9 -f "[z]one_map_server" 2>/dev/null
  pkill -9 -f "[d]ynamic_obstacle_manager" 2>/dev/null
  pkill -9 -f "[r]obot_state_publisher" 2>/dev/null
  pkill -9 -f "[l]ifecycle_manager" 2>/dev/null
  sleep 3
}

for planner in pbs lns; do
  if [ "$planner" = "pbs" ]; then
    LAUNCH_FILE=swarm_pbs_mapf.launch.py
  else
    LAUNCH_FILE=swarm_lns.launch.py
  fi

  for N in 1 4 8 12 16 20; do
    TAG="${planner}_n${N}"
    LLOG="$OUT/e4_${TAG}.launchlog"
    echo "=== [$(date -u +%FT%TZ)] START $TAG ===" >> "$SWEEP"

    ros2 launch iros_llm_swarm_bringup "$LAUNCH_FILE" \
      scenario:=amongus num_robots:=$N use_rviz:=false \
      > "$LLOG" 2>&1 &
    LPID=$!

    if ! wait_ready "$LLOG"; then
      echo "=== [$(date -u +%FT%TZ)] READY_TIMEOUT $TAG ===" >> "$SWEEP"
      teardown "$LPID"
      continue
    fi
    echo "=== [$(date -u +%FT%TZ)] READY $TAG ===" >> "$SWEEP"

    ros2 run iros_llm_swarm_mapf test_send_goals \
      --goal-x 2.7 --goal-y 10.1 --num $N --timeout 300 \
      --json-out "$OUT/e4_${TAG}_converge.json" \
      > "$OUT/e4_${TAG}_converge.log" 2>&1
    echo "=== [$(date -u +%FT%TZ)] DONE_CONVERGE $TAG ===" >> "$SWEEP"

    ros2 run iros_llm_swarm_mapf test_send_goals \
      --random --center-x 2.7 --center-y 10.1 --radius 4.0 --num $N --timeout 300 \
      --json-out "$OUT/e4_${TAG}_random.json" \
      > "$OUT/e4_${TAG}_random.log" 2>&1
    echo "=== [$(date -u +%FT%TZ)] DONE_RANDOM $TAG ===" >> "$SWEEP"

    ros2 run iros_llm_swarm_mapf test_send_goals \
      --json-file "$OUT/e4_dispersal_n${N}.json" --timeout 300 \
      --json-out "$OUT/e4_${TAG}_dispersal.json" \
      > "$OUT/e4_${TAG}_dispersal.log" 2>&1
    echo "=== [$(date -u +%FT%TZ)] DONE_DISPERSAL $TAG ===" >> "$SWEEP"

    teardown "$LPID"
    echo "=== [$(date -u +%FT%TZ)] DONE $TAG ===" >> "$SWEEP"
  done
done
echo "=== [$(date -u +%FT%TZ)] SWEEP COMPLETE ===" >> "$SWEEP"
