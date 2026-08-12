#!/usr/bin/env bash
# Wait out the repair already running, then close the remaining cells.
#
# The guard matters more than the convenience. A `docker compose exec` that is
# killed on the host does NOT kill the process inside the container -- that is
# how two sweeps once wrote the same result file for 53 minutes. So this waits
# on the container-side pid, not on any host-side wrapper, and refuses to start
# a second harness while one is alive.
set -uo pipefail
cd "$(dirname "$0")/../../.." || exit 1
set +a; . ./.env

echo "$(date '+%H:%M:%S') жду завершения текущего прогона..."
while docker compose exec -T terminal bash -lc \
        'pgrep -f "benchmark_ch3.py --map amongus" >/dev/null'; do
  sleep 60
done
echo "$(date '+%H:%M:%S') свободно, запускаю добивку остальных ячеек"

exec paper/results/raw/close_blocked_cells.sh
