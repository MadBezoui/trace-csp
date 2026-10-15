#!/bin/sh
# Shield runs of the second revision. They are deadline-limited, so they start only once the
# baselines are done and the machine is on AC power (Low Power Mode applies on battery).
cd "$(dirname "$0")/../.."
export PYTHONPATH=src
until grep -q "^baselines" results/i210-final/run-v2.log; do sleep 30; done
until pmset -g batt | grep -q "AC Power"; do sleep 30; done
date -u +"shield start %FT%TZ"; pmset -g batt | head -1
xargs -P 3 -L 1 sh -c '.venv/bin/python -u scripts/run_i210_trace.py "$@" > /dev/null 2>&1 && echo "done $*" || echo "FAILED $*"' _ < results/i210-final/jobs-v2-shield-pending.txt
date -u +"shield end %FT%TZ"
