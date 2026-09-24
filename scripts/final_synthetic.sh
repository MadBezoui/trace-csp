#!/bin/sh
# Synthetic experiments: certificates on 800 corridors, infeasibility proofs on small
# instances, learned-guidance comparison, the 12,960-episode closed-loop matrix and the
# corridor closed loop with a softened-MPC weight sweep. Several endpoints are wall-clock
# times: run on an otherwise idle machine. Outputs go to results/final/.
set -e
PY=.venv/bin/python
OUT=results/final
mkdir -p $OUT/closed-loop-v3 $OUT/closed-loop-corridor

# 1. Certificates on 800 multi-ramp corridors.
$PY scripts/evaluate_corridors.py --output $OUT/certificates-10x6.jsonl --count 800 --workers 8

# 2. CP-SAT time to prove infeasibility on the small held-out instances (new encoding).
$PY scripts/time_small_infeasible.py --output $OUT/small-infeasible-timing.json

# 3. Guidance comparison (paired, end-to-end deadline), 5 HGT seeds.
$PY scripts/compare_guidance_matrix.py --input data/processed/synthetic-12000-v3.jsonl.gz --output-dir $OUT --tag v3 \
    --architecture hgt --seeds 5 --limit 200 --splits test_id test_ood_structure --checkpoint-dir results/v3

# 4. Small-instance closed-loop matrix (12,960 episodes), one process per family.
for f in normal mild_incident severe_incident sensor_fault demand_shift communication_fault compound structural_ood; do
  $PY scripts/run_closed_loop.py --output $OUT/closed-loop-v3/episodes-$f.jsonl.gz --seeds 30 --periods 12 \
    --controllers fixed alinea hero_proxy cp_sat warm_cp_sat learned_cp_sat --families $f \
    --demand-levels 0.7 1.0 1.4 --incident-factors 1000 800 500 --checkpoint results/v3/hgt-v3-seed0.pt &
done
wait
$PY scripts/analyse_closed_loop.py --pattern "$OUT/closed-loop-v3/episodes-*.jsonl.gz" --output $OUT/closed-loop-v3-analysis.json

# 5. Corridor closed loop, including a soft-MPC penalty-weight sweep.
for f in normal incident surge compound; do
  for c in cp_sat degraded_cp_sat; do
    $PY scripts/run_closed_loop.py --output $OUT/closed-loop-corridor/$f-$c.jsonl.gz --seeds 20 --periods 12 --controllers $c \
        --families $f --corridor-cells 8 --corridor-horizon 4 --time-limit 5 &
  done
  wait
  for w in 1 10 100; do
    $PY scripts/run_closed_loop.py --output $OUT/closed-loop-corridor/$f-soft_mpc-w$w.jsonl.gz --seeds 20 --periods 12 --controllers soft_mpc \
        --families $f --corridor-cells 8 --corridor-horizon 4 --time-limit 5 --soft-weight $w &
  done
  wait
done
$PY scripts/run_closed_loop.py --output $OUT/closed-loop-corridor/baselines.jsonl.gz --seeds 20 --periods 12 \
    --controllers fixed alinea hero_proxy --families normal incident surge compound --corridor-cells 8 --corridor-horizon 4 \
    --time-limit 5 --ungated-baselines
echo done > $OUT/SYNTHETIC_DONE
