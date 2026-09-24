#!/bin/sh
# Every SUMO I-210 run reported in the article, with the exact options used.
# Each run simulates 7 hours (25,200 s). Runs are independent and can be started in
# parallel; the shield runs use CP-SAT with a wall-clock budget, so keep the machine
# otherwise idle. Seed 5 is the development seed; seeds 6-10 are evaluation seeds.
set -e
PY=.venv/bin/python
RUN="$PY scripts/run_i210_trace.py --seconds 25200"
CAL="--calibration results/pems-i210e-calibration.json --time-limit 6"
F=results/i210-final; D=results/i210-dev
mkdir -p $F $D

# Upstream reproduction with the original run scripts (seed 5).
$PY scripts/run_i210_baseline.py --algorithm ALINEA --seconds 25200 --seed 5 --tripinfo --output results/i210-alinea-seed5-full.json
$PY scripts/run_i210_baseline.py --algorithm HERO   --seconds 25200 --seed 5 --tripinfo --output results/i210-hero-seed5-full.json
# Harness check: HERO on the HERO-folder scenario (extract the upstream zip first).
$RUN --mode hero --seed 5 --scenario-root "data/raw/i210/Ramp Metering/run_fewIncidents_HERO_benchmark/SUMO (few incidents)" \
     --output $F/validation-hero-upstream-scenario-s5.json

# Baselines, seeds 5-10.
for s in 5 6 7 8 9 10; do
  $RUN --mode alinea --seed $s --output $F/alinea-s$s.json
  $RUN --mode hero   --seed $s --output $F/hero-s$s.json
done

# Development seed 5: standalone controllers.
$RUN --mode degrade --seed 5 $CAL --workers 4 --output $D/degrade-v2-s5.json
$RUN --mode soft    --seed 5 $CAL --workers 4 --output $D/soft-v2-s5.json
$RUN --mode degrade --seed 5 $CAL --workers 2 --capacity-drop 100 --no-congestion --output $D/degrade-v3-s5.json
# Development seed 5: shield variants.
for f in 1.0 0.8; do
  $RUN --mode shield --seed 5 $CAL --workers 2 --no-congestion --storage-fraction $f --output $D/shield-f$f-s5.json
  $RUN --mode shield --seed 5 $CAL --workers 3 --no-congestion --storage-fraction $f --approach-count halting --output $D/shield-halt-f$f-s5.json
done
$RUN --mode shield --seed 5 $CAL --workers 4 --no-congestion --storage-fraction 0.8 --approach-count halting \
     --reference-weighting increasing --max-rate-change 2 --output $D/shield-halt-f0.8-inc-dm2-s5.json

# Evaluation seeds 6-10: frozen shield (80% storage) and the 100% threshold.
for s in 6 7 8 9 10; do
  for f in 0.8 1.0; do
    $RUN --mode shield --seed $s $CAL --workers 2 --no-congestion --storage-fraction $f --approach-count halting \
         --output $F/shield-halt-f$f-s$s.json
  done
done
