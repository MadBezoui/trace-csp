#!/bin/sh
# Every SUMO I-210 run reported in the article, with the exact options used.
# Each run simulates 7 hours (25,200 s). Runs are independent and can be started in
# parallel; the shield runs use CP-SAT with a wall-clock budget, so run at most three of
# them at a time on a 10-core machine and keep it otherwise idle (the reported runs did so,
# see results/i210-final/*.json "provenance"). Seed 5 is the development seed; seeds 6-10
# are evaluation seeds.
#
# The development runs on seed 5 listed under "Development history" were produced with
# release 1.0.0 of this repository, whose harness verified a 10-s plan and held its first
# rate for 30 s. They document how the design was chosen and are kept unchanged in
# results/i210-dev and results/i210-v1-final; the current harness holds and verifies
# commands over the whole 30-s period and does not reproduce them exactly.
set -e
PY=.venv/bin/python
RUN="$PY scripts/run_i210_trace.py --seconds 25200"
CAL="--calibration results/pems-i210e-calibration.json --time-limit 6"
F=results/i210-final; D=results/i210-dev; V1=results/i210-v1-final
mkdir -p $F $D

# Upstream reproduction with the original run scripts (seed 5).
$PY scripts/run_i210_baseline.py --algorithm ALINEA --seconds 25200 --seed 5 --tripinfo --output results/i210-alinea-seed5-full.json
$PY scripts/run_i210_baseline.py --algorithm HERO   --seconds 25200 --seed 5 --tripinfo --output results/i210-hero-seed5-full.json
# Harness check: HERO on the HERO-folder scenario (extract the upstream zip first).
$RUN --mode hero --seed 5 --scenario-root "data/raw/i210/Ramp Metering/run_fewIncidents_HERO_benchmark/SUMO (few incidents)" \
     --output $F/validation-hero-upstream-scenario-s5.json

# Baselines, seeds 5-10 (deterministic; identical to the 1.0.0 runs on the common metrics).
for s in 5 6 7 8 9 10; do
  $RUN --mode alinea --seed $s --output $F/alinea-s$s.json
  $RUN --mode hero   --seed $s --output $F/hero-s$s.json
done

# Development history, seed 5. These runs need release 1.0.0 (doi:10.5281/zenodo.22947010);
# with that checkout, remove the leading "# " of the commands below.
# standalone controllers.
# $RUN --mode degrade --seed 5 $CAL --workers 4 --output $D/degrade-v2-s5.json
# $RUN --mode soft    --seed 5 $CAL --workers 4 --output $D/soft-v2-s5.json
# $RUN --mode degrade --seed 5 $CAL --workers 2 --capacity-drop 100 --no-congestion --output $D/degrade-v3-s5.json
# shield variants.
# for f in 1.0 0.8; do
#   $RUN --mode shield --seed 5 $CAL --workers 2 --no-congestion --storage-fraction $f --output $D/shield-f$f-s5.json
#   $RUN --mode shield --seed 5 $CAL --workers 3 --no-congestion --storage-fraction $f --approach-count halting --output $D/shield-halt-f$f-s5.json
# done
# $RUN --mode shield --seed 5 $CAL --workers 4 --no-congestion --storage-fraction 0.8 --approach-count halting \
#      --reference-weighting increasing --max-rate-change 2 --output $D/shield-halt-f0.8-inc-dm2-s5.json

# Current harness. Development seed 5: frozen shield and the post-hoc rate-limited variant.
SH="$RUN --mode shield $CAL --workers 2 --no-congestion --approach-count halting"
mkdir -p $D/v2
$SH --seed 5 --storage-fraction 0.8 --output $D/v2/shield-halt-f0.8-s5.json
$SH --seed 5 --storage-fraction 0.8 --reference-weighting increasing --max-rate-change 2 --output $D/v2/shield-halt-f0.8-inc-dm2-s5.json

# Current harness. Evaluation seeds 6-10: frozen shield (80% storage) and the 100% threshold.
for s in 6 7 8 9 10; do
  for f in 0.8 1.0; do
    $SH --seed $s --storage-fraction $f --output $F/shield-halt-f$f-s$s.json
  done
done

# Sensor-matched baseline (release 1.2.0): ALINEA with a queue override that sees the
# shield's queue (lane-area plus halting approach vehicles) and the same 80% limit.
for s in 6 7 8 9 10; do
  $RUN --mode override --seed $s --storage-fraction 0.8 --output $F/override-f0.8-s$s.json
done

# Independent replay of every logged shield decision (no SUMO needed). The two held commands
# of the development run were logged before first_period_excess existed.
$PY scripts/replay_i210_decisions.py $F/shield-halt-f*-s*.json
$PY scripts/replay_i210_decisions.py --allow-undeclared-hold $D/v2/*.json

# Same-state analysis of the storage margin, and the benchmark manifest checked against the
# input hashes recorded by every run.
$PY scripts/analyse_threshold_counterfactual.py
$PY scripts/benchmark_manifest.py --check $F/*-s*.json $D/v2/*.json
