# TRACE-CSP

**Certified infeasibility diagnosis and a runtime-assurance shield for freeway ramp metering**

This repository contains the experimental code and the result data for the article

> L. Ouali, M. Bezoui, K. Amroun and A. Bounceur, *Certified Infeasibility Diagnosis and Graceful Degradation for Freeway Ramp Metering: A Constraint-Programming Runtime-Assurance Shield*, submitted to IEEE Transactions on Intelligent Transportation Systems.

It provides:

- an integer cell transmission model (CTM) with off-ramps, unmetered inflows and an optional capacity drop;
- a finite-scenario lexicographic CP-SAT controller for ramp metering and variable speed limits;
- an independent verifier that replays the model in exact arithmetic;
- interval-reachability certificates that prove the infeasibility of ramp-storage limits before any search;
- a graceful-degradation relaxation and a runtime-assurance shield around ALINEA;
- a common closed-loop harness for the NCHRP I-210 SUMO benchmark;
- the PeMS calibration pipeline;
- every result file behind the article's tables and figures.

## Repository layout

| Path | Content |
|---|---|
| `src/trace_csp/` | Python package: model (`domain`, `traffic.ctm`), CP-SAT encoding and solver (`csp.builder`, `csp.solver`), verifier (`csp.verifier`), controller and shield (`csp.controller`), certificates (`graph.physics`), learned predictors (`learning`), synthetic generators (`data`, `corridor`), I-210 layout (`sumo_i210`), PeMS fitting (`data_sources.fd`) |
| `scripts/` | Experiment drivers (see *Reproducing the experiments*) |
| `tests/` | Unit and consistency tests (CTM, solver and verifier agreement, certificate soundness) |
| `configs/` | Experiment, model, solver and data-source settings |
| `data/processed/` | Synthetic instances and CP-SAT labels used in the article |
| `data/interim/pems/` | PeMS station positions on the SUMO network (derived) |
| `results/` | All result files (see *Results*) |

## Installation

Tested with Python 3.11 on macOS (Apple M1 Pro, 10 cores, 16 GB).

```bash
python3.11 -m venv .venv
.venv/bin/python -m pip install -e '.[learning,sumo,dev]'
.venv/bin/python -m pytest -q
```

Pinned versions: OR-Tools 9.14, SUMO and TraCI 1.27.1 (pip wheels), PyTorch 2.9, PyTorch Geometric 2.8. Tests that need the I-210 benchmark are skipped until it is installed. Conda users can use `environment.yml`; `Dockerfile` builds an image that runs the test suite.

## External data

**NCHRP I-210 SUMO benchmark** (public; required for the SUMO experiments and the calibration):

```bash
git clone https://github.com/NCHRP-BSM-Traffic-Measures/CAT-Data-For-Freeway-Operations data/raw/i210
git -C data/raw/i210 checkout eb8bbf2781f4a5c437ea9283901b2df78e6fd135
```

The benchmark was developed for SUMO 1.15; all runs here use SUMO 1.27.1. All controllers run on the scenario of `Ramp Metering/run_fewIncidents_ALINEA_benchmark`. Only the HERO harness check uses the HERO-folder scenario, which is distributed as `Ramp Metering/run_fewIncidents_HERO_benchmark/SUMO (few incidents).zip` and must be extracted in place.

**Caltrans PeMS** (registered account required; raw files are not redistributed). From the PeMS Data Clearinghouse, download into `data/raw/pems/`, keeping the original file names:

- *Station 5-Minute*, District 7: the 81 days used in the article (January 1–22, April 1–30, May 1–30 and August 23–30, 2026);
- *Station Metadata*, District 7: `d07_text_meta_2023_12_22.txt`.

## Reproducing the experiments

Every step reads and writes the paths below, so the result files in this repository can be regenerated one by one. Steps marked *timing* report wall-clock times and should run on an otherwise idle machine.

**1. Synthetic instances and labels** (already provided in `data/processed/`)

```bash
make data labels
```

**2. Learned constraint predictors** (MLP, bipartite GNN and HGT, five seeds each) and the learning-free physics baseline

```bash
make train evaluate          # -> results/v3/
```

**3. Certificates, solver guidance, synthetic closed loop and scalability** (*timing*, about 1 h)

```bash
make synthetic               # -> results/final/
```

**4. PeMS calibration** (needs the PeMS files and the I-210 benchmark)

```bash
.venv/bin/python scripts/prepare_pems_i210.py   # extraction, per-station fits, station projection
.venv/bin/python scripts/calibrate_i210.py      # per-cell parameters and held-out validation
```

**5. SUMO I-210 closed loop** (*timing*; 7 simulated hours per run, about 20 min for ALINEA or HERO and 1–2 h for the shield). Seed 5 was used for all design decisions; seeds 6–10 were used only for evaluation.

```bash
S=6   # any seed
# Baselines (upstream ALINEA and HERO implementations)
.venv/bin/python scripts/run_i210_trace.py --mode alinea --seed $S --seconds 25200 --output results/i210-final/alinea-s$S.json
.venv/bin/python scripts/run_i210_trace.py --mode hero   --seed $S --seconds 25200 --output results/i210-final/hero-s$S.json
# Runtime-assurance shield around ALINEA (evaluated configuration)
.venv/bin/python scripts/run_i210_trace.py --mode shield --seed $S --seconds 25200 \
    --calibration results/pems-i210e-calibration.json --workers 2 --time-limit 6 \
    --no-congestion --storage-fraction 0.8 --approach-count halting \
    --output results/i210-final/shield-halt-f0.8-s$S.json
```

`scripts/run_i210_experiments.sh` lists every SUMO run reported in the article with its exact options: the upstream reproduction checks, the baselines for seeds 5–10, the standalone and shield variants on the development seed, and the frozen shield at 80% and 100% of storage on the evaluation seeds.

`--state oracle` reads vehicle counts from SUMO instead of estimating them from detectors.

## Results

| Experiment | Files | Produced by |
|---|---|---|
| Constraint prediction and the physics baseline | `results/v3/model-comparison-v3.json`, `results/v3/physics-comparison-v3.json`, `results/v3/*.pt` | `train_matrix.py`, `evaluate_matrix.py`, `evaluate_physics.py` |
| Certificates on 800 corridors | `results/final/certificates-10x6.jsonl` | `evaluate_corridors.py` |
| Missed corridors and the joint merge bound | `results/final/missed-corridors.json`, `results/final/joint-merge-coverage.json` | `analyse_missed_corridors.py`, `evaluate_joint_merge.py` |
| Validation on 400 fresh corridors | `results/final/certificates-fresh-10x6.jsonl` | `evaluate_corridors.py --start 500000` |
| Infeasibility proofs on small instances | `results/final/small-infeasible-timing.json` | `time_small_infeasible.py` |
| Learned solver guidance | `results/final/guidance-comparison-v3.json`, `results/final/solver-v3-*.json` | `compare_guidance_matrix.py` |
| Synthetic closed loop (12,960 episodes) | `results/final/closed-loop-v3/`, `results/final/closed-loop-v3-analysis.json` | `run_closed_loop.py`, `analyse_closed_loop.py` |
| Corridor closed loop and penalty-weight sweep | `results/final/closed-loop-corridor/` | `run_closed_loop.py` |
| Scalability (25–200 cells) | `results/final/scalability.json` | `scalability.py` |
| PeMS fundamental diagrams and validation | `results/pems-fd-i210-calibration.csv`, `results/pems-i210e-calibration.json` | `prepare_pems_i210.py`, `calibrate_i210.py` |
| I-210 evaluation (seeds 6–10) and threshold sensitivity | `results/i210-final/{alinea,hero,shield-halt-f0.8,shield-halt-f1.0}-s*.json` | `run_i210_trace.py` |
| I-210 development runs (seed 5) | `results/i210-dev/`, `results/i210-final/dev-v1/` | `run_i210_trace.py` |
| Upstream reproduction checks | `results/i210-alinea-seed5-full.json`, `results/i210-hero-seed5-full.json`, `results/i210-final/validation-hero-upstream-scenario-s5.json` | `run_i210_baseline.py`, `run_i210_trace.py` |

Each SUMO result file records the configuration, aggregate metrics (network and insertion-backlog vehicle-hours, completed trips, trip waiting time, ramp-storage excess), the decision log (mode, certified core, applied and reference rates, solve time) and, for the runs made after it was added to the harness, a 30-s time series.

## Notes on reproducibility

- The common harness reproduces the upstream ALINEA run and, on the upstream HERO scenario, the upstream HERO run exactly for seed 5 (identical completed trips and cumulative travel and waiting times).
- The upstream ALINEA and HERO folders ship scenarios with different driver parameters (`tau` 1.5 s versus 1.4 s and one mainline calibrator), so their published results are not directly comparable. Every controller here runs on the ALINEA-folder scenario.
- SUMO is deterministic for a given seed. Controller decisions depend on CP-SAT time limits and can therefore vary slightly with machine load.
- The results in `results/i210-final/dev-v1/` were produced by an earlier revision of the harness, which observed ramp queues only through the lane-area detectors. In the SUMO evaluation runs, the hold branch of the shield held the previously applied action; the choice between the held applied and the held reference action was added afterwards and was used in 0.6% of the control periods.
- OR-Tools 9.14 aborted one development run with an internal presolve assertion (`CpModelPresolver::DetectDuplicateColumns`). That configuration is not reported.

## License

The code and the result data are released under the MIT License (see `LICENSE`). The I-210 benchmark and the PeMS data are subject to their providers' terms and are not included.

## Citation

Please cite the article above; software citation metadata is provided in `CITATION.cff`.

## Contact

Lillia Ouali (corresponding author), Department of Computer Science, University of Bejaia, Algeria: lillia.ouali@univ-bejaia.dz
