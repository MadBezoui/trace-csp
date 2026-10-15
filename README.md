# TRACE-CSP

**Certified infeasibility diagnosis and a runtime-assurance shield for freeway ramp metering**

This repository contains the experimental code and the result data for the article

> L. Ouali, M. Bezoui, K. Amroun and A. Bounceur, *Certified Infeasibility Diagnosis and Graceful Degradation for Freeway Ramp Metering: A Constraint-Programming Runtime-Assurance Shield*, submitted to Transportation Research Part C: Emerging Technologies.

It provides:

- an integer cell transmission model (CTM) with off-ramps, unmetered inflows, an optional capacity drop and device-specific meter commands held over an actuation period,
- a finite-scenario lexicographic CP-SAT controller for ramp metering and variable speed limits,
- an independent verifier that replays the model in exact arithmetic,
- interval-reachability certificates that prove the infeasibility of ramp-storage limits before any search,
- a graceful-degradation relaxation and a runtime-assurance shield around ALINEA,
- a common closed-loop harness for the NCHRP I-210 SUMO benchmark, with an independent replay of every logged decision,
- the PeMS calibration pipeline,
- every result file behind the article's tables and figures.

## Repository layout

| Path | Content |
|---|---|
| `src/trace_csp/` | Python package: model (`domain`, `traffic.ctm`), CP-SAT encoding and solver (`csp.builder`, `csp.solver`), verifier (`csp.verifier`), controller and shield (`csp.controller`), certificates (`graph.physics`), learned predictors (`learning`), synthetic generators (`data`, `corridor`), I-210 layout and meter model (`sumo_i210`), PeMS fitting (`data_sources.fd`), run provenance (`provenance`) |
| `scripts/` | Experiment drivers (see *Reproducing the experiments*) |
| `tests/` | Unit and consistency tests (CTM, solver and verifier agreement, certificate soundness, meter model, audit regressions) |
| `configs/` | Experiment, model, solver and data-source settings, including the list of PeMS dates |
| `data/processed/` | Synthetic instances and CP-SAT labels used in the article |
| `data/interim/pems/` | PeMS station positions on the SUMO network (derived) |
| `results/` | All result files (see *Results*) |

## Installation

Tested with Python 3.11 on macOS (Apple M1 Pro, 10 cores, 16 GB).

```bash
python3.11 -m venv .venv
.venv/bin/python -m pip install -r requirements-lock.txt -e '.[learning,sumo,calibration,dev]'
.venv/bin/python -m pytest -q
```

`requirements-lock.txt` lists the exact versions of the environment that produced the results (OR-Tools 9.14, SUMO, sumolib and TraCI 1.27.1, PyTorch 2.9.1, PyTorch Geometric 2.8). `pyproject.toml` declares compatible ranges. The `calibration` extra (pandas, pyarrow) is needed only for the PeMS pipeline. Tests that need the I-210 benchmark are skipped until it is installed. Conda users can use `environment.yml`. The `Dockerfile` builds an image that runs the test suite. It does not contain the benchmark, so it does not reproduce the SUMO experiments.

## External data

**NCHRP I-210 SUMO benchmark** (public, required for the SUMO experiments and the calibration):

```bash
git clone https://github.com/NCHRP-BSM-Traffic-Measures/CAT-Data-For-Freeway-Operations data/raw/i210
git -C data/raw/i210 checkout eb8bbf2781f4a5c437ea9283901b2df78e6fd135
```

The benchmark was developed for SUMO 1.15. All runs here use SUMO 1.27.1. All controllers run on the scenario of `Ramp Metering/run_fewIncidents_ALINEA_benchmark`. Only the HERO harness check uses the HERO-folder scenario, which is distributed as `Ramp Metering/run_fewIncidents_HERO_benchmark/SUMO (few incidents).zip` and must be extracted in place. Every SUMO result file records the SHA-256 of the benchmark files it read.

**Caltrans PeMS** (registered account required, raw files are not redistributed). From the PeMS Data Clearinghouse, download into `data/raw/pems/`, keeping the original file names:

- *Station 5-Minute*, District 7: the 81 dates listed in `configs/data/pems_i210_dates.txt`. They come from the ranges January 1 to 22, April 1 to 30, May 1 to 30 and August 23 to 30, 2026, from which nine dates are absent. The extraction stops if the files present differ from that list.
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

**5. SUMO I-210 closed loop** (*timing*, 7 simulated hours per run, about 20 min for ALINEA or HERO and 1 to 2 h for the shield). Seed 5 was used for all design decisions. Seeds 6 to 10 were used only for evaluation.

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
# Sensor-matched baseline: ALINEA with a queue override that sees the shield's queue and limit
.venv/bin/python scripts/run_i210_trace.py --mode override --seed $S --seconds 25200 \
    --storage-fraction 0.8 --output results/i210-final/override-f0.8-s$S.json
# Independent replay and verification of every logged decision (no SUMO needed)
.venv/bin/python scripts/replay_i210_decisions.py results/i210-final/shield-halt-f0.8-s$S.json
```

The replay rebuilds each logged decision from its recorded state, demand and exit shares and checks, with the exact verifier, that the executed command meets the queue limits over its 30-s period up to the exceedance the decision declared. It also checks the declared exceedance against the recorded solver stage, recomputes the interval certificate and compares the requested and executed commands. It exits with status 1 on any failure or missing field. It checks consistency with the recorded inputs, not the optimality of the full-horizon plans.

`scripts/run_i210_experiments.sh` lists every SUMO run reported in the article with its exact options: the upstream reproduction checks, the baselines for seeds 5 to 10, the development runs on seed 5 and the frozen shield at 80% and 100% of storage on the evaluation seeds. The reported shield runs were executed three at a time on the 10-core machine.

The meters of the benchmark repeat a 2-s green and a variable red phase within each 30-s update period, so they can only issue one to seven green phases per period. The harness uses these device commands directly. The model plans in 10-s steps, holds every command over the 30-s period and releases at most the vehicles allowed by the green phases that start in each step (`trace_csp.sumo_i210.meter_release_table`). The meter then executes exactly the verified signal plan, and ALINEA's own commands belong to the same set.

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
| Scalability (25 to 200 cells) | `results/final/scalability.json` | `scalability.py` |
| PeMS fundamental diagrams and validation | `results/pems-fd-i210-calibration.csv`, `results/pems-i210e-calibration.json` | `prepare_pems_i210.py`, `calibrate_i210.py` |
| I-210 layout and meter data used by the summaries | `results/i210-layout.json` | `trace_csp.sumo_i210.extract_layout` |
| I-210 evaluation (seeds 6 to 10) and threshold sensitivity | `results/i210-final/{alinea,hero,override-f0.8,shield-halt-f0.8,shield-halt-f1.0}-s*.json` | `run_i210_trace.py` |
| Same-state analysis of the storage margin | `results/i210-final/threshold-counterfactual.json` | `analyse_threshold_counterfactual.py` |
| Benchmark files read by the harness | `results/i210-benchmark-manifest.json` | `benchmark_manifest.py` |
| I-210 development runs (seed 5) | `results/i210-dev/v2/` (current harness), `results/i210-dev/` and `results/i210-v1-final/dev-v1/` (release 1.0.0 harness) | `run_i210_trace.py` |
| Upstream reproduction checks | `results/i210-alinea-seed5-full.json`, `results/i210-hero-seed5-full.json`, `results/i210-final/validation-hero-upstream-scenario-s5.json` | `run_i210_baseline.py`, `run_i210_trace.py` |
| Every number reported in the article | `results/paper-numbers.json` | manuscript build (see below) |

Each SUMO result file records the configuration, aggregate metrics (network and insertion-backlog vehicle-hours, completed trips, trip waiting time, ramp-storage excess on the lane-area detector edges and including halting vehicles on the ramp approach, approach occupancy, vehicles still waiting for insertion at the end), a 30-s time series and a provenance block (package version and source digest, package versions, SHA-256 of the calibration and of the main benchmark files, command line, machine load). Runs made with release 1.2.0 or later also record the digest of `scripts/`, the git revision and a manifest of every benchmark file the harness can read. For the earlier runs, `results/i210-benchmark-manifest.json` lists those files, and `benchmark_manifest.py --check` confirms that the hashes the runs recorded match it. Shield runs also log every decision: the observed state, the measured demand and exit shares, the requested, reference and executed commands, the red phase set at each meter, the vehicles released in the previous period, the cause of any degraded decision and the proof status, incumbent value and bound of every lexicographic stage.

The tables and figures of the article were generated from these files by a script that belongs to the manuscript sources, which are not distributed. Every quantity it produced is listed by name in `results/paper-numbers.json`, and the table above maps each one to its result file.

## Notes on reproducibility

- The common harness reproduces the upstream ALINEA run and, on the upstream HERO scenario, the upstream HERO run exactly for seed 5 (identical completed trips and cumulative travel and waiting times).
- The upstream ALINEA and HERO folders ship scenarios with different driver parameters (`tau` 1.5 s versus 1.4 s and one mainline calibrator), so their published results are not directly comparable. Every controller here runs on the ALINEA-folder scenario.
- SUMO is deterministic for a given seed. Shield decisions depend on CP-SAT time limits and can therefore vary slightly with machine load.
- Release 1.2.0 adds the sensor-matched override baseline, a stricter decision replay, the same-state analysis of the storage margin and fuller run provenance. The controller and the shield are unchanged from 1.1.0, so the 1.1.0 shield runs remain valid.
- Release 1.1.0 corrects the actuation interface of release 1.0.0. In 1.0.0 the shield verified a plan whose metering rate could change every 10 s while SUMO held its first rate for 30 s, a rate of zero could be clipped to the meter's positive minimum, and the verifier did not check queues at the measured initial state. The shield evaluation was rerun with the corrected code. The 1.0.0 runs are kept in `results/i210-v1-final/` (see its README), and the development runs on seed 5 in `results/i210-dev/` still come from 1.0.0. The synthetic results are unchanged. The only record the corrected verifier would write differently is the label of one decision of the ungated HERO-like corridor baseline, which drained an initial queue above storage, and no reported number depends on it.
- Episode files in `results/final/closed-loop-*` were written before the field `hard_violations` was computed and record 0 for every controller. The realized exceedance is in `queue_excess_vehicle_steps` and `max_queue_excess`.
- OR-Tools 9.14 aborted one development run with an internal presolve assertion (`CpModelPresolver::DetectDuplicateColumns`). That configuration is not reported. A deployment should run the solver in a separate process.

## License

The code and the result data are released under the MIT License (see `LICENSE`). The I-210 benchmark and the PeMS data are subject to their providers' terms and are not included.

## Citation

Please cite the article above. The code and data are archived on Zenodo. All versions are at [doi:10.5281/zenodo.22947007](https://doi.org/10.5281/zenodo.22947007), which resolves to the latest version, and version 1.0.0 is at [doi:10.5281/zenodo.22947010](https://doi.org/10.5281/zenodo.22947010). Software citation metadata is provided in `CITATION.cff`.

## Contact

Lillia Ouali (corresponding author), Department of Computer Science, University of Bejaia, Algeria: lillia.ouali@univ-bejaia.dz
