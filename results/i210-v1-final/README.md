# I-210 runs of release 1.0.0 (superseded)

These files are the SUMO runs reported in the first version of the article, produced with release 1.0.0 of this repository (doi:10.5281/zenodo.22947010). They are kept for transparency and are not used by the current article.

In that harness the shield planned in 10-s steps and could change the metering rate at every step, while SUMO held the first rate for the whole 30-s meter update. A rate of zero vehicles could also be clipped to the meter's positive minimum, and when the shield kept ALINEA's rounded rate the meter executed ALINEA's own plan. The command checked by the verifier and the signal plan executed by the meter could therefore differ.

The current harness (`scripts/run_i210_trace.py`) holds every command over the 30-s period in the model and in SUMO, uses the meter's own commands (green phases per period) and executes exactly the verified plan. Its results are in `results/i210-final` and `results/i210-dev/v2`.

The ALINEA and HERO runs here are identical to those of the current harness on every metric both versions record, since those controllers do not use the shield.

`dev-v1/` holds the first standalone controller runs on the development seed, made with an earlier revision that observed ramp queues only through the lane-area detectors.
