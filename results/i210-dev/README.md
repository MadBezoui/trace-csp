# Development runs on seed 5

The files in this folder are the controller designs tried on the development seed, in the order listed in `scripts/run_i210_experiments.sh` and in Table S1 of the article. They were produced with release 1.0.0 of this repository, whose harness verified a plan with 10-s rate changes and held its first rate for 30 s (see `results/i210-v1-final/README.md`). They document how the shield design was chosen.

`v2/` holds the development runs made with the corrected harness of release 1.1.0: the frozen shield (halting approach vehicles, 80% of storage) and the post-hoc variant with deviation weights increasing along the horizon and a command change of at most two green phases between periods.
