PYTHON ?= .venv/bin/python

.PHONY: test data labels train evaluate synthetic calibrate

test:
	$(PYTHON) -m pytest -q

# Synthetic instances (12,000 small instances and the 900-instance OOD challenge).
data:
	$(PYTHON) -m trace_csp.cli generate --count 12000 --horizon 2 --output data/processed/synthetic-12000-v3.jsonl.gz
	$(PYTHON) scripts/generate_ood_challenge.py --count-per-family 300 --output data/processed/synthetic-ood-challenge-900-v3.jsonl.gz

# CP-SAT labels (0.3 s for the main set, 1 s for the challenge).
labels:
	$(PYTHON) -m trace_csp.cli solve-offline --input data/processed/synthetic-12000-v3.jsonl.gz --output data/processed/labelled-12000-v3.jsonl.gz --seconds 0.3
	$(PYTHON) -m trace_csp.cli solve-offline --input data/processed/synthetic-ood-challenge-900-v3.jsonl.gz --output data/processed/labelled-ood-challenge-900-v3.jsonl.gz --seconds 1

# Learned constraint predictors (3 architectures x 5 seeds) and their evaluation.
train:
	for a in mlp bipartite hgt; do $(PYTHON) scripts/train_matrix.py --input data/processed/labelled-12000-v3.jsonl.gz --output-dir results/v3 --tag v3 --architectures $$a --seeds 5 --epochs 2; done

evaluate:
	$(PYTHON) scripts/evaluate_matrix.py --train data/processed/labelled-12000-v3.jsonl.gz --challenge data/processed/labelled-ood-challenge-900-v3.jsonl.gz --output-dir results/v3 --tag v3
	$(PYTHON) scripts/evaluate_physics.py --instances data/processed/synthetic-12000-v3.jsonl.gz --labels data/processed/labelled-12000-v3.jsonl.gz --challenge-instances data/processed/synthetic-ood-challenge-900-v3.jsonl.gz --challenge-labels data/processed/labelled-ood-challenge-900-v3.jsonl.gz --output-dir results/v3 --tag v3

# Certificates, guidance, synthetic closed loop and scalability (run on an idle machine).
synthetic:
	sh scripts/final_synthetic.sh
	$(PYTHON) scripts/analyse_missed_corridors.py
	$(PYTHON) scripts/evaluate_joint_merge.py
	$(PYTHON) scripts/evaluate_corridors.py --output results/final/certificates-fresh-10x6.jsonl --count 400 --start 500000
	$(PYTHON) scripts/scalability.py --output results/final/scalability.json

# PeMS calibration (requires the raw PeMS files and the I-210 benchmark, see README).
calibrate:
	$(PYTHON) scripts/calibrate_i210.py
