# Test image: installs the package and runs the unit and consistency tests.
# Reproducing the SUMO experiments also needs the I-210 benchmark (see README).
FROM python:3.11-slim
WORKDIR /app
COPY pyproject.toml README.md ./
COPY src ./src
COPY requirements-lock.txt ./
RUN pip install --no-cache-dir --extra-index-url https://download.pytorch.org/whl/cpu -r requirements-lock.txt '.[learning,sumo,calibration,dev]'
COPY tests ./tests
COPY scripts ./scripts
COPY configs ./configs
COPY results/pems-fd-i210-calibration.csv results/pems-i210e-calibration.json ./results/
COPY data/interim ./data/interim
CMD ["python", "-m", "pytest", "-q"]
