# SITL Setup

This project includes a host-side SITL-style flow that builds control tests, runs a controller scenario, writes logs, and plots results.

## Prerequisites

- Python 3
- CMake
- A C++ compiler available on your host machine

## One-time environment setup

From `ares_embedded`:

```sh
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

On Windows (PowerShell), activate with:

```powershell
.venv\Scripts\Activate.ps1
```

## Run SITL pipeline

From `ares_embedded`:

```sh
./run_sitl.sh
```

If execution permission is missing:

```sh
bash run_sitl.sh
```

The script will:

1. Configure and build the host target in `build/`
2. Run the controller test case and generate `logs/controller_sim_case1.csv`
3. Plot with `viz/plot_controller_logs.py`

## Plot an existing log manually

```sh
source .venv/bin/activate
python3 viz/plot_controller_logs.py logs/controller_sim_case1.csv
```
