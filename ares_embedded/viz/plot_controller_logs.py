#!/usr/bin/env python3
"""
Plot controller SITL logs to match models/actuation-controller/actuation_controller_sim.m:
  1) Body velocity (goal vs true) and yaw-rate (goal vs true)
  2) Wheel speeds: goal vs measured (4 subplots)
  3) XY path from goal profiles (perfect tracking, time-colored)
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def load_log(csv_path: Path) -> pd.DataFrame:
    df = pd.read_csv(csv_path)
    required = [
        "t",
        "v_goal",
        "psi_goal",
        "v_true",
        "psi_true",
        "w_cmd_1",
        "w_cmd_2",
        "w_cmd_3",
        "w_cmd_4",
        "w_meas_1",
        "w_meas_2",
        "w_meas_3",
        "w_meas_4",
    ]
    missing = [c for c in required if c not in df.columns]
    if missing:
        raise SystemExit(f"CSV missing columns: {missing}")
    return df


def _ensure_same_timebase(runs: list[tuple[str, pd.DataFrame]]) -> None:
    """Warn if time vectors differ noticeably between runs."""
    if not runs:
        return
    _, df0 = runs[0]
    t0 = df0["t"].to_numpy()
    for label, df in runs[1:]:
        t = df["t"].to_numpy()
        if t.shape != t0.shape or not np.allclose(t, t0, atol=1e-6, rtol=1e-5):
            print(
                f"Warning: time vector for run '{label}' "
                "differs from the first run; plots may be slightly misaligned.",
                file=sys.stderr,
            )
            break


def plot_body_and_yaw(runs: list[tuple[str, pd.DataFrame]]) -> None:
    """Figure 1: Body velocity and yaw-rate for one or more runs.

    - Plots v/psi goals from the first run (assumed identical across runs).
    - Overlays v_true/psi_true from all runs with labels derived from filenames.
    """
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 5.2))

    _ensure_same_timebase(runs)

    label0, df0 = runs[0]
    t0 = df0["t"]

    # Goals (assumed common)
    ax1.plot(t0, df0["v_goal"], "--", linewidth=1.2, color="k", label="v goal")
    ax2.plot(
        t0,
        df0["psi_goal"],
        "--",
        linewidth=1.2,
        color="k",
        label=r"$\dot{\psi}$ goal",
    )

    # True signals for each run
    colors = plt.cm.tab10(np.linspace(0.0, 1.0, len(runs)))
    for (label, df), color in zip(runs, colors):
        t = df["t"]
        ax1.plot(
            t,
            df["v_true"],
            "-",
            linewidth=1.6,
            label=f"v (true) [{label}]",
            color=color,
        )
        ax2.plot(
            t,
            df["psi_true"],
            "-",
            linewidth=1.6,
            label=fr"$\dot{{\psi}}$ (true) [{label}]",
            color=color,
        )

    ax1.set_title("Body Velocity (True)")
    ax1.set_xlabel("t [s]")
    ax1.set_ylabel("v [m/s]")
    ax1.legend()
    ax1.grid(True)

    ax2.set_title("Yaw-rate (True)")
    ax2.set_xlabel("t [s]")
    ax2.set_ylabel(r"$\dot{\psi}$ [deg/s]")
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()


def plot_wheel_speeds(runs: list[tuple[str, pd.DataFrame]]) -> None:
    """Figure 2: Wheel speeds (goal dashed, measured solid), 2x2 subplots.

    Goals are taken from the first run; measured traces from all runs
    are overlaid with labels derived from filenames.
    """
    fig, axes = plt.subplots(2, 2, figsize=(11, 5.2))
    axes = axes.flatten()

    _ensure_same_timebase(runs)

    label0, df0 = runs[0]
    t0 = df0["t"]
    colors = plt.cm.tab10(np.linspace(0.0, 1.0, len(runs)))

    for i, ax in enumerate(axes):
        # Goal from first run
        w_goal = df0[f"w_cmd_{i+1}"]
        ax.plot(
            t0,
            w_goal,
            "--",
            linewidth=1.2,
            color="k",
            label="goal",
        )

        # Measured from all runs
        for (label, df), color in zip(runs, colors):
            t = df["t"]
            w_meas = df[f"w_meas_{i+1}"]
            ax.plot(
                t,
                w_meas,
                "-",
                linewidth=1.0,
                label=f"measured [{label}]",
                color=color,
            )

        ax.set_ylabel(f"w{i+1} [deg/s]")
        ax.set_xlabel("t [s]")
        ax.legend()
        ax.grid(True)

    plt.tight_layout()


def plot_xy_path(df: pd.DataFrame) -> None:
    """Figure 3: XY path from goal profiles (perfect tracking), time as color."""
    t = df["t"].values
    v = df["v_goal"].values
    psi_dot_deg = df["psi_goal"].values
    psi_dot_rad = np.deg2rad(psi_dot_deg)

    psi = np.zeros_like(t)
    psi[1:] = np.cumsum(0.5 * (psi_dot_rad[:-1] + psi_dot_rad[1:]) * np.diff(t))
    v_cos = v * np.cos(psi)
    v_sin = v * np.sin(psi)
    x = np.zeros_like(t)
    y = np.zeros_like(t)
    x[1:] = np.cumsum(0.5 * (v_cos[:-1] + v_cos[1:]) * np.diff(t))
    y[1:] = np.cumsum(0.5 * (v_sin[:-1] + v_sin[1:]) * np.diff(t))

    fig, ax = plt.subplots(figsize=(5.6, 5))
    sc = ax.scatter(x, y, s=35, c=t, cmap="turbo", marker=".")
    cb = plt.colorbar(sc, ax=ax)
    cb.set_label("Time [s]", fontsize=11)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_title("XY Path (Assuming Perfect Tracking)")
    ax.axis("equal")
    ax.grid(True)
    plt.tight_layout()


def main() -> int:
    parser = argparse.ArgumentParser(description="Plot controller SITL CSV logs (MATLAB sim style)")
    parser.add_argument(
        "csv",
        nargs="*",
        default=None,
        help=(
            "Zero or more CSV log paths. "
            "If omitted, all logs/controller_sim_case*.csv files are plotted."
        ),
    )
    args = parser.parse_args()

    base = Path(__file__).resolve().parent.parent
    if args.csv:
        raw_paths = [Path(p) for p in args.csv]
    else:
        raw_paths = sorted((base / "logs").glob("controller_sim_case*.csv"))

    if not raw_paths:
        print(
            "Error: no CSV logs found. "
            "Specify one or more paths, or generate logs in logs/controller_sim_case*.csv.",
            file=sys.stderr,
        )
        return 1

    runs: list[tuple[str, pd.DataFrame]] = []
    seen: set[Path] = set()

    # Expand each requested path to include a matching *_fuzzy partner if present.
    expanded_paths: list[Path] = []
    for p in raw_paths:
        csv_path = p if p.is_absolute() else base / p
        expanded_paths.append(csv_path)
        stem = csv_path.stem
        if not stem.endswith("_fuzzy"):
            fuzzy_path = csv_path.with_name(stem + "_fuzzy" + csv_path.suffix)
            expanded_paths.append(fuzzy_path)

    # Deduplicate while preserving order.
    dedup_paths: list[Path] = []
    for p in expanded_paths:
        if p not in seen:
            seen.add(p)
            dedup_paths.append(p)

    for csv_path in dedup_paths:
        if not csv_path.exists():
            print(f"Warning: log file not found, skipping: {csv_path}", file=sys.stderr)
            continue
        df = load_log(csv_path)
        stem = csv_path.stem
        if stem.endswith("_fuzzy"):
            label = f"{stem[:-6]} (fuzzy)"
        else:
            label = f"{stem} (baseline)"
        runs.append((label, df))

    if not runs:
        print("Error: no valid CSV logs to plot.", file=sys.stderr)
        return 1

    # Use the first run for the XY path (goal-only path is identical across runs).
    plot_body_and_yaw(runs)
    plot_wheel_speeds(runs)
    plot_xy_path(runs[0][1])
    plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
