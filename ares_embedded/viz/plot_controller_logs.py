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
        "t", "v_goal", "psi_goal", "v_true", "psi_true",
        "w_cmd_1", "w_cmd_2", "w_cmd_3", "w_cmd_4",
        "w_meas_1", "w_meas_2", "w_meas_3", "w_meas_4",
    ]
    missing = [c for c in required if c not in df.columns]
    if missing:
        raise SystemExit(f"CSV missing columns: {missing}")
    return df


def plot_body_and_yaw(df: pd.DataFrame) -> None:
    """Figure 1: Body velocity and yaw-rate (goal dashed, true solid)."""
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11, 5.2))
    t = df["t"]

    ax1.plot(t, df["v_goal"], "--", linewidth=1.2, label="v goal")
    ax1.plot(t, df["v_true"], "-", linewidth=1.6, label="v (true)")
    ax1.set_title("Body Velocity (True)")
    ax1.set_xlabel("t [s]")
    ax1.set_ylabel("v [m/s]")
    ax1.legend()
    ax1.grid(True)

    ax2.plot(t, df["psi_goal"], "--", linewidth=1.2, label=r"$\dot{\psi}$ goal")
    ax2.plot(t, df["psi_true"], "-", linewidth=1.6, label=r"$\dot{\psi}$ (true)")
    ax2.set_title("Yaw-rate (True)")
    ax2.set_xlabel("t [s]")
    ax2.set_ylabel(r"$\dot{\psi}$ [deg/s]")
    ax2.legend()
    ax2.grid(True)

    plt.tight_layout()


def plot_wheel_speeds(df: pd.DataFrame) -> None:
    """Figure 2: Wheel speeds (goal dashed, measured solid), 2x2 subplots."""
    fig, axes = plt.subplots(2, 2, figsize=(11, 5.2))
    axes = axes.flatten()
    t = df["t"]

    for i, ax in enumerate(axes):
        w_goal = df[f"w_cmd_{i+1}"]
        w_meas = df[f"w_meas_{i+1}"]
        ax.plot(t, w_goal, "--", linewidth=1.2, label="goal")
        ax.plot(t, w_meas, "-", linewidth=1.0, label="measured")
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
        nargs="?",
        default=None,
        help="Path to controller_sim_caseN.csv (default: logs/controller_sim_case1.csv)",
    )
    args = parser.parse_args()

    base = Path(__file__).resolve().parent.parent
    csv_path = Path(args.csv) if args.csv else base / "logs" / "controller_sim_case1.csv"
    if not csv_path.is_absolute() and not csv_path.exists():
        csv_path = base / csv_path
    if not csv_path.exists():
        print(f"Error: log file not found: {csv_path}", file=sys.stderr)
        return 1

    df = load_log(csv_path)

    plot_body_and_yaw(df)
    plot_wheel_speeds(df)
    plot_xy_path(df)
    plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
