#!/usr/bin/env python3
"""
nlink_montecarlo.py
==============================================================================
Monte Carlo mass-sweep experiment for N-link manipulator.
Loads the controller CSV from nlink_reach_example (synthesized at mass=0)
and replays it under increasing payload mass to measure success percentage.

USAGE
    python3 nlink_montecarlo.py nlink_2_m0.csv    # for 2-link
    python3 nlink_montecarlo.py nlink_3_m0.csv    # for 3-link
    etc.

OUTPUT
    10-row mass sweep (1 g -> 1000 g) with success percentage per row.
==============================================================================
"""

import sys
import csv
import numpy as np

# Detect N from the CSV filename (e.g., "nlink_2_m0.csv" -> N=2)
if len(sys.argv) < 2:
    print("usage: python3 nlink_montecarlo.py nlink_N_m0.csv [POS_PTS] [VEL_PTS] [VEL_RANGE]")
    sys.exit(1)

csv_path = sys.argv[1]
# Parse N_LINKS from the filename
import re
match = re.search(r'nlink_(\d+)_m', csv_path)
if not match:
    print("Error: filename must match pattern 'nlink_N_m*.csv'")
    sys.exit(1)
N_LINKS = int(match.group(1))

# Optional overrides: python3 nlink_montecarlo.py csv POS_PTS VEL_PTS VEL_RANGE
# must match the -DPOS_PTS/-DVEL_PTS/-DVEL_RANGE used to compile that CSV's
# controller, since the target ball is derived from the grid spacing.
_POS_PTS_ARG = int(sys.argv[2]) if len(sys.argv) > 2 else 17
_VEL_PTS_ARG = int(sys.argv[3]) if len(sys.argv) > 3 else 17
_VEL_RANGE_ARG = float(sys.argv[4]) if len(sys.argv) > 4 else 2.0

TAU_S = 0.1
N_TRIALS = 1
N_MASS_STEPS = 10
MASS_MIN_G, MASS_MAX_G = 1.0, 1000.0
SIM_HORIZON_STEPS = 40

# Link parameters (must match nlink_reach_example.cc exactly)
L_LINK = np.array([0.5, 0.4, 0.3, 0.25, 0.2, 0.15, 0.1])[:N_LINKS]
M_LINK = np.array([2.0, 1.5, 1.0, 0.8, 0.6, 0.4, 0.2])[:N_LINKS]
R_LINK = np.array([0.25, 0.2, 0.15, 0.125, 0.1, 0.075, 0.05])[:N_LINKS]
I_EFF  = np.array([1.0, 0.6, 0.4, 0.3, 0.15, 0.1, 0.06])[:N_LINKS]
B_DAMP = np.array([0.5, 0.4, 0.3, 0.2, 0.1, 0.08, 0.05])[:N_LINKS]
G_ACC = 9.81

# Grid resolution (must match nlink_reach_example.cc POS_PTS/VEL_PTS/VEL_RANGE
# defaults) -- used to reconstruct the same cell-extent-aware target ball
# the controller was actually synthesized for.
POS_PTS = _POS_PTS_ARG
VEL_PTS = _VEL_PTS_ARG
VEL_RANGE = _VEL_RANGE_ARG
ETA_Q = np.pi / (POS_PTS - 1)
ETA_V = 2.0 * VEL_RANGE / (VEL_PTS - 1)

# Target derived from grid (matches SCOTS synthesis)
TARGET_Q_RADIUS = 1.5 * ETA_Q
TARGET_V_RADIUS = 1.5 * ETA_V


def gravity_torques(q, payload_mass_kg):
    """Compute G_i(q) for each joint i."""
    G = np.zeros(N_LINKS)
    for i in range(N_LINKS):
        s = 0.0
        cum_angle = 0.0
        for j in range(i, N_LINKS):
            cum_angle += q[j]
            m = M_LINK[j]
            if j == N_LINKS - 1:
                m += payload_mass_kg
            r = R_LINK[j]
            s += m * r * np.cos(cum_angle)
        G[i] = G_ACC * s
    return G


def ode_rhs(x, u, payload_mass_kg):
    """Return [q̇, q̈]."""
    q, qd = x[:N_LINKS], x[N_LINKS:]
    G = gravity_torques(q, payload_mass_kg)
    qdd = (u - G - B_DAMP * qd) / I_EFF
    return np.concatenate([qd, qdd])


def rk4_step(x, u, dt, payload_mass_kg, substeps=10):
    h = dt / substeps
    for _ in range(substeps):
        k1 = ode_rhs(x, u, payload_mass_kg)
        k2 = ode_rhs(x + 0.5 * h * k1, u, payload_mass_kg)
        k3 = ode_rhs(x + 0.5 * h * k2, u, payload_mass_kg)
        k4 = ode_rhs(x + h * k3, u, payload_mass_kg)
        x = x + (h / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
    return x


def load_controller(csv_path):
    states, controls = [], []
    with open(csv_path) as f:
        reader = csv.reader(f)
        header = next(reader)
        for row in reader:
            vals = [float(v) for v in row]
            states.append(vals[:2*N_LINKS])
            controls.append(vals[2*N_LINKS:])
    return np.array(states), np.array(controls)


def lookup_control(states, controls, x):
    d = np.sum((states - x) ** 2, axis=1)
    idx = np.argmin(d)
    return controls[idx]


def in_target(x):
    q = x[:N_LINKS]
    qd = x[N_LINKS:]
    return bool(np.all(np.abs(q) < TARGET_Q_RADIUS) and np.all(np.abs(qd) < TARGET_V_RADIUS))


def run_trial(states, controls, payload_mass_kg, rng):
    # Start from a state the nominal-mass controller actually has a winning
    # control for (the synthesized winning domain is a basin around the
    # target, not the whole state space) -- this is what makes the nominal
    # (mass=0) success rate ~100% and the mass-sweep show genuine
    # degradation rather than failures caused by starting outside the basin.
    idx = rng.integers(len(states))
    x = states[idx].copy()
    for _ in range(SIM_HORIZON_STEPS):
        if in_target(x):
            return True
        u = lookup_control(states, controls, x)
        x = rk4_step(x, u, TAU_S, payload_mass_kg)
        x[:N_LINKS] = np.clip(x[:N_LINKS], -np.pi/2, np.pi/2)
        x[N_LINKS:] = np.clip(x[N_LINKS:], -0.5, 0.5)
    return in_target(x)


def main():
    states, controls = load_controller(csv_path)
    print(f"Loaded controller: {len(states)} winning states, N={N_LINKS}\n")

    rng = np.random.default_rng(0)
    masses_g = np.linspace(MASS_MIN_G, MASS_MAX_G, N_MASS_STEPS)

    print(f"{'mass [g]':>10} | {'successes':>10} | {'success %':>10}")
    print("-" * 38)
    for m_g in masses_g:
        m_kg = m_g / 1000.0
        successes = sum(run_trial(states, controls, m_kg, rng) for _ in range(N_TRIALS))
        pct = 100.0 * successes / N_TRIALS
        print(f"{m_g:10.1f} | {successes:10d} | {pct:9.1f}%")

    print(f"\nNominal (mass=0g) sanity check:")
    successes0 = int(run_trial(states, controls, 0.0, rng))
    print(f"  {successes0}/1 success ({100.0*successes0:.1f}%)")


if __name__ == "__main__":
    main()
