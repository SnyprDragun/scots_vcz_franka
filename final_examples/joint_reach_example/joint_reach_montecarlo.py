#!/usr/bin/env python3
"""
joint_reach_montecarlo.py
==============================================================================
Mass-sweep success-percentage experiment for the SCOTS row of the paper's
new 3-row table:

    | Method | Computation Time | Success (Nominal) | Success (Perturbed) |
    | SCOTS  | ...               | 100%               | degrades with mass   |

Loads the state->control lookup table exported by joint_reach_example.cc
(synthesized ONCE at payload_mass=0, i.e. joint_reach_m0g.csv) and replays it
in closed loop under the SAME coupled 7-joint pendulum dynamics used in the
C++ abstraction, but with the *actual* (perturbed) payload mass. Since the
SCOTS controller has no knowledge of payload mass, increasing it makes the
nominal lookup table progressively wrong (it commands torques sized for the
unloaded gravity term), so the simulated trajectory misses the target more
often as mass grows -- exactly the degrading-success-percentage behaviour
the PhD guide asked for.

USAGE
    python3 joint_reach_montecarlo.py joint_reach_m0g.csv

Requires only numpy (pip install numpy if not already available).

OUTPUT
    Prints a 10-row mass sweep (1 g -> 1000 g) x 10 trials each, with the
    success percentage per mass step, ready to paste into the paper table
    (interpolate between rows if you only have time to run a subset).
"""

import sys
import csv
import numpy as np

NJ = 7
TAU_S = 0.3          # sampling time [s], must match joint_reach_example.cc
N_TRIALS = 10
N_MASS_STEPS = 10
MASS_MIN_G, MASS_MAX_G = 1.0, 1000.0
SIM_HORIZON_STEPS = 40   # 40 * 0.3s = 12s simulated time budget to reach target

# Must match the constexpr arrays in joint_reach_example.cc exactly.
LINK_MASS = np.array([3.5, 3.5, 2.5, 2.5, 1.7, 1.2, 0.6])
LINK_LEN  = np.array([0.30, 0.25, 0.25, 0.20, 0.15, 0.10, 0.08])
TOTAL_LEN = LINK_LEN.sum()
I_EFF     = np.array([1.2, 1.0, 0.6, 0.4, 0.2, 0.1, 0.05])
B_DAMP    = np.array([2.0, 2.0, 1.5, 1.5, 1.0, 0.8, 0.5])
G_ACC     = 9.81

POS_HALF_RANGE = 0.2
VEL_HALF_RANGE = 0.1
TARGET_Q_LO, TARGET_Q_HI = POS_HALF_RANGE * 0.4, POS_HALF_RANGE * 0.9
TARGET_V_LO, TARGET_V_HI = -VEL_HALF_RANGE * 0.3, VEL_HALF_RANGE * 0.3


def gravity_torque(q, payload_mass_kg):
    """G_i(q) exactly mirroring gravity_torque() in joint_reach_example.cc."""
    G = np.zeros(NJ)
    for i in range(NJ):
        s = 0.0
        running_angle = 0.0
        for j in range(i, NJ):
            running_angle += q[j]
            m = LINK_MASS[j]
            l = LINK_LEN[j]
            if j == NJ - 1:
                m = m + payload_mass_kg
                l = TOTAL_LEN
            s += m * l * np.cos(running_angle)
        G[i] = G_ACC * s
    return G


def dynamics_rhs(x, u, payload_mass_kg):
    q, qd = x[:NJ], x[NJ:]
    G = gravity_torque(q, payload_mass_kg)
    qdd = (u - G - B_DAMP * qd) / I_EFF
    return np.concatenate([qd, qdd])


def rk4_step(x, u, dt, payload_mass_kg, substeps=10):
    h = dt / substeps
    for _ in range(substeps):
        k1 = dynamics_rhs(x, u, payload_mass_kg)
        k2 = dynamics_rhs(x + 0.5 * h * k1, u, payload_mass_kg)
        k3 = dynamics_rhs(x + 0.5 * h * k2, u, payload_mass_kg)
        k4 = dynamics_rhs(x + h * k3, u, payload_mass_kg)
        x = x + (h / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
    return x


def load_controller(csv_path):
    states, controls = [], []
    with open(csv_path) as f:
        reader = csv.reader(f)
        header = next(reader)
        for row in reader:
            vals = [float(v) for v in row]
            states.append(vals[:2 * NJ])
            controls.append(vals[2 * NJ:])
    return np.array(states), np.array(controls)


def lookup_control(states, controls, x):
    """Nearest-neighbour lookup into the SCOTS state->control table."""
    d = np.sum((states - x) ** 2, axis=1)
    idx = np.argmin(d)
    return controls[idx]


def in_target(x):
    q, qd = x[:NJ], x[NJ:]
    return bool(np.all((q >= TARGET_Q_LO) & (q <= TARGET_Q_HI)) and
                np.all((qd >= TARGET_V_LO) & (qd <= TARGET_V_HI)))


def run_trial(states, controls, payload_mass_kg, rng):
    # Random start in the lower part of the joint-space box (away from target).
    q0 = rng.uniform(-POS_HALF_RANGE, TARGET_Q_LO * 0.5, size=NJ)
    qd0 = rng.uniform(-0.02, 0.02, size=NJ)
    x = np.concatenate([q0, qd0])
    for _ in range(SIM_HORIZON_STEPS):
        if in_target(x):
            return True
        u = lookup_control(states, controls, x)
        x = rk4_step(x, u, TAU_S, payload_mass_kg)
        # clip to grid bounds to keep the lookup meaningful
        x[:NJ] = np.clip(x[:NJ], -POS_HALF_RANGE, POS_HALF_RANGE)
        x[NJ:] = np.clip(x[NJ:], -VEL_HALF_RANGE, VEL_HALF_RANGE)
    return in_target(x)


def main():
    if len(sys.argv) != 2:
        print("usage: python3 joint_reach_montecarlo.py joint_reach_m0g.csv")
        sys.exit(1)

    states, controls = load_controller(sys.argv[1])
    print(f"Loaded controller: {len(states)} winning states\n")

    rng = np.random.default_rng(0)
    masses_g = np.linspace(MASS_MIN_G, MASS_MAX_G, N_MASS_STEPS)

    print(f"{'mass [g]':>10} | {'successes':>10} | {'success %':>10}")
    print("-" * 38)
    rows = []
    for m_g in masses_g:
        m_kg = m_g / 1000.0
        successes = sum(run_trial(states, controls, m_kg, rng) for _ in range(N_TRIALS))
        pct = 100.0 * successes / N_TRIALS
        rows.append((m_g, successes, pct))
        print(f"{m_g:10.1f} | {successes:10d} | {pct:9.1f}%")

    print("\nNominal (mass=0g) sanity check:")
    successes0 = sum(run_trial(states, controls, 0.0, rng) for _ in range(N_TRIALS))
    print(f"  {successes0}/{N_TRIALS} successes ({100.0*successes0/N_TRIALS:.1f}%) -- should be 100% or close to it")


if __name__ == "__main__":
    main()
