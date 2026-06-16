/* nlink_reach_example.cc
 * ============================================================================
 * N-link planar manipulator (N=2,3,4,5...) using gravity-dominated dynamics:
 *   q̈_i = (τ_i - G_i(q) - b_i*q̇_i) / I_i_eff
 *
 * where G_i(q) is the gravitational torque at joint i (depends on configuration),
 * and I_i_eff is the effective inertia including all downstream links.
 *
 * This is a valid simplification of full Euler-Lagrange dynamics that:
 *  - Is fast to compute (linear in N)
 *  - Captures the essential payload mass effect
 *  - Scales easily to any N (just extend the link arrays)
 *
 * Compile for different numbers of links:
 *   clang++ -std=c++17 -O2 -DN_LINKS=2 ... nlink_reach_example.cc -o nlink_2
 *   clang++ -std=c++17 -O2 -DN_LINKS=3 ... nlink_reach_example.cc -o nlink_3
 *   clang++ -std=c++17 -O2 -DN_LINKS=5 ... nlink_reach_example.cc -o nlink_5
 * ============================================================================
 */

#include <array>
#include <cmath>
#include <cstdio>
#include <vector>
#include <fstream>
#include <iostream>
#include <sys/time.h>
#include <sys/resource.h>

#include "scots.hh"
#include "TicToc.hh"
#include "RungeKutta4.hh"

struct rusage usage;

#ifndef N_LINKS
#define N_LINKS 2
#endif
#ifndef PAYLOAD_MASS_G
#define PAYLOAD_MASS_G 0
#endif
#ifndef DEBUG_HUGE_TARGET
#define DEBUG_HUGE_TARGET 0
#endif
#ifndef POS_PTS
#define POS_PTS 17
#endif
#ifndef VEL_PTS
#define VEL_PTS 17
#endif
#ifndef VEL_RANGE
#define VEL_RANGE 2.0
#endif
#ifndef INPUT_PTS
#define INPUT_PTS 5
#endif

constexpr size_t NL        = N_LINKS;
constexpr size_t STATE_DIM = 2 * NL;   /* [q1..qN, qd1..qdN] */
constexpr size_t INPUT_DIM = NL;       /* [tau1..tauN] */
constexpr double tau_s     = 0.1;      /* sampling time [s] -- kept short so velocity
                                         * changes per step stay within the grid bounds */
constexpr double g_acc     = 9.81;
constexpr double payload_mass = PAYLOAD_MASS_G / 1000.0;

/* Link parameters (enough for up to 7 links). Inertia/torque chosen so that
 * worst-case acceleration * tau_s stays within the velocity grid range
 * (+-5 rad/s, see s_ub/s_lb below) -- this is what was wrong before: with
 * tau_max=20 and I_eff=0.5 over tau_s=0.3s, every post-state flew off the
 * grid and the abstraction had zero valid transitions. */
constexpr double L_link[7] = {0.5, 0.4, 0.3, 0.25, 0.2, 0.15, 0.1};    /* length [m] */
constexpr double m_link[7] = {2.0, 1.5, 1.0, 0.8, 0.6, 0.4, 0.2};      /* mass [kg] */
constexpr double r_link[7] = {0.25, 0.2, 0.15, 0.125, 0.1, 0.075, 0.05}; /* COM distance [m] */
constexpr double I_eff[7]  = {1.0, 0.6, 0.4, 0.3, 0.15, 0.1, 0.06};     /* effective inertia [kg m^2] */
constexpr double b_damp[7] = {0.5, 0.4, 0.3, 0.2, 0.1, 0.08, 0.05};     /* damping [N m s/rad] */
constexpr double tau_max[7] = {15, 8, 6, 5, 3, 2, 1.5};                 /* torque bound [N m],
                                                                          * sized to exceed each
                                                                          * joint's worst-case
                                                                          * gravity torque */

using namespace std;
using namespace scots;
using state_type = array<double, STATE_DIM>;
using input_type = array<double, INPUT_DIM>;

/* Compute gravitational torque at joint i. */
inline void gravity_torques(const state_type& q, array<double, NL>& G) {
    for (size_t i = 0; i < NL; ++i) {
        double sum = 0.0;
        double cum_angle = 0.0;
        for (size_t j = i; j < NL; ++j) {
            cum_angle += q[j];
            double m = m_link[j];
            if (j == NL - 1) m += payload_mass;
            double r = r_link[j];
            sum += m * r * std::cos(cum_angle);
        }
        G[i] = g_acc * sum;
    }
}

auto system_post = [](state_type &x, const input_type &u) -> void {
    auto rhs = [](state_type &xdot, const state_type &x, const input_type &u) -> void {
        array<double, NL> G;
        gravity_torques(x, G);
        for (size_t i = 0; i < NL; ++i) {
            xdot[i] = x[NL + i];
            xdot[NL + i] = (u[i] - G[i] - b_damp[i] * x[NL + i]) / I_eff[i];
        }
    };
    runge_kutta_fixed4(rhs, x, u, STATE_DIM, tau_s, 10);
};

auto radius_post = [](state_type &r, const state_type &x, const input_type &u) {
    for (size_t i = 0; i < STATE_DIM; ++i) {
        r[i] = std::abs(r[i]);
    }
};

int main(int argc, char** argv) {
    TicToc tt;
    cout << "\nN-link planar manipulator (gravity-dominant EL dynamics)\n";
    cout << "N_LINKS = " << NL << ", STATE_DIM = " << STATE_DIM << ", INPUT_DIM = " << INPUT_DIM << "\n";
    cout << "Payload mass = " << payload_mass << " kg (" << PAYLOAD_MASS_G << " g)\n";

    /* ---- State space: [q1..qN (rad), qd1..qdN (rad/s)] ----
     * Grid must be fine enough that the target ball spans several cells --
     * with a coarse grid (eta/2 bigger than the target tolerance) the target
     * collapses to a single isolated point that can never be confirmed
     * "robustly winning" by the backward fixed point (see POS_PTS/VEL_PTS). */
    state_type s_lb, s_ub, s_eta;
    for (size_t i = 0; i < NL; ++i) {
        s_lb[i] = -M_PI / 2.0;        s_ub[i] = M_PI / 2.0;      s_eta[i] = M_PI / (POS_PTS - 1);
        s_lb[NL + i] = -VEL_RANGE;    s_ub[NL + i] = VEL_RANGE;  s_eta[NL + i] = 2.0 * VEL_RANGE / (VEL_PTS - 1);
    }

    UniformGrid ss(STATE_DIM, s_lb, s_ub, s_eta);
    cout << "\nState space grid:\n";
    ss.print_info();

    /* ---- Input space: joint torques [N m] ---- */
    input_type i_lb, i_ub, i_eta;
    for (size_t i = 0; i < NL; ++i) {
        i_lb[i] = -tau_max[i];
        i_ub[i] = tau_max[i];
        i_eta[i] = 2.0 * tau_max[i] / (INPUT_PTS - 1);
    }

    UniformGrid is(INPUT_DIM, i_lb, i_ub, i_eta);
    cout << "\nInput space grid:\n";
    is.print_info();

    /* No obstacles. */
    auto obstacle = [](const abs_type&) { return false; };

    cout << "\nComputing transition function:\n";
    TransitionFunction tf;
    Abstraction<state_type, input_type> abs(ss, is);

    tt.tic();
    abs.compute_gb(tf, system_post, radius_post, obstacle);
    tt.toc();

    if (!getrusage(RUSAGE_SELF, &usage)) {
        if (tf.get_no_transitions() > 0)
            cout << "Memory per transition: " << usage.ru_maxrss / (double)tf.get_no_transitions() << "\n";
    }
    cout << "Number of transitions: " << tf.get_no_transitions() << "\n";

    /* Target: stabilization ball around origin. Cell-extent-aware (like
     * reach_example.cc): a state is in target only if its WHOLE cell
     * (center +- eta/2) fits inside the ball, not just the center point.
     * The ball must span >=2-3 cells per dim so the growth-bound radius
     * (which always touches the neighboring-cell boundary) keeps posts of
     * near-zero control inside the target region instead of escaping it. */
    double target_q_radius = 1.5 * s_eta[0];
    double target_v_radius = 1.5 * s_eta[NL];
    auto target = [&ss, &s_eta, target_q_radius, target_v_radius](const abs_type& idx) {
        state_type x;
        ss.itox(idx, x);
        for (size_t i = 0; i < NL; ++i) {
            if (std::abs(x[i]) + s_eta[i] / 2.0 > target_q_radius) return false;
            if (std::abs(x[NL + i]) + s_eta[NL + i] / 2.0 > target_v_radius) return false;
        }
        return true;
    };

    cout << "\nSynthesis (reachability game):\n";
    tt.tic();
    WinningDomain win = solve_reachability_game(tf, target);
    tt.toc();
    cout << "Winning domain size: " << win.get_size() << " / " << ss.size() << "\n";

    /* Write output files. */
    char fname[256];
    snprintf(fname, sizeof(fname), "nlink_%d_m%d", (int)NL, (int)PAYLOAD_MASS_G);
    cout << "\nWrite controller to " << fname << ".scs\n";
    StaticController controller(ss, is, std::move(win));
    if (write_to_file(controller, fname))
        cout << "Done.\n";

    /* CSV export of the state->control lookup table (winning states only),
     * consumed by nlink_montecarlo.py for the mass-sweep success-rate
     * experiment. */
    char csvname[256];
    snprintf(csvname, sizeof(csvname), "%s.csv", fname);
    ofstream csvfile(csvname);
    for (size_t i = 0; i < NL; ++i) csvfile << "q" << i << ",";
    for (size_t i = 0; i < NL; ++i) csvfile << "qd" << i << ",";
    for (size_t i = 0; i < NL; ++i) csvfile << "tau" << i << (i + 1 == NL ? "\n" : ",");

    state_type x;
    vector<input_type> controls;
    size_t rows_written = 0;
    for (abs_type si = 0; si < ss.size(); ++si) {
        ss.itox(si, x);
        try {
            controls = controller.get_control<state_type, input_type>(x);
        } catch (const runtime_error &) {
            continue;
        }
        if (controls.empty()) continue;
        const input_type& uc = controls[0];
        for (size_t d = 0; d < STATE_DIM; ++d) csvfile << x[d] << ",";
        for (size_t d = 0; d < INPUT_DIM; ++d)
            csvfile << uc[d] << (d + 1 == INPUT_DIM ? "\n" : ",");
        ++rows_written;
    }
    csvfile.close();
    cout << "State-control lookup written to " << csvname << " (rows: " << rows_written << ")\n";

    return 0;
}
