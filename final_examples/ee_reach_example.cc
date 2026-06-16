/* ee_reach_example.cc
 * ============================================================================
 * 3D End-Effector space reachability (3D state: position only).
 * Comparison baseline for joint_reach_example (14D joint space).
 *
 * STATE: [x, y, z (m)]
 * INPUT: [vx, vy, vz (m/s)] -- velocity commands (single integrator)
 * DYNAMICS: ẋ = vx, ẏ = vy, ż = vz
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

#ifndef POS_HALF_RANGE
#define POS_HALF_RANGE 0.5
#endif
#ifndef POS_PTS
#define POS_PTS 3
#endif
#ifndef VEL_PTS
#define VEL_PTS 3
#endif

constexpr size_t STATE_DIM = 3;         /* [x, y, z] position only */
constexpr size_t INPUT_DIM = 3;         /* [vx, vy, vz] velocity commands */
constexpr double tau_s     = 0.3;       /* sampling time [s] */
constexpr double v_max     = 1.0;       /* max velocity per axis [m/s] */

using namespace std;
using namespace scots;
using state_type = array<double, STATE_DIM>;
using input_type = array<double, INPUT_DIM>;

/* Single integrator: ẋ = u (velocity commands) */
auto system_post = [](state_type &x, const input_type &u) -> void {
    auto rhs = [](state_type &xdot, const state_type &x, const input_type &u) -> void {
        xdot[0] = u[0];
        xdot[1] = u[1];
        xdot[2] = u[2];
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
    cout << "3D EE-space SCOTS reach example" << endl;
    cout << "STATE_DIM = " << STATE_DIM << ", INPUT_DIM = " << INPUT_DIM << endl;

    /* ---- State space: [x, y, z (m)] ---- */
    state_type s_lb, s_ub, s_eta;
    for (size_t i = 0; i < STATE_DIM; ++i) {
        s_lb[i] = -POS_HALF_RANGE;
        s_ub[i] = POS_HALF_RANGE;
        s_eta[i] = (2.0 * POS_HALF_RANGE) / (POS_PTS - 1);
    }

    UniformGrid ss(STATE_DIM, s_lb, s_ub, s_eta);
    cout << "State space grid:" << endl;
    ss.print_info();

    /* ---- Input space: velocity commands [m/s] ---- */
    input_type i_lb, i_ub, i_eta;
    for (size_t i = 0; i < INPUT_DIM; ++i) {
        i_lb[i] = -v_max;
        i_ub[i] = v_max;
        i_eta[i] = (2.0 * v_max) / (VEL_PTS - 1);
    }

    UniformGrid is(INPUT_DIM, i_lb, i_ub, i_eta);
    cout << "Input space grid:" << endl;
    is.print_info();

    auto obstacle = [](const abs_type&) { return false; };

    cout << "Computing transition function:" << endl;
    TransitionFunction tf;
    Abstraction<state_type, input_type> abs(ss, is);

    tt.tic();
    abs.compute_gb(tf, system_post, radius_post, obstacle);
    tt.toc();

    if (!getrusage(RUSAGE_SELF, &usage)) {
        if (tf.get_no_transitions() > 0)
            cout << "Memory per transition: " << usage.ru_maxrss / (double)tf.get_no_transitions() << endl;
    }
    cout << "Number of transitions: " << tf.get_no_transitions() << endl;

    /* Target: reach origin (cell-extent-aware, spans ~2-3 cells) */
    double target_radius = 1.5 * s_eta[0];  /* 1.5 * grid spacing */

    auto target = [&](const abs_type& idx) {
        state_type x;
        ss.itox(idx, x);
        for (size_t i = 0; i < STATE_DIM; ++i) {
            if (std::abs(x[i]) > target_radius) return false;
        }
        return true;
    };

    cout << "\nSynthesis (reachability game):" << endl;
    tt.tic();
    WinningDomain win = solve_reachability_game(tf, target);
    tt.toc();
    cout << "Winning domain size: " << win.get_size() << " / " << ss.size() << endl;

    char fname[256];
    snprintf(fname, sizeof(fname), "ee_reach");
    cout << "\nWrite controller to " << fname << ".scs\n";
    StaticController controller(ss, is, std::move(win));
    if (write_to_file(controller, fname))
        cout << "Done.\n";

    return 0;
}
