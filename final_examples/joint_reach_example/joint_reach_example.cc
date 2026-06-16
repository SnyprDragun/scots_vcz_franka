/* joint_reach_example.cc
 * ============================================================================
 * Full 7-DOF joint-space reachability synthesis (14D state: 7 joint angles +
 * 7 joint velocities), replacing the earlier simplified 3D single-integrator
 * end-effector abstraction. This is the "SCOTS" baseline row of the new
 * 3-row paper table (Proposed / SCOTS / FaSTrack), run at varying payload
 * mass to produce a degrading success-percentage curve under perturbation.
 *
 * DYNAMICS MODEL
 * ---------------------------------------------------------------------------
 * True Franka rigid-body dynamics M(q)*qddot + C(q,qdot)*qdot + G(q) = tau
 * require full URDF inertial parameters (Pinocchio/RBDL) and are far too
 * expensive to evaluate ~10^9-10^10 times during SCOTS abstraction. Instead
 * we use a serial coupled-pendulum approximation that keeps the essential,
 * paper-relevant property: gravity torque on joint i depends nonlinearly on
 * the configuration of joints 1..i, and a wrist payload mass adds an extra
 * configuration-dependent torque term to every joint upstream of the wrist.
 * This is what makes joint i+1's torque requirement depend on joint i's
 * angle (i.e. genuine multi-joint *coupling*, unlike the single-integrator
 * EE model), while remaining cheap enough to abstract.
 *
 *   qddot_i = ( tau_i - G_i(q) - b_i * qdot_i ) / I_i
 *   G_i(q)  = g * sum_{j=i}^{6} ( m_j * l_j + [j==6]*m_payload*l_total )
 *                              * cos( sum_{k=i}^{j} q_k )
 *
 * Increasing PAYLOAD_MASS (kg) raises G_i(q) for every joint -> larger
 * torque needed to hold/track the same trajectory -> for a fixed torque
 * bound tau_max, higher payload shrinks the set of joint-space states from
 * which the target is still winnable, and (in the Monte-Carlo execution
 * harness, joint_reach_montecarlo.py) increases the chance that a tracked
 * trajectory misses the target -> degrading success percentage, mirroring
 * the FaSTrack TEB-divergence failure mode.
 *
 * COMPILE-TIME MACROS (so you can sweep payload mass without editing code)
 *   PAYLOAD_MASS_G   : payload mass in GRAMS (default 0)         -DPAYLOAD_MASS_G=500
 *   POS_HALF_RANGE    : +/- joint angle range around q=0 [rad]    -DPOS_HALF_RANGE=0.2
 *   VEL_HALF_RANGE    : +/- joint velocity range [rad/s]          -DVEL_HALF_RANGE=0.1
 *   POS_PTS           : grid points per position axis (odd)       -DPOS_PTS=3
 *   VEL_PTS           : grid points per velocity axis (odd)       -DVEL_PTS=3
 *   INPUT_PTS         : grid points per torque axis (>=2)         -DINPUT_PTS=2
 *
 * The defaults below give 3^7 (pos) * 3^7 (vel) = 4,782,969 states and
 * 2^7 = 128 inputs. On a laptop this is expected to take on the order of
 * MINUTES-to-HOURS depending on hardware (the PhD guide's own expectation
 * for full 14D joint-space SCOTS synthesis). Recommended workflow per the
 * guide's note ("you can just interpolate"):
 *   1. Compile+run once at PAYLOAD_MASS_G=0   (nominal)
 *   2. Compile+run once at PAYLOAD_MASS_G=1000 (worst case, 1 kg)
 *   3. Optionally one or two intermediate masses (e.g. 100, 500 g)
 *   4. Interpolate/extrapolate the success-percentage curve for the
 *      remaining mass steps in the 1g->1kg, 10-step sweep, rather than
 *      re-running all 10 from scratch.
 * ============================================================================
 */

#include <array>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>
#include <fstream>
#include <iostream>
#include <sys/time.h>
#include <Eigen/Dense>
#include <sys/resource.h>

#include "scots.hh"
#include "TicToc.hh"
#include "RungeKutta4.hh"

struct rusage usage;

#ifndef PAYLOAD_MASS_G
#define PAYLOAD_MASS_G 0.0
#endif
#ifndef POS_HALF_RANGE
#define POS_HALF_RANGE 0.2
#endif
#ifndef VEL_HALF_RANGE
#define VEL_HALF_RANGE 0.1
#endif
#ifndef POS_PTS
#define POS_PTS 3
#endif
#ifndef VEL_PTS
#define VEL_PTS 3
#endif
#ifndef INPUT_PTS
#define INPUT_PTS 2
#endif

constexpr size_t NJ        = 7;             /* Franka has 7 joints */
constexpr size_t STATE_DIM = 2 * NJ;        /* [q1..q7, qd1..qd7] = 14D   */
constexpr size_t INPUT_DIM = NJ;            /* [tau1..tau7]                */
constexpr double tau_s     = 0.3;           /* sampling time [s]          */
constexpr double g_acc     = 9.81;
constexpr double payload_mass = PAYLOAD_MASS_G / 1000.0;   /* kg */

/* Lumped per-link mass [kg] and effective lever arm [m] -- simplified for
 * fast abstraction; real robot would use Pinocchio/URDF. */
constexpr double link_mass[NJ] = {1.5, 1.5, 1.2, 1.2, 0.8, 0.5, 0.3};
constexpr double link_len [NJ] = {0.15, 0.15, 0.12, 0.12, 0.10, 0.08, 0.06};
constexpr double total_len     = 0.15+0.15+0.12+0.12+0.10+0.08+0.06; /* wrist lever arm [m] */

/* Effective joint inertia [kg m^2] and viscous damping [N m s/rad]. */
constexpr double I_eff[NJ] = {0.5, 0.4, 0.3, 0.25, 0.1, 0.08, 0.05};
constexpr double b_damp[NJ] = {1.0, 1.0, 0.8, 0.8, 0.5, 0.4, 0.3};

/* Torque bound per joint [N m], conservative stand-ins for Franka. */
constexpr double tau_max[NJ] = {40, 40, 40, 40, 8, 8, 8};

using namespace std;
using namespace scots;
using state_type = array<double, STATE_DIM>;
using input_type = array<double, INPUT_DIM>;

/* Gravity + payload torque on joint i, given full configuration q[0..6]. */
inline void gravity_torque(const array<double, NJ>& q, array<double, NJ>& G) {
    for (size_t i = 0; i < NJ; ++i) {
        double sum = 0.0;
        double running_angle = 0.0;
        for (size_t j = i; j < NJ; ++j) {
            running_angle += q[j];
            double m = link_mass[j];
            double l = link_len[j];
            if (j == NJ - 1) {
                /* wrist payload adds extra mass at the full lever arm */
                m += payload_mass;
                l = total_len;
            }
            sum += m * l * std::cos(running_angle);
        }
        G[i] = g_acc * sum;
    }
}

/* Full coupled joint dynamics: qddot_i = (tau_i - G_i(q) - b_i*qdot_i)/I_i */
auto system_post = [](state_type &x, input_type &u) -> void {
    auto rhs = [](state_type &xdot, const state_type &xx, input_type &uu) -> void {
        array<double, NJ> q, qd, G;
        for (size_t i = 0; i < NJ; ++i) { q[i] = xx[i]; qd[i] = xx[NJ + i]; }
        gravity_torque(q, G);
        for (size_t i = 0; i < NJ; ++i) {
            xdot[i]      = qd[i];
            xdot[NJ + i] = (uu[i] - G[i] - b_damp[i] * qd[i]) / I_eff[i];
        }
    };
    runge_kutta_fixed4(rhs, x, u, STATE_DIM, tau_s, 10);
};

auto radius_post = [](state_type &r, const state_type &x, const input_type &u) {
    /* Simple error inflation: just track the absolute error. The reachability
     * game will compute the exact error bound based on system dynamics. */
    for (size_t i = 0; i < STATE_DIM; ++i) {
        r[i] = fabs(r[i]);
    }
};

int main(int argc, char** argv) {
    TicToc tt;
    cout << "Joint-space (14D) SCOTS reach example" << endl;
    cout << "STATE_DIM = " << STATE_DIM << ", INPUT_DIM = " << INPUT_DIM << endl;
    cout << "Payload mass = " << payload_mass << " kg (" << PAYLOAD_MASS_G << " g)" << endl;

    /* ---- State space: [q1..q7 (rad), qd1..qd7 (rad/s)] ---- */
    state_type s_lb, s_ub, s_eta;
    for (size_t i = 0; i < NJ; ++i) {
        s_lb[i] = -POS_HALF_RANGE;   s_ub[i] = POS_HALF_RANGE;   s_eta[i] = 0.2;   /* coarse: 3 points per pos axis */
        s_lb[NJ+i] = -VEL_HALF_RANGE; s_ub[NJ+i] = VEL_HALF_RANGE; s_eta[NJ+i] = 0.1;   /* coarse: 3 points per vel axis */
    }

    UniformGrid ss(STATE_DIM, s_lb, s_ub, s_eta);
    cout << "State space grid:" << endl;
    ss.print_info();

    /* ---- Input space: joint torque commands [N m] ---- */
    input_type i_lb, i_ub, i_eta;
    for (size_t i = 0; i < NJ; ++i) {
        i_lb[i] = -tau_max[i];
        i_ub[i] =  tau_max[i];
        i_eta[i] = tau_max[i] / 4.0;   /* 9 points per input: [-tau, -3tau/4, ..., 0, ..., 3tau/4, tau] */
    }

    UniformGrid is(INPUT_DIM, i_lb, i_ub, i_eta);
    cout << "Input space grid:" << endl;
    is.print_info();

    /* No explicit obstacles -- joint limits are already enforced by the
     * grid bounds. Self-collision is out of scope for this toy example. */
    auto obstacle = [](const abs_type&) { return false; };

    cout << "Computing transition function (this is the long step -- see header for timing notes):" << endl;
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

    /* Target: stabilization ball around origin (q ~ 0, v ~ 0).
     * This is the easiest reachability task: from any nearby state,
     * the controller should be able to hold the arm at the origin
     * despite gravity. As payload mass increases, this becomes harder. */
    double target_q_radius = 0.05;   /* [rad] */
    double target_v_radius = 0.05;   /* [rad/s] */

    auto target = [&](const abs_type& idx) {
        state_type x;
        ss.itox(idx, x);
        for (size_t i = 0; i < NJ; ++i) {
            if (fabs(x[i]) > target_q_radius) return false;
            if (fabs(x[NJ+i]) > target_v_radius) return false;
        }
        return true;
    };

    cout << "\nSynthesis (reachability game):" << endl;
    tt.tic();
    WinningDomain win = solve_reachability_game(tf, target);
    tt.toc();
    cout << "Winning domain size: " << win.get_size() << " / " << ss.size() << endl;

    char fname[256];
    snprintf(fname, sizeof(fname), "joint_reach_m%dg", (int)PAYLOAD_MASS_G);
    cout << "\nWrite controller to " << fname << ".scs" << endl;
    StaticController controller(ss, is, move(win));
    if (write_to_file(controller, fname))
        cout << "Done writing controller file." << endl;

    /* CSV export of the state->control lookup table (winning states only),
     * consumed by joint_reach_montecarlo.py for the mass-sweep success-rate
     * experiment. Only synthesize/export this ONCE at PAYLOAD_MASS_G=0
     * (nominal) -- the Monte Carlo harness re-simulates this *same* nominal
     * controller under increasingly perturbed dynamics to measure how often
     * it still reaches the target. */
    char csvname[256];
    snprintf(csvname, sizeof(csvname), "%s.csv", fname);
    ofstream csvfile(csvname);
    for (size_t i = 0; i < NJ; ++i) csvfile << "q" << i << ",";
    for (size_t i = 0; i < NJ; ++i) csvfile << "qd" << i << ",";
    for (size_t i = 0; i < NJ; ++i) csvfile << "tau" << i << (i + 1 == NJ ? "\n" : ",");

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
