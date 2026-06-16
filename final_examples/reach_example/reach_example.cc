#include <array>
#include <cmath>
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

#ifndef N_LINKS
#define N_LINKS 3
#endif

constexpr size_t NL        = (size_t)N_LINKS;
constexpr size_t STATE_DIM = NL;
constexpr size_t INPUT_DIM = NL;
constexpr double tau = 0.3;   /* sampling time [s] */

using namespace std;
using namespace scots;
using state_type = array<double, STATE_DIM>;
using input_type = array<double, INPUT_DIM>;

/* Single-integrator VCZ dynamics: xi_dot = u */
auto system_post = [](state_type &x, input_type &u) -> void {
    auto rhs = [](state_type &xdot, const state_type &x, input_type &uu) -> void {
        for (size_t i = 0; i < NL; ++i) xdot[i] = uu[i];
    };
    runge_kutta_fixed4(rhs, x, u, STATE_DIM, tau, 10);
};

auto radius_post = [](state_type &r, const state_type &x, const input_type &u) {
    state_type r_next;
    for (size_t i = 0; i < NL; ++i) r_next[i] = abs(r[i]);
    r = r_next;
};

int main(int argc, char** argv) {
    TicToc tt;
    cout << "N_LINKS = " << NL << ", STATE_DIM = " << STATE_DIM << ", INPUT_DIM = " << INPUT_DIM << endl;

    /*
     * VCZ specification tightening (for reference):
     *   lambda      = 0.05 m        (VCZ ball radius)
     *   u_bar       = 0.05 m/s      (matches i_ub below)
     *   delta       = u_bar*tau/2 + eta_s = 0.05*0.15 + 0.01 = 0.0175 m
     *   lambda+delta ~ 0.07 m       (total tightening per dimension)
     *
     * Original goal G (for robot execution / paper reporting):
     *   x in [0.47, 0.69],  y in [-0.11, 0.11],  z in [0.26, 0.50]
     *
     * Tightened goal G_{lambda+delta} (what SCOTS synthesizes for):
     *   x in [0.54, 0.62],  y in [-0.04, 0.04],  z in [0.33, 0.43]
     */

    /* ---- State space: 3D EE Cartesian position [m] ---- */
    state_type s_lb, s_ub, s_eta;

    s_lb[0]  =  0.28;  s_ub[0]  =  0.65;  s_eta[0] = 0.01;   /* X: forward */
    s_lb[1]  = -0.35;  s_ub[1]  =  0.35;  s_eta[1] = 0.01;   /* Y: lateral */
    s_lb[2]  =  0.02;  s_ub[2]  =  0.55;  s_eta[2] = 0.01;   /* Z: vertical */

    UniformGrid ss(STATE_DIM, s_lb, s_ub, s_eta);
    cout << "State space grid:" << endl;
    ss.print_info();

    /* ---- Input space: VCZ velocity command [m/s] ---- */
    input_type i_lb, i_ub, i_eta;
    for (size_t i = 0; i < 3; ++i) {
        i_lb[i]  = -0.05;
        i_ub[i]  =  0.05;
        i_eta[i] =  0.01;
    }

    UniformGrid is(INPUT_DIM, i_lb, i_ub, i_eta);
    cout << "Input space grid:" << endl;
    is.print_info();

    cout << "Defining 3D obstacle set..." << endl;

    /* Obstacles: Franka base column + workspace boundary walls.
     * Format: [x_lo, x_hi, y_lo, y_hi, z_lo, z_hi] */
    const size_t NO_OBSTACLES = 5;
    const double ObstacleBounds3D[NO_OBSTACLES][6] = {
        /* ===== Franka base / robot body column ===== */
        { -0.28,  0.28, -0.28,  0.28,  0.02,  0.70 },
        /* ===== Workspace boundary walls ===== */
        {  0.63,  0.70, -0.40,  0.40,  0.02,  0.60 },   /* +X wall */
        { -0.40,  0.70,  0.33,  0.40,  0.02,  0.60 },   /* +Y wall */
        { -0.40,  0.70, -0.40, -0.33,  0.02,  0.60 },   /* -Y wall */
        { -0.40,  0.70, -0.40,  0.40,  0.53,  0.65 },   /* +Z ceiling */
    };

    auto obstacle = [&ObstacleBounds3D, NO_OBSTACLES, &ss, &s_eta](const abs_type& idx) {
        state_type x;
        ss.itox(idx, x);

        double c[3];
        c[0] = s_eta[0] / 2.0 + 1e-10;
        c[1] = s_eta[1] / 2.0 + 1e-10;
        c[2] = s_eta[2] / 2.0 + 1e-10;

        for (size_t i = 0; i < NO_OBSTACLES; i++) {
            const double *bounds = ObstacleBounds3D[i];
            bool x_overlap = (bounds[0] - c[0]) <= x[0] && x[0] <= (bounds[1] + c[0]);
            bool y_overlap = (bounds[2] - c[1]) <= x[1] && x[1] <= (bounds[3] + c[1]);
            bool z_overlap = (bounds[4] - c[2]) <= x[2] && x[2] <= (bounds[5] + c[2]);
            if (x_overlap && y_overlap && z_overlap)
                return true;
        }
        return false;
    };
    write_to_file(ss, obstacle, "reach_example_obstacle");

    cout << "Computing transition function (pruning obstacles):" << endl;
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

    /* Tightened target G_{lambda+delta}: x in [0.54,0.62], y in [-0.04,0.04], z in [0.33,0.43] */
    auto target = [&ss, &s_eta](const abs_type& idx) {
        state_type x;
        ss.itox(idx, x);
        bool cond = true;

        double vx = x[0]; double hx = s_eta[0] / 2.0;
        if (!(0.54 <= (vx - hx) && (vx + hx) <= 0.62)) cond = false;

        double vy = x[1]; double hy = s_eta[1] / 2.0;
        if (!(-0.04 <= (vy - hy) && (vy + hy) <= 0.04)) cond = false;

        double vz = x[2]; double hz = s_eta[2] / 2.0;
        if (!(0.33 <= (vz - hz) && (vz + hz) <= 0.43)) cond = false;

        return cond;
    };
    write_to_file(ss, target, "reach_example_target");

    cout << "\nSynthesis (reachability game):" << endl;
    tt.tic();
    WinningDomain win = solve_reachability_game(tf, target);
    tt.toc();
    cout << "Winning domain size: " << win.get_size() << endl;

    cout << "\nWrite controller to reach_example.scs" << endl;
    StaticController controller(ss, is, move(win));
    if (write_to_file(controller, "reach_example"))
        cout << "Done writing controller file." << endl;

    /* CSV export: state-input lookup table for robot execution */
    ofstream csvfile("reach_example.csv");
    csvfile << "x,y,z,vx,vy,vz\n";

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
        for (const auto& uc : controls) {
            for (size_t d = 0; d < STATE_DIM; ++d) csvfile << x[d] << ",";
            for (size_t d = 0; d < INPUT_DIM; ++d)
                csvfile << uc[d] << (d + 1 == INPUT_DIM ? "\n" : ",");
            ++rows_written;
        }
    }
    csvfile.close();
    cout << "State-input pairs written to reach_example.csv (rows: " << rows_written << ")\n";

    return 0;
}
