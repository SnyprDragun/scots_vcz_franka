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

constexpr size_t NL = (size_t)N_LINKS;
constexpr size_t STATE_DIM = NL;
constexpr size_t INPUT_DIM = NL;
constexpr double tau = 0.3;

using namespace std;
using namespace scots;
using state_type = array<double, STATE_DIM>;
using input_type = array<double, INPUT_DIM>;

/* ----------------- Dynamics: single integrator x' = u ----------------- */
auto system_post = [](state_type &x, input_type &u) -> void {
    auto rhs = [](state_type &xdot, const state_type &x, input_type &uu) -> void {
        for (size_t i = 0; i < NL; ++i) xdot[i] = uu[i];
    };
    runge_kutta_fixed4(rhs, x, u, STATE_DIM, tau, 10);
};

/* ----------------- Radius propagation for x' = u ----------------- */
auto radius_post = [](state_type &r, const state_type &x, const input_type &u) {
    state_type r_next;
    for (size_t i = 0; i < NL; ++i) r_next[i] = abs(r[i]);
    r = r_next;
};

/* ----------------- Main program ----------------- */
int main(int argc, char** argv) {
    TicToc tt;
    cout << "N_LINKS = " << NL << ", STATE_DIM = " << STATE_DIM << ", INPUT_DIM = " << INPUT_DIM << endl;
    state_type s_lb, s_ub, s_eta;

    for (size_t i = 0; i < 3; ++i) {
        s_ub[i]  =  0.7;
        s_eta[i] =  0.02;
    }
    s_lb[0] =  0.1; 
    s_lb[1] = -0.7;
    s_lb[2] =  0.02;

    UniformGrid ss(STATE_DIM, s_lb, s_ub, s_eta);
    cout << "Uniform grid details:" << endl;
    ss.print_info();

    input_type i_lb, i_ub, i_eta;

    for (size_t i = 0; i < 3; ++i) {
        i_lb[i]  =  -0.25;
        i_ub[i]  =  0.25;
        i_eta[i] =  0.04;
    }

    UniformGrid is(INPUT_DIM, i_lb, i_ub, i_eta);
    cout << "Input grid details:" << endl;
    is.print_info();

    cout << "Defining 3D obstacle set..." << endl;

    const size_t NO_OBSTACLES = 10;
    const double ObstacleBounds3D[NO_OBSTACLES][6] = {
        /* ================= STATE SPACE BOUNDARY ================= */
        { -0.28, 0.28, -0.28, 0.28, 0.02, 0.7 },
        {  0.6 , 0.7 , -0.7 , 0.7 , 0.02, 0.7 },
        { -0.7 , -0.6, -0.7 , 0.7 , 0.02, 0.7 },
        { -0.6 , 0.6 ,  0.6 , 0.7 , 0.02, 0.7 },
        { -0.6 , 0.6 , -0.7 ,-0.6 , 0.02, 0.7 },

        /* ================= INTERNAL BOUNDARY ================= */
        {  0.5 , 0.6 , -0.6 , 0.6 , 0.2 , 0.7 },
        {  0.15, 0.25, -0.6 ,-0.28, 0.02, 0.7 },
        {  0.15, 0.25,  0.28, 0.6 , 0.02, 0.7 },

        /* ================= OBSTACLES ================= */
        {  0.25, 0.6 ,  0.15, 0.35, 0.02, 0.5 },
        {  0.25, 0.6 , -0.35,-0.15, 0.02, 0.5 }
    };

    auto obstacle = [&ObstacleBounds3D, NO_OBSTACLES, &ss, &s_eta](const abs_type& idx) {
        state_type x;
        ss.itox(idx, x);

        double c[3];
        c[0] = s_eta[0] / 2.0 + 1e-10;
        c[1] = s_eta[1] / 2.0 + 1e-10;
        c[2] = s_eta[2] / 2.0 + 1e-10;

        for(size_t i = 0; i < NO_OBSTACLES; i++) {
            const double *bounds = ObstacleBounds3D[i];
            bool x_overlap = (bounds[0] - c[0]) <= x[0] && x[0] <= (bounds[1] + c[0]);
            bool y_overlap = (bounds[2] - c[1]) <= x[1] && x[1] <= (bounds[3] + c[1]);
            bool z_overlap = (bounds[4] - c[2]) <= x[2] && x[2] <= (bounds[5] + c[2]);

            if (x_overlap && y_overlap && z_overlap)
                return true;
        }
        return false;
    };
    write_to_file(ss, obstacle, "example_2_obstacle");

    cout << "Computing the transition function (PRUNING OBSTACLES): " << endl;
    TransitionFunction tf;
    Abstraction<state_type, input_type> abs(ss, is);

    tt.tic();
    abs.compute_gb(tf, system_post, radius_post, obstacle);
    tt.toc();

    if(!getrusage(RUSAGE_SELF, &usage)) {
        if (tf.get_no_transitions() > 0) {
            cout << "Memory per transition: " << usage.ru_maxrss / (double)tf.get_no_transitions() << endl;
        }
    }
    cout << "Number of transitions: " << tf.get_no_transitions() << endl;

    auto target = [&ss, &s_eta](const abs_type& idx) {
        state_type x;
        ss.itox(idx, x);
        bool cond = true;

        /* ================= TARGET 1 ================= */
        double vx = x[0]; double hx = s_eta[0] / 2.0;
        if (!(0.5 <= (vx - hx) && (vx + hx) <= 0.6)) cond = false;

        double vy = x[1]; double hy = s_eta[1] / 2.0;
        if (!(-0.05 <= (vy - hy) && (vy + hy) <= 0.05)) cond = false;

        double vz = x[2]; double hz = s_eta[2] / 2.0;
        if (!(0.02 <= (vz - hz) && (vz + hz) <= 0.08)) cond = false;

        /* ================= TARGET 2 ================= */
        // double vx = x[0]; double hx = s_eta[0] / 2.0;
        // if (!(0.35 <= (vx - hx) && (vx + hx) <= 0.45)) cond = false;

        // double vy = x[1]; double hy = s_eta[1] / 2.0;
        // if (!(0.5 <= (vy - hy) && (vy + hy) <= 0.6)) cond = false;

        // double vz = x[2]; double hz = s_eta[2] / 2.0;
        // if (!(0.02 <= (vz - hz) && (vz + hz) <= 0.08)) cond = false;

        /* ================= TARGET 3 ================= */
        // double vx = x[0]; double hx = s_eta[0] / 2.0;
        // if (!(0.35 <= (vx - hx) && (vx + hx) <= 0.45)) cond = false;

        // double vy = x[1]; double hy = s_eta[1] / 2.0;
        // if (!(-0.6 <= (vy - hy) && (vy + hy) <= -0.5)) cond = false;

        // double vz = x[2]; double hz = s_eta[2] / 2.0;
        // if (!(0.02 <= (vz - hz) && (vz + hz) <= 0.08)) cond = false;

        /* ================= TARGET 4 ================= */
        // double vx = x[0]; double hx = s_eta[0] / 2.0;
        // if (!(0.3 <= (vx - hx) && (vx + hx) <= 0.35)) cond = false;

        // double vy = x[1]; double hy = s_eta[1] / 2.0;
        // if (!(-0.05 <= (vy - hy) && (vy + hy) <= 0.05)) cond = false;

        // double vz = x[2]; double hz = s_eta[2] / 2.0;
        // if (!(0.45 <= (vz - hz) && (vz + hz) <= 0.55)) cond = false;

        return cond;
    };

    write_to_file(ss, target, "example_2_target");


    cout << "\nSynthesis (Simple Reachability on Pruned Graph): " << endl;
    tt.tic();
    WinningDomain win = solve_reachability_game(tf, target); 
    tt.toc();
    cout << "Winning domain size: " << win.get_size() << endl;

    cout << "\nWrite controller to example_2.scs \n";
    StaticController controller(ss, is, move(win));
    if (write_to_file(controller, "example_2")) {
        cout << "Done writing controller file. \n";
    }

    /* ================= CSV Export ================= */
    ofstream csvfile("example_2_T1.csv");
    csvfile << "x";
    csvfile << ",y";
    csvfile << ",z";
    csvfile << ",vx";
    csvfile << ",vy";
    csvfile << ",vz";
    csvfile << "\n";

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
    cout << "State–input pairs written to example_2.csv (rows: " << rows_written << ")\n";

    return 0;
}
