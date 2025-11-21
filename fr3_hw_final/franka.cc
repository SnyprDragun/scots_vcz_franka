#include <array>
#include <vector>
#include <cmath>
#include <limits>
#include <iostream>
#include <fstream>
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

using namespace std;
using namespace scots;

constexpr size_t NL = (size_t)N_LINKS;
constexpr size_t STATE_DIM = NL;     // State: [x, y, z]
constexpr size_t INPUT_DIM = NL;     // Input: [vx, vy, vz]
// REDUCED TAU: Matching the reference script's time step for better maneuverability
constexpr double tau = 0.3;

using state_type = array<double, STATE_DIM>;
using input_type = array<double, INPUT_DIM>;

/* ----------------- Dynamics: single integrator x' = u ----------------- */
auto system_post = [](state_type &x, input_type &u) -> void {
  // rhs: xdot = u  (independent of x)
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

    // State grid: [x, y, z]
    state_type s_lb, s_ub, s_eta;

    // --- X, Y, Z (meters) ---
    for (size_t i = 0; i < 3; ++i) {
        // s_lb[i]  =  -0.7;   // Position lower bound (m)
        s_ub[i]  =  0.7;   // Position upper bound (m)
        s_eta[i] =  0.02;   // Cell size for position i (m)
    }
    s_lb[0] = -0.7; 
    s_lb[1] = -0.7;
    s_lb[2] =  0.3;

    UniformGrid ss(STATE_DIM, s_lb, s_ub, s_eta);
    cout << "Uniform grid details:" << endl;
    ss.print_info();

    // Input grid: [vx, vy, vz] in m/s
    input_type i_lb, i_ub, i_eta;

    // --- Linear Velocities (m/s) ---
    for (size_t i = 0; i < 3; ++i) {
        i_lb[i]  =  -0.25;   // Corrected: Allow movement in negative direction
        i_ub[i]  =  0.25;   // max linear rate (m/s)
        i_eta[i] =  0.04;   // quantization
    }

    UniformGrid is(INPUT_DIM, i_lb, i_ub, i_eta);
    cout << "Input grid details:" << endl;
    is.print_info();

    // --- START: 3D Obstacle Set Definition ---
    cout << "Defining 3D obstacle set..." << endl;
    
    // 3D Constraint matrix: [x_min, x_max, y_min, y_max, z_min, z_max]
    // Obstacle: A central pillar that forces the system to go around it in Y or Z.
    // X in [1.5, 1.7] (Narrow wall)
    // Y in [1.0, 3.0] (Blocks the middle Y path)
    // Z in [1.0, 3.0] (Blocks the middle Z path)
    const size_t NO_OBSTACLES = 11;
    const double ObstacleBounds3D[NO_OBSTACLES][6] = {
      // Bounds: X-span, Y-span, Z-span
      { -0.3, 0.3, -0.3, 0.3, 0.2, 0.7 },
      { 0.6, 0.7, -0.7, 0.7, 0.2, 0.7 },
      { -0.7, -0.6, -0.7, 0.7, 0.2, 0.7 },
      { -0.6, 0.6, 0.6, 0.7, 0.2, 0.7 },
      { -0.6, 0.6, -0.7, -0.6, 0.2, 0.7 },
      { 0.15, 0.25, -0.6, -0.3, 0.4, 0.7 },
      { -0.25, -0.15, -0.6, -0.3, 0.4, 0.7 },
      { -0.05, 0.05, -0.6, -0.3, 0.2, 0.5 },
      { 0.15, 0.25, 0.3, 0.6, 0.4, 0.7 },
      { -0.25, -0.15, 0.3, 0.6, 0.4, 0.7 },
      { -0.05, 0.05, 0.3, 0.6, 0.2, 0.5 }
    };

    // Obstacle function: returns true if the state cell overlaps with the 3D obstacle (AVOID SET)
    auto obstacle = [&ObstacleBounds3D, NO_OBSTACLES, &ss, &s_eta](const abs_type& idx) {
        state_type x;
        ss.itox(idx, x);
        
        // Half cell size in x, y, and z dimensions (with tolerance)
        double c[3];
        c[0] = s_eta[0] / 2.0 + 1e-10; // X (0.03)
        c[1] = s_eta[1] / 2.0 + 1e-10; // Y (0.03)
        c[2] = s_eta[2] / 2.0 + 1e-10; // Z (0.03)

        for(size_t i = 0; i < NO_OBSTACLES; i++) {
          // Check for overlap in X, Y, AND Z dimensions
          const double *bounds = ObstacleBounds3D[i];
          
          // Overlap check: center (x[i]) must be within [bounds_min - c[i], bounds_max + c[i]]
          bool x_overlap = (bounds[0] - c[0]) <= x[0] && x[0] <= (bounds[1] + c[0]);
          bool y_overlap = (bounds[2] - c[1]) <= x[1] && x[1] <= (bounds[3] + c[1]);
          bool z_overlap = (bounds[4] - c[2]) <= x[2] && x[2] <= (bounds[5] + c[2]);

          if (x_overlap && y_overlap && z_overlap)
            return true;
        }
        return false;
    };
    // --- END: 3D Obstacle Set Definition ---
    
    // --- Write obstacle set to file for visualization and debugging ---
    write_to_file(ss, obstacle, "franka_obstacle");

    cout << "Computing the transition function (PRUNING OBSTACLES): " << endl;
    TransitionFunction tf;
    Abstraction<state_type, input_type> abs(ss, is);

    tt.tic();
    // NOW PASSING 'obstacle' to prune unsafe states from the graph, matching the reference script.
    abs.compute_gb(tf, system_post, radius_post, obstacle);
    tt.toc();

    if(!getrusage(RUSAGE_SELF, &usage)) {
        if (tf.get_no_transitions() > 0) {
            cout << "Memory per transition: "
                 << usage.ru_maxrss / (double)tf.get_no_transitions() << endl;
        }
    }
    cout << "Number of transitions: " << tf.get_no_transitions() << endl;

    // Define target set
    auto target = [&ss, &s_eta](const abs_type& idx) {
        state_type x;
        ss.itox(idx, x);
        bool cond = true;

        // --- X, Y, Z Target (meters) ---
        // Goal: x, y, z in [3.6, 3.9]
        // const double p_lo = 0.5;
        // const double p_hi = 0.6;

        // for(size_t i = 0; i < 3; ++i) {
        //     double v = x[i]; double h = s_eta[i] / 2.0; // h = 0.03
        //     // Requires cell to be strictly contained in [p_lo, p_hi]
        //     if (!(p_lo <= (v - h) && (v + h) <= p_hi)) cond = false;
        // }

        double vx = x[0]; double hx = s_eta[0] / 2.0;
        if (!(-0.6 <= (vx - hx) && (vx + hx) <= -0.5)) cond = false;

        double vy = x[1]; double hy = s_eta[1] / 2.0;
        if (!(-0.1 <= (vy - hy) && (vy + hy) <= 0.0)) cond = false;

        double vz = x[2]; double hz = s_eta[2] / 2.0;
        if (!(0.5 <= (vz - hz) && (vz + hz) <= 0.6)) cond = false;
        
        return cond;
    };

    write_to_file(ss, target, "franka_target");


    cout << "\nSynthesis (Simple Reachability on Pruned Graph): " << endl;
    tt.tic();
    // Simple reachability on the graph already pruned of obstacle states.
    WinningDomain win = solve_reachability_game(tf, target); 
    tt.toc();
    cout << "Winning domain size: " << win.get_size() << endl;

    cout << "\nWrite controller to franka.scs \n";
    StaticController controller(ss, is, move(win));
    if (write_to_file(controller, "franka")) {
        cout << "Done writing controller file. \n";
    }

    /* ================= CSV Export ================= */
    ofstream csvfile("franka.csv");
    // header
    csvfile << "x_m";
    csvfile << ",y_m";
    csvfile << ",z_m";
    // csvfile << ",roll_rad";
    // csvfile << ",pitch_rad";
    // csvfile << ",yaw_rad";
    csvfile << ",vx_mps";
    csvfile << ",vy_mps";
    csvfile << ",vz_mps";
    // csvfile << ",wroll_radps";
    // csvfile << ",wpitch_radps";
    // csvfile << ",wyaw_radps";
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
    cout << "State–input pairs written to franka.csv (rows: "
         << rows_written << ")\n";

    return 0;
}
