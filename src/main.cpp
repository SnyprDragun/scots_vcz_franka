#include "ScotsVczControl.hpp"

int main() {
    string filename = "/home/focaslab/SCOTS/SCOTS_ros2_v2/SCOTS/examples/franka/new/xyz_roll_pitch_yaw.csv"; 

    try {
        ScotsVczControl controller(filename);

        arma::vec x(controller.STATE_DIM); 
        x[0] = 0.6;     
        x[1] = 0.72;    
        x[2] = 2.52;

        cout << "\nQuerying controller for state x = " << x.t();
        vector<arma::vec> U = controller.getValidControls(x);
        controller.printInputs("\nValid control inputs:", U);

    } catch (const exception& e) {
        cerr << "\n--- ERROR ---" << endl;
        cerr << e.what() << endl;
        cerr << "-------------" << endl;
        return 1;
    }

    return 0;
}