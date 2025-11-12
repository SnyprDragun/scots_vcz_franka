#ifndef SCOTS_VCZ_FRANKA_HPP
#define SCOTS_VCZ_FRANKA_HPP

#include <map>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <armadillo> 
#include <stdexcept>

using namespace std;

// --- CONFIGURATION ---
const int STATE_DIM = 3;
const int INPUT_DIM = 3;
const int STATE_PRECISION = 2;
// ---------------------

/**
 * @brief Utility function to create a consistent string key from a state vector.
 * This simulates the UniformGrid::xtoi discretization.
 */
string createStateKey(const arma::vec& x, int precision);

/**
 * @class ScotsVczFranka
 * @brief Reads controller data from a CSV file and provides state-to-control mapping.
 */
class ScotsVczFranka {
private:
    // Map to store the controller: Key (discretized state string) -> Value (list of valid control vectors)
    map<string, vector<arma::vec>> m_controller_map;

public:
    /**
     * @brief Constructor that loads the controller from a CSV file.
     */
    ScotsVczFranka(const string& filename);

    /**
     * @brief Get the valid control inputs for a given state.
     * The input state 'x' is discretized and used as a key to lookup controls.
     */
    vector<arma::vec> getValidControls(const arma::vec& x) const;
};

// Helper function to print the results
void printInputs(const string& title, const vector<arma::vec>& inputs);

#endif // SCOTS_VCZ_FRANKA_HPP