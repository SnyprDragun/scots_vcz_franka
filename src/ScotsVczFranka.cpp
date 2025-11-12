#include "ScotsVczFranka.hpp"

// --- Implementation of createStateKey ---
string createStateKey(const arma::vec& x, int precision) {
    stringstream ss;
    ss << fixed << setprecision(precision);
    for (size_t i = 0; i < x.n_elem; ++i) {
        ss << x[i];
        if (i < x.n_elem - 1) {
            ss << ",";
        }
    }
    return ss.str();
}

// --- Implementation of ScotsVczFranka Constructor ---
ScotsVczFranka::ScotsVczFranka(const string& filename) {
    cout << "Loading controller from CSV: " << filename << "..." << endl;
    ifstream file(filename);

    if (!file.is_open()) {
        throw runtime_error("Error: Could not open CSV file: " + filename);
    }

    string line;
    while (getline(file, line)) {
        stringstream lineStream(line);
        string cell;
        vector<double> row_data;

        // Simple parsing assuming comma separation
        while (getline(lineStream, cell, ',')) {
            try {
                row_data.push_back(stod(cell));
            } catch (...) {
                // Skip if conversion fails (e.g., empty cell or header)
            }
        }

        if (row_data.size() != STATE_DIM + INPUT_DIM) {
            continue;
        }

        // Extract State and Input vectors
        arma::vec state_vec(STATE_DIM);
        for (int i = 0; i < STATE_DIM; ++i) {
            state_vec[i] = row_data[i];
        }

        arma::vec input_vec(INPUT_DIM);
        for (int i = 0; i < INPUT_DIM; ++i) {
            input_vec[i] = row_data[STATE_DIM + i];
        }

        // Create the map key from the state vector (the cell center)
        string key = createStateKey(state_vec, STATE_PRECISION);

        // Add the control input to the map for this state key
        m_controller_map[key].push_back(input_vec);
    }

    if (m_controller_map.empty()) {
         throw runtime_error("Error: CSV file loaded but found no valid controller mappings.");
    }
    cout << "Controller loaded successfully. Total " << m_controller_map.size() << " unique states." << endl;
}

// --- Implementation of getValidControls ---
vector<arma::vec> ScotsVczFranka::getValidControls(const arma::vec& x) const {
    if (x.n_elem != STATE_DIM) {
        throw runtime_error("Input state vector dimension mismatch.");
    }
    
    // Discretize the input state 'x' to find the corresponding cell center key
    string key = createStateKey(x, STATE_PRECISION);

    auto it = m_controller_map.find(key);
    
    if (it == m_controller_map.end()) {
        // Replicating the SCOTS error for consistency
        stringstream os;
        os << "\nscots::StaticController: state " << x.t() 
           << " is out of winning domain: no progress possible. (Key: " << key << ")";
        throw runtime_error(os.str());
    }

    return it->second;
}

// --- Implementation of printInputs ---
void printInputs(const string& title, const vector<arma::vec>& inputs) {
    cout << title << endl;
    if (inputs.empty()) {
        cout << "  (No valid inputs)" << endl;
        return;
    }
    for (size_t i = 0; i < inputs.size(); ++i) {
        cout << "  Input " << i + 1 << ": " << inputs[i].t();
    }
}