#include "ScotsVczFranka.hpp"

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

        while (getline(lineStream, cell, ',')) {
            try {
                row_data.push_back(stod(cell));
            } catch (...) {
                
            }
        }

        this-> STATE_DIM = row_data.size() / 2;
        this-> INPUT_DIM = row_data.size() / 2;
        this-> STATE_PRECISION = 2;

        arma::vec state_vec(this->STATE_DIM);
        for (int i = 0; i < this->STATE_DIM; ++i) {
            state_vec[i] = row_data[i];
        }

        arma::vec input_vec(this->INPUT_DIM);
        for (int i = 0; i < this->INPUT_DIM; ++i) {
            input_vec[i] = row_data[this->STATE_DIM + i];
        }

        string key = createStateKey(state_vec, this->STATE_PRECISION);
        m_controller_map[key].push_back(input_vec);
    }

    if (m_controller_map.empty()) {
         throw runtime_error("Error: CSV file loaded but found no valid controller mappings.");
    }
    cout << "Controller loaded successfully. Total " << m_controller_map.size() << " unique states." << endl;
}

vector<arma::vec> ScotsVczFranka::getValidControls(const arma::vec& x) const {
    if (x.n_elem != this->STATE_DIM) {
        throw runtime_error("Input state vector dimension mismatch.");
    }
    string key = ScotsVczFranka::createStateKey(x, this->STATE_PRECISION);
    auto it = m_controller_map.find(key);

    if (it == m_controller_map.end()) {
        stringstream os;
        os << "\nscots::StaticController: state " << x.t() 
           << " is out of winning domain: no progress possible. (Key: " << key << ")";
        throw runtime_error(os.str());
    }
    return it->second;
}

string ScotsVczFranka::createStateKey(const arma::vec& x, int precision) {
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

void ScotsVczFranka::printInputs(const string& title, const vector<arma::vec>& inputs) {
    cout << title << endl;
    if (inputs.empty()) {
        cout << "  (No valid inputs)" << endl;
        return;
    }
    for (size_t i = 0; i < inputs.size(); ++i) {
        cout << "  Input " << i + 1 << ": " << inputs[i].t();
    }
}
