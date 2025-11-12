#ifndef SCOTS_VCZ_CONTROL_HPP
#define SCOTS_VCZ_CONTROL_HPP

#include <map>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <armadillo> 
#include <stdexcept>

using namespace std;

class ScotsVczControl {
    private:
        map<string, vector<arma::vec>> m_controller_map;

    public:
        int STATE_DIM;
        int INPUT_DIM;
        int STATE_PRECISION;

        ScotsVczControl(const string& filename);
        vector<arma::vec> getValidControls(const arma::vec& x) const;
        static string createStateKey(const arma::vec& x, int precision);
        void printInputs(const string& title, const vector<arma::vec>& inputs);
};

#endif
