#include "FIR_filter.h"

#include <stdexcept>
#include <algorithm>


FIR_filter::FIR_filter(const std::vector<double>& coefficient)
{
    // Check parameter consistency
    if (coefficient.size()<1)
    {
        this->coefficient.push_back(0.0);
        this->state.push_back(0.0);

        throw std::invalid_argument( "FIR order should be greater than or equal to one.");
    }
    else
    {
        // Initialize coefficient vector
        std::vector<double>::const_iterator it_coeff;
        for (it_coeff = coefficient.begin(); it_coeff != coefficient.end(); ++it_coeff)
        {
            this->coefficient.push_back(*it_coeff);
        }

        // Initialize state vector
        state.assign((int)coefficient.size()-1, 0.0);
    }
}

FIR_filter::~FIR_filter()
{
    // Do nothing
}

void FIR_filter::evaluate(double input, double& output)
{
    // If the filter is algebraic, directly compute the output end exit
    if ((int)coefficient.size()==1) {
        output = coefficient.at(0)*input;
        return;
    }

    // Otherwise, evaluate the filter output
    std::vector<double>::iterator it_state;
    std::vector<double>::iterator it_coeff = coefficient.begin();

    output = *it_coeff*input;
    for (it_coeff = coefficient.begin()+1, it_state = state.begin(); it_coeff != coefficient.end(); ++it_coeff, ++it_state)
    {
        output += *it_coeff*(*it_state);
    }

    // Update the filter state
    std::rotate(state.rbegin(), state.rbegin() + 1, state.rend());
    state.at(0) = input;    
}

void FIR_filter::reset_state()
{
    state.assign((int)coefficient.size()-1, 0.0);
}
