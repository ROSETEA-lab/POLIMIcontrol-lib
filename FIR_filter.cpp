#include "FIR_filter.h"

#include <stdexcept>
#include <algorithm>


FIR_filter::FIR_filter(const std::vector<double>& coefficient) :
    FIR_filter(coefficient, std::vector<double>((int)coefficient.size()-1, 0.0))
{
    // Do nothing
}

FIR_filter::FIR_filter(const std::vector<double>& coefficient, const std::vector<double>& initial_state)
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

        // Numerator and denominator size
        m = (int)coefficient.size();
        lw = m-1;

        // Initialize state vector
        state = initial_state;
    }
}

FIR_filter::~FIR_filter()
{
    // Do nothing
}

void FIR_filter::evaluate(double input, double& output)
{
//    if(lw > 0)
//      for index = 1:L
//        y(index) = w(1) + b(1)*x(index);
//        if ( lw > 1)
//          # Update state vector
//          w(1:lw-1) = w(2:lw) + b(2:lw)*x(index);
//          w(lw) = b(MN)*x(index);
//        else
//          w(1) = b(2)*x(index);
//        endif
//      endfor
//    else
//      # Handle special case where there is no delay separately.
//      y = b(1)*x;
//    endif

    // Evaluate filter output
    if (lw>0) {
        output = state.at(0)+coefficient.at(0)*input;
        if (lw>1) {
            // Update state vector
            for (auto k=0; k<lw-1; k++) {
                state.at(k) = state.at(k+1)+coefficient.at(k+1)*input;
            }
            state.at(lw-1) = coefficient.at(m-1)*input;
        }
        else {
            state.at(0) = coefficient.at(1)*input;
        }
    }
    else {              // If the filter is algebraic, directly compute the output end exit
        output = coefficient.at(0)*input;
        return;
    }
}

void FIR_filter::reset_state()
{
    state.assign((int)coefficient.size()-1, 0.0);
}
