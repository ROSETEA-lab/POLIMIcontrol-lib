#include "discrete_tf.h"

#include <iostream>
#include <stdexcept>
#include <algorithm>


discrete_tf::discrete_tf(const std::vector<double>& num_coeff, const std::vector<double>& den_coeff)
{
    // Check parameter consistency
    if (num_coeff.size()>den_coeff.size())
    {
        this->num_coeff.push_back(0.0);
        this->den_coeff.push_back(0.0);
        this->in_state.push_back(0.0);
        this->out_state.push_back(0.0);

        throw std::invalid_argument( "The order of the denominator should be greater than or equal to the order of the numerator.");
    }
    else if ((num_coeff.size()<1) || (den_coeff.size()<1))
    {
        this->num_coeff.push_back(0.0);
        this->den_coeff.push_back(0.0);
        this->in_state.push_back(0.0);
        this->out_state.push_back(0.0);

        throw std::invalid_argument( "The order of the numerator and denominator should be greater than or equal to one.");
    }
    else
    {
        // Initialize coefficient vectors
        std::vector<double>::const_iterator it_num_coeff;
        for (it_num_coeff = num_coeff.begin(); it_num_coeff != num_coeff.end(); ++it_num_coeff)
        {
            this->num_coeff.push_back(*it_num_coeff);
        }

        std::vector<double>::const_iterator it_den_coeff;
        for (it_den_coeff = den_coeff.begin(); it_den_coeff != den_coeff.end(); ++it_den_coeff)
        {
            this->den_coeff.push_back(*it_den_coeff);
        }

        // Initialize state vectors
        in_state.assign((int)den_coeff.size()-1, 0.0);
        out_state.assign((int)den_coeff.size()-1, 0.0);
    }
}

discrete_tf::~discrete_tf()
{
    // Do nothing
}

void discrete_tf::evaluate(double input, double& output)
{
    // Numerator and denominator order
    int n = (int)den_coeff.size()-1;
    int m = (int)num_coeff.size()-1;

    // If the filter is algebraic, directly compute the output end exit
    if ((n==0) and (m==0)) {
        output = num_coeff.at(0)/den_coeff.at(0)*input;
        return;
    }

    // Otherwise, evaluate the filter output
    output = 0.0;

    std::vector<double>::iterator it_den_coeff, it_out_state;
    for (it_den_coeff = den_coeff.begin()+1, it_out_state = out_state.begin(); it_den_coeff != den_coeff.end(); ++it_den_coeff, ++it_out_state)
    {
        output -= *it_den_coeff/den_coeff.at(0)*(*it_out_state);
    }

    if (n-m==0) {  // Not strictly proper transfer function
        std::vector<double>::iterator it_num_coeff, it_in_state;
        for (it_num_coeff = num_coeff.begin()+1, it_in_state = in_state.begin(); it_num_coeff != num_coeff.end(); ++it_num_coeff, ++it_in_state)
        {
            output += *it_num_coeff/den_coeff.at(0)*(*it_in_state);
        }

        output += num_coeff.at(0)/den_coeff.at(0)*input;
    } else {       // Strictly proper transfer function
        std::vector<double>::iterator it_num_coeff, it_in_state;
        for (it_num_coeff = num_coeff.begin(), it_in_state = in_state.begin()+n-m-1;
             it_num_coeff != num_coeff.end(); ++it_num_coeff, ++it_in_state)
        {
            output += *it_num_coeff/den_coeff.at(0)*(*it_in_state);
        }
    }

    // Update the filter state
    std::rotate(in_state.rbegin(), in_state.rbegin() + 1, in_state.rend());
    in_state.at(0) = input;    

    std::rotate(out_state.rbegin(), out_state.rbegin() + 1, out_state.rend());
    out_state.at(0) = output;
}

void discrete_tf::reset_state()
{
    in_state.assign((int)num_coeff.size()-1, 0.0);
    out_state.assign((int)den_coeff.size()-1, 0.0);
}
