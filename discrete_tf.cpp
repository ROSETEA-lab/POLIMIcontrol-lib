#include "discrete_tf.h"

#include <iostream>
#include <stdexcept>
#include <algorithm>


discrete_tf::discrete_tf(const std::vector<double>& num_coeff, const std::vector<double>& den_coeff) :
    discrete_tf(num_coeff, den_coeff, std::vector<double>(std::max((int)den_coeff.size(),(int)num_coeff.size())-1, 0.0))
{
    // Do nothing
}

discrete_tf::discrete_tf(const std::vector<double>& num_coeff, const std::vector<double>& den_coeff, const std::vector<double>& initial_state)
{
    // Check parameter consistency
    if (num_coeff.size()>den_coeff.size())
    {
        this->num_coeff.push_back(0.0);
        this->den_coeff.push_back(0.0);
        this->state.push_back(0.0);

        throw std::invalid_argument("The order of the denominator should be greater than or equal to the order of the numerator.");
    }
    else if ((num_coeff.size()<1) || (den_coeff.size()<1))
    {
        this->num_coeff.push_back(0.0);
        this->den_coeff.push_back(0.0);
        this->state.push_back(0.0);

        throw std::invalid_argument("The order of the numerator and denominator should be greater than or equal to one.");
    }
    else if (initial_state.size()!=(std::max((int)den_coeff.size(),(int)num_coeff.size())-1))
    {
        this->num_coeff.push_back(0.0);
        this->den_coeff.push_back(0.0);
        this->state.push_back(0.0);

        throw std::invalid_argument("The size of the initial state vector is wrong.");
    }
    else if (den_coeff.at(0) == 0.0)
    {
        this->num_coeff.push_back(0.0);
        this->den_coeff.push_back(0.0);
        this->state.push_back(0.0);

        throw std::invalid_argument("The first coefficient of the denominator cannot be equal to zero.");
    }
    else
    {
        // Initialize coefficient vectors and normalize them (a(m)=1)
        for (auto it_num_coeff = num_coeff.begin(); it_num_coeff != num_coeff.end(); ++it_num_coeff)
        {
            this->num_coeff.push_back(*it_num_coeff/den_coeff.at(0));
        }

        for (auto it_den_coeff = den_coeff.begin(); it_den_coeff != den_coeff.end(); ++it_den_coeff)
        {
            this->den_coeff.push_back(*it_den_coeff/den_coeff.at(0));
        }

        // Numerator and denominator size
        n = (int)den_coeff.size();
        m = (int)num_coeff.size();
        MN = std::max(n,m);
        lw = MN-1;

        // It is convenient to pad the coefficient vectors to the same length
        if((MN-m%MN)%MN > 0)
        {
            this->num_coeff.resize(m+(MN-m%MN)%MN, 0.0);
            this->den_coeff.resize(n+(MN-n%MN)%MN, 0.0);
        }

        // Initialize state vectors
        state = initial_state;
    }
}

discrete_tf::~discrete_tf()
{
    // Do nothing
}

void discrete_tf::evaluate(double input, double& output)
{
    // Output computation
    if (lw>0)
    {
        output = state.at(0)+num_coeff.at(0)*input;
    }
    else
    {
        output = num_coeff.at(0)*input;
    }


    // Update state vector
    if (lw>1)
    {
        for (auto k=0; k<lw-1; k++)
        {
            state.at(k) = state.at(k+1)-den_coeff.at(k+1)*output+num_coeff.at(k+1)*input;
        }
        state.at(lw-1) = num_coeff.at(MN-1)*input-den_coeff.at(MN-1)*output;
    }
    else if (lw>0)
    {
        state.at(0) = num_coeff.at(MN-1)*input-den_coeff.at(MN-1)*output;
    }
    else
    {
        // Do nothing
    }
//      y(index) = w(1) + b(1)*x(index);
//      # Update state vector
//      if(lw > 1)
//        w(1:(lw-1)) = w(2:lw) - a(2:lw)*y(index) + b(2:lw)*x(index);
//        w(lw) = b(MN)*x(index) - a(MN) * y(index);
//      else
//        w(1) = b(MN)*x(index) - a(MN) * y(index);
//      endif
}

void discrete_tf::reset_state()
{
    state.assign(std::max((int)den_coeff.size(),(int)num_coeff.size())-1, 0.0);
}
