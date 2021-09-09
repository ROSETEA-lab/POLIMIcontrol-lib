#include "discrete_derivative.h"

discrete_derivative::discrete_derivative(double gain, double sampling_time) :
    discrete_tf(std::vector<double>{gain, -gain}, std::vector<double>{sampling_time, 0.0})
{
     // Do nothing 
}
