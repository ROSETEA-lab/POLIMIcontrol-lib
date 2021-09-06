#include "discrete_integrator.h"

discrete_integrator_fwEul::discrete_integrator_fwEul(double gain, double sampling_time) :
    discrete_tf(std::vector<double>{0.0, gain*sampling_time}, std::vector<double>{1.0, -1.0})
{
     // Do nothing 
}

discrete_integrator_fwEul::discrete_integrator_fwEul(double gain, double sampling_time, double initial_state) :
    discrete_tf(std::vector<double>{0.0, gain*sampling_time}, std::vector<double>{1.0, -1.0}, std::vector<double>{initial_state})
{
     // Do nothing
}

discrete_integrator_bwEul::discrete_integrator_bwEul(double gain, double sampling_time) :
    discrete_tf(std::vector<double>{gain*sampling_time, 0.0}, std::vector<double>{1.0, -1.0})
{
     // Do nothing
}

discrete_integrator_bwEul::discrete_integrator_bwEul(double gain, double sampling_time, double initial_state) :
    discrete_tf(std::vector<double>{gain*sampling_time, 0.0}, std::vector<double>{1.0, -1.0}, std::vector<double>{initial_state})
{
     // Do nothing
}

discrete_integrator_trapz::discrete_integrator_trapz(double gain, double sampling_time) :
    discrete_tf(std::vector<double>{gain*sampling_time, gain*sampling_time}, std::vector<double>{2.0, -2.0})
{
     // Do nothing
}

discrete_integrator_trapz::discrete_integrator_trapz(double gain, double sampling_time, double initial_state) :
    discrete_tf(std::vector<double>{gain*sampling_time, gain*sampling_time}, std::vector<double>{2.0, -2.0}, std::vector<double>{initial_state})
{
     // Do nothing
}
