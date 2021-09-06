#ifndef DISCRETE_INTEGRATOR_H_
#define DISCRETE_INTEGRATOR_H_

#include <vector>
#include <discrete_tf.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Discrete integrator class
//
// Implements the tranfer function of a discrete integrator using Forward Euler, Backward Euler or Trapezoidal method
//
//  K Ts      K Ts z    K Ts (z+1)
// ------ ,  ------- , -----------
//  z-1        z-1       2 (z-1)
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class discrete_integrator_fwEul : public discrete_tf
{
    public:
        discrete_integrator_fwEul(double gain, double sampling_time);
        discrete_integrator_fwEul(double gain, double sampling_time, double initial_state);
};

class discrete_integrator_bwEul : public discrete_tf
{
    public:
        discrete_integrator_bwEul(double gain, double sampling_time);
        discrete_integrator_bwEul(double gain, double sampling_time, double initial_state);
};

class discrete_integrator_trapz : public discrete_tf
{
    public:
        discrete_integrator_trapz(double gain, double sampling_time);
        discrete_integrator_trapz(double gain, double sampling_time, double initial_state);
};

#endif // DISCRETE_INTEGRATOR_H_