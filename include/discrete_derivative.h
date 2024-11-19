#ifndef DISCRETE_DERIVATIVE_H_
#define DISCRETE_DERIVATIVE_H_

#include <vector>
#include <discrete_tf.h>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Discrete derivative class
//
// Implements the transfer function of a discrete derivative
//
//  K (z-1)
// ---------
//   Ts z
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class discrete_derivative : public discrete_tf
{
    public:
        discrete_derivative(double gain, double sampling_time);
};

#endif // DISCRETE_DERIVATIVE_H_