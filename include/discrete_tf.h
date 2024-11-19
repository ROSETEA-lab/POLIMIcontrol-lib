#ifndef DISCRETE_TF_H_
#define DISCRETE_TF_H_

#include <vector>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Discrete transfer function class
//
// Implements the transfer function of a discrete time system
//
//        b(m) z^m + b(m-1) z^(m-1) + b(m-2) z^(m-2) + ... + b(0)
// F(z) = -------------------------------------------------------
//        a(n) z^n + a(n-1) z^(n-1) + a(n-2) z^(n-2) + ... + a(0)
//
// where:
//   num_coeff = [b(m) b(m-1) b(m-2) ... b(0)]
//   den_coeff = [a(n) a(n-1) a(n-2) ... a(0)]
//
// The implementation is based on transposed Direct Form II
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class discrete_tf
{
    private:
        int n, m, MN, lw;

        std::vector<double> num_coeff, den_coeff;
        std::vector<double> state;

    public:
        discrete_tf(const std::vector<double>& num_coeff, const std::vector<double>& den_coeff);
        discrete_tf(const std::vector<double>& num_coeff, const std::vector<double>& den_coeff, const std::vector<double>& initial_state);
        ~discrete_tf();

        void evaluate(double input, double& output);
        void reset_state();
};

#endif // DISCRETE_TF_H_