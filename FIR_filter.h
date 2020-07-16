#ifndef FIR_FILTER_H_
#define FIR_FILTER_H_

#include <vector>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// FIR filter class
//
// Implements the tranfer function of a discrete time FIR filter
//
//        a(n) z^n + a(n-1) z^(n-1) + a(n-2) z^(n-2) + ... + a(0)
// F(z) = -------------------------------------------------------
//                               z^n
//
// where:
//   coefficient = [a(n) a(n-1) a(n-2) ... a(0)]
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class FIR_filter
{
    private:
        std::vector<double> state, coefficient;

    public:
        FIR_filter(const std::vector<double>& coefficient);
        ~FIR_filter();

        void evaluate(double input, double& output);
        void reset_state();
};

#endif // FIR_FILTER_H_