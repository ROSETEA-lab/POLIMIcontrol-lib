#include "MatlabDataArray.hpp"
#include "MatlabEngine.hpp"
#include <iostream>
#include <functional>
#include <algorithm>

#include "discrete_tf.h"

#define NUM_TEST 1000


template <typename UnaryFunctionT /*= std::less<double>*/ >
int count_all(const std::vector<double>& vec, UnaryFunctionT func )
{
    return std::count_if( vec.begin(), vec.end(), func );
}
bool is_nan (double d) { return std::isnan(d); }
bool is_inf (double d) { return std::isinf(d); }


int main() {
    using namespace matlab::engine;

    // Initialize tests
    std::vector<double> test_error(NUM_TEST, 0.0);

    // Start MATLAB engine synchronously
    std::unique_ptr<MATLABEngine> matlabPtr = startMATLAB();

    // Execute tests
    std::cout << "Executing " << NUM_TEST << " tests on discrete_tf class" << std::endl;
    for (auto k=0; k<NUM_TEST; k++) {
        // Simulate filter in Matlab
        matlabPtr->eval(u"test_discrete_tf;");

        // Get filter data from Matlab
        matlab::data::TypedArray<double> num = matlabPtr->getVariable(u"num");
        std::vector<double> num_coeff;
        for (auto i=0; i<num.getNumberOfElements(); i++) {
            num_coeff.push_back(num[i]);
        }

        matlab::data::TypedArray<double> den = matlabPtr->getVariable(u"den");
        std::vector<double> den_coeff;
        for (auto i=0; i<den.getNumberOfElements(); i++) {
            den_coeff.push_back(den[i]);
        }

        matlab::data::TypedArray<double> in  = matlabPtr->getVariable(u"in");
        matlab::data::TypedArray<double> out = matlabPtr->getVariable(u"out");

        // Simulate filter in C++ and compare
        discrete_tf tfd(num_coeff, den_coeff);

        std::vector<double> error;
        for (auto i=0; i<in.getNumberOfElements(); i++) {
            double tmp_out;

            tfd.evaluate(in[i], tmp_out);
            error.push_back(std::fabs(out[i]-tmp_out));
        }

        test_error.at(k) = *std::max_element(error.begin(), error.end());
    }
    std::cout << "Tests completed, plotting results" << std::endl;

    // Check NaN and Inf values
    if (count_all(test_error, is_nan)>0) {
        std::cout << "Test aborted, there are NaN values in the error" << std::endl;
    }
    if (count_all(test_error, is_inf)>0) {
        std::cout << "Test aborted, there are Inf values in the error" << std::endl;
    }

    // Plot test results
    matlab::data::ArrayFactory factory;
    matlab::data::TypedArray<double> m_error = factory.createArray({NUM_TEST,1}, test_error.begin(), test_error.end());
    matlabPtr->eval(u"clear all; close all;");
    matlabPtr->setVariable(u"error", std::move(m_error));
    matlabPtr->eval(u"figure,bar(1:1:length(error),error),grid");
    matlabPtr->eval(u"pause");

    // Terminate MATLAB session
    matlab::engine::terminateEngineClient();

	return 0;
}