#include "MatlabDataArray.hpp"
#include "MatlabEngine.hpp"
#include <iostream>
#include <functional>
#include <algorithm>

#include "discrete_derivative.h"

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
    std::cout << "Executing " << NUM_TEST << " tests on discrete_derivative class" << std::endl;
    for (auto k=0; k<NUM_TEST; k++) {
        // Simulate integrator in Matlab
        matlabPtr->eval(u"test_discrete_derivative;");

        // Get integrator data from Matlab
        matlab::data::TypedArray<double> K  = matlabPtr->getVariable(u"K");
        matlab::data::TypedArray<double> Ts = matlabPtr->getVariable(u"Ts");
        double gain = K[0];
        double sampling_time = Ts[0];

        matlab::data::TypedArray<double> in  = matlabPtr->getVariable(u"in");
        matlab::data::TypedArray<double> out = matlabPtr->getVariable(u"out");

        // Simulate filter in C++ and compare
        discrete_derivative der(gain, sampling_time);

        std::vector<double> error;
        for (auto i=0; i<in.getNumberOfElements(); i++) {
            double tmp_out;

            der.evaluate(in[i], tmp_out);
            if (std::fabs(out[i])<1.0e-16) {
                if (std::fabs(tmp_out)<1.0e-16) {
                    error.push_back(0.0);
                }
                else {
                    error.push_back(100.0);
                }
            }
            else {
                error.push_back(std::fabs((out[i]-tmp_out)/out[i])*100.0);
            }
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
    matlabPtr->eval(u"figure,bar(1:1:length(error),error),grid,xlabel('Test number'),ylabel('Percentage error')");
    matlabPtr->eval(u"pause");

    // Terminate MATLAB session
    matlab::engine::terminateEngineClient();

	return 0;
}