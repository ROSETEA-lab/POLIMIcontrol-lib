#include "MatlabDataArray.hpp"
#include "MatlabEngine.hpp"
#include <iostream>
#include <functional>
#include <algorithm>

#include "PIDISA_controller.h"

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
    std::cout << "Executing " << NUM_TEST << " tests on PIDISA_controller class" << std::endl;
    for (auto k=0; k<NUM_TEST; k++) {
        // Simulate filter in Matlab
        matlabPtr->eval(u"test_PIDISA_controller;");

        // Get filter data from Matlab
        matlab::data::TypedArray<double> PID_param = matlabPtr->getVariable(u"PID_param");
        matlab::data::TypedArray<double> ysp = matlabPtr->getVariable(u"ysp");
        matlab::data::TypedArray<double> y   = matlabPtr->getVariable(u"y");
        matlab::data::TypedArray<double> out = matlabPtr->getVariable(u"out");

        // Simulate filter in C++ and compare
        PIDISA_controller PID(PID_param[0], PID_param[4], PID_param[1], PID_param[2], PID_param[3], PID_param[5], PID_param[6], PID_param[7], PID_param[8]);

        std::vector<double> error;
        for (auto i=0; i<y.getNumberOfElements(); i++) {
            double tmp_out;

            PID.evaluate(y[i], ysp[i], 0.0, tmp_out);
            error.push_back(std::fabs(out[i]-tmp_out));
        }

        test_error.at(k) = *std::max_element(error.begin(), error.end());

        if (std::abs(test_error.at(k))>0.1) {
            std::cout << "Test " << k << " with error " << test_error.at(k) << " - Kc=" << PID_param[0] <<
            " Ti=" << PID_param[1] << " Td=" << PID_param[2] << " N=" << PID_param[3] << " b=" << PID_param[4] << " c=" << PID_param[5] << std::endl;
        }
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
