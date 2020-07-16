#include "MatlabDataArray.hpp"
#include "MatlabEngine.hpp"
#include <iostream>

#include "PID_controller.h"

#define NUM_TEST 1000


int main() {
    using namespace matlab::engine;

    // Initialize tests
    std::vector<double> test_error(NUM_TEST, 0.0);

    // Start MATLAB engine synchronously
    std::unique_ptr<MATLABEngine> matlabPtr = startMATLAB();

    // Execute tests
    std::cout << "Executing " << NUM_TEST << " tests on PID_controller class" << std::endl;
    for (auto k=0; k<NUM_TEST; k++) {
        // Simulate filter in Matlab
        matlabPtr->eval(u"test_PID_controller;");

        // Get filter data from Matlab
        matlab::data::TypedArray<double> PID_param = matlabPtr->getVariable(u"PID_param");
        matlab::data::TypedArray<double> in  = matlabPtr->getVariable(u"in");
        matlab::data::TypedArray<double> out = matlabPtr->getVariable(u"out");

        // Simulate filter in C++ and compare
        PID_controller PID(PID_param[0], PID_param[1], PID_param[2], PID_param[3], PID_param[4], PID_param[5], PID_param[6]);

        std::vector<double> error;
        for (auto i=0; i<in.getNumberOfElements(); i++) {
            double tmp_out;

            PID.evaluate(0.0, in[i], tmp_out);
            error.push_back(std::fabs(out[i]-tmp_out));
        }

        test_error.at(k) = *std::max_element(error.begin(), error.end());
    }
    std::cout << "Tests completed, plotting results" << std::endl;

    // Plot test results
    matlab::data::ArrayFactory factory;
    matlab::data::TypedArray<double> m_error = factory.createArray({NUM_TEST,1}, test_error.begin(), test_error.end());
    matlabPtr->eval(u"clear all; close all;");
    matlabPtr->setVariable(u"error", std::move(m_error));
    matlabPtr->eval(u"figure,bar(1:1:length(error),error),grid");
    matlabPtr->eval(u"pause");

	return 0;
}