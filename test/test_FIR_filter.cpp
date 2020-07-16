#include "MatlabDataArray.hpp"
#include "MatlabEngine.hpp"
#include <iostream>

#include "FIR_filter.h"

#define NUM_TEST 1000


int main() {
    using namespace matlab::engine;

    // Initialize tests
    std::vector<double> test_error(NUM_TEST, 0.0);

    // Start MATLAB engine synchronously
    std::unique_ptr<MATLABEngine> matlabPtr = startMATLAB();

    // Execute tests
    std::cout << "Executing " << NUM_TEST << " tests on FIR_filter class" << std::endl;
    for (auto k=0; k<NUM_TEST; k++) {
        // Simulate filter in Matlab
        matlabPtr->eval(u"test_FIR_filter;");

        // Get filter data from Matlab
        matlab::data::TypedArray<double> coeff = matlabPtr->getVariable(u"coeff");
        std::vector<double> coeffs;
        for (auto i=0; i<coeff.getNumberOfElements(); i++) {
            coeffs.push_back(coeff[i]);
        }

        matlab::data::TypedArray<double> in  = matlabPtr->getVariable(u"in");
        matlab::data::TypedArray<double> out = matlabPtr->getVariable(u"out");

        // Simulate filter in C++ and compare
        FIR_filter filter(coeffs);

        std::vector<double> error;
        for (auto i=0; i<in.getNumberOfElements(); i++) {
            double tmp_out;

            filter.evaluate(in[i], tmp_out);
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