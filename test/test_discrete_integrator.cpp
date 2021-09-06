#include "MatlabDataArray.hpp"
#include "MatlabEngine.hpp"
#include <iostream>
#include <functional>
#include <algorithm>

#include "discrete_integrator.h"

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
    std::vector<double> test_error_fwEul(NUM_TEST, 0.0);
    std::vector<double> test_error_bwEul(NUM_TEST, 0.0);
    std::vector<double> test_error_trapz(NUM_TEST, 0.0);

    // Start MATLAB engine synchronously
    std::unique_ptr<MATLABEngine> matlabPtr = startMATLAB();

    // Execute tests
    std::cout << "Executing " << NUM_TEST << " tests on discrete_integrator class" << std::endl;
    for (auto k=0; k<NUM_TEST; k++) {
        // Simulate integrator in Matlab
        matlabPtr->eval(u"test_discrete_integrator;");

        // Get integrator data from Matlab
        matlab::data::TypedArray<double> K  = matlabPtr->getVariable(u"K");
        matlab::data::TypedArray<double> Ts = matlabPtr->getVariable(u"Ts");
        double gain = K[0];
        double sampling_time = Ts[0];

        matlab::data::TypedArray<double> initial = matlabPtr->getVariable(u"initial_condition");
        double initial_condition = initial[0];

        matlab::data::TypedArray<double> in        = matlabPtr->getVariable(u"in");
        matlab::data::TypedArray<double> out_fwEul = matlabPtr->getVariable(u"out_fwEul");
        matlab::data::TypedArray<double> out_bwEul = matlabPtr->getVariable(u"out_bwEul");
        matlab::data::TypedArray<double> out_trapz = matlabPtr->getVariable(u"out_trapz");

        // Simulate filter in C++ and compare
        discrete_integrator_fwEul fwEul(gain, sampling_time, initial_condition);
        discrete_integrator_bwEul bwEul(gain, sampling_time, initial_condition);
        discrete_integrator_trapz trapz(gain, sampling_time, initial_condition);

        std::vector<double> error_fwEul, error_bwEul, error_trapz;
        for (auto i=0; i<in.getNumberOfElements(); i++) {
            double tmp_out;

            fwEul.evaluate(in[i], tmp_out);
            if (std::fabs(out_fwEul[i])<1.0e-16) {
                if (std::fabs(tmp_out)<1.0e-16) {
                    error_fwEul.push_back(0.0);
                }
                else {
                    error_fwEul.push_back(100.0);
                }
            }
            else {
                error_fwEul.push_back(std::fabs((out_fwEul[i]-tmp_out)/out_fwEul[i])*100.0);
            }

            bwEul.evaluate(in[i], tmp_out);
            if (std::fabs(out_bwEul[i])<1.0e-16) {
                if (std::fabs(tmp_out)<1.0e-16) {
                    error_bwEul.push_back(0.0);
                }
                else {
                    error_bwEul.push_back(100.0);
                }
            }
            else {
                error_bwEul.push_back(std::fabs((out_bwEul[i]-tmp_out)/out_bwEul[i])*100.0);
            }

            trapz.evaluate(in[i], tmp_out);
            if (std::fabs(out_trapz[i])<1.0e-16) {
                if (std::fabs(tmp_out)<1.0e-16) {
                    error_trapz.push_back(0.0);
                }
                else {
                    error_trapz.push_back(100.0);
                }
            }
            else {
                error_trapz.push_back(std::fabs((out_trapz[i]-tmp_out)/out_trapz[i])*100.0);
            }
        }

        test_error_fwEul.at(k) = *std::max_element(error_fwEul.begin(), error_fwEul.end());
        test_error_bwEul.at(k) = *std::max_element(error_bwEul.begin(), error_bwEul.end());
        test_error_trapz.at(k) = *std::max_element(error_trapz.begin(), error_trapz.end());
    }
    std::cout << "Tests completed, plotting results" << std::endl;

    // Check NaN and Inf values
    if (count_all(test_error_fwEul, is_nan)>0) {
        std::cout << "Test aborted, there are NaN values in the error (forward euler test)" << std::endl;
    }
    if (count_all(test_error_bwEul, is_nan)>0) {
        std::cout << "Test aborted, there are NaN values in the error (backward euler test)" << std::endl;
    }
    if (count_all(test_error_trapz, is_nan)>0) {
        std::cout << "Test aborted, there are NaN values in the error (trapezoidal test)" << std::endl;
    }
    if (count_all(test_error_fwEul, is_inf)>0) {
        std::cout << "Test aborted, there are Inf values in the error (forward euler test)" << std::endl;
    }
    if (count_all(test_error_bwEul, is_inf)>0) {
        std::cout << "Test aborted, there are Inf values in the error (backward euler test)" << std::endl;
    }
    if (count_all(test_error_trapz, is_inf)>0) {
        std::cout << "Test aborted, there are Inf values in the error (trapezoidal test)" << std::endl;
    }

    // Plot test results
    matlab::data::ArrayFactory factory;
    matlab::data::TypedArray<double> m_error_fwEul = factory.createArray({NUM_TEST,1}, test_error_fwEul.begin(), test_error_fwEul.end());
    matlab::data::TypedArray<double> m_error_bwEul = factory.createArray({NUM_TEST,1}, test_error_bwEul.begin(), test_error_bwEul.end());
    matlab::data::TypedArray<double> m_error_trapz = factory.createArray({NUM_TEST,1}, test_error_trapz.begin(), test_error_trapz.end());
    matlabPtr->eval(u"clear all; close all;");
    matlabPtr->setVariable(u"error_fwEul", std::move(m_error_fwEul));
    matlabPtr->setVariable(u"error_bwEul", std::move(m_error_bwEul));
    matlabPtr->setVariable(u"error_trapz", std::move(m_error_trapz));
    matlabPtr->eval(u"figure,bar(1:1:length(error_fwEul),error_fwEul),grid,xlabel('Test number'),ylabel('Percentage error'),title('Forward Euler')");
    matlabPtr->eval(u"figure,bar(1:1:length(error_bwEul),error_bwEul),grid,xlabel('Test number'),ylabel('Percentage error'),title('Backward Euler')");
    matlabPtr->eval(u"figure,bar(1:1:length(error_trapz),error_trapz),grid,xlabel('Test number'),ylabel('Percentage error'),title('Trapezoidal')");
    matlabPtr->eval(u"pause");

    // Terminate MATLAB session
    matlab::engine::terminateEngineClient();

	return 0;
}