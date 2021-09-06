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
    std::vector<double> test_error_z(NUM_TEST, 0.0);
    std::vector<double> test_error_nz(NUM_TEST, 0.0);

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

        matlab::data::TypedArray<double> in      = matlabPtr->getVariable(u"in");
        matlab::data::TypedArray<double> out_z   = matlabPtr->getVariable(u"out_z");
        matlab::data::TypedArray<double> out_nz  = matlabPtr->getVariable(u"out_nz");

        matlab::data::TypedArray<double> initial = matlabPtr->getVariable(u"initial_condition");
        std::vector<double> initial_state;
        for (auto i=0; i<initial.getNumberOfElements(); i++) {
            initial_state.push_back(initial[i]);
        }

        // Simulate filter in C++ and compare
        discrete_tf tfd_z(num_coeff, den_coeff);
        discrete_tf tfd_nz(num_coeff, den_coeff, initial_state);

        std::vector<double> error_z, error_nz;
        for (auto i=0; i<in.getNumberOfElements(); i++) {
            double tmp_out;

            tfd_z.evaluate(in[i], tmp_out);
            if (std::fabs(out_z[i])<1.0e-16) {
                if (std::fabs(tmp_out)<1.0e-16) {
                    error_z.push_back(0.0);
                }
                else {
                    error_z.push_back(100.0);
                }
            }
            else {
                error_z.push_back(std::fabs((out_z[i]-tmp_out)/out_z[i])*100.0);
            }

            tfd_nz.evaluate(in[i], tmp_out);
            if (std::fabs(out_nz[i])<1.0e-16) {
                if (std::fabs(tmp_out)<1.0e-16) {
                    error_nz.push_back(0.0);
                }
                else {
                    error_nz.push_back(100.0);
                }
            }
            else {
                error_nz.push_back(std::fabs((out_nz[i]-tmp_out)/out_nz[i])*100.0);
            }
        }

        test_error_z.at(k) = *std::max_element(error_z.begin(), error_z.end());
        test_error_nz.at(k) = *std::max_element(error_nz.begin(), error_nz.end());
    }
    std::cout << "Tests completed, plotting results" << std::endl;

    // Check NaN and Inf values
    if (count_all(test_error_z, is_nan)>0) {
        std::cout << "Test aborted, there are NaN values in the error (zero initial condition test)" << std::endl;
    }
    if (count_all(test_error_nz, is_nan)>0) {
        std::cout << "Test aborted, there are NaN values in the error (non-zero initial condition test)" << std::endl;
    }
    if (count_all(test_error_z, is_inf)>0) {
        std::cout << "Test aborted, there are Inf values in the error (zero initial condition test)" << std::endl;
    }
    if (count_all(test_error_nz, is_inf)>0) {
        std::cout << "Test aborted, there are Inf values in the error (non-zero initial condition test)" << std::endl;
    }

    // Plot test results
    matlab::data::ArrayFactory factory;
    matlab::data::TypedArray<double> m_error_z = factory.createArray({NUM_TEST,1}, test_error_z.begin(), test_error_z.end());
    matlab::data::TypedArray<double> m_error_nz = factory.createArray({NUM_TEST,1}, test_error_nz.begin(), test_error_nz.end());
    matlabPtr->eval(u"clear all; close all;");
    matlabPtr->setVariable(u"error_z", std::move(m_error_z));
    matlabPtr->setVariable(u"error_nz", std::move(m_error_nz));
    matlabPtr->eval(u"figure,bar(1:1:length(error_z),error_z),grid,xlabel('Test number'),ylabel('Percentage error'),title('Zero initial conditions')");
    matlabPtr->eval(u"figure,bar(1:1:length(error_nz),error_nz),grid,xlabel('Test number'),ylabel('Percentage error'),title('Non-zero initial conditions')");
    matlabPtr->eval(u"pause");

    // Terminate MATLAB session
    matlab::engine::terminateEngineClient();

	return 0;
}