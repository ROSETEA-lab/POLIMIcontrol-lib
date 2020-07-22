#include "MatlabDataArray.hpp"
#include "MatlabEngine.hpp"
#include <iostream>
#include <functional>
#include <algorithm>
#include <Eigen/Dense>

#include "discrete_ss.h"

#define NUM_TEST 1000


template <typename UnaryFunctionT /*= std::less<double>*/ >
int count_all(const std::vector<double>& vec, UnaryFunctionT func )
{
    return std::count_if( vec.begin(), vec.end(), func );
}
bool is_nan (double d) { return std::isnan(d); }
bool is_inf (double d) { return std::isinf(d); }

Eigen::MatrixXd A0, A1, B0, B1, C0, C1, D0, D1;

// Time-varying system matrix computation
void compute_matrix_A(Eigen::MatrixXd& A,int k) {
    A = A0+pow((double)k,0.5)*A1;
}
void compute_matrix_B(Eigen::MatrixXd& B,int k) {
    B = B0+pow((double)k,0.5)*B1;
}
void compute_matrix_C(Eigen::MatrixXd& C,int k) {
    C = C0+pow((double)k,0.5)*C1;
}
void compute_matrix_D(Eigen::MatrixXd& D,int k) {
    D = D0+pow((double)k,0.5)*D1;
}


int main() {
    using namespace matlab::engine;

    // Initialize tests
    std::vector<double> test_state_error(NUM_TEST, 0.0);
    std::vector<double> test_output_error(NUM_TEST, 0.0);

    // Start MATLAB engine synchronously
    std::unique_ptr<MATLABEngine> matlabPtr = startMATLAB();

    // Execute tests
    std::cout << "Executing " << NUM_TEST << " tests on discrete_ss_tv class" << std::endl;
    for (auto k=0; k<NUM_TEST; k++) {
        // Simulate filter in Matlab
        matlabPtr->eval(u"test_discrete_ss_tv;");

        // Get system data from Matlab
        matlab::data::TypedArray<double> n = matlabPtr->getVariable(u"n");
        matlab::data::TypedArray<double> m = matlabPtr->getVariable(u"m");
        matlab::data::TypedArray<double> p = matlabPtr->getVariable(u"p");

        A0 = Eigen::MatrixXd::Zero((int)n[0],(int)n[0]);
        matlab::data::TypedArray<double> m_A0 = matlabPtr->getVariable(u"A0");
        for (auto i=0; i<(int)n[0]; i++) {
            for (auto j=0; j<(int)n[0]; j++) {
                A0(i,j) = m_A0[i][j];
            }
        }
        A1 = Eigen::MatrixXd::Zero((int)n[0],(int)n[0]);
        matlab::data::TypedArray<double> m_A1 = matlabPtr->getVariable(u"A1");
        for (auto i=0; i<(int)n[0]; i++) {
            for (auto j=0; j<(int)n[0]; j++) {
                A1(i,j) = m_A1[i][j];
            }
        }

        B0 = Eigen::MatrixXd::Zero((int)n[0],(int)m[0]);
        matlab::data::TypedArray<double> m_B0 = matlabPtr->getVariable(u"B0");
        for (auto i=0; i<(int)n[0]; i++) {
            for (auto j=0; j<(int)m[0]; j++) {
                B0(i,j) = m_B0[i][j];
            }
        }
        B1 = Eigen::MatrixXd::Zero((int)n[0],(int)m[0]);
        matlab::data::TypedArray<double> m_B1 = matlabPtr->getVariable(u"B1");
        for (auto i=0; i<(int)n[0]; i++) {
            for (auto j=0; j<(int)m[0]; j++) {
                B1(i,j) = m_B1[i][j];
            }
        }

        C0 = Eigen::MatrixXd::Zero((int)p[0],(int)n[0]);
        matlab::data::TypedArray<double> m_C0 = matlabPtr->getVariable(u"C0");
        for (auto i=0; i<(int)p[0]; i++) {
            for (auto j=0; j<(int)n[0]; j++) {
                C0(i,j) = m_C0[i][j];
            }
        }
        C1 = Eigen::MatrixXd::Zero((int)p[0],(int)n[0]);
        matlab::data::TypedArray<double> m_C1 = matlabPtr->getVariable(u"C1");
        for (auto i=0; i<(int)p[0]; i++) {
            for (auto j=0; j<(int)n[0]; j++) {
                C1(i,j) = m_C1[i][j];
            }
        }

        D0 = Eigen::MatrixXd::Zero((int)p[0],(int)m[0]);
        matlab::data::TypedArray<double> m_D0 = matlabPtr->getVariable(u"D0");
        for (auto i=0; i<(int)p[0]; i++) {
            for (auto j=0; j<(int)m[0]; j++) {
                D0(i,j) = m_D0[i][j];
            }
        }
        D1 = Eigen::MatrixXd::Zero((int)p[0],(int)m[0]);
        matlab::data::TypedArray<double> m_D1 = matlabPtr->getVariable(u"D1");
        for (auto i=0; i<(int)p[0]; i++) {
            for (auto j=0; j<(int)m[0]; j++) {
                D1(i,j) = m_D1[i][j];
            }
        }

        Eigen::VectorXd initial_state = Eigen::VectorXd::Zero((int)n[0]);
        matlab::data::TypedArray<double> m_initial_state = matlabPtr->getVariable(u"initial_state");
        for (auto i=0; i<(int)n[0]; i++) {
            initial_state(i) = m_initial_state[i];
        }

        matlab::data::TypedArray<double> m_in = matlabPtr->getVariable(u"in");
        matlab::data::ArrayDimensions m_in_size = m_in.getDimensions();
        Eigen::MatrixXd input = Eigen::MatrixXd::Zero((int)m[0],m_in_size.at(0));
        for (auto i=0; i<m_in_size.at(0); i++) {
            for (auto j=0; j<(int)m[0]; j++) {
                input(j,i) = m_in[i][j];
            }
        }

        matlab::data::TypedArray<double> m_output = matlabPtr->getVariable(u"output");
        matlab::data::TypedArray<double> m_state  = matlabPtr->getVariable(u"state");

        // Simulate system in C++ and compare
        discrete_ss ssd(compute_matrix_A, compute_matrix_B, compute_matrix_C, compute_matrix_D, initial_state);

        Eigen::VectorXd output, state;
        std::vector<double> state_error, output_error;

        for (auto i=0; i<input.cols(); i++) {
            Eigen::VectorXd tmp_in((int)m[0]);
            for (auto k=0; k<(int)m[0]; k++) {
                tmp_in(k) = input(k,i);
            }

            ssd.evaluate(tmp_in);
            ssd.get_state(state);
            ssd.get_output(output);

            double state_error_norm = 0.0;
            for (auto k=0; k<(int)n[0]; k++) {
                state_error_norm += std::pow(state(k)-m_state[i][k],2);
            }
            state_error.push_back(std::sqrt(state_error_norm));

            double output_error_norm = 0.0;
            for (auto k=0; k<(int)p[0]; k++) {
                output_error_norm += std::pow(output(k)-m_output[i][k],2);
            }
            output_error.push_back(std::sqrt(output_error_norm));
        }

        test_state_error.at(k) = *std::max_element(state_error.begin(), state_error.end());
        test_output_error.at(k) = *std::max_element(output_error.begin(), output_error.end());
    }
    std::cout << "Tests completed, plotting results" << std::endl;

    // Check NaN and Inf values
    if ((count_all(test_state_error, is_nan)>0) || (count_all(test_output_error, is_nan)>0)) {
        std::cout << "Test aborted, there are NaN values in the errors" << std::endl;
    }
    if ((count_all(test_state_error, is_inf)>0) || (count_all(test_output_error, is_inf)>0)) {
        std::cout << "Test aborted, there are Inf values in the errors" << std::endl;
    }

    // Plot test results
    matlab::data::ArrayFactory factory;
    matlab::data::TypedArray<double> m_state_error = factory.createArray({NUM_TEST,1}, test_state_error.begin(), test_state_error.end());
    matlab::data::TypedArray<double> m_output_error = factory.createArray({NUM_TEST,1}, test_output_error.begin(), test_output_error.end());
    matlabPtr->eval(u"clear all; close all;");
    matlabPtr->setVariable(u"state_error", std::move(m_state_error));
    matlabPtr->setVariable(u"output_error", std::move(m_output_error));
    matlabPtr->eval(u"figure,bar(1:1:length(state_error),state_error),grid");
    matlabPtr->eval(u"figure,bar(1:1:length(output_error),output_error),grid");
    matlabPtr->eval(u"pause");

    // Terminate MATLAB session
    matlab::engine::terminateEngineClient();

	return 0;
}