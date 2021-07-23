#include "MatlabDataArray.hpp"
#include "MatlabEngine.hpp"
#include <iostream>
#include <functional>
#include <algorithm>
#include <Eigen/Dense>

#include "continuous_ss.h"


int main(int argc, char** argv) {
    using namespace matlab::engine;

    // Start MATLAB engine synchronously
    std::unique_ptr<MATLABEngine> matlabPtr = startMATLAB();

    // Load test data
    std::string matlabLine;
    matlabLine = "load('" + std::string(argv[1]) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));

    // Get system data from Matlab
    matlab::data::TypedArray<double> n = matlabPtr->getVariable(u"n");
    matlab::data::TypedArray<double> m = matlabPtr->getVariable(u"m");
    matlab::data::TypedArray<double> p = matlabPtr->getVariable(u"p");
    matlab::data::TypedArray<double> sampling_time = matlabPtr->getVariable(u"Ts");

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero((int)n[0],(int)n[0]);
    matlab::data::TypedArray<double> m_A = matlabPtr->getVariable(u"A");
    for (auto i=0; i<(int)n[0]; i++) {
        for (auto j=0; j<(int)n[0]; j++) {
            A(i,j) = m_A[i][j];
        }
    }

    Eigen::MatrixXd B = Eigen::MatrixXd::Zero((int)n[0],(int)m[0]);
    matlab::data::TypedArray<double> m_B = matlabPtr->getVariable(u"B");
    for (auto i=0; i<(int)n[0]; i++) {
        for (auto j=0; j<(int)m[0]; j++) {
            B(i,j) = m_B[i][j];
        }
    }

    Eigen::MatrixXd C = Eigen::MatrixXd::Zero((int)p[0],(int)n[0]);
    matlab::data::TypedArray<double> m_C = matlabPtr->getVariable(u"C");
    for (auto i=0; i<(int)p[0]; i++) {
        for (auto j=0; j<(int)n[0]; j++) {
            C(i,j) = m_C[i][j];
        }
    }

    Eigen::MatrixXd D = Eigen::MatrixXd::Zero((int)p[0],(int)m[0]);
    matlab::data::TypedArray<double> m_D = matlabPtr->getVariable(u"D");
    for (auto i=0; i<(int)p[0]; i++) {
        for (auto j=0; j<(int)m[0]; j++) {
            D(i,j) = m_D[i][j];
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
    continuous_ss ssc(A, B, C, D, (double)sampling_time[0], initial_state);

    Eigen::VectorXd output, state;
    std::vector<double> state_error, output_error;
    std::vector<std::vector<double> > state_vect((int)n[0], std::vector<double>(input.cols()));
    std::vector<std::vector<double> > output_vect((int)p[0], std::vector<double>(input.cols()));

    bool nan_flag = false;
    for (auto i=0; i<input.cols(); i++) {
        ssc.evaluate(input.col(i));
        ssc.get_state(state);
        ssc.get_output(output);

        double state_error_norm = 0.0;
        for (auto ss=0; ss<(int)n[0]; ss++) {
            if (std::isnan(m_state[i][ss])) {
                nan_flag = true;
                std::cout << "At time " << i << " Matlab state is NaN!" << std::endl;
            }
            if (std::isnan(state(ss))) {
                nan_flag = true;
                std::cout << "At time " << i << " C++ state is NaN!" << std::endl;
            }

            if (nan_flag) {
                break;
            }
            else {
                state_vect.at(ss).at(i) = state(ss);
                state_error_norm += std::sqrt(std::pow((state(ss)-m_state[i][ss])/m_state[i][ss],2))*(100.0/(input.cols()*(int)n[0]));
            }
        }

        double output_error_norm = 0.0;
        for (auto out=0; out<(int)p[0]; out++) {
            if (std::isnan(m_output[i][out])) {
                nan_flag = true;
                std::cout << "At time " << i << " Matlab output is NaN!" << std::endl;
            }
            if (std::isnan(output(out))) {
                nan_flag = true;
                std::cout << "At time " << i << " C++ output is NaN!" << std::endl;
            }

            if (nan_flag) {
                break;
            }
            else {
                output_vect.at(out).at(i) = output(out);
                output_error_norm += std::sqrt(std::pow((output(out)-m_output[i][out])/m_output[i][out],2))*(100.0/(input.cols()*(int)p[0]));
            }
        }

        if (nan_flag) {
            break;
        }
        else {
            state_error.push_back(state_error_norm);
            output_error.push_back(output_error_norm);
        }
    }

    // Plot a comparison between cpp and Matlab state/output
    for (auto i=0; i<(int)n[0]; i++) {
        // Transfer data to Matlab
        matlab::data::ArrayFactory factory;
        matlab::data::TypedArray<double> m_state_cpp= factory.createArray({1,(long unsigned int)input.cols()}, state_vect.at(i).begin(), state_vect.at(i).end());
        matlabPtr->setVariable(u"state_cpp", std::move(m_state_cpp));

        // Plot comparison between cpp and Matlab results
        std::string matlabLine;

        matlabPtr->eval(u"figure;");
        matlabLine = "subplot(2,1,1),plot(t,state(:," + std::to_string(i+1) + "), t,state_cpp,'r--'),grid,title('State " + std::to_string(i+1) + "'),legend('matlab','cpp')";
        matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
        matlabLine = "subplot(2,1,2),plot(t,abs(state(:," + std::to_string(i+1) + ")-state_cpp')),grid";
        matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    }

    for (auto i=0; i<(int)p[0]; i++) {
        // Transfer data to Matlab
        matlab::data::ArrayFactory factory;
        matlab::data::TypedArray<double> m_output_cpp = factory.createArray({1,(long unsigned int)input.cols()}, output_vect.at(i).begin(), output_vect.at(i).end());
        matlabPtr->setVariable(u"output_cpp", std::move(m_output_cpp));

        // Plot comparison between cpp and Matlab results
        std::string matlabLine;

        matlabPtr->eval(u"figure;");
        matlabLine = "subplot(2,1,1),plot(t,output(:," + std::to_string(i+1) + "), t,output_cpp,'r--'),grid,title('Output " + std::to_string(i+1) + "'),legend('matlab','cpp')";
        matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
        matlabLine = "subplot(2,1,2),plot(t,abs(output(:," + std::to_string(i+1) + ")-output_cpp')),grid";
        matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    }

    // Skip this test in case of NaN or record the state and output errors
    if (nan_flag) {
        std::cout << "Test aborted, due to NaN values!" << std::endl << std::endl;
    }
    else {
        std::cout << "Tests completed, maximum state error " << *std::max_element(state_error.begin(), state_error.end()) << ", maximum output error " << *std::max_element(output_error.begin(), output_error.end()) << std::endl;
    }

    // Terminate MATLAB session
    matlab::engine::terminateEngineClient();

	return 0;
}
