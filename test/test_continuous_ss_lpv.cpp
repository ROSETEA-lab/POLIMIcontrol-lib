#include "MatlabDataArray.hpp"
#include "MatlabEngine.hpp"
#include <iostream>
#include <functional>
#include <algorithm>
#include <Eigen/Dense>

#include "continuous_ss.h"

#define NUM_TEST        1000
#define STATE_ERR_THD   5
#define OUTPUT_ERR_THD  5


template <typename UnaryFunctionT /*= std::less<double>*/ >
int count_all(const std::vector<double>& vec, UnaryFunctionT func )
{
    return std::count_if( vec.begin(), vec.end(), func );
}
bool is_nan (double d) { return std::isnan(d); }
bool is_inf (double d) { return std::isinf(d); }


// Derive a new class specifing time-varying system matrix computation
class continuous_ss_lpv : public continuous_ss {
private:
    Eigen::MatrixXd A0, B0, C0, D0, A1, B1, C1, D1;
    std::vector<Eigen::MatrixXd> Ap, Bp, Cp, Dp;

    void compute_state_matrices(Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::MatrixXd& C, Eigen::MatrixXd& D, double time, const Eigen::VectorXd& param)
    {
        A = A0+pow(time,0.5)*A1;

        for (auto i=0; i<param.size(); i++) {
            A += Ap.at(i)*param(i);
        }

        B = B0+pow(time,0.5)*B1;

        for (auto i=0; i<param.size(); i++) {
            B += Bp.at(i)*param(i);
        }

        C = C0+pow(time,0.5)*C1;

        for (auto i=0; i<param.size(); i++) {
            C += Cp.at(i)*param(i);
        }

        D = D0+pow(time,0.5)*D1;

        for (auto i=0; i<param.size(); i++) {
            D += Dp.at(i)*param(i);
        }
    }

public:
    continuous_ss_lpv(double sampling_time, int num_param, const Eigen::VectorXd& initial_state) :
        continuous_ss(sampling_time, num_param, initial_state) {};

    void set_base_matrix(const Eigen::MatrixXd& A0, const Eigen::MatrixXd& A1, const Eigen::MatrixXd& B0, const Eigen::MatrixXd& B1,
                         const Eigen::MatrixXd& C0, const Eigen::MatrixXd& C1, const Eigen::MatrixXd& D0, const Eigen::MatrixXd& D1,
                         const std::vector<Eigen::MatrixXd>& Ap, const std::vector<Eigen::MatrixXd>& Bp, const std::vector<Eigen::MatrixXd>& Cp, const std::vector<Eigen::MatrixXd>& Dp)
    {
        this->A0 = A0; this->A1 = A1;
        this->B0 = B0; this->B1 = B1;
        this->C0 = C0; this->C1 = C1;
        this->D0 = D0; this->D1 = D1;
        this->Ap = Ap; this->Bp = Bp; this->Cp = Cp; this->Dp = Dp;
    }
};


int main() {
    using namespace matlab::engine;

    // Initialize tests
    std::vector<double> test_state_error(NUM_TEST, 0.0);
    std::vector<double> test_output_error(NUM_TEST, 0.0);

    // Start MATLAB engine synchronously
    std::vector<String> optionVec;
    optionVec.push_back(u"-nodesktop -nosplash");
    std::unique_ptr<MATLABEngine> matlabPtr = startMATLAB(optionVec);

    // Execute tests
    std::cout << "Executing " << NUM_TEST << " tests on continuous_ss_lpv class" << std::endl;
    for (auto k=0; k<NUM_TEST; k++) {
        // Simulate system in Matlab
        matlabPtr->eval(u"test_continuous_ss_lpv;");

        // Get system data from Matlab
        matlab::data::TypedArray<double> n = matlabPtr->getVariable(u"n");
        matlab::data::TypedArray<double> m = matlabPtr->getVariable(u"m");
        matlab::data::TypedArray<double> p = matlabPtr->getVariable(u"p");
        matlab::data::TypedArray<double> sampling_time = matlabPtr->getVariable(u"Ts");
        matlab::data::TypedArray<double> num_param = matlabPtr->getVariable(u"num_param");

        Eigen::MatrixXd A0 = Eigen::MatrixXd::Zero((int)n[0],(int)n[0]);
        matlab::data::TypedArray<double> m_A0 = matlabPtr->getVariable(u"A0");
        for (auto i=0; i<(int)n[0]; i++) {
            for (auto j=0; j<(int)n[0]; j++) {
                A0(i,j) = m_A0[i][j];
            }
        }
        Eigen::MatrixXd A1 = Eigen::MatrixXd::Zero((int)n[0],(int)n[0]);
        matlab::data::TypedArray<double> m_A1 = matlabPtr->getVariable(u"A1");
        for (auto i=0; i<(int)n[0]; i++) {
            for (auto j=0; j<(int)n[0]; j++) {
                A1(i,j) = m_A1[i][j];
            }
        }
        std::vector<Eigen::MatrixXd> Ap;
        matlab::data::TypedArray<double> m_Ap = matlabPtr->getVariable(u"Ap");
        for (auto pm=0; pm<(int)num_param[0]; pm++) {
            Eigen::MatrixXd Apk = Eigen::MatrixXd::Zero((int)n[0],(int)n[0]);
            for (auto i=0; i<(int)n[0]; i++) {
                for (auto j=0; j<(int)n[0]; j++) {
                    Apk(i,j) = m_Ap[i][j][pm];
                }
            }
            Ap.push_back(Apk);
        }

        Eigen::MatrixXd B0 = Eigen::MatrixXd::Zero((int)n[0],(int)m[0]);
        matlab::data::TypedArray<double> m_B0 = matlabPtr->getVariable(u"B0");
        for (auto i=0; i<(int)n[0]; i++) {
            for (auto j=0; j<(int)m[0]; j++) {
                B0(i,j) = m_B0[i][j];
            }
        }
        Eigen::MatrixXd B1 = Eigen::MatrixXd::Zero((int)n[0],(int)m[0]);
        matlab::data::TypedArray<double> m_B1 = matlabPtr->getVariable(u"B1");
        for (auto i=0; i<(int)n[0]; i++) {
            for (auto j=0; j<(int)m[0]; j++) {
                B1(i,j) = m_B1[i][j];
            }
        }
        std::vector<Eigen::MatrixXd> Bp;
        matlab::data::TypedArray<double> m_Bp = matlabPtr->getVariable(u"Bp");
        for (auto pm=0; pm<(int)num_param[0]; pm++) {
            Eigen::MatrixXd Bpk = Eigen::MatrixXd::Zero((int)n[0],(int)m[0]);
            for (auto i=0; i<(int)n[0]; i++) {
                for (auto j=0; j<(int)m[0]; j++) {
                    Bpk(i,j) = m_Bp[i][j][pm];
                }
            }
            Bp.push_back(Bpk);
        }

        Eigen::MatrixXd C0 = Eigen::MatrixXd::Zero((int)p[0],(int)n[0]);
        matlab::data::TypedArray<double> m_C0 = matlabPtr->getVariable(u"C0");
        for (auto i=0; i<(int)p[0]; i++) {
            for (auto j=0; j<(int)n[0]; j++) {
                C0(i,j) = m_C0[i][j];
            }
        }
        Eigen::MatrixXd C1 = Eigen::MatrixXd::Zero((int)p[0],(int)n[0]);
        matlab::data::TypedArray<double> m_C1 = matlabPtr->getVariable(u"C1");
        for (auto i=0; i<(int)p[0]; i++) {
            for (auto j=0; j<(int)n[0]; j++) {
                C1(i,j) = m_C1[i][j];
            }
        }
        std::vector<Eigen::MatrixXd> Cp;
        matlab::data::TypedArray<double> m_Cp = matlabPtr->getVariable(u"Cp");
        for (auto pm=0; pm<(int)num_param[0]; pm++) {
            Eigen::MatrixXd Cpk = Eigen::MatrixXd::Zero((int)p[0],(int)n[0]);
            for (auto i=0; i<(int)p[0]; i++) {
                for (auto j=0; j<(int)n[0]; j++) {
                    Cpk(i,j) = m_Cp[i][j][pm];
                }
            }
            Cp.push_back(Cpk);
        }

        Eigen::MatrixXd D0 = Eigen::MatrixXd::Zero((int)p[0],(int)m[0]);
        matlab::data::TypedArray<double> m_D0 = matlabPtr->getVariable(u"D0");
        for (auto i=0; i<(int)p[0]; i++) {
            for (auto j=0; j<(int)m[0]; j++) {
                D0(i,j) = m_D0[i][j];
            }
        }
        Eigen::MatrixXd D1 = Eigen::MatrixXd::Zero((int)p[0],(int)m[0]);
        matlab::data::TypedArray<double> m_D1 = matlabPtr->getVariable(u"D1");
        for (auto i=0; i<(int)p[0]; i++) {
            for (auto j=0; j<(int)m[0]; j++) {
                D1(i,j) = m_D1[i][j];
            }
        }
        std::vector<Eigen::MatrixXd> Dp;
        matlab::data::TypedArray<double> m_Dp = matlabPtr->getVariable(u"Dp");
        for (auto pm=0; pm<(int)num_param[0]; pm++) {
            Eigen::MatrixXd Dpk = Eigen::MatrixXd::Zero((int)p[0],(int)m[0]);
            for (auto i=0; i<(int)p[0]; i++) {
                for (auto j=0; j<(int)m[0]; j++) {
                    Dpk(i,j) = m_Dp[i][j][pm];
                }
            }
            Dp.push_back(Dpk);
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

        matlab::data::TypedArray<double> m_param = matlabPtr->getVariable(u"param");
        matlab::data::ArrayDimensions m_param_size = m_param.getDimensions();
        Eigen::MatrixXd param = Eigen::MatrixXd::Zero((int)num_param[0],m_param_size.at(0));
        for (auto i=0; i<m_param_size.at(0); i++) {
            for (auto j=0; j<(int)num_param[0]; j++) {
                param(j,i) = m_param[i][j];
            }
        }

        matlab::data::TypedArray<double> m_output = matlabPtr->getVariable(u"output");
        matlab::data::TypedArray<double> m_state  = matlabPtr->getVariable(u"state");

        // Simulate system in C++ and compare
        continuous_ss_lpv ssc((double)sampling_time[0], (int)num_param[0], initial_state);
        ssc.set_base_matrix(A0, A1, B0, B1, C0, C1, D0, D1, Ap, Bp, Cp, Dp);

        Eigen::VectorXd output, state;
        std::vector<double> state_error, output_error;
        std::vector<std::vector<double> > state_vect((int)n[0], std::vector<double>(input.cols()));
        std::vector<std::vector<double> > output_vect((int)p[0], std::vector<double>(input.cols()));

        bool nan_flag = false;
        for (auto i=0; i<input.cols(); i++) {
            ssc.evaluate(input.col(i),param.col(i));
            ssc.get_state(state);
            ssc.get_output(output);

            double state_error_norm = 0.0;
            for (auto ss=0; ss<(int)n[0]; ss++) {
                if (std::isnan(m_state[i][ss])) {
                    nan_flag = true;
                    std::cout << "Test " << k << ": at time " << i << " Matlab state is NaN!" << std::endl;
                }
                if (std::isnan(state(ss))) {
                    nan_flag = true;
                    std::cout << "Test " << k << ": at time " << i << " C++ state is NaN!" << std::endl;
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
                    std::cout << "Test " << k << ": at time " << i << " Matlab output is NaN!" << std::endl;
                }
                if (std::isnan(output(out))) {
                    nan_flag = true;
                    std::cout << "Test " << k << ": at time " << i << " C++ output is NaN!" << std::endl;
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

        // In case a single test is performed, plot a comparison between cpp and Matlab state/output
        if (NUM_TEST==1) {
            for (auto i=0; i<(int)n[0]; i++) {
                // Transfer data to Matlab
                matlab::data::ArrayFactory factory;
                matlab::data::TypedArray<double> m_state_cpp = factory.createArray({1,(long unsigned int)input.cols()}, state_vect.at(i).begin(), state_vect.at(i).end());
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
        }

        // Skip this test in case of NaN or record the state and output errors
        if (nan_flag) {
            std::cout << "Test " << k << " aborted, due to NaN values!" << std::endl << std::endl;

            test_state_error.at(k)  = 0.0;
            test_output_error.at(k) = 0.0;
        }
        else {
            test_state_error.at(k)  = *std::max_element(state_error.begin(), state_error.end());
            test_output_error.at(k) = *std::max_element(output_error.begin(), output_error.end());

            // In case the error exceeds the threshold, save in a mat file all the data
            if ((test_state_error.at(k)>STATE_ERR_THD) || (test_output_error.at(k)>OUTPUT_ERR_THD)) {
                std::string matlabLine;
                matlabLine = "save('test_continuous_ss_lpv_" + std::to_string(k+1) + ".mat','n','m','p','num_param','A0','A1','Ap','B0','B1','Bp','C0','C1','Cp','D0','D1','Dp','initial_state','t','in','state','output','param');";
                matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
            }
        }
    }
    std::cout << "Tests completed, plotting results" << std::endl;

    // Check NaN and Inf values
    if ((count_all(test_state_error, is_inf)>0) || (count_all(test_output_error, is_inf)>0)) {
        std::cout << "Test aborted, there are Inf values in the errors" << std::endl;
    }

    // Plot test results
    if (NUM_TEST>1) {
        matlab::data::ArrayFactory factory;
        matlab::data::TypedArray<double> m_state_error = factory.createArray({NUM_TEST,1}, test_state_error.begin(), test_state_error.end());
        matlab::data::TypedArray<double> m_output_error = factory.createArray({NUM_TEST,1}, test_output_error.begin(), test_output_error.end());
        matlabPtr->eval(u"clear all; close all;");
        matlabPtr->setVariable(u"state_error", std::move(m_state_error));
        matlabPtr->setVariable(u"output_error", std::move(m_output_error));
        matlabPtr->eval(u"figure('visible', 'off'),bar(1:1:length(state_error),state_error),grid,xlabel('Test number'),ylabel('Percentage error')");
        matlabPtr->eval(u"print -djpeg test_continuous_ss_lpv_state_error.jpg");
        matlabPtr->eval(u"figure('visible', 'off'),bar(1:1:length(output_error),output_error),grid,xlabel('Test number'),ylabel('Percentage error')");
        matlabPtr->eval(u"print -djpeg test_continuous_ss_lpv_output_error.jpg");
    }

    // Terminate MATLAB session
    matlab::engine::terminateEngineClient();

	return 0;
}