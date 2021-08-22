#include "MatlabDataArray.hpp"
#include "MatlabEngine.hpp"

#include <iostream>
#include <AGS_controller.h>

void getMatlabMatrix(std::string matvar_name, Eigen::MatrixXd& matrix, std::unique_ptr<matlab::engine::MATLABEngine>& matEngine);


int main(int argc, char **argv)
{
    std::string matlabLine;
    using namespace matlab::engine;

    // Start Matlab engine synchronously
    std::unique_ptr<MATLABEngine> matlabPtr = startMATLAB();

    // Initialize the Matlab simulation
    matlabPtr->eval(u"clear all; close all");
    matlabPtr->eval(u"load('test_AGS_compute_matrices.mat');");

    // Create theta vector and load to Matlab
    const double Vx = 5.0;
    const double xi_d = 0.7;
    Eigen::Vector3d theta(1.0/Vx, 1.0/std::pow(Vx, 2.0), xi_d);

    std::vector<double> theta_vect;
    theta_vect.resize(theta.size());
    Eigen::VectorXd::Map(&theta_vect[0], theta.size()) = theta;

    matlab::data::ArrayFactory factory;
    matlab::data::TypedArray<double> m_theta = factory.createArray({theta_vect.size(),1}, theta_vect.begin(), theta_vect.end());
    matlabPtr->setVariable(u"theta", std::move(m_theta));

    // Execute Matlab script to compute controller matrices
    matlabPtr->eval(u"test_AGS_compute_matrices;");

    // Open mat file
    const char *fileName = "test_AGS_controller.mat";

    // Create controller object
    AGS_controller* controller = NULL;
    controller = new AGS_controller(fileName, 0.01, 3, Eigen::VectorXd::Zero(4));
    if (controller) {
        std::cout << "Controller created" << std::endl;
    }
    else {
        std::cout << "Cannot create controller" << std::endl;
    }

    // Compute AGS control matrices
    Eigen::Vector2d AGS_input(0.0, 0.0);
    controller->evaluate(AGS_input, theta);

    Eigen::MatrixXd X, Y, Ak_hat, Bk_hat, Ck_hat, Dk_hat, A, B, C, NMt, N, Mt, Ak, Bk, Ck, Dk;
    controller->get_lyapunov_matrix(X, Y);
    controller->get_controller_hat_matrix(Ak_hat, Bk_hat, Ck_hat, Dk_hat);
    controller->get_plant_matrix(A, B, C);
    controller->get_projection_matrix(NMt, N, Mt);
    controller->get_controller_matrix(Ak, Bk, Ck, Dk);

    // Extract matrices from Matlab
    Eigen::MatrixXd m_X, m_Y, m_Ak_hat, m_Bk_hat, m_Ck_hat, m_Dk_hat, m_A, m_B, m_C, m_NMt, m_N, m_Mt, m_Ak, m_Bk, m_Ck, m_Dk;
    getMatlabMatrix(std::string("X"), m_X, matlabPtr);
    getMatlabMatrix(std::string("Y"), m_Y, matlabPtr);
    getMatlabMatrix(std::string("Ak_hat"), m_Ak_hat, matlabPtr);
    getMatlabMatrix(std::string("Bk_hat"), m_Bk_hat, matlabPtr);
    getMatlabMatrix(std::string("Ck_hat"), m_Ck_hat, matlabPtr);
    getMatlabMatrix(std::string("Dk_hat"), m_Dk_hat, matlabPtr);
    getMatlabMatrix(std::string("A"), m_A, matlabPtr);
    getMatlabMatrix(std::string("B"), m_B, matlabPtr);
    getMatlabMatrix(std::string("C"), m_C, matlabPtr);
    getMatlabMatrix(std::string("NMt"), m_NMt, matlabPtr);
    getMatlabMatrix(std::string("N"), m_N, matlabPtr);
    getMatlabMatrix(std::string("Mt"), m_Mt, matlabPtr);
    getMatlabMatrix(std::string("Ak"), m_Ak, matlabPtr);
    getMatlabMatrix(std::string("Bk"), m_Bk, matlabPtr);
    getMatlabMatrix(std::string("Ck"), m_Ck, matlabPtr);
    getMatlabMatrix(std::string("Dk"), m_Dk, matlabPtr);

    // Print results
    std::cout << "err X" << std::endl << X-m_X << std::endl;
    std::cout << "err Y" << std::endl << Y-m_Y << std::endl << std::endl;
    std::cout << "err Ak_hat" << std::endl << Ak_hat-m_Ak_hat << std::endl;
    std::cout << "err Bk_hat" << std::endl << Bk_hat-m_Bk_hat << std::endl;
    std::cout << "err Ck_hat" << std::endl << Ck_hat-m_Ck_hat << std::endl;
    std::cout << "err Dk_hat" << std::endl << Dk_hat-m_Dk_hat << std::endl << std::endl;
    std::cout << "err A" << std::endl << A-m_A << std::endl;
    std::cout << "err B" << std::endl << B-m_B << std::endl;
    std::cout << "err C" << std::endl << C-m_C << std::endl << std::endl;
    std::cout << "err NMt" << std::endl << NMt-m_NMt << std::endl;
    std::cout << "err N" << std::endl << N-m_N << std::endl;
    std::cout << "err Mt" << std::endl << Mt-m_Mt << std::endl << std::endl;
    std::cout << "err Ak" << std::endl << Ak-m_Ak << std::endl;
    std::cout << "err Bk" << std::endl << Bk-m_Bk << std::endl;
    std::cout << "err Ck" << std::endl << Ck-m_Ck << std::endl << std::endl;
    std::cout << "err Dk" << std::endl << Dk-m_Dk << std::endl << std::endl;

    // Terminate MATLAB session
    matlab::engine::terminateEngineClient();

    // Destroy controller object
    if (controller) {
        delete controller;
    }

	return 0;
}


void getMatlabMatrix(std::string matvar_name, Eigen::MatrixXd& matrix, std::unique_ptr<matlab::engine::MATLABEngine>& matEngine)
{
    matlab::data::TypedArray<double> mat   = matEngine->getVariable(matvar_name);
    matlab::data::ArrayDimensions mat_size = mat.getDimensions();

    matrix = Eigen::MatrixXd::Zero(mat_size.at(0),mat_size.at(1));
    for (auto j=0; j<mat_size.at(0); j++) {
        for (auto k=0; k<mat_size.at(1); k++) {
            matrix(j,k) = mat[j][k];
        }
    }
}