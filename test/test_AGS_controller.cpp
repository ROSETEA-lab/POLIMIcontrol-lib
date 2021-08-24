#include "MatlabDataArray.hpp"
#include "MatlabEngine.hpp"

#include <iostream>
#include <AGS_controller.h>

#define SAMPLING_TIME   0.001
#define T_END           20.0

int main(int argc, char **argv)
{
    std::string matlabLine;
    using namespace matlab::engine;

    // Start MATLAB engine synchronously
    std::unique_ptr<MATLABEngine> matlabPtr = startMATLAB();

    // Initialize the MATLAB simulation
    matlabPtr->eval(u"clear all; close all");
    matlabPtr->eval(u"load('test_AGS_controller.mat');");
    matlabPtr->eval(u"open_system('test_AGS_controller_sim');");

    matlabLine = "set_param('AGS_controller_sim/Vx','SampleTime','" + std::to_string(SAMPLING_TIME) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('AGS_controller_sim/xi_d','SampleTime','" + std::to_string(SAMPLING_TIME) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('AGS_controller_sim/beta','SampleTime','" + std::to_string(SAMPLING_TIME) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('AGS_controller_sim/r','SampleTime','" + std::to_string(SAMPLING_TIME) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('AGS_controller_sim/t','SampleTime','" + std::to_string(SAMPLING_TIME) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('AGS_controller_sim/Fy','SampleTime','" + std::to_string(SAMPLING_TIME) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));

    // Start Simulink simulation
    matlabLine = "simres = sim('AGS_controller_sim', [0 " + std::to_string(T_END) + "]);";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));

    // Open mat file
    const char *fileName = "test_AGS_controller.mat";

    // Create controller object
    AGS_controller* controller = NULL;
    controller = new AGS_controller(fileName, SAMPLING_TIME, 3, Eigen::VectorXd::Zero(4));
    if (controller) {
        std::cout << "Controller created" << std::endl;
    }
    else {
        std::cout << "Cannot create controller" << std::endl;
    }

    // Extract simulation results from Matlab
    matlab::data::TypedArray<double> Vx   = matlabPtr->getVariable(u"Vx");
    matlab::data::ArrayDimensions Vx_size = Vx.getDimensions();
    std::vector<double> Vx_vect;
    for (auto k=0; k<Vx_size.at(0); k++) {
        Vx_vect.push_back(Vx[k]);
    }

    matlab::data::TypedArray<double> xi_d   = matlabPtr->getVariable(u"xi_d");
    matlab::data::ArrayDimensions xi_d_size = xi_d.getDimensions();
    std::vector<double> xi_d_vect;
    for (auto k=0; k<xi_d_size.at(0); k++) {
        xi_d_vect.push_back(xi_d[k]);
    }

    matlab::data::TypedArray<double> beta   = matlabPtr->getVariable(u"beta");
    matlab::data::ArrayDimensions beta_size = beta.getDimensions();
    std::vector<double> beta_vect;
    for (auto k=0; k<beta_size.at(0); k++) {
        beta_vect.push_back(beta[k]);
    }

    matlab::data::TypedArray<double> r    = matlabPtr->getVariable(u"r");
    matlab::data::ArrayDimensions r_size  = r.getDimensions();
    std::vector<double> r_vect;
    for (auto k=0; k<r_size.at(0); k++) {
        r_vect.push_back(r[k]);
    }

    matlab::data::TypedArray<double> t   = matlabPtr->getVariable(u"t");
    matlab::data::ArrayDimensions t_size = t.getDimensions();
    std::vector<double> t_vect;
    for (auto k=0; k<t_size.at(0); k++) {
        t_vect.push_back(t[k]);
    }

    // Execute the simulation
    std::vector<double> control;

    std::cout << "Executing the simulation..." << std::endl;
    for (auto k=0; k<Vx_vect.size(); k++) {
        // Compute AGS control law
        Eigen::Vector2d AGS_input(beta_vect.at(k), r_vect.at(k));
        Eigen::Vector3d AGS_param(1.0/Vx_vect.at(k), 1.0/std::pow(Vx_vect.at(k), 2.0), xi_d_vect.at(k));
        controller->evaluate(AGS_input, AGS_param);

        Eigen::VectorXd Fyf = Eigen::VectorXd::Zero(1);
        controller->get_output(Fyf);

        // Store simulation data
        control.push_back(Fyf(0));
    }

    matlabPtr->eval(u"close_system('AGS_controller_sim',0);");

    std::cout << "Test completed, plotting results" << std::endl;

    matlab::data::ArrayFactory factory;
    matlab::data::TypedArray<double> m_control = factory.createArray({control.size(),1}, control.begin(), control.end());
    matlabPtr->setVariable(u"control", std::move(m_control));

    matlabPtr->eval(u"figure,plot(t,control, t,Fy,'r--'),grid,xlabel('Time [s]'),ylabel('Control signal'),legend('C++','Matlab')");
    matlabPtr->eval(u"figure,plot(t,abs(control-Fy)),grid,xlabel('Time [s]'),ylabel('Control signal error')");
    matlabPtr->eval(u"figure,plot(t,Vx),grid,xlabel('Time [s]'),ylabel('Velocity [m/s]')");
    matlabPtr->eval(u"figure,plot(t,beta),grid,xlabel('Time [s]'),ylabel('Sideslip [rad]')");
    matlabPtr->eval(u"figure,plot(t,r),grid,xlabel('Time [s]'),ylabel('Yaw rate [rad/s]')");

    // Terminate MATLAB session
    matlab::engine::terminateEngineClient();

    // Destroy controller object
    if (controller) {
        delete controller;
    }

	return 0;
}
