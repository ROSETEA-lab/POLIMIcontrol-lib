#include "MatlabDataArray.hpp"
#include "MatlabEngine.hpp"
#include <iostream>
#include <unistd.h>

#include <differentiator.h>

#define DIFFERENTIATOR
#define LINEAR_DIFFERENTIATOR
#define RE_DIFFERENTIATOR
#define URE_DIFFERENTIATOR

using namespace matlab::engine;

void execute_test_differentiator(std::unique_ptr<MATLABEngine>& matlabPtr, unsigned int n, unsigned int nf, double d, double r, unsigned int m, double Ts);
void execute_test_lineardifferentiator(std::unique_ptr<MATLABEngine>& matlabPtr, unsigned int n, unsigned int nf, double r, unsigned int m, double Ts);
void execute_test_REdifferentiator(std::unique_ptr<MATLABEngine>& matlabPtr, unsigned int n, unsigned int nf, double r, unsigned int m, double Ts);
void execute_test_UREdifferentiator(std::unique_ptr<MATLABEngine>& matlabPtr, unsigned int n, unsigned int nf, double r, double mu, double Ts);


int main(int argc, char **argv)
{
    const double Ts       = 0.001;
    const unsigned int n  = 2;

    // Start Matlab engine synchronously
    std::unique_ptr<MATLABEngine> matlabPtr = startMATLAB();

    // Execute tests (n, nf, d, r, m, mu, Ts)
#ifdef DIFFERENTIATOR
    execute_test_differentiator(matlabPtr, n, 1, -1.0, 1.0, 1, Ts);

    std::cout << "Press enter to continue...";
    do {
        usleep(1000);
    } while (std::cin.get() != '\n');

    execute_test_differentiator(matlabPtr, n, 0, -1.0, 1.0, 1, Ts);

    std::cout << "Press enter to continue...";
    do {
        usleep(1000);
    } while (std::cin.get() != '\n');
#endif

#ifdef LINEAR_DIFFERENTIATOR
    execute_test_lineardifferentiator(matlabPtr, n, 1, 2.0, 0, Ts);

    std::cout << "Press enter to continue...";
    do {
        usleep(1000);
    } while (std::cin.get() != '\n');

    execute_test_lineardifferentiator(matlabPtr, n, 1, 1.5, 1, Ts);

    std::cout << "Press enter to continue...";
    do {
        usleep(1000);
    } while (std::cin.get() != '\n');
#endif

#ifdef RE_DIFFERENTIATOR
    execute_test_REdifferentiator(matlabPtr, n, 1, 2.0, 0, Ts);

    std::cout << "Press enter to continue...";
    do {
        usleep(1000);
    } while (std::cin.get() != '\n');

    execute_test_REdifferentiator(matlabPtr, n, 1, 1.5, 1, Ts);

    std::cout << "Press enter to continue...";
    do {
        usleep(1000);
    } while (std::cin.get() != '\n');
#endif

#ifdef URE_DIFFERENTIATOR
    execute_test_UREdifferentiator(matlabPtr, n, 1, 2.0, 1.0, Ts);

    std::cout << "Press enter to continue...";
    do {
        usleep(1000);
    } while (std::cin.get() != '\n');

    execute_test_UREdifferentiator(matlabPtr, n, 1, 1.5, 2.5, Ts);
#endif

    // Terminate Matlab session
    matlab::engine::terminateEngineClient();

	return 0;
}

void execute_test_differentiator(std::unique_ptr<MATLABEngine>& matlabPtr, unsigned int n, unsigned int nf, double d, double r, unsigned int m, double Ts)
{
    std::string matlabLine;

    // Initialize Matlab simulation
    matlabPtr->eval(u"clear all; close all");
    matlabPtr->eval(u"open_system('test_differentiator');");

    matlabLine = "set_param('test_differentiator/Differentiator','n','" + std::to_string(n) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('test_differentiator/Differentiator','r','" + std::to_string(r) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('test_differentiator/Differentiator','Ts','" + std::to_string(Ts) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('test_differentiator/Differentiator','d','" + std::to_string(d) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('test_differentiator/Differentiator','m','" + std::to_string(m) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    if (nf==0) {
        matlabLine = "set_param('test_differentiator/Differentiator','filtering','off');";
        matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    }
    else {
        matlabLine = "set_param('test_differentiator/Differentiator','nf','" + std::to_string(nf) + "');";
        matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    }

    matlabLine = "set_param('test_differentiator/f','SampleTime','" + std::to_string(Ts) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('test_differentiator/u','SampleTime','" + std::to_string(Ts) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('test_differentiator/x0','SampleTime','" + std::to_string(Ts) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('test_differentiator/z0','SampleTime','" + std::to_string(Ts) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('test_differentiator/z','SampleTime','" + std::to_string(Ts) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('test_differentiator/t','SampleTime','" + std::to_string(Ts) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));

    // Start Simulink simulation
    matlabLine = "simres = sim('test_differentiator', [0 " + std::to_string(20) + "]);";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabPtr->eval(u"close_system('test_differentiator',0);");

    // Extract simulation results from Matlab
    matlab::data::TypedArray<double> u   = matlabPtr->getVariable(u"u");
    matlab::data::ArrayDimensions u_size = u.getDimensions();
    std::vector<double> u_vect;
    for (auto k=0; k<u_size.at(0); k++) {
        u_vect.push_back(u[k]);
    }

    // Create differentiator object
    differentiator diff(n, nf, d, r, m, 0.0, Ts);

    // Execute the simulation
    std::vector<double> x0_vect, z0_vect, z_1_vect, z_2_vect;
    for (std::vector<double>::iterator it=u_vect.begin(); it!=u_vect.end(); it++) {
        // Call the differentiator
        diff.evaluate(*it);

        // Get results
        double x0, z0;
        diff.get_x0(x0);
        diff.get_z0(z0);
        x0_vect.push_back(x0);
        z0_vect.push_back(z0);

        Eigen::VectorXd z;
        diff.get_z(z);
        z_1_vect.push_back(z(0));
        z_2_vect.push_back(z(1));
    }

    std::cout << "Test completed, plotting results" << std::endl;

    matlab::data::ArrayFactory factory;
    matlab::data::TypedArray<double> m_x0 = factory.createArray({x0_vect.size(),1}, x0_vect.begin(), x0_vect.end());
    matlabPtr->setVariable(u"m_x0", std::move(m_x0));
    matlab::data::TypedArray<double> m_z0 = factory.createArray({z0_vect.size(),1}, z0_vect.begin(), z0_vect.end());
    matlabPtr->setVariable(u"m_z0", std::move(m_z0));
    matlab::data::TypedArray<double> m_z_1 = factory.createArray({z_1_vect.size(),1}, z_1_vect.begin(), z_1_vect.end());
    matlabPtr->setVariable(u"m_z_1", std::move(m_z_1));
    matlab::data::TypedArray<double> m_z_2 = factory.createArray({z_2_vect.size(),1}, z_2_vect.begin(), z_2_vect.end());
    matlabPtr->setVariable(u"m_z_2", std::move(m_z_2));

    matlabPtr->eval(u"figure,subplot(2,1,1),plot(t,x0, t,m_x0,'r--'),grid,xlabel('Time [s]'),ylabel('x0'),legend('Matlab','C++')");
    matlabPtr->eval(u"subplot(2,1,2),plot(t,abs(x0-m_x0)),grid,xlabel('Time [s]'),ylabel('x0 error')");
    matlabPtr->eval(u"figure,subplot(2,1,1),plot(t,z0, t,m_z0,'r--'),grid,xlabel('Time [s]'),ylabel('z0'),legend('Matlab','C++')");
    matlabPtr->eval(u"subplot(2,1,2),plot(t,abs(z0-m_z0)),grid,xlabel('Time [s]'),ylabel('z0 error')");
    matlabPtr->eval(u"figure,subplot(2,1,1),plot(t,z(:,1), t,m_z_1,'r--'),grid,xlabel('Time [s]'),ylabel('First derivative'),legend('Matlab','C++')");
    matlabPtr->eval(u"subplot(2,1,2),plot(t,abs(z(:,1)-m_z_1)),grid,xlabel('Time [s]'),ylabel('First derivative error')");
    matlabPtr->eval(u"figure,subplot(2,1,1),plot(t,z(:,2), t,m_z_2,'r--'),grid,xlabel('Time [s]'),ylabel('Second derivative'),legend('Matlab','C++')");
    matlabPtr->eval(u"subplot(2,1,2),plot(t,abs(z(:,2)-m_z_2)),grid,xlabel('Time [s]'),ylabel('Second derivative error')");
}

void execute_test_lineardifferentiator(std::unique_ptr<MATLABEngine>& matlabPtr, unsigned int n, unsigned int nf, double r, unsigned int m, double Ts)
{
    std::string matlabLine;

    // Initialize Matlab simulation
    matlabPtr->eval(u"clear all; close all");
    matlabPtr->eval(u"open_system('test_differentiator');");

    matlabLine = "set_param('test_differentiator/Differentiator','n','" + std::to_string(n) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('test_differentiator/Differentiator','r','" + std::to_string(r) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('test_differentiator/Differentiator','Ts','" + std::to_string(Ts) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('test_differentiator/Differentiator','differentiator_type','Linear Differentiator');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('test_differentiator/Differentiator','m','" + std::to_string(m) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    if (nf==0) {
        matlabLine = "set_param('test_differentiator/Differentiator','filtering','off');";
        matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    }
    else {
        matlabLine = "set_param('test_differentiator/Differentiator','nf','" + std::to_string(nf) + "');";
        matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    }

    matlabLine = "set_param('test_differentiator/f','SampleTime','" + std::to_string(Ts) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('test_differentiator/u','SampleTime','" + std::to_string(Ts) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('test_differentiator/x0','SampleTime','" + std::to_string(Ts) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('test_differentiator/z0','SampleTime','" + std::to_string(Ts) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('test_differentiator/z','SampleTime','" + std::to_string(Ts) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('test_differentiator/t','SampleTime','" + std::to_string(Ts) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));

    // Start Simulink simulation
    matlabLine = "simres = sim('test_differentiator', [0 " + std::to_string(20) + "]);";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabPtr->eval(u"close_system('test_differentiator',0);");

    // Extract simulation results from Matlab
    matlab::data::TypedArray<double> u   = matlabPtr->getVariable(u"u");
    matlab::data::ArrayDimensions u_size = u.getDimensions();
    std::vector<double> u_vect;
    for (auto k=0; k<u_size.at(0); k++) {
        u_vect.push_back(u[k]);
    }

    // Create differentiator object
    linear_differentiator diff(n, nf, r, m, Ts);

    // Execute the simulation
    std::vector<double> x0_vect, z0_vect, z_1_vect, z_2_vect;
    for (std::vector<double>::iterator it=u_vect.begin(); it!=u_vect.end(); it++) {
        // Call the differentiator
        diff.evaluate(*it);

        // Get results
        double x0, z0;
        diff.get_x0(x0);
        diff.get_z0(z0);
        x0_vect.push_back(x0);
        z0_vect.push_back(z0);

        Eigen::VectorXd z;
        diff.get_z(z);
        z_1_vect.push_back(z(0));
        z_2_vect.push_back(z(1));
    }

    std::cout << "Test completed, plotting results" << std::endl;

    matlab::data::ArrayFactory factory;
    matlab::data::TypedArray<double> m_x0 = factory.createArray({x0_vect.size(),1}, x0_vect.begin(), x0_vect.end());
    matlabPtr->setVariable(u"m_x0", std::move(m_x0));
    matlab::data::TypedArray<double> m_z0 = factory.createArray({z0_vect.size(),1}, z0_vect.begin(), z0_vect.end());
    matlabPtr->setVariable(u"m_z0", std::move(m_z0));
    matlab::data::TypedArray<double> m_z_1 = factory.createArray({z_1_vect.size(),1}, z_1_vect.begin(), z_1_vect.end());
    matlabPtr->setVariable(u"m_z_1", std::move(m_z_1));
    matlab::data::TypedArray<double> m_z_2 = factory.createArray({z_2_vect.size(),1}, z_2_vect.begin(), z_2_vect.end());
    matlabPtr->setVariable(u"m_z_2", std::move(m_z_2));

    matlabPtr->eval(u"figure,subplot(2,1,1),plot(t,x0, t,m_x0,'r--'),grid,xlabel('Time [s]'),ylabel('x0'),legend('Matlab','C++')");
    matlabPtr->eval(u"subplot(2,1,2),plot(t,abs(x0-m_x0)),grid,xlabel('Time [s]'),ylabel('x0 error')");
    matlabPtr->eval(u"figure,subplot(2,1,1),plot(t,z0, t,m_z0,'r--'),grid,xlabel('Time [s]'),ylabel('z0'),legend('Matlab','C++')");
    matlabPtr->eval(u"subplot(2,1,2),plot(t,abs(z0-m_z0)),grid,xlabel('Time [s]'),ylabel('z0 error')");
    matlabPtr->eval(u"figure,subplot(2,1,1),plot(t,z(:,1), t,m_z_1,'r--'),grid,xlabel('Time [s]'),ylabel('First derivative'),legend('Matlab','C++')");
    matlabPtr->eval(u"subplot(2,1,2),plot(t,abs(z(:,1)-m_z_1)),grid,xlabel('Time [s]'),ylabel('First derivative error')");
    matlabPtr->eval(u"figure,subplot(2,1,1),plot(t,z(:,2), t,m_z_2,'r--'),grid,xlabel('Time [s]'),ylabel('Second derivative'),legend('Matlab','C++')");
    matlabPtr->eval(u"subplot(2,1,2),plot(t,abs(z(:,2)-m_z_2)),grid,xlabel('Time [s]'),ylabel('Second derivative error')");
}

void execute_test_REdifferentiator(std::unique_ptr<MATLABEngine>& matlabPtr, unsigned int n, unsigned int nf, double r, unsigned int m, double Ts)
{
    std::string matlabLine;

    // Initialize Matlab simulation
    matlabPtr->eval(u"clear all; close all");
    matlabPtr->eval(u"open_system('test_differentiator');");

    matlabLine = "set_param('test_differentiator/Differentiator','n','" + std::to_string(n) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('test_differentiator/Differentiator','r','" + std::to_string(r) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('test_differentiator/Differentiator','Ts','" + std::to_string(Ts) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('test_differentiator/Differentiator','differentiator_type','Robust Exact Differentiator');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('test_differentiator/Differentiator','m','" + std::to_string(m) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    if (nf==0) {
        matlabLine = "set_param('test_differentiator/Differentiator','filtering','off');";
        matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    }
    else {
        matlabLine = "set_param('test_differentiator/Differentiator','nf','" + std::to_string(nf) + "');";
        matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    }

    matlabLine = "set_param('test_differentiator/f','SampleTime','" + std::to_string(Ts) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('test_differentiator/u','SampleTime','" + std::to_string(Ts) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('test_differentiator/x0','SampleTime','" + std::to_string(Ts) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('test_differentiator/z0','SampleTime','" + std::to_string(Ts) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('test_differentiator/z','SampleTime','" + std::to_string(Ts) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('test_differentiator/t','SampleTime','" + std::to_string(Ts) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));

    // Start Simulink simulation
    matlabLine = "simres = sim('test_differentiator', [0 " + std::to_string(20) + "]);";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabPtr->eval(u"close_system('test_differentiator',0);");

    // Extract simulation results from Matlab
    matlab::data::TypedArray<double> u   = matlabPtr->getVariable(u"u");
    matlab::data::ArrayDimensions u_size = u.getDimensions();
    std::vector<double> u_vect;
    for (auto k=0; k<u_size.at(0); k++) {
        u_vect.push_back(u[k]);
    }

    // Create differentiator object
    robust_exact_differentiator diff(n, nf, r, m, Ts);

    // Execute the simulation
    std::vector<double> x0_vect, z0_vect, z_1_vect, z_2_vect;
    for (std::vector<double>::iterator it=u_vect.begin(); it!=u_vect.end(); it++) {
        // Call the differentiator
        diff.evaluate(*it);

        // Get results
        double x0, z0;
        diff.get_x0(x0);
        diff.get_z0(z0);
        x0_vect.push_back(x0);
        z0_vect.push_back(z0);

        Eigen::VectorXd z;
        diff.get_z(z);
        z_1_vect.push_back(z(0));
        z_2_vect.push_back(z(1));
    }

    std::cout << "Test completed, plotting results" << std::endl;

    matlab::data::ArrayFactory factory;
    matlab::data::TypedArray<double> m_x0 = factory.createArray({x0_vect.size(),1}, x0_vect.begin(), x0_vect.end());
    matlabPtr->setVariable(u"m_x0", std::move(m_x0));
    matlab::data::TypedArray<double> m_z0 = factory.createArray({z0_vect.size(),1}, z0_vect.begin(), z0_vect.end());
    matlabPtr->setVariable(u"m_z0", std::move(m_z0));
    matlab::data::TypedArray<double> m_z_1 = factory.createArray({z_1_vect.size(),1}, z_1_vect.begin(), z_1_vect.end());
    matlabPtr->setVariable(u"m_z_1", std::move(m_z_1));
    matlab::data::TypedArray<double> m_z_2 = factory.createArray({z_2_vect.size(),1}, z_2_vect.begin(), z_2_vect.end());
    matlabPtr->setVariable(u"m_z_2", std::move(m_z_2));

    matlabPtr->eval(u"figure,subplot(2,1,1),plot(t,x0, t,m_x0,'r--'),grid,xlabel('Time [s]'),ylabel('x0'),legend('Matlab','C++')");
    matlabPtr->eval(u"subplot(2,1,2),plot(t,abs(x0-m_x0)),grid,xlabel('Time [s]'),ylabel('x0 error')");
    matlabPtr->eval(u"figure,subplot(2,1,1),plot(t,z0, t,m_z0,'r--'),grid,xlabel('Time [s]'),ylabel('z0'),legend('Matlab','C++')");
    matlabPtr->eval(u"subplot(2,1,2),plot(t,abs(z0-m_z0)),grid,xlabel('Time [s]'),ylabel('z0 error')");
    matlabPtr->eval(u"figure,subplot(2,1,1),plot(t,z(:,1), t,m_z_1,'r--'),grid,xlabel('Time [s]'),ylabel('First derivative'),legend('Matlab','C++')");
    matlabPtr->eval(u"subplot(2,1,2),plot(t,abs(z(:,1)-m_z_1)),grid,xlabel('Time [s]'),ylabel('First derivative error')");
    matlabPtr->eval(u"figure,subplot(2,1,1),plot(t,z(:,2), t,m_z_2,'r--'),grid,xlabel('Time [s]'),ylabel('Second derivative'),legend('Matlab','C++')");
    matlabPtr->eval(u"subplot(2,1,2),plot(t,abs(z(:,2)-m_z_2)),grid,xlabel('Time [s]'),ylabel('Second derivative error')");
}

void execute_test_UREdifferentiator(std::unique_ptr<MATLABEngine>& matlabPtr, unsigned int n, unsigned int nf, double r, double mu, double Ts)
{
    std::string matlabLine;

    // Initialize Matlab simulation
    matlabPtr->eval(u"clear all; close all");
    matlabPtr->eval(u"open_system('test_differentiator');");

    matlabLine = "set_param('test_differentiator/Differentiator','n','" + std::to_string(n) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('test_differentiator/Differentiator','r','" + std::to_string(r) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('test_differentiator/Differentiator','Ts','" + std::to_string(Ts) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('test_differentiator/Differentiator','differentiator_type','Uniform Robust Exact Differentiator');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('test_differentiator/Differentiator','mu','" + std::to_string(mu) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    if (nf==0) {
        matlabLine = "set_param('test_differentiator/Differentiator','filtering','off');";
        matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    }
    else {
        matlabLine = "set_param('test_differentiator/Differentiator','nf','" + std::to_string(nf) + "');";
        matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    }

    matlabLine = "set_param('test_differentiator/f','SampleTime','" + std::to_string(Ts) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('test_differentiator/u','SampleTime','" + std::to_string(Ts) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('test_differentiator/x0','SampleTime','" + std::to_string(Ts) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('test_differentiator/z0','SampleTime','" + std::to_string(Ts) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('test_differentiator/z','SampleTime','" + std::to_string(Ts) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabLine = "set_param('test_differentiator/t','SampleTime','" + std::to_string(Ts) + "');";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));

    // Start Simulink simulation
    matlabLine = "simres = sim('test_differentiator', [0 " + std::to_string(20) + "]);";
    matlabPtr->eval(convertUTF8StringToUTF16String(matlabLine));
    matlabPtr->eval(u"close_system('test_differentiator',0);");

    // Extract simulation results from Matlab
    matlab::data::TypedArray<double> u   = matlabPtr->getVariable(u"u");
    matlab::data::ArrayDimensions u_size = u.getDimensions();
    std::vector<double> u_vect;
    for (auto k=0; k<u_size.at(0); k++) {
        u_vect.push_back(u[k]);
    }

    // Create differentiator object
    uniform_robust_exact_differentiator diff(n, nf, r, mu, Ts);

    // Execute the simulation
    std::vector<double> x0_vect, z0_vect, z_1_vect, z_2_vect;
    for (std::vector<double>::iterator it=u_vect.begin(); it!=u_vect.end(); it++) {
        // Call the differentiator
        diff.evaluate(*it);

        // Get results
        double x0, z0;
        diff.get_x0(x0);
        diff.get_z0(z0);
        x0_vect.push_back(x0);
        z0_vect.push_back(z0);

        Eigen::VectorXd z;
        diff.get_z(z);
        z_1_vect.push_back(z(0));
        z_2_vect.push_back(z(1));
    }

    std::cout << "Test completed, plotting results" << std::endl;

    matlab::data::ArrayFactory factory;
    matlab::data::TypedArray<double> m_x0 = factory.createArray({x0_vect.size(),1}, x0_vect.begin(), x0_vect.end());
    matlabPtr->setVariable(u"m_x0", std::move(m_x0));
    matlab::data::TypedArray<double> m_z0 = factory.createArray({z0_vect.size(),1}, z0_vect.begin(), z0_vect.end());
    matlabPtr->setVariable(u"m_z0", std::move(m_z0));
    matlab::data::TypedArray<double> m_z_1 = factory.createArray({z_1_vect.size(),1}, z_1_vect.begin(), z_1_vect.end());
    matlabPtr->setVariable(u"m_z_1", std::move(m_z_1));
    matlab::data::TypedArray<double> m_z_2 = factory.createArray({z_2_vect.size(),1}, z_2_vect.begin(), z_2_vect.end());
    matlabPtr->setVariable(u"m_z_2", std::move(m_z_2));

    matlabPtr->eval(u"figure,subplot(2,1,1),plot(t,x0, t,m_x0,'r--'),grid,xlabel('Time [s]'),ylabel('x0'),legend('Matlab','C++')");
    matlabPtr->eval(u"subplot(2,1,2),plot(t,abs(x0-m_x0)),grid,xlabel('Time [s]'),ylabel('x0 error')");
    matlabPtr->eval(u"figure,subplot(2,1,1),plot(t,z0, t,m_z0,'r--'),grid,xlabel('Time [s]'),ylabel('z0'),legend('Matlab','C++')");
    matlabPtr->eval(u"subplot(2,1,2),plot(t,abs(z0-m_z0)),grid,xlabel('Time [s]'),ylabel('z0 error')");
    matlabPtr->eval(u"figure,subplot(2,1,1),plot(t,z(:,1), t,m_z_1,'r--'),grid,xlabel('Time [s]'),ylabel('First derivative'),legend('Matlab','C++')");
    matlabPtr->eval(u"subplot(2,1,2),plot(t,abs(z(:,1)-m_z_1)),grid,xlabel('Time [s]'),ylabel('First derivative error')");
    matlabPtr->eval(u"figure,subplot(2,1,1),plot(t,z(:,2), t,m_z_2,'r--'),grid,xlabel('Time [s]'),ylabel('Second derivative'),legend('Matlab','C++')");
    matlabPtr->eval(u"subplot(2,1,2),plot(t,abs(z(:,2)-m_z_2)),grid,xlabel('Time [s]'),ylabel('Second derivative error')");
}
