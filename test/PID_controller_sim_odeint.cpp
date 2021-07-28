#include <iostream>
#include <boost/math/constants/constants.hpp>
#include <boost/numeric/odeint.hpp>
#include <fstream>

#include "matplotlibcpp.h"
#include "PID_controller.h"

using namespace boost::numeric::odeint;

typedef std::vector< double > state_type;

class closed_loop_system
{
public:
    closed_loop_system(double deltaT, double y0);

    void updatePlant();
    void updateController();

    void getTimeVect(std::vector< double > &time) { time = data_t; };
    void getControlVect(std::vector< double > &control) { control = data_u; };
    void getMeasureVect(std::vector< double > &measure) { measure = data_y; };
private:
    // Simulator and integrator variables
    double t, dt, u;
    state_type y;
    runge_kutta_dopri5 < state_type > stepper;

    // Controller variable
    PID_controller* controller;

    // Variables to store data for plotting
    std::vector< double > data_y, data_t, data_u;

    // ODE function
    void plantODE(const state_type &y, state_type &dy, double t);
};

closed_loop_system::closed_loop_system(double deltaT, double y0) : dt(deltaT), t(0.0), y(1), u(0)
{
    /*
       x = y[0]
      dx = dy[0] = ODE equation
     */

    // Initial values
    y[0] = y0;  //  x1

    // Controller initialization
    const double Kc = 20.0;
    const double Ti = 10.0;
    const double uMin = -100.0;
    const double uMax = 100.0;
    controller = new PID_controller(Kc, Ti, dt, uMin, uMax);
}

void closed_loop_system::updatePlant()
{
    // Store data for plotting
    data_t.push_back(t);
    data_y.push_back(y[0]);

    // Integrate for one step ahead
    using namespace std::placeholders;
    stepper.do_step(std::bind(&closed_loop_system::plantODE, this, _1, _2, _3), y, t, dt);
    t += dt;
}

void closed_loop_system::updateController()
{
    // Controller evaluation
    const double ysp = 5.0;
    controller->evaluate(y[0], ysp, u);

    // Store data for plotting
    data_u.push_back(u);
}

void closed_loop_system::plantODE(const state_type &y, state_type &dy, double t)
{
    // ODE Equation
    dy[0] = -0.1*y[0] + 0.1*u;
}

int main(int argc, char **argv)
{
    // Create the closed-loop system
    const double dt(0.001);
    const double y0(0.0);
    closed_loop_system sys(dt,y0);

    // Integration loop
    for (double t(0.0); t <= 10.0; t += dt){
        sys.updateController();
        sys.updatePlant();
    }

    // Plot results
    std::vector< double > t, y, u;
    sys.getTimeVect(t);
    sys.getControlVect(u);
    sys.getMeasureVect(y);

    matplotlibcpp::figure(1);
    matplotlibcpp::plot(t,y);
    matplotlibcpp::title("Process variable");
    matplotlibcpp::figure(2);
    matplotlibcpp::plot(t,u);
    matplotlibcpp::title("Control signal");
    matplotlibcpp::show();

    return 0;
}
