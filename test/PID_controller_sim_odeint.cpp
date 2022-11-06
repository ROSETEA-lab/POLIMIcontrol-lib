#include <iostream>
#include <boost/math/constants/constants.hpp>
#include <boost/numeric/odeint.hpp>
#include <fstream>

#include "matplotlibcpp.h"
#include "PID_controller.h"

using namespace boost::numeric::odeint;

typedef std::vector<double> state_type;

class closed_loop_system {
public:
    closed_loop_system(double deltaT, double y0);

    void updatePlant();

    void updateController();

    void getTimeVect(std::vector<double> &time) { time = data_t; };

    void getControlVect(std::vector<double> &control) { control = data_u; };

    void getMeasureVect(std::vector<double> &measure) { measure = data_y; };

    void getSetpointVect(std::vector<double> &setpoint) { setpoint = data_ysp; };

private:
    // Simulator and integrator variables
    double t, dt, u;
    state_type y;
    runge_kutta_dopri5<state_type> stepper;

    // Controller variable
    PID_controller *controller1, *controller2;

    // Variables to store data for plotting
    std::vector<double> data_y, data_ysp, data_t, data_u;

    // ODE function
    void plantODE(const state_type &y, state_type &dy, double t);
};

closed_loop_system::closed_loop_system(double deltaT, double y0) : dt(deltaT), t(0.0), y(1), u(0) {
    /*
       x = y[0]
      dx = dy[0] = ODE equation
     */

    // Initial values
    y[0] = y0;  //  x1

    // Controller initialization
    const double Kc = 4.0;
    const double Ti = 2.0;
    const double uMin = -10.0;
    const double uMax = 10.0;
    controller1 = new PID_controller(Kc, dt, uMin, uMax);
    controller2 = new PID_controller(Kc, Ti, dt, uMin, uMax);
}

void closed_loop_system::updatePlant() {
    // Store data for plotting
    data_t.push_back(t);
    data_y.push_back(y[0]);

    // Integrate for one step ahead
    using namespace std::placeholders;
    stepper.do_step(std::bind(&closed_loop_system::plantODE, this, _1, _2, _3), y, t, dt);
    t += dt;
}

void closed_loop_system::updateController() {
    // Setpoint generation
    double ysp;
    if (t<=20.0) {
        ysp = -9.0;
    } else if (t<=40.0) {
        ysp = 9.0;
    } else if (t<=60.0) {
        ysp = 0.0;
    } else if (t<=80.0) {
        ysp = 9.0;
    } else {
        ysp = 0.0;
    }

    // Controller evaluation
    if (t<=10.0) {
        controller1->setControllerState(PID_controller::PID_state::AUTO);
        controller1->evaluate(y[0], ysp, 0.0, u);

        double utmp;
        controller2->setControllerState(PID_controller::PID_state::TRACKING);
        controller2->setControlSignalState(PID_controller::control_state::NO_FREEZE);
        controller2->evaluate(y[0], ysp, u, utmp);
    }
    else if ((t>=62.5) && (t<=64.5)) {
        controller2->setControllerState(PID_controller::PID_state::AUTO);
        controller2->setControlSignalState(PID_controller::control_state::FREEZE_DOWN);
        controller2->evaluate(y[0], ysp, 0.0, u);
    }
    else if ((t>=81.0) && (t<=82.5)) {
        controller2->setControllerState(PID_controller::PID_state::AUTO);
        controller2->setControlSignalState(PID_controller::control_state::FREEZE_UP);
        controller2->evaluate(y[0], ysp, 0.0, u);
    }
    else {
        controller2->setControllerState(PID_controller::PID_state::AUTO);
        controller2->setControlSignalState(PID_controller::control_state::NO_FREEZE);
        controller2->evaluate(y[0], ysp, 0.0, u);
    }

    // Store data for plotting
    data_u.push_back(u);
    data_ysp.push_back(ysp);
}

void closed_loop_system::plantODE(const state_type &y, state_type &dy, double t) {
    // ODE Equation
    dy[0] = -0.5 * y[0] + 0.5 * u;
}

int main(int argc, char **argv) {
    // Create the closed-loop system
    const double dt(0.001);
    const double y0(0.0);
    closed_loop_system sys(dt, y0);

    // Integration loop
    for (double t(0.0); t <= 100.0; t += dt) {
        sys.updateController();
        sys.updatePlant();
    }

    // Plot results
    std::vector<double> t, y, ysp, u, err;
    sys.getTimeVect(t);
    sys.getControlVect(u);
    sys.getMeasureVect(y);
    sys.getSetpointVect(ysp);

    for (auto i = 0; i < ysp.size(); i++) {
        err.push_back(ysp.at(i) - y.at(i));
    }

    matplotlibcpp::figure(1);
    matplotlibcpp::plot(t, y);
    matplotlibcpp::plot(t, ysp, "r");
    matplotlibcpp::title("Process variable");
    matplotlibcpp::grid(true);
    matplotlibcpp::figure(2);
    matplotlibcpp::plot(t, u);
    matplotlibcpp::title("Control signal");
    matplotlibcpp::grid(true);
    matplotlibcpp::figure(3);
    matplotlibcpp::plot(t, err);
    matplotlibcpp::title("Error signal");
    matplotlibcpp::grid(true);
    matplotlibcpp::show();

    return 0;
}
