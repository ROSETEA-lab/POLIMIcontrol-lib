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
    closed_loop_system(double deltaT, double y01, double y02);

    void updatePlant();

    void updateController();

    void getTimeVect(std::vector<double> &time) { time = data_t; };

    void getControlVect(std::vector<double> &control) { control = data_u; };

    void getPIDControlVect(std::vector<double> &control) { control = data_uPID; };

    void getR1ControlVect(std::vector<double> &control) { control = data_uR1; };

    void getR2ControlVect(std::vector<double> &control) { control = data_uR2; };

    void getMeasureVect(std::vector<double> &measure) { measure = data_y; };

    void getSetpointVect(std::vector<double> &setpoint) { setpoint = data_ysp; };

private:
    // Simulator and integrator variables
    double t, dt, u;
    state_type y;
    runge_kutta_dopri5<state_type> stepper;

    // Controller variable
    PID_controller *R1, *R2, *PID;

    // Variables to store data for plotting
    std::vector<double> data_y, data_ysp, data_t, data_u, data_uR1, data_uR2, data_uPID;

    // ODE function
    void plantODE(const state_type &y, state_type &dy, double t);
};

closed_loop_system::closed_loop_system(double deltaT, double y01, double y02) : dt(deltaT), t(0.0), y(2), u(0) {
    /*
       x = y[0]
      dx = dy[0] = ODE equation
     */

    // Initial values
    y[0] = y01;  //  x1
    y[1] = y02;  //  x2

    // Cascaded controller initialization
    const double R1_Kc = 2.0;
    const double R1_Ti = 1.0;
    const double R1_uMin = -5.0;
    const double R1_uMax = 5.0;
    R1 = new PID_controller(R1_Kc, R1_Ti, dt, R1_uMin, R1_uMax);    // Inner control

    const double R2_Kc = 0.2;
    const double R2_Ti = 10.0;
    const double R2_uMin = -10.0;
    const double R2_uMax = 10.0;
    R2 = new PID_controller(R2_Kc, R2_Ti, dt, R2_uMin, R2_uMax);    // Outer control

    // Single loop controller initialization
    const double PID_Kc = 0.22;
    const double PID_Ti = 11.0;
    const double PID_Td = 10.0/11.0;
    const double PID_N = 5.0/11.0;
    const double PID_uMin = -5.0;
    const double PID_uMax = 5.0;
    PID = new PID_controller(PID_Kc, PID_Ti, PID_Td, PID_N, dt, PID_uMin, PID_uMax);
}

void closed_loop_system::updatePlant() {
    // Store data for plotting
    data_t.push_back(t);
    data_y.push_back(y[1]);

    // Integrate for one step ahead
    using namespace std::placeholders;
    stepper.do_step(std::bind(&closed_loop_system::plantODE, this, _1, _2, _3), y, t, dt);
    t += dt;
}

void closed_loop_system::updateController() {
    // Setpoint generation
    double ysp;
    if (t<=30.0) {
        ysp = -9.0;
    } else if (t<=60.0) {
        ysp = 9.0;
    } else if (t<=90.0) {
        ysp = 0.0;
    } else if (t<=120.0) {
        ysp = 40.0;
    } else {
        ysp = 40.0;
    }

    // Controller evaluation
    if (t<=35.0) {
        // PID controller is controlling the plant
        PID->setControllerState(PID_controller::PID_state::AUTO);
        PID->evaluate(y[1], ysp, 0.0, u);

        // Cascaded controller is in tracking
        double u1, u2;
        R2->setControllerState(PID_controller::PID_state::TRACKING);
        R2->setControlSignalState(PID_controller::control_state::NO_FREEZE);
        R2->evaluate(y[1], ysp, y[0], u2);
        R1->setControllerState(PID_controller::PID_state::TRACKING);
        R1->setControlSignalState(PID_controller::control_state::NO_FREEZE);
        R1->evaluate(y[0], u2, u, u1);

        // Store data for plotting
        data_uPID.push_back(u);
        data_uR1.push_back(u1);
        data_uR2.push_back(u2);
    }
    else {
        // Manage the cascaded controller saturation
        if (R1->getActuationState() == PID_controller::actuator_state::SATURATION_UP) {
            R2->setControlSignalState(PID_controller::control_state::FREEZE_UP);
        } else if (R1->getActuationState() == PID_controller::actuator_state::SATURATION_DOWN) {
            R2->setControlSignalState(PID_controller::control_state::FREEZE_DOWN);
        } else {
            R2->setControlSignalState(PID_controller::control_state::NO_FREEZE);
        }
        R1->setControlSignalState(PID_controller::control_state::NO_FREEZE);

        // Cascaded controller is controlling the plant
        double u2;
        R2->setControllerState(PID_controller::PID_state::AUTO);
        R2->evaluate(y[1], ysp, 0.0, u2);
        R1->setControllerState(PID_controller::PID_state::AUTO);
        R1->evaluate(y[0], u2, 0.0, u);

        // Store data for plotting
        data_uPID.push_back(0.0);
        data_uR1.push_back(u);
        data_uR2.push_back(u2);
    }

    // Store data for plotting
    data_u.push_back(u);
    data_ysp.push_back(ysp);
}

void closed_loop_system::plantODE(const state_type &y, state_type &dy, double t) {
    // ODE Equation
    dy[0] = - y[0] + u;
    dy[1] =   y[0] - 0.1 * y[1];
}

int main(int argc, char **argv) {
    // Create the closed-loop system
    const double dt(0.001);
    const double y01(0.0);
    const double y02(0.0);
    closed_loop_system sys(dt, y01, y02);

    // Integration loop
    for (double t(0.0); t <= 150.0; t += dt) {
        sys.updateController();
        sys.updatePlant();
    }

    // Plot results
    std::vector<double> t, y, ysp, u, uR1, uR2, uPID, err;
    sys.getTimeVect(t);
    sys.getControlVect(u);
    sys.getPIDControlVect(uPID);
    sys.getR1ControlVect(uR1);
    sys.getR2ControlVect(uR2);
    sys.getMeasureVect(y);
    sys.getSetpointVect(ysp);

    for (auto i = 0; i < ysp.size(); i++) {
        err.push_back(ysp.at(i) - y.at(i));
    }

    matplotlibcpp::figure(1);
    matplotlibcpp::plot(t, y);
    matplotlibcpp::plot(t, ysp, "r");
    matplotlibcpp::xlabel("Time [s]");
    matplotlibcpp::ylabel("Process variable");
    matplotlibcpp::grid(true);

    matplotlibcpp::figure(2);
    matplotlibcpp::plot(t, u);
    matplotlibcpp::xlabel("Time [s]");
    matplotlibcpp::ylabel("Control signal");
    matplotlibcpp::grid(true);

    matplotlibcpp::figure(3);
    matplotlibcpp::subplot(3,1,1);
    matplotlibcpp::plot(t, uPID);
    matplotlibcpp::ylabel("PID control");
    matplotlibcpp::grid(true);
    matplotlibcpp::subplot(3,1,2);
    matplotlibcpp::plot(t, uR1);
    matplotlibcpp::ylabel("R1 control");
    matplotlibcpp::grid(true);
    matplotlibcpp::subplot(3,1,3);
    matplotlibcpp::plot(t, uR2);
    matplotlibcpp::xlabel("Time [s]");
    matplotlibcpp::ylabel("R2 control");
    matplotlibcpp::grid(true);

    matplotlibcpp::figure(4);
    matplotlibcpp::plot(t, err);
    matplotlibcpp::xlabel("Time [s]");
    matplotlibcpp::ylabel("Error signal");
    matplotlibcpp::grid(true);
    matplotlibcpp::show();

    return 0;
}
