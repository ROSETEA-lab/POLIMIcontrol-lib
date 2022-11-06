#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// PID controller class
//
// Implements the transfer function of a digital PID controller, that is obtained applying Backward-Euler transformation
// to the following continuous time transfer function
//
//           [       1          s Td     ]
// R(s) = Kc [ 1 + ------ + ------------ ]
//           [      s Ti     1 + s Td/N  ]
//
// An anti-windup, based on conditional integration, and a tracking action are also implemented.
// When PID is in TRACKING mode the output signal is equal to the tracking signal.
// No bumpless transition from AUTO to TRACKING is guaranteed as during tracking mode the PID is disconnected from the
// plant; bumpless transition from TRACKING to AUTO is guaranteed if the tracking signal is modulated accordingly.
//
// For cascaded control:
// - when the inner controller saturates up/down set the outer controller to freeze up/down
// - when the inner controller is set to TRACKING the outer controller should be set to TRACKING as well
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class PID_controller {
public:
    enum class PID_state {
        AUTO, TRACKING
    };
    enum class actuator_state {
        NO_SATURATION, SATURATION_UP, SATURATION_DOWN
    };
    enum class control_state {
        NO_FREEZE, FREEZE_UP, FREEZE_DOWN
    };


    PID_controller(double Kc, double Ts, double uMin, double uMax);

    PID_controller(double Kc, double Ti, double Ts, double uMin, double uMax);

    PID_controller(double Kc, double Td, double N, double Ts, double uMin, double uMax);

    PID_controller(double Kc, double Ti, double Td, double N, double Ts, double uMin, double uMax);

    ~PID_controller();


    void evaluate(double y, double ysp, double trk, double &u);

    void resetState();


    PID_state getControllerState() { return controller_state; }

    void setControllerState(PID_state controller_state) { this->controller_state = controller_state; }

    actuator_state getActuationState() { return actuation_state; }

    control_state getControlSignalState() { return controlSignal_state; }

    void setControlSignalState(control_state controlSignal_state) { this->controlSignal_state = controlSignal_state; }

private:
    /* PID parameters */
    double Ts;                      // Sampling time
    double Kc, Ti, Td, N;           // PID parameters (proportional gain, integral/derivative time)
    double uMin, uMax;              // Output saturation values
    double a1, b1, b2;              // PID equation internal coefficients

    /* PID state variables */
    double e_old, u_old, uI, uD;   // PID internal states

    /* PID logic states */
    PID_state controller_state;
    actuator_state actuation_state;
    control_state controlSignal_state;
};

#endif /* PID_CONTROLLER_H_ */
