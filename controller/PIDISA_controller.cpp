#include "PIDISA_controller.h"

#include <stdexcept>


PIDISA_controller::PIDISA_controller(double Kc, double b, double Ti, double Td, double N, double c, double Ts, double uMin, double uMax) {
    // Check parameter consistency
    if (Ts == 0.0)
        throw std::invalid_argument("PID sampling time cannot be zero");
    if ((Kc == 0.0) && (Ti == 0.0) && (Td == 0.0))
        throw std::invalid_argument("PID parameters cannot be all zero");
    if (uMin > uMax)
        throw std::invalid_argument("Lower control saturation cannot be greater then higher");
    if ((b < 0) || (b > 1))
        throw std::invalid_argument("Weight b should be in the range [0 1]");
    if ((c < 0) || (c > 1))
        throw std::invalid_argument("Weight c should be in the range [0 1]");

    // Initialise PID parameters
    this->Ts = Ts;
    this->Kc = Kc;
    this->b = b;
    this->Ti = Ti;
    this->Td = Td;
    this->c = c;
    this->N = N;
    this->uMin = uMin;
    this->uMax = uMax;

    if (Ti == 0.0)                        // No integral action
        this->a1 = 0.0;
    else {
        if (Kc == 0.0)                    // Integral action without proportional action
            this->a1 = Ts / Ti;
        else                              // Integral action with proportional action
            this->a1 = Kc * Ts / Ti;
    }

    if (Td == 0.0)                        // No derivative action
        this->b1 = this->b2 = 0.0;
    else {
        if (Kc == 0.0)                    // Derivative action without proportional action
        {
            this->b1 = Td / (N * Ts + Td);
            this->b2 = N * this->b1;
        } else                            // Derivative action with proportional action
        {
            this->b1 = Td / (N * Ts + Td);
            this->b2 = Kc * N * this->b1;
        }
    }

    // Initialise PID state variables
    y_old = ysp_old = u_old = uI = uD = 0.0;

    // Initialize PID logic states
    controller_state = PID_state::AUTO;
    actuation_state = actuator_state::NO_SATURATION;
    controlSignal_state = control_state::NO_FREEZE;
}

PIDISA_controller::PIDISA_controller(double Kc, double b, double Ti, double Ts, double uMin, double uMax) {
    // Check parameter consistency
    if (Ts == 0.0)
        throw std::invalid_argument("PID sampling time cannot be zero");
    if ((Kc == 0.0) && (Ti == 0.0))
        throw std::invalid_argument("PID parameters cannot be all zero");
    if (uMin > uMax)
        throw std::invalid_argument("Lower control saturation cannot be greater then higher");
    if ((b < 0) || (b > 1))
        throw std::invalid_argument("Weight b should be in the range [0 1]");

    // Initialise PID parameters
    this->Ts = Ts;
    this->Kc = Kc;
    this->b = b;
    this->Ti = Ti;
    this->Td = 0.0;
    this->N = 0.0;
    this->uMin = uMin;
    this->uMax = uMax;

    if (Ti == 0.0)                        // No integral action
        this->a1 = 0.0;
    else {
        if (Kc == 0.0)                    // Integral action without proportional action
            this->a1 = Ts / Ti;
        else                              // Integral action with proportional action
            this->a1 = Kc * Ts / Ti;
    }

    // No derivative action
    this->b1 = this->b2 = 0.0;

    // Initialise PID state variables
    y_old = ysp_old = u_old = uI = uD = 0.0;

    // Initialize PID logic states
    controller_state = PID_state::AUTO;
    actuation_state = actuator_state::NO_SATURATION;
    controlSignal_state = control_state::NO_FREEZE;
}

PIDISA_controller::PIDISA_controller(double Kc, double b, double Td, double N, double c, double Ts, double uMin, double uMax) {
    // Check parameter consistency
    if (Ts == 0.0)
        throw std::invalid_argument("PID sampling time cannot be zero");
    if ((Kc == 0.0) && (Td == 0.0))
        throw std::invalid_argument("PID parameters cannot be all zero");
    if (uMin > uMax)
        throw std::invalid_argument("Lower control saturation cannot be greater then higher");
    if ((b < 0) || (b > 1))
        throw std::invalid_argument("Weight b should be in the range [0 1]");
    if ((c < 0) || (c > 1))
        throw std::invalid_argument("Weight c should be in the range [0 1]");

    // Initialise PID parameters
    this->Ts = Ts;
    this->Kc = Kc;
    this->b = b;
    this->Ti = 0.0;
    this->Td = Td;
    this->c = c;
    this->N = N;
    this->uMin = uMin;
    this->uMax = uMax;

    // No integral action
    this->a1 = 0.0;

    if (Td == 0.0)                        // No derivative action
        this->b1 = this->b2 = 0.0;
    else {
        if (Kc == 0.0)                    // Derivative action without proportional action
        {
            this->b1 = Td / (N * Ts + Td);
            this->b2 = N * this->b1;
        } else                            // Derivative action with proportional action
        {
            this->b1 = Td / (N * Ts + Td);
            this->b2 = Kc * N * this->b1;
        }
    }

    // Initialise PID state variables
    y_old = ysp_old = u_old = uI = uD = 0.0;

    // Initialize PID logic states
    controller_state = PID_state::AUTO;
    actuation_state = actuator_state::NO_SATURATION;
    controlSignal_state = control_state::NO_FREEZE;
}

PIDISA_controller::PIDISA_controller(double Kc, double b, double Ts, double uMin, double uMax) {
    // Check parameter consistency
    if (Ts == 0.0)
        throw std::invalid_argument("PID sampling time cannot be zero");
    if (Kc == 0.0)
        throw std::invalid_argument("PID parameters cannot be all zero");
    if (uMin > uMax)
        throw std::invalid_argument("Lower control saturation cannot be greater then higher");
    if ((b < 0) || (b > 1))
        throw std::invalid_argument("Weight b should be in the range [0 1]");

    // Initialise PID parameters
    this->Ts = Ts;
    this->Kc = Kc;
    this->b = b;
    this->Ti = 0;
    this->Td = 0;
    this->N = 0;
    this->uMin = uMin;
    this->uMax = uMax;

    // No integral action
    this->a1 = 0.0;

    // No derivative action
    this->b1 = this->b2 = 0.0;

    // Initialise PID state variables
    y_old = ysp_old = u_old = uI = uD = 0.0;

    // Initialize PID logic states
    controller_state = PID_state::AUTO;
    actuation_state = actuator_state::NO_SATURATION;
    controlSignal_state = control_state::NO_FREEZE;
}

PIDISA_controller::~PIDISA_controller() {
    // Do nothing
}

void PIDISA_controller::evaluate(double y, double ysp, double trk, double &u) {
    // Compute derivative action
    uD = b1 * uD + b2 * ( (c*ysp-y) - (c*ysp_old-y_old));

    // Manage AUTO/TRACKING behaviour
    if (controller_state == PID_state::AUTO) {
        u = Kc * (b*ysp-y) + uI + uD;
    } else if (controller_state == PID_state::TRACKING) {
        u = trk;
    } else {
        u = 0.0;
        throw std::invalid_argument("Unknown PID state");
    }

    // Check windup and freeze conditions
    if (u > uMax) {
        u = uMax;
        actuation_state = actuator_state::SATURATION_UP;
    } else if (u < uMin) {
        u = uMin;
        actuation_state = actuator_state::SATURATION_DOWN;
    } else if ((controller_state == PID_state::AUTO) && (controlSignal_state == control_state::NO_FREEZE)) {
        uI += a1 * (ysp - y);
        actuation_state = actuator_state::NO_SATURATION;
    } else if ((controller_state == PID_state::AUTO) && (controlSignal_state == control_state::FREEZE_UP)) {
        if (u > u_old) {
            u = u_old;
            uI = u - Kc * (b*ysp-y) - uD;
        } else {
            uI += a1 * (ysp - y);
        }
        actuation_state = actuator_state::NO_SATURATION;
    } else if ((controller_state == PID_state::AUTO) && (controlSignal_state == control_state::FREEZE_DOWN)) {
        if (u < u_old) {
            u = u_old;
            uI = u - Kc * (b*ysp-y) - uD;
        } else {
            uI += a1 * (ysp - y);
        }
        actuation_state = actuator_state::NO_SATURATION;
    } else {
        uI = u - Kc * (b*ysp-y) - uD;
        actuation_state = actuator_state::NO_SATURATION;
    }

    // Update PID states
    y_old = y;
    ysp_old = ysp;
    u_old = u;
}

void PIDISA_controller::resetState() {
    // Reset PID state
    y_old = ysp_old = u_old = uI = uD = 0.0;
}
