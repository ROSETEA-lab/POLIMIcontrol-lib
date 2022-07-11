#ifndef PID_CONTROLLER_H_
#define PID_CONTROLLER_H_

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// PID controller class
//
// Implements the tranfer function of a digital PID controller, that is obtained applying Backward-Euler transformation
// to the following continuos time transfer function
//
//           [       1          s Td     ]
// R(s) = Kc [ 1 + ------ + ------------ ]
//           [      s Ti     1 + s Td/N  ]
//
// An anti-windup is also implemented, based on conditional integration
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class PID_controller
{
    private:
        /* PID parameters */
        double Ts;                      // Sampling time
        double Kc, Ti, Td, N;           // PID parameters (proportional gain, integral/derivative time)
        double uMin, uMax;              // Output saturation values
        double a1, b1, b2;              // PID equation internal coefficients
    
        /* PID states */
        double e_old, uI_old, uD_old;   // PID internal states

    public:
        PID_controller(double Kc, double Ts, double uMin, double uMax);
        PID_controller(double Kc, double Ti, double Ts, double uMin, double uMax);
        PID_controller(double Kc, double Td, double N, double Ts, double uMin, double uMax);
        PID_controller(double Kc, double Ti, double Td, double N, double Ts, double uMin, double uMax);
        ~PID_controller();

        void evaluate(double y, double ysp, double& u);
        void resetState();
};

#endif /* PID_CONTROLLER_H_ */
