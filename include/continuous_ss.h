#ifndef CONTINUOUS_SS_H_
#define CONTINUOUS_SS_H_

#include <vector>
#include <eigen3/Eigen/Dense>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Continuous state-space system class
//
// Implements the state-space form of a linear time-invariant/time-variant/LPV continuous time system
//
// dx/dt = A(t,theta) x(t) + B(t,theta) u(t)
// y(t)  = C(t,theta) x(t) + D(t,theta) u(t)
//
// The state derivative is discretized using backward Euler
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class continuous_ss
{
    private:
        double time;
        double sampling_time;
        int n, m, p, num_param;

        Eigen::MatrixXd A, B, C, D;
        Eigen::VectorXd state, state_next, output;

        virtual void compute_state_matrices(Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::MatrixXd& C, Eigen::MatrixXd& D, double time, const Eigen::VectorXd& param);

    public:
        continuous_ss(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C, const Eigen::MatrixXd& D, const double sampling_time);
        continuous_ss(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C, const Eigen::MatrixXd& D, const double sampling_time, const Eigen::VectorXd initial_state);

        continuous_ss(double sampling_time, int num_param, const Eigen::VectorXd& initial_state);

        ~continuous_ss();

        void evaluate(const Eigen::VectorXd& input);
        void evaluate(const Eigen::VectorXd& input, const Eigen::VectorXd& params);
        void reset_state();

        void get_state(Eigen::VectorXd& state) { state = this->state; };
        void get_output(Eigen::VectorXd& output) { output = this->output; };
};

#endif // CONTINUOUS_SS_H_
