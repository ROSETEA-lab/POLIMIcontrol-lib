#ifndef DISCRETE_SS_H_
#define DISCRETE_SS_H_

#include <vector>
#include <eigen3/Eigen/Dense>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Discrete state-space system class
//
// Implements the state-space form of a linear time-invariant/time-variant/LPV discrete time system
//
// x(k+1) = A(k,theta) x(k) + B(k,theta) u(k)
// y(k)   = C(k,theta) x(k) + D(k,theta) u(k)
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class discrete_ss
{
    private:
        int time;
        int n, m, p, num_param;

        Eigen::MatrixXd A, B, C, D;
        Eigen::VectorXd state, state_next, output;

        virtual void compute_state_matrices(Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::MatrixXd& C, Eigen::MatrixXd& D, int time, const Eigen::VectorXd& param);

    public:
        discrete_ss(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C, const Eigen::MatrixXd& D);
        discrete_ss(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C, const Eigen::MatrixXd& D, const Eigen::VectorXd initial_state);

        discrete_ss(int num_param, const Eigen::VectorXd& initial_state);

        ~discrete_ss();

        void evaluate(const Eigen::VectorXd& input);
        void evaluate(const Eigen::VectorXd& input, const Eigen::VectorXd& params);
        void reset_state();

        void get_state(Eigen::VectorXd& state) { state = this->state; };
        void get_output(Eigen::VectorXd& output) { output = this->output; };
};

#endif // DISCRETE_SS_H_
