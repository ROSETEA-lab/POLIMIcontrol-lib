#ifndef DISCRETE_SS_H_
#define DISCRETE_SS_H_

#include <vector>
#include <Eigen/Dense>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Discrete state-space system class
//
// Implements the state-space form of a linear time-invariant discrete time system
//
// x(k+1) = A x(k) + B u(k)
// y(k)   = C x(k) + D u(k)
//
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class discrete_ss
{
    private:
        Eigen::MatrixXd A, B, C, D;
        Eigen::VectorXd state, output;

    public:
        discrete_ss(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C);
        discrete_ss(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C, const Eigen::MatrixXd& D);
        ~discrete_ss();

        void evaluate(Eigen::VectorXd& input);
        void reset_state();

        void get_state(Eigen::VectorXd& state) { state = this->state; };
        void get_output(Eigen::VectorXd& output) { output = this->output; };
};

#endif // DISCRETE_SS_H_