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
        int time;
        int n, m, p, num_param;

        Eigen::MatrixXd A, B, C, D;
        Eigen::VectorXd state, state_next, output;

        bool extern_matrix_computation;
        typedef void (*compute_matrix)(Eigen::MatrixXd&,int,const Eigen::VectorXd&);
        compute_matrix compute_matrix_A, compute_matrix_B, compute_matrix_C, compute_matrix_D;

    public:
        discrete_ss(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C, const Eigen::MatrixXd& D);
        discrete_ss(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C, const Eigen::MatrixXd& D, const Eigen::VectorXd initial_state);

        discrete_ss(const compute_matrix pMatrix_A, const compute_matrix pMatrix_B, const compute_matrix pMatrix_C, const compute_matrix pMatrix_D, const int num_param, const Eigen::VectorXd& initial_state);

        ~discrete_ss();

        void evaluate(const Eigen::VectorXd& input);
        void evaluate(const Eigen::VectorXd& input, const Eigen::VectorXd& params);
        void reset_state();

        void get_state(Eigen::VectorXd& state) { state = this->state; };
        void get_output(Eigen::VectorXd& output) { output = this->output; };
};

#endif // DISCRETE_SS_H_
