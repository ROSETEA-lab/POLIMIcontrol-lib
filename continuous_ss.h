#ifndef CONTINUOUS_SS_H_
#define CONTINUOUS_SS_H_

#include <vector>
#include <Eigen/Dense>

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

        bool extern_matrix_computation;
        typedef void (*compute_matrix)(Eigen::MatrixXd&,double,const Eigen::VectorXd&);
        compute_matrix compute_matrix_A, compute_matrix_B, compute_matrix_C, compute_matrix_D;
        typedef void (*compute_matrices)(Eigen::MatrixXd&,Eigen::MatrixXd&,Eigen::MatrixXd&,Eigen::MatrixXd&,double,const Eigen::VectorXd&);
        compute_matrices compute_matrices_ABCD;

    public:
        continuous_ss(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C, const Eigen::MatrixXd& D, const double sampling_time);
        continuous_ss(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C, const Eigen::MatrixXd& D, const double sampling_time, const Eigen::VectorXd initial_state);

        continuous_ss(const compute_matrix pMatrix_A, const compute_matrix pMatrix_B, const compute_matrix pMatrix_C, const compute_matrix pMatrix_D, const double sampling_time, const int num_param, const Eigen::VectorXd& initial_state);
        continuous_ss(const compute_matrices pMatrices_ABCD, const double sampling_time, const int num_param, const Eigen::VectorXd& initial_state);

        ~continuous_ss();

        void evaluate(const Eigen::VectorXd& input);
        void evaluate(const Eigen::VectorXd& input, const Eigen::VectorXd& params);
        void reset_state();

        void get_state(Eigen::VectorXd& state) { state = this->state; };
        void get_output(Eigen::VectorXd& output) { output = this->output; };
};

#endif // CONTINUOUS_SS_H_
