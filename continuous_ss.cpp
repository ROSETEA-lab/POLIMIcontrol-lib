#include "continuous_ss.h"

#include <iostream>
#include <stdexcept>


continuous_ss::continuous_ss(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C, const Eigen::MatrixXd& D, const double sampling_time, const Eigen::VectorXd initial_state)
{
    // Check parameter consistency
    if (((A.rows()<1) || (A.cols()<1)) || ((B.rows()<1) || (B.cols()<1)) || ((C.rows()<1) || (C.cols()<1))) {
        this->A = Eigen::MatrixXd::Zero(1,1);
        this->B = Eigen::MatrixXd::Zero(1,1);
        this->C = Eigen::MatrixXd::Zero(1,1);
        this->D = Eigen::MatrixXd::Zero(1,1);

        this->n = this->m = this->p = 1;

        this->time = 0.0;
        this->sampling_time = sampling_time;

        throw std::invalid_argument("Matrix A, B, C should have at least one column and one row.");
    }
    else if (A.rows()!=A.cols()) {
        this->A = Eigen::MatrixXd::Zero(1,1);
        this->B = Eigen::MatrixXd::Zero(1,1);
        this->C = Eigen::MatrixXd::Zero(1,1);
        this->D = Eigen::MatrixXd::Zero(1,1);

        this->n = this->m = this->p = 1;

        this->time = 0.0;
        this->sampling_time = sampling_time;

        throw std::invalid_argument("Matrix A should be square.");
    }
    else if (B.rows()!=A.rows()) {
        this->A = Eigen::MatrixXd::Zero(1,1);
        this->B = Eigen::MatrixXd::Zero(1,1);
        this->C = Eigen::MatrixXd::Zero(1,1);
        this->D = Eigen::MatrixXd::Zero(1,1);

        this->n = this->m = this->p = 1;

        this->time = 0.0;
        this->sampling_time = sampling_time;

        throw std::invalid_argument("Matrix B has a wrong number of rows.");
    }
    else if (C.cols()!=A.rows()) {
        this->A = Eigen::MatrixXd::Zero(1,1);
        this->B = Eigen::MatrixXd::Zero(1,1);
        this->C = Eigen::MatrixXd::Zero(1,1);
        this->D = Eigen::MatrixXd::Zero(1,1);

        this->n = this->m = this->p = 1;

        this->time = 0.0;
        this->sampling_time = sampling_time;

        throw std::invalid_argument("Matrix C has a wrong number of cols.");
    }
    else if ((D.rows()!=C.rows()) || (D.cols()!=B.cols())) {
        this->A = Eigen::MatrixXd::Zero(1,1);
        this->B = Eigen::MatrixXd::Zero(1,1);
        this->C = Eigen::MatrixXd::Zero(1,1);
        this->D = Eigen::MatrixXd::Zero(1,1);

        this->n = this->m = this->p = 1;

        this->time = 0.0;
        this->sampling_time = sampling_time;

        throw std::invalid_argument("Matrix D has a wrong number of rows or cols.");
    }
    else if (sampling_time<0.0) {
        this->A = Eigen::MatrixXd::Zero(1,1);
        this->B = Eigen::MatrixXd::Zero(1,1);
        this->C = Eigen::MatrixXd::Zero(1,1);
        this->D = Eigen::MatrixXd::Zero(1,1);

        this->n = this->m = this->p = 1;

        this->time = 0.0;
        this->sampling_time = 1;

        throw std::invalid_argument("Sampling time cannot be negative.");
    }
    else {
        // Initialize coefficient vectors
        this->A = A;
        this->B = B;
        this->C = C;
        this->D = D;

        this->n = A.rows();
        this->m = B.cols();
        this->p = C.rows();

        this->time = 0.0;
        this->sampling_time = sampling_time;

        // Initialize state vectors
        state      = initial_state;
        state_next = Eigen::VectorXd::Zero(n);
        output     = Eigen::VectorXd::Zero(p);
    }
}

continuous_ss::continuous_ss(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C, const Eigen::MatrixXd& D, const double sampling_time) :
    continuous_ss(A, B, C, D, sampling_time, Eigen::VectorXd::Zero(A.rows()))
{
    // Do nothing
}

continuous_ss::continuous_ss(double sampling_time, int num_param, const Eigen::VectorXd& initial_state)
{
    // Dummy initialization of system variables (should be rewritten by user implemented virtual function)
    this->A = Eigen::MatrixXd::Zero(1,1);
    this->B = Eigen::MatrixXd::Zero(1,1);
    this->C = Eigen::MatrixXd::Zero(1,1);
    this->D = Eigen::MatrixXd::Zero(1,1);

    this->n = this->m = this->p = 1;

    // Initializing class attributes
    this->time = 0.0;
    this->sampling_time = sampling_time;
    this->num_param = num_param;

    // Determine matrix size
    Eigen::VectorXd params = Eigen::VectorXd::Zero(1);
    compute_state_matrices(A, B, C, D, time, params);

    n = A.rows();
    m = B.cols();
    p = C.rows();

    // Initialize state vectors
    state      = initial_state;
    state_next = Eigen::VectorXd::Zero(n);
    output     = Eigen::VectorXd::Zero(p);
}

continuous_ss::~continuous_ss()
{
    // Do nothing
}

void continuous_ss::compute_state_matrices(Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::MatrixXd& C, Eigen::MatrixXd& D, double time, const Eigen::VectorXd& param)
{
    // Standard behaviour for LTI systems
    // This function should be reimplemented in case of LTV or LPV systems
    A = this->A;
    B = this->B;
    C = this->C;
    D = this->D;
}

void continuous_ss::evaluate(const Eigen::VectorXd& input)
{
    // Check parameter consistency
    if (num_param!=0) {
        state      = Eigen::VectorXd::Zero(n);
        state_next = Eigen::VectorXd::Zero(n);
        output     = Eigen::VectorXd::Zero(p);

        time = 0.0;

        throw std::invalid_argument("If state-space matrices depend on parameters you must use the appropriate evaluate function.");
    }
    else {
        // Compute state matrices
        Eigen::VectorXd params = Eigen::VectorXd::Zero(1);
        compute_state_matrices(A, B, C, D, time, params);

        // Update state and output
        if (time<sampling_time) {
            state_next = state + (A*state+B*input)*sampling_time;
            output     = C*state+D*input;
        } else {
            state       = state_next;
            output      = C*state+D*input;
            state_next  = state + (A*state+B*input)*sampling_time;
        }

        // Increment time
        time += sampling_time;
    }
}

void continuous_ss::evaluate(const Eigen::VectorXd& input, const Eigen::VectorXd& params)
{
    // Check parameter consistency
    if (num_param!=params.size()) {
        state      = Eigen::VectorXd::Zero(n);
        state_next = Eigen::VectorXd::Zero(n);
        output     = Eigen::VectorXd::Zero(p);

        time = 0.0;

        throw std::invalid_argument("Evaluate called with a wrong number of parameters.");
    }
    else if (num_param==0) {
        state      = Eigen::VectorXd::Zero(n);
        output     = Eigen::VectorXd::Zero(p);

        time = 0.0;

        throw std::invalid_argument("If state-space matrices do not depend on parameters you must use the appropriate evaluate function.");
    }
    else {
        // Compute state matrices
        compute_state_matrices(A, B, C, D, time, params);

        // Update state and output
        if (time<sampling_time) {
            state_next = state + (A*state+B*input)*sampling_time;
            output     = C*state+D*input;
        } else {
            state       = state_next;
            output      = C*state+D*input;
            state_next  = state + (A*state+B*input)*sampling_time;
        }

        // Increment time
        time += sampling_time;
    }
}

void continuous_ss::reset_state()
{
    state      = Eigen::VectorXd::Zero(n);
    output     = Eigen::VectorXd::Zero(p);
}
