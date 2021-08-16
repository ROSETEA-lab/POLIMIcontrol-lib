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

        this->extern_matrix_computation = false;
        this->compute_matrix_A = this->compute_matrix_B = this->compute_matrix_C = this->compute_matrix_D = NULL;

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

        this->extern_matrix_computation = false;
        this->compute_matrix_A = this->compute_matrix_B = this->compute_matrix_C = this->compute_matrix_D = NULL;

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

        this->extern_matrix_computation = false;
        this->compute_matrix_A = this->compute_matrix_B = this->compute_matrix_C = this->compute_matrix_D = NULL;

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

        this->extern_matrix_computation = false;
        this->compute_matrix_A = this->compute_matrix_B = this->compute_matrix_C = this->compute_matrix_D = NULL;

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

        this->extern_matrix_computation = false;
        this->compute_matrix_A = this->compute_matrix_B = this->compute_matrix_C = this->compute_matrix_D = NULL;

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

        this->extern_matrix_computation = false;
        this->compute_matrix_A = this->compute_matrix_B = this->compute_matrix_C = this->compute_matrix_D = NULL;

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

        this->extern_matrix_computation = false;
        this->compute_matrix_A = this->compute_matrix_B = this->compute_matrix_C = this->compute_matrix_D = NULL;

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

continuous_ss::continuous_ss(const compute_matrix pMatrix_A, const compute_matrix pMatrix_B, const compute_matrix pMatrix_C, const compute_matrix pMatrix_D, const double sampling_time, const int num_param, const Eigen::VectorXd& initial_state)
{
    // Initialize pointers to external functions computing system matrices
    this->compute_matrix_A = pMatrix_A;
    this->compute_matrix_B = pMatrix_B;
    this->compute_matrix_C = pMatrix_C;
    this->compute_matrix_D = pMatrix_D;

    this->compute_matrices_ABCD = NULL;

    this->num_param = num_param;

    this->extern_matrix_computation = true;

    this->time = 0.0;
    this->sampling_time = sampling_time;

    // Determine matrix size
    Eigen::VectorXd params = Eigen::VectorXd::Zero(1);
    compute_matrix_A(A, time, params);
    compute_matrix_B(B, time, params);
    compute_matrix_C(C, time, params);
    compute_matrix_D(D, time, params);

    n = A.rows();
    m = B.cols();
    p = C.rows();

    // Initialize state vectors
    state      = initial_state;
    state_next = Eigen::VectorXd::Zero(n);
    output     = Eigen::VectorXd::Zero(p);
}

continuous_ss::continuous_ss(const compute_matrices pMatrices_ABCD, const double sampling_time, const int num_param, const Eigen::VectorXd& initial_state)
{
    // Initialize pointer to external function computing system matrices
    this->compute_matrices_ABCD = pMatrices_ABCD;

    this->compute_matrix_A = NULL;
    this->compute_matrix_B = NULL;
    this->compute_matrix_C = NULL;
    this->compute_matrix_D = NULL;

    this->num_param = num_param;

    this->extern_matrix_computation = true;

    this->time = 0.0;
    this->sampling_time = sampling_time;

    // Determine matrix size
    Eigen::VectorXd params = Eigen::VectorXd::Zero(1);
    compute_matrices_ABCD(A, B, C, D, time, params);

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
        // Compute state matrices using external functions
        if (extern_matrix_computation && !this->compute_matrices_ABCD) {
            Eigen::VectorXd params = Eigen::VectorXd::Zero(1);
            compute_matrix_A(A, time, params);
            compute_matrix_B(B, time, params);
            compute_matrix_C(C, time, params);
            compute_matrix_D(D, time, params);
        }
        if (extern_matrix_computation && this->compute_matrices_ABCD) {
            Eigen::VectorXd params = Eigen::VectorXd::Zero(1);
            compute_matrices_ABCD(A, B, C, D, time, params);
        }

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
    else if (!extern_matrix_computation) {
        state      = Eigen::VectorXd::Zero(n);
        output     = Eigen::VectorXd::Zero(p);

        time = 0.0;

        throw std::invalid_argument("Evaluate with parameters can be called only in the case of state-space matrices computed using user-defined functions.");
    }
    else {
        // Compute state matrices using external functions
        if (!this->compute_matrices_ABCD) {
            compute_matrix_A(A, time, params);
            compute_matrix_B(B, time, params);
            compute_matrix_C(C, time, params);
            compute_matrix_D(D, time, params);
        }
        else {
            compute_matrices_ABCD(A, B, C, D, time, params);
        }

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
