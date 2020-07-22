#include "discrete_ss.h"

#include <iostream>
#include <stdexcept>


discrete_ss::discrete_ss(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C, const Eigen::MatrixXd& D, const Eigen::VectorXd initial_state)
{
    // Check parameter consistency
    if (((A.rows()<1) || (A.cols()<1)) || ((B.rows()<1) || (B.cols()<1)) || ((C.rows()<1) || (C.cols()<1))) {
        this->A = Eigen::MatrixXd::Zero(1,1);
        this->B = Eigen::MatrixXd::Zero(1,1);
        this->C = Eigen::MatrixXd::Zero(1,1);
        this->D = Eigen::MatrixXd::Zero(1,1);

        this->n = this->m = this->p = 1;

        this->time = 0;

        this->extern_matrix_computation = false;
        this->compute_matrix_A = this->compute_matrix_B = this->compute_matrix_C = this->compute_matrix_D = NULL;

        throw std::invalid_argument( "Matrix A, B, C should have at least one column and one row.");
    }
    else if (A.rows()!=A.cols()) {
        this->A = Eigen::MatrixXd::Zero(1,1);
        this->B = Eigen::MatrixXd::Zero(1,1);
        this->C = Eigen::MatrixXd::Zero(1,1);
        this->D = Eigen::MatrixXd::Zero(1,1);

        this->n = this->m = this->p = 1;

        this->time = 0;

        this->extern_matrix_computation = false;
        this->compute_matrix_A = this->compute_matrix_B = this->compute_matrix_C = this->compute_matrix_D = NULL;

        throw std::invalid_argument( "Matrix A should be square.");
    }
    else if (B.rows()!=A.rows()) {
        this->A = Eigen::MatrixXd::Zero(1,1);
        this->B = Eigen::MatrixXd::Zero(1,1);
        this->C = Eigen::MatrixXd::Zero(1,1);
        this->D = Eigen::MatrixXd::Zero(1,1);

        this->n = this->m = this->p = 1;

        this->time = 0;

        this->extern_matrix_computation = false;
        this->compute_matrix_A = this->compute_matrix_B = this->compute_matrix_C = this->compute_matrix_D = NULL;

        throw std::invalid_argument( "Matrix B has a wrong number of rows.");
    }
    else if (C.cols()!=A.rows()) {
        this->A = Eigen::MatrixXd::Zero(1,1);
        this->B = Eigen::MatrixXd::Zero(1,1);
        this->C = Eigen::MatrixXd::Zero(1,1);
        this->D = Eigen::MatrixXd::Zero(1,1);

        this->n = this->m = this->p = 1;

        this->time = 0;

        this->extern_matrix_computation = false;
        this->compute_matrix_A = this->compute_matrix_B = this->compute_matrix_C = this->compute_matrix_D = NULL;

        throw std::invalid_argument( "Matrix C has a wrong number of cols.");
    }
    else if ((D.rows()!=C.rows()) || (D.cols()!=B.cols())) {
        this->A = Eigen::MatrixXd::Zero(1,1);
        this->B = Eigen::MatrixXd::Zero(1,1);
        this->C = Eigen::MatrixXd::Zero(1,1);
        this->D = Eigen::MatrixXd::Zero(1,1);

        this->n = this->m = this->p = 1;

        this->time = 0;

        this->extern_matrix_computation = false;
        this->compute_matrix_A = this->compute_matrix_B = this->compute_matrix_C = this->compute_matrix_D = NULL;

        throw std::invalid_argument( "Matrix D has a wrong number of rows or cols.");
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

        this->time = 0;

        this->extern_matrix_computation = false;
        this->compute_matrix_A = this->compute_matrix_B = this->compute_matrix_C = this->compute_matrix_D = NULL;

        // Initialize state vectors
        state      = initial_state;
        state_next = Eigen::VectorXd::Zero(n);
        output     = Eigen::VectorXd::Zero(p);
    }
}

discrete_ss::discrete_ss(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C, const Eigen::MatrixXd& D) :
    discrete_ss(A, B, C, D, Eigen::VectorXd::Zero(A.rows()))
{
    // Do nothing
}

discrete_ss::discrete_ss(compute_matrix pMatrix_A, compute_matrix pMatrix_B, compute_matrix pMatrix_C, compute_matrix pMatrix_D, Eigen::VectorXd initial_state)
{
    // Initialize pointers to external functions computing system matrices
    this->compute_matrix_A = pMatrix_A;
    this->compute_matrix_B = pMatrix_B;
    this->compute_matrix_C = pMatrix_C;
    this->compute_matrix_D = pMatrix_D;

    this->extern_matrix_computation = true;

    this->time = 0;

    // Determine matrix size
    compute_matrix_A(A, time);
    compute_matrix_B(B, time);
    compute_matrix_C(C, time);
    compute_matrix_D(D, time);

    n = A.rows();
    m = B.cols();
    p = C.rows();

    // Initialize state vectors
    state      = initial_state;
    state_next = Eigen::VectorXd::Zero(n);
    output     = Eigen::VectorXd::Zero(p);
}

discrete_ss::~discrete_ss()
{
    // Do nothing
}

void discrete_ss::evaluate(Eigen::VectorXd& input)
{
    // Compute state matrices using external functions
    if (extern_matrix_computation) {
        compute_matrix_A(A, time);
        compute_matrix_B(B, time);
        compute_matrix_C(C, time);
        compute_matrix_D(D, time);
    }

    // Update state and output
    if (time==0) {
        state_next = A*state+B*input;
        output     = C*state+D*input;
    } else {
        state       = state_next;
        output      = C*state+D*input;
        state_next  = A*state+B*input;
    }

    // Increment time
    time++;
}

void discrete_ss::reset_state()
{
    state      = Eigen::VectorXd::Zero(n);
    state_next = Eigen::VectorXd::Zero(n);
    output     = Eigen::VectorXd::Zero(p);
}
