#include "discrete_ss.h"

#include <iostream>
#include <stdexcept>


discrete_ss::discrete_ss(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C)
{
    // Check parameter consistency
    if (((A.rows()<1) || (A.cols()<1)) || ((B.rows()<1) || (B.cols()<1)) || ((C.rows()<1) || (C.cols()<1))) {
        this->A = Eigen::MatrixXd::Zero(1,1);
        this->B = Eigen::MatrixXd::Zero(1,1);
        this->C = Eigen::MatrixXd::Zero(1,1);
        this->D = Eigen::MatrixXd::Zero(1,1);

        throw std::invalid_argument( "Matrix A, B, C should have at least one column and one row.");
    }
    else if (A.rows()!=A.cols()) {
        this->A = Eigen::MatrixXd::Zero(1,1);
        this->B = Eigen::MatrixXd::Zero(1,1);
        this->C = Eigen::MatrixXd::Zero(1,1);
        this->D = Eigen::MatrixXd::Zero(1,1);

        throw std::invalid_argument( "Matrix A should be square.");
    }
    else if (B.rows()!=A.rows()) {
        this->A = Eigen::MatrixXd::Zero(1,1);
        this->B = Eigen::MatrixXd::Zero(1,1);
        this->C = Eigen::MatrixXd::Zero(1,1);
        this->D = Eigen::MatrixXd::Zero(1,1);

        throw std::invalid_argument( "Matrix B has a wrong number of rows.");
    }
    else if (C.cols()!=A.rows()) {
        this->A = Eigen::MatrixXd::Zero(1,1);
        this->B = Eigen::MatrixXd::Zero(1,1);
        this->C = Eigen::MatrixXd::Zero(1,1);
        this->D = Eigen::MatrixXd::Zero(1,1);

        throw std::invalid_argument( "Matrix C has a wrong number of cols.");
    }
    else {
        // Initialize coefficient vectors
        this->A = A;
        this->B = B;
        this->C = C;
        this->D = Eigen::MatrixXd::Zero(C.rows(),B.cols());

        // Initialize state vectors
        state  = Eigen::VectorXd::Zero(A.rows());
        output = Eigen::VectorXd::Zero(A.rows());
    }
}

discrete_ss::discrete_ss(const Eigen::MatrixXd& A, const Eigen::MatrixXd& B, const Eigen::MatrixXd& C, const Eigen::MatrixXd& D)
{
    // Check parameter consistency
    if (((A.rows()<1) || (A.cols()<1)) || ((B.rows()<1) || (B.cols()<1)) || ((C.rows()<1) || (C.cols()<1))) {
        this->A = Eigen::MatrixXd::Zero(1,1);
        this->B = Eigen::MatrixXd::Zero(1,1);
        this->C = Eigen::MatrixXd::Zero(1,1);
        this->D = Eigen::MatrixXd::Zero(1,1);

        throw std::invalid_argument( "Matrix A, B, C should have at least one column and one row.");
    }
    else if (A.rows()!=A.cols()) {
        this->A = Eigen::MatrixXd::Zero(1,1);
        this->B = Eigen::MatrixXd::Zero(1,1);
        this->C = Eigen::MatrixXd::Zero(1,1);
        this->D = Eigen::MatrixXd::Zero(1,1);

        throw std::invalid_argument( "Matrix A should be square.");
    }
    else if (B.rows()!=A.rows()) {
        this->A = Eigen::MatrixXd::Zero(1,1);
        this->B = Eigen::MatrixXd::Zero(1,1);
        this->C = Eigen::MatrixXd::Zero(1,1);
        this->D = Eigen::MatrixXd::Zero(1,1);

        throw std::invalid_argument( "Matrix B has a wrong number of rows.");
    }
    else if (C.cols()!=A.rows()) {
        this->A = Eigen::MatrixXd::Zero(1,1);
        this->B = Eigen::MatrixXd::Zero(1,1);
        this->C = Eigen::MatrixXd::Zero(1,1);
        this->D = Eigen::MatrixXd::Zero(1,1);

        throw std::invalid_argument( "Matrix C has a wrong number of cols.");
    }
    else if ((D.rows()!=C.rows()) || (D.cols()!=B.cols())) {
        this->A = Eigen::MatrixXd::Zero(1,1);
        this->B = Eigen::MatrixXd::Zero(1,1);
        this->C = Eigen::MatrixXd::Zero(1,1);
        this->D = Eigen::MatrixXd::Zero(1,1);

        throw std::invalid_argument( "Matrix D has a wrong number of rows or cols.");
    }
    else {
        // Initialize coefficient vectors
        this->A = A;
        this->B = B;
        this->C = C;
        this->D = D;

        // Initialize state vectors
        state  = Eigen::VectorXd::Zero(A.rows());
        output = Eigen::VectorXd::Zero(A.rows());
    }
}

discrete_ss::~discrete_ss()
{
    // Do nothing
}

void discrete_ss::evaluate(Eigen::VectorXd& input)
{
    // Evaluate the filter state and output
    output = C*state+D*input;
    state  = A*state+B*input;
}

void discrete_ss::reset_state()
{
    state  = Eigen::VectorXd::Zero(A.rows());
    output = Eigen::VectorXd::Zero(A.rows());
}
