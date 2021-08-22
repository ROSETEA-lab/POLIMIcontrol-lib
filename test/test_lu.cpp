#include <iostream>
#include <Eigen/Dense>

int main(int argc, char **argv)
{
    // A=[1,0,2; 2,1,-1; 0,3,-1];
    Eigen::MatrixXd A(3,3);
    A << 1,0,2, 2,1,-1, 0,3,-1;

    Eigen::PartialPivLU<Eigen::MatrixXd> A_lu = Eigen::PartialPivLU<Eigen::MatrixXd>(A);

    Eigen::MatrixXd L = A_lu.matrixLU().triangularView<Eigen::UpLoType::UnitLower>();
    Eigen::MatrixXd U = A_lu.matrixLU().triangularView<Eigen::UpLoType::Upper>();
    Eigen::MatrixXd P = A_lu.permutationP().transpose();
    L = P*L;

    std::cout << "A:" << std::endl << A << std::endl << std::endl;
    std::cout << "L:" << std::endl << L << std::endl << std::endl;
    std::cout << "U:" << std::endl << U << std::endl << std::endl;

	return 0;
}
