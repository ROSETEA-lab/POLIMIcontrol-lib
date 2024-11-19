#ifndef MATFILE_FUN_H_
#define MATFILE_FUN_H_

#include <vector>
#include <matio.h>
#include <eigen3/Eigen/Dense>

void extractMatrixFromCellarray(Eigen::Ref<Eigen::MatrixXd> matrix, matvar_t **matrixMat, int mat_idx);
void extractCellArray(std::vector<Eigen::MatrixXd>& matrixVect, matvar_t *cellarrayMat);

#endif /* MATFILE_FUN_H_ */
