#include "matfile_fun.h"

void extractMatrixFromCellarray(Eigen::Ref<Eigen::MatrixXd> matrix, matvar_t **matrixMat, int mat_idx)
{
    // Get matrix dimensions
    int mat_row = (int) matrixMat[mat_idx]->dims[0];
    int mat_col = (int) matrixMat[mat_idx]->dims[1];

    // Initialize matrix
    matrix = Eigen::MatrixXd::Zero(mat_row, mat_col);

    // Get a pointer to matrix data
    double *matrixMat_data = (double *)matrixMat[mat_idx]->data;

    // Copy data to an eigen matrix
    int data_idx = 0;
    for (auto col=0; col<mat_col; col++)
    {
        for (auto row=0; row<mat_row; row++)
        {
            matrix(row,col) = matrixMat_data[data_idx++];
        }
    }
}

void extractCellArray(std::vector<Eigen::MatrixXd>& matrixVect, matvar_t *cellarrayMat)
{
    // Get a pointer to cell array elements
    matvar_t **cellarray = (matvar_t **)cellarrayMat->data;

    // Get number of elements in cell array
    int cellarray_num = (int) cellarrayMat->dims[1];

    // Extract matrices
    Eigen::MatrixXd eigenMat(cellarray[0]->dims[0],cellarray[0]->dims[1]);
    for (auto k=0; k<cellarray_num; k++)
    {
        extractMatrixFromCellarray(eigenMat, cellarray, k);
        matrixVect.push_back(eigenMat);
    }
}
