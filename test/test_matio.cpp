#include <vector>
#include <iostream>
#include <Eigen/Dense>
#include <matio.h>

void extractMatrixFromCellarray(Eigen::Ref<Eigen::MatrixXd> matrix, matvar_t **matrixMat, int mat_idx);
void extractCellArray(std::vector<Eigen::MatrixXd>& matrixVect, matvar_t *cellarrayMat);


int main(int argc, char **argv)
{
    // Open mat file
    const char *fileName = "test_matio.mat";
    mat_t *mat = Mat_Open(fileName,MAT_ACC_RDONLY);

    if(mat)
    {
        std::cout << "mat file opened successfully!" << std::endl;

        // Reading matrices from structure with fields
        // controller = struct with fields:
        //        Aki: cell array of matrix
        //        Bki: cell array of matrix
        //        Cki: cell array of matrix
        //        Dki: cell array of matrix

        matvar_t *matVar = NULL;
        matVar = Mat_VarRead(mat, (char*)"controller") ;
        if(matVar)
        {
            // Get a pointer to the structure fields
            matvar_t **fields = (matvar_t **)matVar->data;
            matvar_t *Aki_cellarray = (matvar_t *)fields[0];
            matvar_t *Bki_cellarray = (matvar_t *)fields[1];
            matvar_t *Cki_cellarray = (matvar_t *)fields[2];
            matvar_t *Dki_cellarray = (matvar_t *)fields[3];

            // Aki
            std::vector<Eigen::MatrixXd> Aki;
            extractCellArray(Aki, Aki_cellarray);
            for (auto k=0; k<Aki.size(); k++)
            {
                std::cout << "Ak " << k+1 << ": " << std::endl << Aki.at(k) << std::endl << std::endl;
            }

            // Bki
            std::vector<Eigen::MatrixXd> Bki;
            extractCellArray(Bki, Bki_cellarray);
            for (auto k=0; k<Bki.size(); k++)
            {
                std::cout << "Bk " << k+1 << ": " << std::endl << Bki.at(k) << std::endl << std::endl;
            }

            // Cki
            std::vector<Eigen::MatrixXd> Cki;
            extractCellArray(Cki, Cki_cellarray);
            for (auto k=0; k<Cki.size(); k++)
            {
                std::cout << "Ck " << k+1 << ": " << std::endl << Cki.at(k) << std::endl << std::endl;
            }

            // Dki
            std::vector<Eigen::MatrixXd> Dki;
            extractCellArray(Dki, Dki_cellarray);
            for (auto k=0; k<Dki.size(); k++)
            {
                std::cout << "Dk " << k+1 << ": " << std::endl << Dki.at(k) << std::endl << std::endl;
            }
        }
        else
        {
            std::cout << "Cannot read variable from mat file!" << std::endl;
        }

        // Reading matrices from cell array
        //        Ai: cell array of matrix
        //        Bi: cell array of matrix
        //        Ci: cell array of matrix
        //        Di: cell array of matrix

        // Ai
        matVar = NULL;
        matVar = Mat_VarRead(mat, (char*)"Ai") ;
        if (matVar)
        {
            std::vector<Eigen::MatrixXd> Ai;
            extractCellArray(Ai, matVar);
            for (auto k=0; k<Ai.size(); k++)
            {
                std::cout << "A " << k+1 << ": " << std::endl << Ai.at(k) << std::endl << std::endl;
            }
        }

        // Bi
        matVar = NULL;
        matVar = Mat_VarRead(mat, (char*)"Bi") ;
        if (matVar)
        {
            std::vector<Eigen::MatrixXd> Bi;
            extractCellArray(Bi, matVar);
            for (auto k=0; k<Bi.size(); k++)
            {
                std::cout << "B " << k+1 << ": " << std::endl << Bi.at(k) << std::endl << std::endl;
            }
        }

        // Ci
        matVar = NULL;
        matVar = Mat_VarRead(mat, (char*)"Ci") ;
        if (matVar)
        {
            std::vector<Eigen::MatrixXd> Ci;
            extractCellArray(Ci, matVar);
            for (auto k=0; k<Ci.size(); k++)
            {
                std::cout << "C " << k+1 << ": " << std::endl << Ci.at(k) << std::endl << std::endl;
            }
        }

        // Di
        matVar = NULL;
        matVar = Mat_VarRead(mat, (char*)"Di") ;
        if (matVar)
        {
            std::vector<Eigen::MatrixXd> Di;
            extractCellArray(Di, matVar);
            for (auto k=0; k<Di.size(); k++)
            {
                std::cout << "D " << k+1 << ": " << std::endl << Di.at(k) << std::endl << std::endl;
            }
        }

        Mat_Close(mat);
    }
    else
    {
        std::cout << "Error opening the mat file" << std::endl;
        return 1;
    }
    return 0;
}


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