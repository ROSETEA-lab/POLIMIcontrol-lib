#include "AGS_controller.h"

#include <iostream>
#include "matfile_fun.h"


AGS_controller::AGS_controller(const char* mat_filename,double sampling_time, int num_param, const Eigen::VectorXd& initial_state) :
    continuous_ss(sampling_time, num_param, initial_state)
{
  // Mat file is supposed to include 4 cell arrays Aki, Bki, Cki, Dki

  // Load controller matrices from mat file
  mat_t* mat_file = Mat_Open(mat_filename,MAT_ACC_RDONLY);

  // Extract controller matrices
  if (mat_file) {
    matvar_t *matVar = NULL;

    // Aki
    matVar = NULL;
    matVar = Mat_VarRead(mat_file, (char*)"Aki") ;
    if (matVar)
    {
      extractCellArray(_Aki, matVar);
    }
    else
    {
      throw std::invalid_argument("Unable to read Matlab controller variable Aki.");
    }

    // Bki
    matVar = NULL;
    matVar = Mat_VarRead(mat_file, (char*)"Bki") ;
    if (matVar)
    {
      extractCellArray(_Bki, matVar);
    }
    else
    {
      throw std::invalid_argument("Unable to read Matlab controller variable Bki.");
    }

    // Cki
    matVar = NULL;
    matVar = Mat_VarRead(mat_file, (char*)"Cki") ;
    if (matVar)
    {
      extractCellArray(_Cki, matVar);
    }
    else
    {
      throw std::invalid_argument("Unable to read Matlab controller variable Cki.");
    }

    // Dki
    matVar = NULL;
    matVar = Mat_VarRead(mat_file, (char*)"Dki") ;
    if (matVar)
    {
      extractCellArray(_Dki, matVar);
    }
    else
    {
      throw std::invalid_argument("Unable to read Matlab controller variable Dki.");
    }

    // Xi
    matVar = NULL;
    matVar = Mat_VarRead(mat_file, (char*)"Xi") ;
    if (matVar)
    {
      extractCellArray(_Xi, matVar);
    }
    else
    {
      throw std::invalid_argument("Unable to read Matlab controller variable Xi.");
    }

    // Yi
    matVar = NULL;
    matVar = Mat_VarRead(mat_file, (char*)"Yi") ;
    if (matVar)
    {
      extractCellArray(_Yi, matVar);
    }
    else
    {
      throw std::invalid_argument("Unable to read Matlab controller variable Yi.");
    }

    // Ai
    matVar = NULL;
    matVar = Mat_VarRead(mat_file, (char*)"Ai") ;
    if (matVar)
    {
      extractCellArray(_Ai, matVar);
    }
    else
    {
      throw std::invalid_argument("Unable to read Matlab controller variable Ai.");
    }

    // Bi
    matVar = NULL;
    matVar = Mat_VarRead(mat_file, (char*)"Bi") ;
    if (matVar)
    {
      extractCellArray(_Bi, matVar);
    }
    else
    {
      throw std::invalid_argument("Unable to read Matlab controller variable Bi.");
    }

    // Ci
    matVar = NULL;
    matVar = Mat_VarRead(mat_file, (char*)"Ci") ;
    if (matVar)
    {
      extractCellArray(_Ci, matVar);
    }
    else
    {
      throw std::invalid_argument("Unable to read Matlab controller variable Ci.");
    }

    // Di
    matVar = NULL;
    matVar = Mat_VarRead(mat_file, (char*)"Di") ;
    if (matVar)
    {
      extractCellArray(_Di, matVar);
    }
    else
    {
      throw std::invalid_argument("Unable to read Matlab controller variable Di.");
    }

    // Close mat file
    Mat_Close(mat_file);
  }
  else {
    std::string filename = std::string("Unable to load controller matrix file ") + std::string(mat_filename) + std::string(" .");
    throw std::ios_base::failure(filename.c_str());
  }
}

void AGS_controller::compute_state_matrices(Eigen::MatrixXd& Ak, Eigen::MatrixXd& Bk, Eigen::MatrixXd& Ck, Eigen::MatrixXd& Dk, double t, const Eigen::VectorXd& theta)
{
  /* Compute X and Y */
  _X = _Xi.at(0);
  for (auto k=1; k<_Xi.size(); k++)
  {
    _X += theta(k-1)*_Xi.at(k);
  }

  _Y = _Yi.at(0);
  for (auto k=1; k<_Yi.size(); k++)
  {
    _Y += theta(k-1)*_Yi.at(k);
  }

  /* Compute hat and plant matrices */
  // Constant term
  _Ak_hat = _Aki.at(0);
  _Bk_hat = _Bki.at(0);
  _Ck_hat = _Cki.at(0);
  _Dk_hat = _Dki.at(0);

  _A = _Ai.at(0);
  _B = _Bi.at(0);
  _C = _Ci.at(0);

  // Theta dependent terms
  for (auto k=1; k<_Aki.size(); k++)
  {
    _Ak_hat += theta(k-1)*_Aki.at(k);
    _Bk_hat += theta(k-1)*_Bki.at(k);
    _Ck_hat += theta(k-1)*_Cki.at(k);
    _Dk_hat += theta(k-1)*_Dki.at(k);

    _A += theta(k-1)*_Ai.at(k);
    _B += theta(k-1)*_Bi.at(k);
    _C += theta(k-1)*_Ci.at(k);
  }

  /* Compute controller matrices */
  _NMt = Eigen::MatrixXd::Identity(_X.rows(),_X.cols()) - _X*_Y;

  Eigen::PartialPivLU<Eigen::MatrixXd> _NMt_lu = Eigen::PartialPivLU<Eigen::MatrixXd>(_NMt);
  _N = _NMt_lu.matrixLU().triangularView<Eigen::UpLoType::UnitLower>();
  _Mt = _NMt_lu.matrixLU().triangularView<Eigen::UpLoType::Upper>();
  Eigen::MatrixXd perm = _NMt_lu.permutationP().transpose();
  _N = perm*_N;

  _Ak = Ak = (_Mt.transpose().lu().solve((_N.lu().solve(_Ak_hat-_X*(_A-_B*_Dk_hat*_C)*_Y-_Bk_hat*_C*_Y-_X*_B*_Ck_hat)).transpose())).transpose();
  _Bk = Bk = _N.lu().solve(_Bk_hat-_X*_B*_Dk_hat);
  _Ck = Ck = (_Mt.transpose().lu().solve((_Ck_hat-_Dk_hat*_C*_Y).transpose())).transpose();
  _Dk = Dk = _Dk_hat;
}
