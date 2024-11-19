#ifndef AGS_CONTROLLER_H_
#define AGS_CONTROLLER_H_

#include <matio.h>
#include <continuous_ss.h>


class AGS_controller : public continuous_ss
{
    private:
        std::vector<Eigen::MatrixXd> _Aki, _Bki, _Cki, _Dki, _Xi, _Yi, _Ai, _Bi, _Ci, _Di;
        Eigen::MatrixXd _X, _Y, _Ak_hat, _Bk_hat, _Ck_hat, _Dk_hat, _A, _B, _C, _NMt, _N, _Mt, _Ak, _Bk, _Ck, _Dk;

        void compute_state_matrices(Eigen::MatrixXd& Ak, Eigen::MatrixXd& Bk, Eigen::MatrixXd& Ck, Eigen::MatrixXd& Dk, double t, const Eigen::VectorXd& theta);

    public:
        AGS_controller(const char* mat_filename,double sampling_time, int num_param, const Eigen::VectorXd& initial_state);

        void get_lyapunov_matrix(Eigen::MatrixXd& X, Eigen::MatrixXd& Y) { X = _X; Y = _Y; };
        void get_controller_hat_matrix(Eigen::MatrixXd& Ak_hat, Eigen::MatrixXd& Bk_hat, Eigen::MatrixXd& Ck_hat, Eigen::MatrixXd& Dk_hat) { Ak_hat = _Ak_hat; Bk_hat = _Bk_hat; Ck_hat = _Ck_hat; Dk_hat = _Dk_hat; };
        void get_plant_matrix(Eigen::MatrixXd& A, Eigen::MatrixXd& B, Eigen::MatrixXd& C) { A = _A; B = _B; C = _C; };
        void get_projection_matrix(Eigen::MatrixXd& NMt, Eigen::MatrixXd& N, Eigen::MatrixXd& Mt) { NMt = _NMt; N = _N; Mt = _Mt; };
        void get_controller_matrix(Eigen::MatrixXd& Ak, Eigen::MatrixXd& Bk, Eigen::MatrixXd& Ck, Eigen::MatrixXd& Dk) { Ak = _Ak; Bk = _Bk; Ck = _Ck; Dk = _Dk; };
};

#endif /* AGS_CONTROLLER_H_ */
