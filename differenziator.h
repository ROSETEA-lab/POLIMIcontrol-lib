#ifndef DIFFERENZIATOR_H_
#define DIFFERENZIATOR_H_

#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Differenziator class
//
// Implements the differenziator described in
//
// B. Andritsch, M. Horn, S. Koch, H. Niederwieser, M. Wetzlinger, M. Reichhartinger
// The Robust Exact Differentiator Toolbox revisited: Filtering and Discretization Features
// 2021 IEEE International Conference on Mechatronics
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

class differenziator
{
    private:
        double _d, _r, _mu, _Ts;
        unsigned int _n, _nf, _m;

        double _x0, _z0;
        unsigned int _sys_n;
        Eigen::VectorXd _zp, _b, _bD, _z;
        Eigen::MatrixXd _A, _P, _Phi, _U, _V_inv, _So_inv;

        double err(double u, double first_state);
        double cont_eigenvalues(double x0);
        double disc_eigenvalues(double s);
        double disc_eigenvalues_URED(double x0);
        void   ackerman(Eigen::VectorXd& lambda, double z);
        void   ackerman_precomputed(Eigen::VectorXd& lambda, double z);
        void   step(Eigen::VectorXd& z, double u, const Eigen::VectorXd& lambda);

    public:
        differenziator(unsigned int n, double r, double Ts);
        differenziator(unsigned int n, unsigned int nf, double d, double r, unsigned int m, double mu, double Ts);
        ~differenziator();

        void evaluate(double u);

        void get_x0(double x0) { x0 = _x0; };
        void get_z0(double z0) { z0 = _z0; };
        void get_z(Eigen::VectorXd& z) { z = _z; };
};


class linear_differenziator : public differenziator
{
    public:
        linear_differenziator(unsigned int n, unsigned int nf, double r, unsigned int m, double Ts) : differenziator(n, nf, 0.0, r, m, 0.0, Ts) {};
        ~linear_differenziator();
};


class robust_exact_differenziator : public differenziator
{
    public:
        robust_exact_differenziator(unsigned int n, unsigned int nf, double r, unsigned int m, double Ts) : differenziator(n, nf, -1.0, r, m, 0.0, Ts) {};
        ~robust_exact_differenziator();
};


class uniform_robust_exact_differenziator : public differenziator
{
    public:
        uniform_robust_exact_differenziator(unsigned int n, unsigned int nf, double r, double mu, double Ts) : differenziator(n, nf, -1.0, r, 2.0, mu, Ts) {};
        ~uniform_robust_exact_differenziator();
};

#endif // DIFFERENZIATOR_H_