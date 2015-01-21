#ifndef KALMANFILTER_H
#define KALMANFILTER_H
#include <cmath>

using namespace std;

class KalmanFilter
{
private:

    double F[4], H[4], B[4], R[4], Q[4];
    double X_tmp[2];
    double P_tmp[4];

    void assignMatrix(double* A, double* B);
    void multiplyMatrix4x4(double *A, double *B, double *C);
    void multiplyMatrix4x2(double* A, double* B, double* C);
    void transposeMatrix(double* A, double* T, unsigned char n);
    void addMatrix(double *A, double *B, double *C, unsigned char n);
    void inverseMatrix(double* A, double* B);


public:

    double X_new[2];
    double P_new[4];

    KalmanFilter(double* varU_, double* varZ_);

    void makeI(double *I, const unsigned char n);
    void makeF();
    void makeB(const double t);
    void makeH();
    void makeQ(double* varU);
    void makeR(double* varZ);
    void predict(double *U, const double t, double *X_old, double *P_old);
    void update(double* Z);
    ~KalmanFilter();
};

#endif // KALMANFILTER_H
