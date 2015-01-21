#include "kalmanfilter.h"

using namespace std;

KalmanFilter::KalmanFilter(double* varU_, double* varZ_)
{
    makeF();
    makeH();

//    assignMatrix(varU_, varU);
//    assignMatrix(varZ_, varZ);

    makeQ(varU_);
    makeR(varZ_);

//    assignMatrix(X0, X_old);
//    assignMatrix(P0, P_old);
}

void KalmanFilter::makeI(double* I, const unsigned char n)
{
    /*** Define a n x n matrix ***/

    unsigned char in = 0;

    for(unsigned char i = 0; i < n; i++){

        for(unsigned char j = 0; j < n; j++){

            if(i == j) I[in] = 1;
            else I[in] = 0;
            in++;

        }
    }
}

void KalmanFilter::makeF()
{
    makeI(F,2);
}

void KalmanFilter::makeB(const double t)
{
    makeI(B,2);
    B[0] = B[3] = t;
}

void KalmanFilter::makeH()
{
    makeI(H,2);
}

void KalmanFilter::makeQ(double *varU)
{
    makeI(Q,2);
    Q[0] = varU[0];
    Q[3] = varU[1];
}

void KalmanFilter::makeR(double* varZ)
{
    makeI(R,2);
    R[0] = varZ[0];
    R[3] = varZ[1];
}

void KalmanFilter::assignMatrix(double *A, double *B)
{
    for(unsigned char i = 0; i < sizeof(A) ; i++)
        B[i] = A[i];
}

void KalmanFilter::multiplyMatrix4x4(double* A, double* B, double* C)
{
    C[0] = A[0] * B[0] + A[1] * B[2];
    C[1] = A[0] * B[1] + A[1] * B[3];
    C[2] = A[2] * B[0] + A[3] * B[2];
    C[3] = A[2] * B[1] + A[3] * B[3];
}

void KalmanFilter::multiplyMatrix4x2(double *A, double *B, double* C)
{
    C[0] = A[0] * B[0] + A[1] * B[1];
    C[1] = A[2] * B[0] + A[3] * B[1];
}

void KalmanFilter::transposeMatrix(double *A, double *T, unsigned char n)
{
    /* TODO: define matrix transpose*/
}

void KalmanFilter::addMatrix(double* A, double* B, double* C, unsigned char n)
{
    for(unsigned char i = 0; i < n; i++){

        C[i] = A[i] + B[i];
    }
}

void KalmanFilter::inverseMatrix(double* A, double* B)
{
    double denum = A[0] * A[3] - A[1] * A[2];
    B[0] = A[3] / denum;
    B[1] = - A[1] / denum;
    B[2] = - A[2] / denum;
    B[3] = A[0] / denum;
}

void KalmanFilter::predict(double* U, const double t, double* X_old, double* P_old)
{

    double FX[2], BU[2];

    makeB(t);

    multiplyMatrix4x2(F, X_old, FX);
    multiplyMatrix4x2(B, U, BU);
    addMatrix(FX, BU, X_tmp, 2);

    double FP[4], FPF[4], Ft[4];

    /* TODO: define matrix transpose*/
    // transposeMatrix(F, Ft, 4);

    assignMatrix(F, Ft);
    multiplyMatrix4x4(Ft, P_old, FP);
    multiplyMatrix4x4(FP, F, FPF);
    addMatrix(FPF, Q, P_tmp, 4);

    assignMatrix(X_tmp, X_new);
    assignMatrix(P_tmp, P_new);
}

void KalmanFilter::update(double *Z)
{

    double HX[2], Y[2];
    multiplyMatrix4x2(H, X_tmp, HX);
    HX[0] = - H[0];
    HX[1] = - H[1];
    addMatrix(Z, HX, Y, 2);

    double Ht[4], HP[4], HPH[4], S[4];

    /* TODO: define matrix transpose*/
    // transposeMatrix(H, Ht, 4);

    assignMatrix(H, Ht);

    multiplyMatrix4x4(H, P_tmp, HP);
    multiplyMatrix4x4(HP, Ht, HPH);
    addMatrix(HPH, Q, S, 4);

    double K[4], PH[4], Sinv[4];
    inverseMatrix(S, Sinv);
    multiplyMatrix4x4(P_tmp, Ht, PH);
    multiplyMatrix4x2(PH, Sinv, K);

    double KY[2];
    multiplyMatrix4x2(K, Y, KY);
    addMatrix(X_tmp, KY, X_new, 2);

    double KH[4];
    multiplyMatrix4x4(K, H, KH);
    KH[0] = 1 - KH[0];
    KH[3] = 1 - KH[3];
    multiplyMatrix4x4(KH, P_tmp, P_new);
}

KalmanFilter::~KalmanFilter()
{}
