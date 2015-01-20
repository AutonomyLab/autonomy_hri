#include "kalmanfilter.h"

using namespace std;

KalmanFilter::KalmanFilter(double varUR_, double varUT_,
                           double varZR_, double varZT_,
                           double time_, double X_old_[],
                           double P_old_[], double u_[], double z_[]):
    varUR (varUR_),
    varUT (varUT_),
    varZR (varZR_),
    varZT (varZT_),
    t (time_)
{
    makeF();
    makeB(t);
    makeQ(varUR, varUT);
    makeH();
    makeQ(varZR, varZT);
    assignMatrix(X_old_, X_old);
    assignMatrix(P_old_, P_old);
    assignMatrix(u_, U);
    assignMatrix(z_, Z);
}

void KalmanFilter::makeI(double I[], const unsigned char n)
{
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

void KalmanFilter::makeB(double T)
{
    makeI(B,2);
    B[0] = B[3] = T;
}

void KalmanFilter::makeH()
{
    makeI(H,2);
}

void KalmanFilter::makeQ(double varUR, double varUT)
{
    makeI(Q,2);
    Q[0] = varUR;
    Q[3] = varUT;
}

void KalmanFilter::makeR(double varZR, double varZT)
{
    makeI(R,2);
    R[0] = varZR;
    R[3] = varZT;
}

void KalmanFilter::assignMatrix(const double A[], double B[])
{
    for(unsigned char i = 0; i < sizeof(A) ; i++)
        B[i] = A[i];
}

void KalmanFilter::multiplyMatrix4x4(const double A[4], const double B[4], double C[4])
{
    C[0] = A[0] * B[0] + A[1] * B[2];
    C[1] = A[0] * B[1] + A[1] * B[3];
    C[2] = A[2] * B[0] + A[3] * B[2];
    C[3] = A[2] * B[1] + A[3] * B[3];
}

void KalmanFilter::multiplyMatrix4x2(const double A[4], const double B[2], double C[2])
{
    C[0] = A[0] * B[0] + A[1] * B[1];
    C[1] = A[2] * B[0] + A[3] * B[1];
}

void KalmanFilter::transposeMatrix(const double A[], double T[], unsigned char n)
{
    /* TODO: define matrix transpose*/
}

void KalmanFilter::addMatrix(const double A[], const double B[], double C[], unsigned char n)
{
    for(unsigned char i = 0; i < n; i++){
        C[i] = A[i] + B[i];
    }
}

void KalmanFilter::inverseMatrix(const double A[], double B[])
{
    double denum = A[0] * A[3] - A[1] * A[2];
    B[0] = A[3] / denum;
    B[1] = - A[1] / denum;
    B[2] = - A[2] / denum;
    B[3] = A[0] / denum;
}

void KalmanFilter::predict(const double u[])
{
    assignMatrix(u, U);
    double FX[2], BU[2];
    multiplyMatrix4x2(F, X_old, FX);
    multiplyMatrix4x2(B, U, BU);
    addMatrix(FX, BU, X_tmp, 2);

    /* TODO: define matrix transpose*/
    double FP[4], FPF[4], Ft[4];
    multiplyMatrix4x4(F, P_old, FP);
    multiplyMatrix4x4(FP, F, FPF);
    addMatrix(FPF, Q, P_tmp, 4);
}

void KalmanFilter::update(const double z[])
{
    assignMatrix(z, Z);
    double HX[2], Y[2];
    multiplyMatrix4x2(H, X_tmp, HX);
    HX[0] = - H[0];
    HX[1] = - H[1];
    addMatrix(Z, HX, Y, 2);

    double HP[4], HPH[4], S[4];
    multiplyMatrix4x4(H, P_tmp, HP);
    multiplyMatrix4x4(HP, H, HPH);
    addMatrix(HPH, Q, S, 4);

    double K[4], PH[4], Sinv[4];
    inverseMatrix(S, Sinv);
    multiplyMatrix4x4(P_tmp, H, PH);
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
