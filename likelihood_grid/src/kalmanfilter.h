#ifndef KALMANFILTER_H
#define KALMANFILTER_H
#include <cmath>

using namespace std;

class KalmanFilter
{
private:
    double F[4], H[4], B[4], R[4], Q[4];
    double X_new[2], X_old[2], X_tmp[2];
    double t;
    double U[2], Z[2];
    double varUR, varUT;
    double varZR, varZT;
    double P_new[4], P_old[4], P_tmp[4];
    void assignMatrix(const double A[], double B[]);
    void multiplyMatrix4x4(const double A[], const double B[], double C[]);
    void multiplyMatrix4x2(const double A[], const double B[], double C[]);
    void transposeMatrix(const double A[], double T[], unsigned char n);
    void addMatrix(const double A[], const double B[], double C[], unsigned char n);
    void inverseMatrix(const double A[], double B[]);

public:
    KalmanFilter(double varUR_, double varUT_, double varZR_,
                 double varZT_, double time_, double X_old_[],
                 double P_old_[], double u_[], double z_[]);
    void makeI(double I[], const unsigned char n);
    void makeF();
    void makeB(double T);
    void makeH();
    void makeQ(double varUR, double varUT);
    void makeR(double varZR, double varZT);
    void predict(const double u[]);
    void update(const double z[]);
};

#endif // KALMANFILTER_H
