#ifndef _POLARCORD_H
#define _POLARCORD_H

#define _USE_MATH_DEFINES

class PolarPose
{
public:
    double range;
    double angle;
    PolarPose():range(0.0), angle(0.0){;}
    PolarPose(const double r, const double a ):range(r), angle(a){;}
    inline void fromCart(const double x, const double y)
    {
        range = sqrt((x * x) + (y * y));
        angle = atan2(y, x);
    }
    inline void toCart(double &x, double &y) {
        x = range * cos(angle);
        y = range * sin(angle);
    }

    inline double distance(const double r, const double a)
    {
        return sqrt( pow(range,2) + pow(r,2) - 2*range*r*cos(angle - a));
    }
};

inline static double toRadian(double t)
{
    return t*M_PI/180.0;
}

#endif

