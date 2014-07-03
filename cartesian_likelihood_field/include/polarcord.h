#ifndef _POLARCORD_H
#define _POLARCORD_H

#define _USE_MATH_DEFINES

class PolarPose
{
public:
    float range;
    float angle;
    PolarPose():range(0.0), angle(0.0){;}
    PolarPose(const float r, const float a ):range(r), angle(a){;}
    inline void fromCart(const float x, const float y)
    {
        range = sqrt((x * x) + (y * y));
        angle = atan2(y, x);
    }
    inline void toCart(float &x, float &y) {
        x = round(range * cos(angle));
        y = round(range * sin(angle));
    }

    inline double distance(const float r, const float a)
    {
        return sqrt( pow(range,2) + pow(r,2) - 2*range*r*cos(angle - a));
    }
};

inline static double toRadian(double t)
{
    return t*M_PI/180;
}

#endif

