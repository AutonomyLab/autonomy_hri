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
        x = range * cos(angle);
        y = range * sin(angle);
    }
};

