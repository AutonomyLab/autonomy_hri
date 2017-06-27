#ifndef _POLARCORD_H
#define _POLARCORD_H

#define _USE_MATH_DEFINES
#include <angles/angles.h>

class PolarPose
{
public:
  float range;
  float angle;
  float var_range;
  float var_angle;
  float cov;
  PolarPose(): range(0.0), angle(0.0) {;}
  PolarPose(const float r, const float a): range(r), angle(a)
  {
    angle = (float) angles::normalize_angle(angle);
  }
  inline void fromCart(const float x, const float y)
  {
    range = sqrtf((x * x) + (y * y));
    angle = (float) angles::normalize_angle(atan2f(y, x));
  }
  inline void toCart(double &x, double &y)
  {
    x = range * cos(angle);
    y = range * sin(angle);
  }

  inline void toCart(float &x, float &y)
  {
    x = range * cosf(angle);
    y = range * sinf(angle);
  }

  inline float distance(const float r, const float a)
  {
    float x2, y2;
    float x1 = r * cos(a);
    float y1 = r * sin(a);

    this->toCart(x2, y2);

    return sqrtf((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
  }

  inline float distance(PolarPose p)
  {
    float x2, y2;
    float x1 = p.range * cos(p.angle);
    float y1 = p.range * sin(p.angle);

    this->toCart(x2, y2);

    return sqrtf((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
  }

  inline void setZeroVar()
  {
    this->var_angle = 1e-6;
    this->var_range = 1e-6;
  }
};


#endif

