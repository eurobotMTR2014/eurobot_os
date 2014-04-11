#ifndef CUSTOM_MATH_H_INCLUDED
#define CUSTOM_MATH_H_INCLUDED

#include "custom_lib.h"

#include "sine.h"

#ifndef PI
   #define PI  3.141592654
#endif

float custom_abs(float x);
int custom_absinthe(int x);
float custom_sin(float th); /// ACCEPTS ONLY VALUES IN [-PI; PI]!!!
float custom_cos(float th);
float custom_sqrt(float x);
float custom_atan(float z);
float custom_min(float v1, float v2);
float custom_max(float v1, float v2);
float custom_sign(float x);
bool compareFloat(float x, float y, float eps);
void seedRandomGen();
unsigned int getPseudoRandomNumber(unsigned int ceil);

typedef struct Point
{
    float x;
    float y;
} Point;

/// Refs must be table of point of size 4, out allocated to contain "nbpts" points
/// out will be spline between refs[1] and refs[2]
/// t is curve
void genSpline(Point* refs, Point* out, float t, unsigned int nbpts);

#endif // CUSTOM_MATH_H_INCLUDED
