#include "custom_math.h"

float custom_abs(float x)
{
    return (x >= 0) ? x : -x;
}

int custom_absinthe(int x)
{
    return (x >= 0) ? x : -x;
}

float custom_sin(float th)
{
    if (th < 0)
        th = th + 2*PI;

    th = th/(2*PI);

    unsigned long fixed32th = (unsigned long) (th * 4294967296.0);

    long fixed16_16 = sine(fixed32th);

    return (float) fixed16_16 / 65536.0;
}

float custom_cos(float th)
{
    float th2 = -PI/2 - th;

    if (th2 > 0)
        th2 = th2 - 2*PI;

    th2 = th2 + PI;

    return custom_sin(th2);
}

float custom_atan(float z){
   int iter = 50;
   float result = (iter*2) + 1;
   if (z > 20.0) {
      result = PI/2;
   }
   else if (z < -20.0) {
      result = -PI/2;
   }
   else {
      for (int i = iter; i > 0; i--) {
         result = ((2*i)-1) + ((i*i*z*z)/result);
      }
      result = z/result;
   }
   return result;
}

float custom_sqrt(float x)
{
    float b = x;

    for (unsigned char i = 0; i < 10; ++i) // Precise to 10^-7 with 10 iterations
    {
        b = (x / b + b) / 2;
    }

    return b;
}

/**
 * @fn genSpline
 * Create a spline between the reference points 2 and 3 (4 ref. points needed).
 * @param refs the reference points (array of size 4)
 * @param out the ouput points generated (must be of size nbpts)
 * @param t ???
 * @param nbpts the number of points to output
 */
void genSpline(Point* refs, Point* out, float t, unsigned int nbpts)
{
    Point P0 = refs[0];
    Point P1 = refs[1];
    Point P2 = refs[2];
    Point P3 = refs[3];

    float s  = 0;
    float s2 = 0;
    float s3 = 0;

    float interval = 1.0/((float)nbpts-1);

    unsigned int i=0;
    for (; s <= 1; ++i)
    {
        s2 = s*s; // s^2
        s3 = s2*s; // s^3

        out[i].x = (P1.x) + (- (P0.x) + (P2.x)) * t * s  + 
                            (2 * t * (P0.x) + (t-3) * (P1.x) + (3-2 * t) * (P2.x)- t * (P3.x)) * (s2)  + 
                            (-t * (P0.x) + (2-t) * (P1.x) + (t-2) * (P2.x) + t * (P3.x)) * (s3);

        out[i].y = (P1.y)  +  (-t * (P0.y) + t * (P2.y)) * (s)  + 
                            (2 * t * (P0.y) + (t-3) * (P1.y) + (3-2 * t) * (P2.y)-t * (P3.y)) * (s2)  + 
                            (-t * (P0.y) + (2-t) * (P1.y) + (t-2) * (P2.y) + t * (P3.y)) * (s3);
        s += interval;
    }

    while (i < nbpts)
    {
        out[i].x = P2.x;
        out[i].y = P2.y;
        ++i;
    }
}

static unsigned long nextRand = 1;

unsigned int myrand(void) {
    nextRand = nextRand  *  1103515245 + 12345;
    return((unsigned)(nextRand/65536) % 32768);
}
void mysrand(unsigned seed) {
    nextRand = seed;
}

void seedRandomGen()
{
   mysrand(xTaskGetTickCount());
}

unsigned int getPseudoRandomNumber(unsigned int ceil) {
   return (myrand()%ceil);
}

float custom_min(float v1, float v2)
{
    return v1 < v2 ? v1 : v2;
}

float custom_max(float v1, float v2)
{
    return v1 > v2 ? v1 : v2;
}

// 
bool compareFloat(float x, float y, float eps)
{
    return custom_abs(y-x) <= eps;
}