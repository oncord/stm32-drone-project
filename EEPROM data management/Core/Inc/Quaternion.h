#ifndef _QUATERNION_H
#define _QUATERNION_H

#include "main.h"
#include <math.h>

extern float BNO080_Roll;
extern float BNO080_Pitch;
extern float BNO080_Yaw;

void Quaternion_Update(float *q);
float invSqrt(float x);
void Quaternion_Multiply(const float *q1, const float *q2, float *out);

#endif
