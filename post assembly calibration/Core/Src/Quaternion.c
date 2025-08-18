#include "Quaternion.h"

#define _180_DIV_PI 57.295779515f // 180 / PI --> radians to degrees

float BNO080_Roll;
float BNO080_Pitch;
float BNO080_Yaw;

// update our quaternion value passed by MotionEngine software
void Quaternion_Update(float *q_in)
{
	float q_corr[4] = {0.0f, 0.0f, -0.70710678f, 0.70710678f};
	float q[4];

	Quaternion_Multiply(q_corr, q_in, q); // apply offset correction

	float q1, q2, q3, q4;
	float norm;

	norm = invSqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]); // normalize the quaternion

	q1 = q[0] * norm; // x
	q2 = q[1] * norm; // y
	q3 = q[2] * norm; // z
	q4 = q[3] * norm; // w

	BNO080_Roll = atan2f(2.0f * (q2*q3 + q1*q4), q1*q1 + q2*q2 - q3*q3 - q4*q4);
	BNO080_Pitch = -asinf(2.0f * (q2*q4 - q1*q3));
	BNO080_Yaw = atan2f(2.0f * (q1*q2 + q3*q4), q1*q1 - q2*q2 - q3*q3 + q4*q4);

	BNO080_Roll *= _180_DIV_PI;
	BNO080_Pitch *= _180_DIV_PI;
	BNO080_Yaw *= _180_DIV_PI;

	if (BNO080_Yaw >= 0)
	{
		BNO080_Yaw = 360.f - BNO080_Yaw;
	}
	else
	{
		BNO080_Yaw = -BNO080_Yaw;
	}

	if (BNO080_Pitch >= 0)
	{
		BNO080_Pitch = 180.f - BNO080_Pitch;
	}
	else
	{
		BNO080_Pitch = -(BNO080_Pitch + 180.f);
	}
}

// fast inverse square-root borrowed from wiki

float invSqrt(float x)
{
	float halfX = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfX * y * y));
	return y;
}

// offset correction
void Quaternion_Multiply(const float *q1, const float *q2, float *out)
{
    float x1 = q1[0], y1 = q1[1], z1 = q1[2], w1 = q1[3];
    float x2 = q2[0], y2 = q2[1], z2 = q2[2], w2 = q2[3];

    out[0] = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2;
    out[1] = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2;
    out[2] = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2;
    out[3] = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2;
}
