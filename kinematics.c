#include "kinematics.h"
#include <math.h>

void calculateJointAngles(Waypoint const* const wp, float* const th1_ptr, float* const th2_ptr,
                          float* const th3_ptr, float* const th4_ptr)
{
	float x,y,z,thdg,d,num,den,th1,th2,th3,th4;

	/* Waypoint coordinates for the last joint (the joint that holds the bucket) */
	x = wp->x;
	y = wp->y;
	/* To make these coordinates essentially bucket coordinates, the
      z coordinate must be adjusted accordingly */
	z = wp->z + BUCKET_LEN - GROUND_OFFSET;
	thdg = wp->thdg;

	/* Calculate angle theta1 */
	th1 = atan2(y, x);
	*th1_ptr = th1;

	/* Calculate angle theta2 */
	d = cos(th1)*x + sin(th1)*y - SLEW_JOINT_LEN;
	num = sqrt(4*BOOM_LEN*BOOM_LEN*(z*z + d*d) - pow((z*z + d*d + BOOM_LEN*BOOM_LEN - ARM_LEN*ARM_LEN), 2));
	den = z*z + d*d + BOOM_LEN*BOOM_LEN - ARM_LEN*ARM_LEN;
	th2 = atan2(z,d) + atan2(num, den);
	*th2_ptr = th2;

	/* Calculate angle theta3 */
	num = cos(th2)*z - sin(th2)*d;
	den = sin(th2)*z + cos(th2)*d - BOOM_LEN;
	th3 = atan2(num, den);
	*th3_ptr = th3;

	/* Calculate angle theta4 */
	th4 = -thdg - th2 + fabs(th3);
	//th4 = thdg - fabs(th3) + th2 - BUCKET_ANGLE;
	*th4_ptr = th4;

}


void calculateBucketCoordinates(float* const x_ptr, float* const y_ptr, float* const z_ptr,
							const float th1, const float th2, const float th3, const float th4)
{
	float cos234 = cos(th2+th3+th4);
	float sin234 = sin(th2+th3+th4);

	*x_ptr = cos(th1)*(BUCKET_LEN*cos234 + ARM_LEN*cos(th2+th3) + BOOM_LEN*cos(th2) + SLEW_JOINT_LEN);

	*y_ptr = sin(th1)*(BUCKET_LEN*cos234 + ARM_LEN*cos(th2+th3) + BOOM_LEN*cos(th2) + SLEW_JOINT_LEN);

	*z_ptr = BUCKET_LEN*sin234 + ARM_LEN*sin(th2+th3) + BOOM_LEN*sin(th2);

}

