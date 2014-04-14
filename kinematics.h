#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <math.h>
#include "waypoint.h"

#define TH1_MAX_DEG 138.67
#define TH1_MIN_DEG 35.19

#define TH2_MAX_DEG 67.00
#define TH2_MIN_DEG 1.28

#define TH3_MAX_DEG -29.38
#define TH3_MIN_DEG -144.49

#define TH4_MAX_DEG 53.36
#define TH4_MIN_DEG -145.90

#define TH1_MAX 138.67*(M_PI/180.0)
#define TH1_MIN 51.15*(M_PI/180.0)

#define TH2_MAX 67.00*(M_PI/180.0)
#define TH2_MIN 1.28*(M_PI/180.0)

#define TH3_MAX -29.38*(M_PI/180.0)
#define TH3_MIN -144.49*(M_PI/180.0)

#define TH4_MAX 53.36*(M_PI/180.0)
#define TH4_MIN -145.90*(M_PI/180.0)

#define SLEW_JOINT_LEN 0.18 // Excavator slew joint length is 0.18m
#define BOOM_LEN 1.34 	// Excavator boom length is 1.34m
#define ARM_LEN 0.945	// Excavator boom length is 0.945m
#define BUCKET_LEN 0.42 // Excavator bucket length from joint to bucket digging edge
#define BUCKET_ANGLE 73*(M_PI/180.0)  // Excavator bucket angle is 75 degrees
#define GROUND_OFFSET 0.445 // Slew joint height from ground

/*
 * Solve joint angles for a given point in 3D space (waypoint)
 * by using inverse kinematics equations.
 *
 * Parameters: pointer to waypoint struct, pointers to angle
 * variables th1, th2, th3, th4 and digging angle thdg.
 * Updates angles th1, th2, th3 and th4.
 */
void calculateJointAngles(Waypoint const* const wp, float* const th1_ptr, float*
                          const th2_ptr, float* const th3_ptr, float* const th4_ptr);

/*
 * Solve the coordinates for the excavator bucket in 3D space for a given
 * set of joint angles th1, th2, th3, th4 by using forward kinematics equations.
 *
 * Takes pointers to current bucket coordinates x,y,z and current angles.
 * Updates current bucket coordinates x, y, z.
 */
void calculateBucketCoordinates(float* const x_ptr, float* const y_ptr, float* const z_ptr,
							const float th1, const float th2, const float th3, const float th4);


#endif
