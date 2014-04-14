#ifndef WAYPOINT_H
#define WAYPOINT_H

#include <stdint.h>

#define WAYPOINT_ARR_SIZE 256

typedef struct Waypoint
{
   float x;
   float y;
   float z;
   float thdg;
} Waypoint;

/*
 * Check whether a given waypoint is reachable or not by the excavator bucket.
 *
 * Parameters: pointer to waypoint struct.
 * Returns '1' if waypoint is reachable and '0' otherwise.
 *
 */

uint8_t checkWaypoint(Waypoint const* const wp);

/*
 * Takes in a set of coordinates for the excavator bucket and places them in a
 * pre-allocated array of waypoints. The validity of the waypoints and reachability
 * of waypoints is also checked.
 *
 * Takes in a pointer to waypoint array 'wp_arr' and the waypoint 'wp' which will
 * be checked and stored to the array if the waypoint can be reached and the array isn't full.
 * Returns '1' if operation was successful and '0' if the operation failed.
 */

uint8_t updateWaypointArray(Waypoint* const wp_arr, Waypoint const* const wp);


#endif
