#include <string.h>
#include "kinematics.h"
#include "waypoint.h"

float max(float x, float y)
{
   return (x > y ? x : y);
}

uint8_t checkWaypoint(const Waypoint* const wp)
{
   float th1, th2, th3, th4, x1, x2, u1, u2, limit, d;
   float x = wp->x;
   float y = wp->y;
   float z = wp->z;

   x1 = 2*BOOM_LEN*BOOM_LEN + 2*ARM_LEN*ARM_LEN - 2*z*z;
   x2 = pow(BOOM_LEN, 4) + pow(ARM_LEN,4) - 2*pow(BOOM_LEN*ARM_LEN, 2) - 2*pow(BOOM_LEN*z, 2) - 2*ARM_LEN*ARM_LEN*z*z + pow(z,4);

   u1 = (-x1 + sqrt(x1*x1 - 4*x2))/(-2);
   u2 = (-x1 - sqrt(x1*x1 - 4*x2))/(-2);

   limit = sqrt(max(u1,u2));
   calculateJointAngles(wp, &th1, &th2, &th3, &th4);
   d = cos(th1)*x + sin(th1)*y - SLEW_JOINT_LEN;
   if (!(fabs(d) < limit)) {return 0;}

   //printf("Th1: %f, Th2: %f, Th3: %f, Th4: %f\n", th1, th2, th3, th4);
   //printf("TH1MAX: %f, TH1_MIN: %f, TH2MAX: %f, TH2_MIN: %f, TH3MAX: %f, TH3_MIN: %f, TH4MAX: %f, TH4_MIN: %f\n", TH1_MAX, TH1_MIN, TH2_MAX, TH2_MIN, TH3_MAX, TH3_MIN, TH4_MAX, TH4_MIN);

   if(th1 > TH1_MAX || th1 < TH1_MIN) {return 0;}
   if(th2 > TH2_MAX || th2 < TH2_MIN) {return 0;}
   if(th3 > TH3_MAX || th3 < TH3_MIN) {return 0;}
   if(th4 > TH4_MAX || th4 < TH4_MIN) {return 0;}

   return 1;
}


uint8_t updateWaypointArray(Waypoint* const wp_arr, Waypoint const* const wp)
{
   if(checkWaypoint(wp))
   {
      Waypoint tmp_wp;
      for (int16_t i=0; i<WAYPOINT_ARR_SIZE; i++)
      {
         tmp_wp = wp_arr[i];
         if(tmp_wp.x == -1)
         {
            wp_arr[i] = *wp;
            return 1;
         }
      }
   }

   return 0;
}

Waypoint parseCoordinate(char* coordinateStr)
{

    // Messages: EII = Error In Input
    char error_message[3];

	Waypoint newWaypoint;
    uint32_t str_length = strlen(coordinateStr);
    uint32_t i = 0;
    uint8_t state = 0;

    float x = 1;
    float y = 1;
    float z = 1;
    float a = 1;

    float* var;

    // Ignore characters after x.xx until , or >

    while (i < str_length)
    {
        switch (state)
        {
            case -1:
                break;
            case 0:
                if (coordinateStr[i] == '<')
                    state = 1;
                break;
            case 1:
                switch (coordinateStr[i])
                {
                    case ('x'):
                        var = &x;
                        state = 2;
                        break;
                    case ('y'):
                        var = &y;
                        state = 2;
                        break;
                    case ('z'):
                        var = &z;
                        state = 2;
                        break;
                    case ('a'):
                        var = &a;
                        state = 2;
                        break;
                        // Error in input (Variable not defined correctly)
                    default:
                        state = -1;
                        break;
                }
                break;
                // x,y,z or a seen last, : expected.
            case 2:
                if (coordinateStr[i] == ':')
                    state = 3;
                else
                    state = -1;
                break;
                // : seen last, number from 0-9 expected.
            case 3:
                if (coordinateStr[i] - '0' > 9 || coordinateStr[i] - '0' < 0)
                {
                    state = -1;
                    break;
                }

                *var = (float)(coordinateStr[i++] - '0'); // What if coordinateStr[i] = 0?
                while (coordinateStr[i] - '0' < 10 && coordinateStr[i] - '0' >= 0)
                {
                    *var *= 10; // Shift to left
                    *var += coordinateStr[i++] - '0';
                }

                if (coordinateStr[i] == '.')
                {
                    i++;
                    if (coordinateStr[i] - '0' > 9 || coordinateStr[i] - '0' < 0)
                    {
                        state = -1;
                        break;
                    }

                    float decimal = (float)(coordinateStr[i++]-'0')/10.0; // Shift to right
                    float temp;
                    int decimal_places = 1;
                    while (coordinateStr[i] - '0' < 10 && coordinateStr[i] - '0' >= 0)
                    {
                        temp = (float)(coordinateStr[i++]-'0');
                        decimal_places++;
                        int a = 0;
                        while (a < decimal_places)
                        { temp = temp/10.0; a++; } // Shift to right {decimal_places} times
                        decimal += temp;
                    }
                    i--;
                    *var += decimal;
                }

                state = 4;
                break;
            // Finished parsing number , or > expected
            case 4:
                if (coordinateStr[i]==',')
                {
                    state = 1;
                }
                else
                    state = -1;
                break;

        }

        i++;
    }

	newWaypoint.x = x;
	newWaypoint.y = y;
	newWaypoint.z = z;
	newWaypoint.thdg = a;

    return newWaypoint;
}








