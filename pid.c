#include <stdint.h>
#include <stdlib.h>
#include "pid.h"
#include "valves.h"

#define MAXFLOW 100

char buffer[7];

uint8_t pid_controller(int16_t* const referenceAngle,
                 int16_t* const currentAngle,
                 uint8_t* const direction,
                 int16_t* const jointErrorIntegral,
				 uint8_t switchDirection,
                 const float Kp,
                 const float Ki)
{

	int16_t error;
	int16_t integral = *jointErrorIntegral;
	int16_t pid, P, I;

	error = *referenceAngle - *currentAngle;     //Calculate error
	
	uart1_puts("PID error term: ");
	itoa(error,buffer, 10);
	uart1_puts(buffer);

	if(abs(error) < 1)
   	{
		*direction = FLOATING;
		pid = 0;
		return (uint8_t)pid;
   	}

	integral += error;		//Calculate integral of the error
	if(integral > MAXFLOW) integral = MAXFLOW;           //Anti-windup for the integral
	else if(integral < -MAXFLOW) integral = -MAXFLOW;   //Anti-windup for the negative side
	//*jointErrorIntegral = integral;
	P = Kp * error;    		//P term
	I = Ki * integral;		//I term
	
	uart1_puts("PID integral term: ");
	itoa(integral,buffer, 10);
	uart1_puts(buffer);

	pid = P + I;

	if(pid < 0)								//Check for negative values
	{
		if(switchDirection)
		{
			*direction = RETRACT;
		}
		else
		{
			*direction = EXTEND;         							
		}
		
	}
	else
	{
		// I term is different when lifting the arm or the boom as
		// more power is required to reach the wanted position
		I = (Ki+1.5) * integral;
		if(switchDirection)
		{
			*direction = EXTEND;
		}
		else
		{
			*direction = RETRACT;          							
		}

		pid = P + I;
	}

	pid = abs(pid);

	if(pid > MAXFLOW) pid = MAXFLOW;        		//Limit the maximum value of the output

	return (uint8_t)pid;  					//Controller output */
}
