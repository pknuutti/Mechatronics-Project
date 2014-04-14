#include <stdint.h>
#include "config.h"
#include "can.h"
#include "at90can_private.h"
#include "can_private.h"
#include "can_buffer.h"
#include "utils.h"
#include "uart.h"

#include "pid.h"
#include "valves.h"

int16_t slewJointErrorIntegral;
int16_t boomJointErrorIntegral;
int16_t armJointErrorIntegral;
int16_t bucketJointErrorIntegral;

char buffer[7];

uint8_t controlSlewJointValve(int16_t* const referenceAngle, int16_t* const currentAngle)
{
	uint8_t pid = 0;
	uint8_t switchDirection = 1;
	uint8_t direction = FLOATING;
	float Kp = 0.8;
	float Ki = 0.1;
	pid = pid_controller(referenceAngle, currentAngle, &direction, &slewJointErrorIntegral, switchDirection, Kp, Ki);
	
	uart1_puts("PID value: ");
	itoa(pid, buffer, 10);
	uart1_puts(buffer);
	uart1_puts("\n");

	can_t valveCtrlMsg;
	setValveMsg(&valveCtrlMsg, 50, 1, SLEW_JOINT_VALVE_CTRLMSG_ID);
	can_send_message(&valveCtrlMsg);
	_delay_ms(50);

	if(pid == 0) 
	{
		slewJointErrorIntegral = 0;
		return 1;
	}
	else { return 0;}

}

uint8_t controlBoomJointValve(int16_t* const referenceAngle, int16_t* const currentAngle)
{
	uint8_t pid = 0;
	uint8_t switchDirection = 0;
	uint8_t direction = FLOATING;
	float Kp = 1.3;
	float Ki = 0;
	pid = pid_controller(referenceAngle, currentAngle, &direction, &boomJointErrorIntegral, switchDirection, Kp, Ki);
	
	uart1_puts("PID value: ");
	itoa(pid, buffer, 10);
	uart1_puts(buffer);
	uart1_puts("\n");
	
	can_t valveCtrlMsg;
	setValveMsg(&valveCtrlMsg, 50, 2, BOOM_JOINT_VALVE_CTRLMSG_ID);
	can_send_message(&valveCtrlMsg);
	_delay_ms(50);

	if(pid == 0) 
	{
		boomJointErrorIntegral = 0;
		return 1;
	}
	else { return 0;}

}

uint8_t controlArmJointValve(int16_t* const referenceAngle, int16_t* const currentAngle)
{
	uint8_t pid = 0;
	uint8_t switchDirection = 0;
	uint8_t direction = FLOATING;
	float Kp = 1.3;
	float Ki = 0.1;
	pid = pid_controller(referenceAngle, currentAngle, &direction, &armJointErrorIntegral, switchDirection, Kp, Ki);
	
	can_t valveCtrlMsg;
	
	uart1_puts("PID value: ");
	itoa(pid, buffer, 10);
	uart1_puts(buffer);
	uart1_puts("\n");

	setValveMsg(&valveCtrlMsg, 50, 2, ARM_JOINT_VALVE_CTRLMSG_ID);

	can_send_message(&valveCtrlMsg);
	_delay_ms(50);

	if(pid == 0) 
	{
		armJointErrorIntegral = 0;
		return 1;
	}
	else { return 0;}

}

uint8_t controlBucketJointValve(int16_t* const referenceAngle, int16_t* const currentAngle)
{
	uint8_t pid = 0;
	uint8_t switchDirection = 1;
	uint8_t direction = FLOATING;
	float Kp = 1.2;
	float Ki = 0.1;
	pid = pid_controller(referenceAngle, currentAngle, &direction, &bucketJointErrorIntegral, switchDirection, Kp, Ki);
	
	can_t valveCtrlMsg;
	
	uart1_puts("PID value: ");
	itoa(pid, buffer, 10);
	uart1_puts(buffer);
	uart1_puts("\n");

	setValveMsg(&valveCtrlMsg, 50, 1, BUCKET_JOINT_VALVE_CTRLMSG_ID);

	can_send_message(&valveCtrlMsg);
	_delay_ms(50);

	if(pid == 0) 
	{
		bucketJointErrorIntegral = 0;
		return 1;
	}
	else { return 0;}

}

void setValveMsg(can_t* const valveMsg, const uint8_t flow, const uint8_t spoolState, const uint32_t valveID)
{   
   valveMsg->id = valveID;
   valveMsg->flags.rtr = 0;	// The sent message is NOT a remote-transmit-request frame
   valveMsg->flags.extended = 1;	// Sends the message with extended ID
   valveMsg->length = 8;	// Message length is 8 bytes

   for (uint8_t k = 0; k < 8; k++)
   {
		valveMsg->data[k] = 0;
   }

   valveMsg->data[0] = flow;
   valveMsg->data[2] = spoolState;

}

ValveState getValveState(can_t* valveMsg)
{
   ValveState valve;
   // Flow percent 0% == 125 and 100% == 225
   uint8_t extendFlow = valveMsg->data[0] - 125;
   uint8_t retractFlow = valveMsg->data[1] - 125;
   uint8_t currentFlow = 0;

   /* CAN message from valve contains both extend and retract flow
    * information. As they both cannot be larger than 0 at the same time,
    * the bigger of the two is chosen. */
   currentFlow = (extendFlow > retractFlow ? extendFlow : retractFlow);

   valve.flowPercent = currentFlow;
   valve.spoolState = valveMsg->data[3];

   return valve;

}

void printValveStateUART(char* flowStr, char* spoolStr, ValveState* valve)
{
	char buffer[7];
	uint8_t flowPercent = 0;
	uint8_t spoolState = 0;
	
	uart1_puts(flowStr);
	
	flowPercent = valve->flowPercent;
	itoa(flowPercent, buffer, 10); // the parameter 10 signifies base 10        
    uart1_puts(buffer);
	uart1_puts("% \n");
	
	uart1_puts(spoolStr);
	spoolState = valve->spoolState;
	itoa(spoolState, buffer, 10); // the parameter 10 signifies base 10        
    uart1_puts(buffer);
	uart1_puts("\n");
}



