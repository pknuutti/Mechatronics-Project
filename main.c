#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>

#define	__AVR_AT90CAN128__		1
#define F_CPU	16000000UL 	/* in Hz */
#define UART_BAUD_RATE	   250000  

/* Heartbeat interval defines how many milliseconds to wait
 * between sending burst of CAN messages. PID control interval
 * defines the interval for */
#define HEARTBEAT_INTERVAL   500			// How often we send the heartbeat msg (in ms)
#define PID_CTRL_INTERVAL 	 50

//#define DEBUG_ON 0

#include "config.h"
#include "at90can_private.h"
#include "can.h"
#include "can_private.h"
#include "can_buffer.h"
#include "utils.h"

#include "uart.h"
#include "kinematics.h"
#include "pid.h"
#include "waypoint.h"
#include "resolvers.h"
#include "valves.h"

// Global variables
uint16_t msCount = 0;
uint8_t f_heartbeat = 0;
uint8_t f_ctrl = 0;


int main(void)
{

	// ------------------------------------------------------------
	// Misc CPU registers
	// ------------------------------------------------------------	  
	__asm__("sei");   				// Global interrupt enable

	WDTCR = (1<<WDCE) | (1<<WDE); 	// Watchdog disable
	WDTCR = 0x00;

	/*
	 ******* Initialization functions *******
	 */
	
	// Initialize microcontroller ports
	initPorts();
	
	// Initialize the UART library
    uart1_init( UART_BAUD_SELECT(UART_BAUD_RATE, F_CPU) );
	
	// Initialize CAN controller
	initCAN();
	
	// Initialize ADC	
	initADC();

	// Initialize resolvers to operational state
	sendResolverNMTMessage();
	
	can_t receiveMsg;
	for(uint8_t i=0; i < 8; i++)
	{
		receiveMsg.data[i] = 0x00;
	}

	uint8_t c;
	char buffer[7];
    int16_t  num=134;
    
    // Transmit string to UART
    uart1_puts("Program start!\n");
	
	float th1,th2,th3,th4;
	Waypoint wp;
	wp.x = 0;
	wp.y = 1.3;
	wp.z = 0.7;
	wp.thdg = M_PI/2;
	calculateJointAngles(&wp, &th1, &th2, &th3, &th4);        

	int16_t angle = 0;
	uint16_t resolverReading = 0;
	char* resolverStr;
	char* resolverRefStr;
	char* flowStr;
	char* spoolStr;

	can_t txMsg;
    txMsg.id = 20; 
    txMsg.flags.rtr = 0;	// The sent message is NOT a remote-transmit-request frame
    //txMsg.flags.extended = 1;	// Sends the message with extended ID
    txMsg.length = 8;	// Message length is 8 bytes
    for (unsigned char k = 0; k < 8; k++)
		txMsg.data[k] = 66;

	uint32_t resolverId;
	ValveState valveState;
	uint8_t slewReachedReference = 0;
	uint8_t boomReachedReference = 0;
	uint8_t armReachedReference = 0;
	uint8_t bucketReachedReference = 0;
	
	can_t valveCtrlMsg;
	for(uint8_t i=0; i<100; i++)
	{
		setValveMsg(&valveCtrlMsg, 0, 0x00, SLEW_JOINT_VALVE_CTRLMSG_ID);
		can_send_message(&valveCtrlMsg);
		setValveMsg(&valveCtrlMsg, 0, 0x00, BOOM_JOINT_VALVE_CTRLMSG_ID);
		can_send_message(&valveCtrlMsg);
		setValveMsg(&valveCtrlMsg, 0, 0x00, ARM_JOINT_VALVE_CTRLMSG_ID);
		can_send_message(&valveCtrlMsg);
		setValveMsg(&valveCtrlMsg, 0, 0x00, BUCKET_JOINT_VALVE_CTRLMSG_ID);
		can_send_message(&valveCtrlMsg);
		_delay_ms(50);
	}

	while(1)
	{		
		if (can_check_message())
		{
			can_get_message(&receiveMsg);
			resolverId = receiveMsg.id;
			
			int16_t angle = 15;
			int16_t slew = -20;
			int16_t boom = 40;
			int16_t arm = -60;
			int16_t bucket = 0;
			controlSlewJointValve(&slew, &angle);
			_delay_ms(50);
			controlBoomJointValve(&boom, &angle);
			_delay_ms(50);
			controlArmJointValve(&arm, &angle);
			_delay_ms(50);
			controlBucketJointValve(&bucket, &angle);
			_delay_ms(50);
			/*switch(resolverId)
		 	{
				case SLEW_JOINT_RESOLVER_ID:
					angle = getResolverAngle(&receiveMsg);

					#ifdef DEBUG_ON
						resolverStr = "Slew joint angle (18A, TH1), 1 degree precision: ";
						printResolverAngleUART(resolverStr, angle);
						resolverRefStr = "Slew joint REFERENCE angle, 1 degree precision: ";
						printResolverReferenceAngleUART(resolverRefStr, th1);
					#endif

					int16_t slew = lrintf(th1*180.0/M_PI);
					if(controlSlewJointValve(&slew, &angle))
					{
						slewReachedReference = 1;
					}
					break;
				case BOOM_JOINT_RESOLVER_ID:
					angle = getResolverAngle(&receiveMsg);

					#ifdef DEBUG_ON
						resolverStr = "Boom joint angle (1A2, TH2), 1 degree precision: ";
						printResolverAngleUART(resolverStr, angle);
						resolverRefStr = "Boom joint REFERENCE angle, 1 degree precision: ";
						printResolverReferenceAngleUART(resolverRefStr, th2);
					#endif

					int16_t boom = lrintf(th2*180.0/M_PI);
					if(controlBoomJointValve(&boom, &angle))
					{
						boomReachedReference = 1;
					}

					break;
				case ARM_JOINT_RESOLVER_ID:
					angle = getResolverAngle(&receiveMsg);

					#ifdef DEBUG_ON
						resolverStr = "Arm joint angle (1A1, TH3), 1 degree precision: ";
						printResolverAngleUART(resolverStr, angle);
						resolverRefStr = "Arm joint REFERENCE angle, 1 degree precision: ";
						printResolverReferenceAngleUART(resolverRefStr, th3);
					#endif

					int16_t arm = lrintf(th3*180.0/M_PI);
					if(controlArmJointValve(&arm, &angle))
					{
						armReachedReference = 1;
					}
					break;
				case BUCKET_JOINT_RESOLVER_ID:
					angle = getResolverAngle(&receiveMsg);

					#ifdef DEBUG_ON
						resolverStr = "Bucket joint angle (1A3, TH4), 1 degree precision: ";
						printResolverAngleUART(resolverStr, angle);
						resolverRefStr = "Bucket joint REFERENCE angle, 1 degree precision: ";
						printResolverReferenceAngleUART(resolverRefStr, th4);
					#endif

					int16_t bucket = lrintf(th4*180.0/M_PI);
					if(controlBucketJointValve(&bucket, &angle))
					{
						bucketReachedReference = 1;
					}
					break;

				//-----------------------------------------------------------------------

				case SLEW_JOINT_VALVE_AVEF_ID:
					flowStr = "\nSlew joint flow percent: ";
					spoolStr = "Slew joint spool state: ";
					valveState = getValveState(&receiveMsg);
					//printValveStateUART(flowStr, spoolStr, &valveState);
					break;
				
				case BOOM_JOINT_VALVE_AVEF_ID:
					flowStr = "\nBoom joint flow percent: ";
					spoolStr = "Boom joint spool state: ";
					valveState = getValveState(&receiveMsg);
					//printValveStateUART(flowStr, spoolStr, &valveState);
					break;					

				case ARM_JOINT_VALVE_AVEF_ID:
					flowStr = "\nArm joint flow percent: ";
					spoolStr = "Arm joint spool state: ";
					valveState = getValveState(&receiveMsg);
					//printValveStateUART(flowStr, spoolStr, &valveState);
					break;

				case BUCKET_JOINT_VALVE_AVEF_ID:
					flowStr = "\nBucket joint flow percent: ";
					spoolStr = "Bucket joint spool state: ";			
					valveState = getValveState(&receiveMsg);
					//printValveStateUART(flowStr, spoolStr, &valveState);
					break;
		 	}*/
	  	
		}
		/*if(slewReachedReference && boomReachedReference && armReachedReference && bucketReachedReference)
		{
			float x,y,z;
			calculateBucketCoordinates(&x, &y, &z, th1, th2, th3, th4);
			uart1_puts("Waypoint reached!\n");
			printCurrentCoordinatesUART("Current coordinates of the bucket tip: ", &x, &y, &z);
			_delay_ms(10000);

		}*/

	
		//can_send_message(&txMsg);
		//_delay_ms(1000);

	}

	return 0;
}



ISR(TIMER0_OVF_vect)
{
	// Set hb flag every 1000ms
	if (msCount++ > HEARTBEAT_INTERVAL)
	{
		f_heartbeat = 1;
		msCount = 0;
	}

	if ((msCount % PID_CTRL_INTERVAL) == 0)
		f_ctrl = 1;

	// Start counting from '6' to make sure next interrupt
	// occurs in a millisecond. 256 - 16MHz/64/1000 = 6
	TCNT0 =  6;
}

