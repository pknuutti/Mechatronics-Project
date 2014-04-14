#include <stdint.h>
#include "can.h"
#include "resolvers.h"
#include "kinematics.h"
#include "uart.h"

void sendResolverNMTMessage(void)
{
	can_t NMTMsg;
    NMTMsg.id = 0x00; 
    NMTMsg.flags.rtr = 0;	// The sent message is NOT a remote-transmit-request frame
    NMTMsg.flags.extended = 0;	// Sends the message with standard ID
    NMTMsg.length = 2;	// Message length is 8 bytes
   
	NMTMsg.data[0] = 0x01;
	NMTMsg.data[1] = 0x00;	//0x00 for all resolvers

	can_send_message(&NMTMsg);
}

// This function is mainly for debugging purposes
uint16_t getResolverReading(can_t* resolverMsg)
{
	uint32_t resolverId = resolverMsg->id;
	uint16_t angle = 0;
	float tmp;
   	angle += resolverMsg->data[1];
	angle <<= 8;
	angle += resolverMsg->data[0];

	return angle;
}


int16_t getResolverAngle(can_t* resolverMsg)
{
	uint32_t resolverId = resolverMsg->id;
	uint16_t resolverReading = 0;
	int16_t angle;

   	resolverReading += resolverMsg->data[1];
	resolverReading = resolverReading << 8;
	resolverReading += resolverMsg->data[0];

	switch(resolverId)
	{
		case SLEW_JOINT_RESOLVER_ID:
			angle = (resolverReading - SLEW_JOINT_RESOLVER_MIN) + TH1_MIN_DEG*100;
			break;
		case BOOM_JOINT_RESOLVER_ID:
			angle = (resolverReading - BOOM_JOINT_RESOLVER_MIN) + TH2_MIN_DEG*100;
			break;
		case ARM_JOINT_RESOLVER_ID:
			angle = (ARM_JOINT_RESOLVER_MIN - resolverReading) + TH3_MIN_DEG*100;
			break;
		case BUCKET_JOINT_RESOLVER_ID:
			angle = calculateBucketAngle(resolverReading);
			break;			
	}

	return angle/100;
}

int16_t calculateBucketAngle(uint16_t resolverReading)
{
	float resolverReadingFloat, tmp;
	int16_t bucketAngle = 0;

	resolverReadingFloat = resolverReading/100.0;
	tmp = (resolverReadingFloat - 260.22)/40.886;
	bucketAngle = -12.37*pow(tmp,2) + 66.704*tmp - 22.757;
	// Multiply by 100 to compensate for division by 100 in getResolverAngle
	bucketAngle = bucketAngle*100; 

	return bucketAngle;
}

void printResolverReadingUART(char* str, uint16_t angle)
{
	char buffer[7];
	uart1_puts(str);
	itoa(angle, buffer, 10); // the parameter 10 signifies base 10        
    uart1_puts(buffer);
	uart1_puts("\n");
}

void printResolverAngleUART(char* str, int16_t angle)
{
	char buffer[7];
	uart1_puts(str);
	itoa(angle, buffer, 10); // the parameter 10 signifies base 10        
    uart1_puts(buffer);
	uart1_puts("\n");
}

void printResolverReferenceAngleUART(char* str, float referenceAngle)
{
	char buffer[15];
	referenceAngle = (referenceAngle * 180.0 / M_PI);
	dtostrf(referenceAngle,5,2,buffer);
	uart1_puts(str);
    uart1_puts(buffer);
	uart1_puts("\n");
}

void printCurrentCoordinatesUART(char* str, float x, float y, float z)
{
	char buffer[15];
	uart1_puts(str);
	dtostrf(x,5,2,buffer);
    uart1_puts(buffer);
	uart1_puts("\n");
}
