#ifndef VALVES_H
#define VALVES_H

#include "can.h"

#define BLOCKED 0x00
#define EXTEND 0x01
#define RETRACT 0x02
#define FLOATING 0x03

#define SLEW_JOINT_VALVE_AVEF_ID       	0xCFE1181
#define BOOM_JOINT_VALVE_AVEF_ID		0xCFE1383
#define ARM_JOINT_VALVE_AVEF_ID     	0xCFE1484
#define BUCKET_JOINT_VALVE_AVEF_ID  	0xCFE1585

#define SLEW_JOINT_VALVE_CTRLMSG_ID    	0xCFE3106
#define BOOM_JOINT_VALVE_CTRLMSG_ID	   	0xCFE3306
#define ARM_JOINT_VALVE_CTRLMSG_ID     	0xCFE3506
#define BUCKET_JOINT_VALVE_CTRLMSG_ID  	0xCFE3406

typedef struct ValveState
{
   uint8_t flowPercent;
   uint8_t spoolState;
} ValveState;

void setValveMsg(can_t* const valveMsg, const uint8_t flow, const uint8_t spoolState, const uint32_t valveID);

ValveState getValveState(can_t* valveMsg);

void printValveStateUART(char* flowStr, char* spoolStr, ValveState* valve);

#endif
