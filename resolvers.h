#ifndef RESOLVERS_H
#define RESOVLERS_H

#include "can.h"

/* Canbus IDs for angle resolvers and angles */
#define SLEW_JOINT_RESOLVER_ID      394 // 0x18A
#define BOOM_JOINT_RESOLVER_ID		418 // 0x1A2
#define ARM_JOINT_RESOLVER_ID       417	// 0x1A1
#define BUCKET_JOINT_RESOLVER_ID    419 // 0x1A3

#define SLEW_JOINT_RESOLVER_MOB     1 
#define BOOM_JOINT_RESOLVER_MOB     2 
#define ARM_JOINT_RESOLVER_MOB      3 
#define BUCKET_JOINT_RESOLVER_MOB   4 

#define SLEW_JOINT_RESOLVER_MIN 	19988
#define BOOM_JOINT_RESOLVER_MIN		16568
#define ARM_JOINT_RESOLVER_MIN		28379
#define BUCKET_JOINT_RESOLVER_MIN	20219


void sendResolverNMTMessage(void);

int16_t getResolverAngle(can_t* resolverMsg);

void printResolverAngleUART(char* str, int16_t angle);


#endif
