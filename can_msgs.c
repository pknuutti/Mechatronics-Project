#include <stdint.h>
#include "canlib/src/config.h"
#include "canlib/can.h"
#include "canlib/src/at90can_private.h"
#include "canlib/src/can_private.h"
#include "canlib/src/can_buffer.h"
#include "canlib/src/utils.h"

#include "can_msgs.h"


can_t setValveMsg(uint8_t flow, uint8_t spoolState, uint32_t valveID)
{
   can_t valveMsg;
   valveMsg.id = valveID;
   valveMsg.flags.rtr = 0;	// The sent message is NOT a remote-transmit-request frame
   valveMsg.flags.extended = 1;	// Sends the message with extended ID
   valveMsg.length = 8;	// Message length is 8 bytes
   for (uint8_t k = 0; k < 8; k++)
   {
      valveMsg.data[k] = 0;
   }
   valveMsg.data[0] = flow;
   valveMsg.data[2] = spoolState;

   return valveMsg;

}

ValveState getValveState(can_t* valveMsg)
{
   ValveState valve;
   // Flow percent 0% == 125 and 100% == 225
   int8_t extendFlow = valveMsg->data[0] - 125;
   int8_t retractFlow = valveMsg->data[1] - 125;
   int8_t currentFlow = 0;

   /* CAN message from valve contains both extend and retract flow
    * information. As they both cannot be larger than 0 at the same time,
    * the bigger of the two is chosen. */
   currentFlow = (extendFlow > retractFlow ? extendFlow : retractFlow);

   valve.flowPercent = currentFlow;
   valve.spoolState = valveMsg->data[3];

   return valve;

}



