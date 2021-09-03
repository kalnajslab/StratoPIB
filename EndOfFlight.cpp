/*
 *  EndOfFlight.cpp
 *  Author:  Alex St. Clair
 *  Created: July 2019
 *
 *  This file implements the RACHuTS end of flight mode.
 */

#include "StratoPIB.h"

enum EFStates_t : uint8_t {
    EF_ENTRY = MODE_ENTRY,

    // add any desired states between entry and shutdown
    EF_LOOP,

    EF_ERROR_LANDING = MODE_ERROR,
    EF_SHUTDOWN = MODE_SHUTDOWN,
    EF_EXIT = MODE_EXIT
};

void StratoPIB::EndOfFlightMode()
{
    switch (inst_substate) {
    case EF_ENTRY:
        // perform setup
        log_nominal("Entering EF");

        // do anything else?
        mcbComm.TX_ASCII(MCB_CANCEL_MOTION);
        delay(100);
        mcbComm.TX_ASCII(MCB_GO_LOW_POWER);

        inst_substate = EF_LOOP;
        break;
    case EF_LOOP:
        // nominal ops
        log_debug("EF loop");
        break;
    case EF_ERROR_LANDING:
        log_debug("EF error");
        break;
    case EF_SHUTDOWN:
        // prep for shutdown
        log_nominal("Shutdown warning received in EF");
        break;
    case EF_EXIT:
        // perform cleanup
        log_nominal("Exiting EF");
        break;
    default:
        // todo: throw error
        log_error("Unknown substate in EF");
        inst_substate = EF_ENTRY; // reset
        break;
    }
}