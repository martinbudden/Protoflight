#include "version.h"
#include "Targets.h"

#if !defined(BOARD_IDENTIFIER)
#define BOARD_IDENTIFIER    "NONE"
#endif

//const unsigned int buildTimeUnix = BUILD_TIME_UNIX;

const char* const targetName = "TARGETNAME";

const char * const buildDate = __DATE__;

const char * const buildTime = __TIME__;

const char* const flightControllerIdentifier = FC_FIRMWARE_IDENTIFIER; // 4 UPPER CASE alpha numeric characters that identify the flight controller.

const char* const boardIdentifier = BOARD_IDENTIFIER;

//const char * const gitShortRevision = "fea0e8c";
