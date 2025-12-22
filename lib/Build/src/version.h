#pragma once
#include <array>

// NOLINTBEGIN(modernize-macro-to-enum)

#if true
#define FC_FIRMWARE_NAME            "Betaflight"
#define FC_FIRMWARE_IDENTIFIER      "BTFL"
#define FC_VERSION_MAJOR            4  // increment when a major release is made (big new feature, etc)
#define FC_VERSION_MINOR            5  // increment when a minor release is made (small new feature, change etc)
#define FC_VERSION_PATCH_LEVEL      0  // increment when a bug is fixed
enum { U_ID_0 = 0 };
enum { U_ID_1 = 1 };
enum { U_ID_2 = 2 };
#else
#define FC_FIRMWARE_NAME            "Protoflight"
#define FC_FIRMWARE_IDENTIFIER      "PTFL"
#define FC_VERSION_MAJOR            0  // increment when a major release is made (big new feature, etc)
#define FC_VERSION_MINOR            0  // increment when a minor release is made (small new feature, change etc)
#define FC_VERSION_PATCH_LEVEL      1  // increment when a bug is fixed
#endif

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#define FC_VERSION_STRING STR(FC_VERSION_MAJOR) "." STR(FC_VERSION_MINOR) "." STR(FC_VERSION_PATCH_LEVEL)

extern const char* const gitRevision; // lower case hexadecimal digits

extern const unsigned int buildTimeUnix;  // yyyy-mm-ddThh:mm:ss+00:00

extern const char* const targetName;

extern const char* const flightControllerIdentifier;

extern const char* const boardIdentifier;

enum { BUILD_DATE_LENGTH = 11};
extern const char* const buildDate; // "MMM DD YYYY"; // MMM = Jan/Feb/...

enum { BUILD_TIME_LENGTH = 8 };
extern const char* const buildTime; // "HH:MM:SS"

// NOLINTEND(modernize-macro-to-enum)
