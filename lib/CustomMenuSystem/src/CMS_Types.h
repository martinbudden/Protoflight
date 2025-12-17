#pragma once

#include <cstdint>


static constexpr uint16_t OME_LABEL = 0;
static constexpr uint16_t OME_SUBMENU = 1;
static constexpr uint16_t OME_FUNCTION_CALL = 2;
static constexpr uint16_t OME_EXIT = 3;
static constexpr uint16_t OME_BACK = 4;
static constexpr uint16_t OME_END = 5;

static constexpr uint16_t OME_STRING = 6;
static constexpr uint16_t OME_TABLE = 7;
static constexpr uint16_t OME_BOOL = 8;
static constexpr uint16_t OME_UINT8 = 9;
static constexpr uint16_t OME_INT8 = 10;
static constexpr uint16_t OME_UINT16 = 11;
static constexpr uint16_t OME_INT16 = 12;
static constexpr uint16_t OME_UINT32 = 13;
static constexpr uint16_t OME_INT32 = 14;
static constexpr uint16_t OME_UINT8_FIXED = 15;
static constexpr uint16_t OME_UINT16_FIXED = 16;

static constexpr uint16_t OME_VISIBLE = 17;
static constexpr uint16_t OME_MENU = 18;

static constexpr uint16_t OME_TYPE_COUNT = OME_MENU;

// Flags bits
static constexpr uint16_t OME_TYPE_MASK         = 0x001F;

static constexpr uint16_t OME_PRINT_VALUE       = 0x0020;  // Value has been changed, need to redraw
static constexpr uint16_t OME_PRINT_LABEL       = 0x0040;  // Text label should be printed
static constexpr uint16_t OME_DYNAMIC           = 0x0080;  // Value should be updated dynamically
static constexpr uint16_t OME_OPTION_STRING     = 0x0100;  // (Temporary) Flag for OME_SUBMENU, indicating function should be called to get a string to display
static constexpr uint16_t OME_REBOOT_REQUIRED   = 0x0200;  // Reboot is required if the value is changed
static constexpr uint16_t OME_SCROLLING_TICKER  = 0x0400;  // Long values are displayed as horizontally scrolling tickers (OME_TABLE only)
static constexpr uint16_t OME_SLIDER_RP         = 0x0800;  // Value should be read only if simplified RP slider is enabled
static constexpr uint16_t OME_SLIDER_RPY        = 0x1000;  // Value should be read only if simplified RPY slider is enabled
static constexpr uint16_t OME_SLIDER_GYRO       = 0x2000;  // Value should be read only if simplified gyro slider is enabled
static constexpr uint16_t OME_SLIDER_DTERM      = 0x4000;  // Value should be read only if simplified D term slider is enabled


struct OSD_String_t {
    uint8_t* val;
};

struct OSD_TABLE_t {
    uint8_t* val;
    uint8_t max;
    const char * const* names;
};

struct OSD_BOOL_t {
    bool* val;
};

struct OSD_UINT8_t {
    uint8_t* val;
    uint8_t min;
    uint8_t max;
    uint8_t step;
};

struct OSD_INT8_t {
    int8_t* val;
    int8_t min;
    int8_t max;
    int8_t step;
};

struct OSD_UINT16_t {
    uint16_t* val;
    uint16_t min;
    uint16_t max;
    uint16_t step;
};

struct OSD_INT16_t {
    int16_t* val;
    int16_t min;
    int16_t max;
    int16_t step;
};

struct OSD_UINT32_t {
    uint32_t* val;
    uint32_t min;
    uint32_t max;
    uint32_t step;
};

struct OSD_INT32_t {
    int32_t* val;
    int32_t min;
    int32_t max;
    int32_t step;
};

struct OSD_UINT8_FIXED_t {
    uint8_t* val;
    uint8_t min;
    uint8_t max;
    uint8_t step;
    uint16_t multiplier;
};

struct OSD_UINT16_FIXED_t {
    uint16_t* val;
    uint16_t min;
    uint16_t max;
    uint16_t step;
    uint16_t multiplier;
};
