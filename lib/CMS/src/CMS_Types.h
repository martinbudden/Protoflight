#pragma once

#include <cstdint>


static constexpr uint16_t OME_Label = 0;
static constexpr uint16_t OME_Back = 1;
static constexpr uint16_t OME_OSD_Exit = 2;
static constexpr uint16_t OME_Submenu = 3;
static constexpr uint16_t OME_FunctionCall = 4;
static constexpr uint16_t OME_Bool = 5;
static constexpr uint16_t OME_INT8 = 6;
static constexpr uint16_t OME_UINT8 = 7;
static constexpr uint16_t OME_UINT16 = 8;
static constexpr uint16_t OME_INT16 = 9;
static constexpr uint16_t OME_UINT32 = 10;
static constexpr uint16_t OME_INT32 = 11;
static constexpr uint16_t OME_String = 12;
static constexpr uint16_t OME_FLOAT = 13;
static constexpr uint16_t OME_VISIBLE = 14;
static constexpr uint16_t OME_TABLE = 15;
static constexpr uint16_t OME_END = 16;
static constexpr uint16_t OME_MENU = 17;

static constexpr uint16_t OME_COUNT = OME_MENU;
static constexpr uint16_t OME_MASK = 0x001F;

// Bits in flags
static constexpr uint16_t PRINT_VALUE      = 0x0020;  // Value has been changed, need to redraw
static constexpr uint16_t PRINT_LABEL      = 0x0040;  // Text label should be printed
static constexpr uint16_t DYNAMIC          = 0x0080;  // Value should be updated dynamically
static constexpr uint16_t OPTSTRING        = 0x0100;  // (Temporary) Flag for OME_Submenu, indicating func should be called to get a string to display
static constexpr uint16_t REBOOT_REQUIRED  = 0x0200;  // Reboot is required if the value is changed
static constexpr uint16_t SCROLLING_TICKER = 0x0400;  // Long values are displayed as horizontally scrolling tickers (OME_TABLE only)
static constexpr uint16_t SLIDER_RP        = 0x0800;  // Value should be read only if simplified RP slider is enabled
static constexpr uint16_t SLIDER_RPY       = 0x1000;  // Value should be read only if simplified RPY slider is enabled
static constexpr uint16_t SLIDER_GYRO      = 0x2000;  // Value should be read only if simplified gyro slider is enabled
static constexpr uint16_t SLIDER_DTERM     = 0x4000;  // Value should be read only if simplified D term slider is enabled


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

struct OSD_INT16_t {
    int16_t* val;
    int16_t min;
    int16_t max;
    int16_t step;
};

struct OSD_UINT16_t {
    uint16_t* val;
    uint16_t min;
    uint16_t max;
    uint16_t step;
};

struct OSD_INT32_t {
    int32_t* val;
    int32_t min;
    int32_t max;
    int32_t step;
};

struct OSD_UINT32_t {
    uint32_t* val;
    uint32_t min;
    uint32_t max;
    uint32_t step;
};

struct OSD_FLOAT_t {
    uint8_t* val;
    uint8_t min;
    uint8_t max;
    uint8_t step;
    uint16_t multiplier;
};

struct OSD_TABLE_t {
    uint8_t* val;
    uint8_t max;
    const char * const* names;
};

struct OSD_String_t {
    uint8_t* val;
};
