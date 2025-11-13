#pragma once

#include <cstdint>


enum osd_menu_element_e {
    OME_Label,
    OME_Back,
    OME_OSD_Exit,
    OME_Submenu,
    OME_Funcall,
    OME_Bool,
    OME_INT8,
    OME_UINT8,
    OME_UINT16,
    OME_INT16,
    OME_UINT32,
    OME_INT32,
    OME_String,
    OME_FLOAT,
    OME_VISIBLE,
    OME_TAB,
    OME_END,

    // Debug aid
    OME_MENU,

    OME_COUNT = OME_MENU,

// Bits in flags
    PRINT_VALUE      = 0x0020,  // Value has been changed, need to redraw
    PRINT_LABEL      = 0x0040,  // Text label should be printed
    DYNAMIC          = 0x0080,  // Value should be updated dynamically
    OPTSTRING        = 0x0100,  // (Temporary) Flag for OME_Submenu, indicating func should be called to get a string to display.
    REBOOT_REQUIRED  = 0x0200,  // Reboot is required if the value is changed
    SCROLLING_TICKER = 0x0400,  // Long values are displayed as horizontally scrolling tickers (OME_TAB only)
    SLIDER_RP        = 0x0800,  // Value should be read only if simplified RP slider is enabled
    SLIDER_RPY       = 0x1000,  // Value should be read only if simplified RPY slider is enabled
    SLIDER_GYRO      = 0x2000,  // Value should be read only if simplified gyro slider is enabled
    SLIDER_DTERM     = 0x4000  // Value should be read only if simplified D term slider is enabled
};

// Special return value(s) for function chaining by CMSMenuFuncPtr
extern int menuChainBack;
#define MENU_CHAIN_BACK  (&menuChainBack) // Causes automatic cmsMenuBack

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

struct OSD_TAB_t {
    uint8_t* val;
    uint8_t max;
    const char * const* names;
};

struct OSD_String_t {
    uint8_t* val;
};
