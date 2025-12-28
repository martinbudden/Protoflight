#pragma once

#include <cstdint>


// Artificial horizon center screen Graphics
#if defined(M5_UNIFIED)
static constexpr uint8_t SYM_AH_CENTER_LINE     = '-';
static constexpr uint8_t SYM_AH_CENTER          = '+';
static constexpr uint8_t SYM_AH_CENTER_LINE_RIGHT = '-';
static constexpr uint8_t SYM_AH_RIGHT           = '>';
static constexpr uint8_t SYM_AH_LEFT            = '<';
static constexpr uint8_t SYM_AH_DECORATION      = '*';

static constexpr uint8_t SYM_AH_BAR9_0          = '#';
#else
static constexpr uint8_t SYM_AH_CENTER_LINE     = 0x72;
static constexpr uint8_t SYM_AH_CENTER          = 0x73;
static constexpr uint8_t SYM_AH_CENTER_LINE_RIGHT = 0x74;
static constexpr uint8_t SYM_AH_RIGHT           = 0x02;
static constexpr uint8_t SYM_AH_LEFT            = 0x03;
static constexpr uint8_t SYM_AH_DECORATION      = 0x13;
// Artificial horizon bars
static constexpr uint8_t SYM_AH_BAR9_0          = 0x80;
#endif

//Misc
static constexpr uint8_t SYM_NONE               = 0x00;
static constexpr uint8_t SYM_END_OF_FONT        = 0xFF;
static constexpr uint8_t SYM_BLANK              = 0x20;
static constexpr uint8_t SYM_HYPHEN             = 0x2D;
static constexpr uint8_t SYM_BLACKBOX_LOG       = 0x10;
static constexpr uint8_t SYM_HOMEFLAG           = 0x11;

// GPS and navigation
static constexpr uint8_t SYM_LAT                = 0x89;
static constexpr uint8_t SYM_LON                = 0x98;
static constexpr uint8_t SYM_ALTITUDE           = 0x7F;
static constexpr uint8_t SYM_TOTAL_DISTANCE     = 0x71;
static constexpr uint8_t SYM_OVER_HOME          = 0x05;

// RSSI
static constexpr uint8_t SYM_RSSI               = 0x01;
static constexpr uint8_t SYM_LINK_QUALITY       = 0x7B;

// Throttle Position (%)
static constexpr uint8_t SYM_THR                = 0x04;

// Unit Icons (Metric)
static constexpr uint8_t SYM_M                  = 0x0C;
static constexpr uint8_t SYM_KM                 = 0x7D;
static constexpr uint8_t SYM_C                  = 0x0E;

// Unit Icons (Imperial)
static constexpr uint8_t SYM_FT                 = 0x0F;
static constexpr uint8_t SYM_MILES              = 0x7E;
static constexpr uint8_t SYM_F                  = 0x0D;

// Heading Graphics
static constexpr uint8_t SYM_HEADING_N          = 0x18;
static constexpr uint8_t SYM_HEADING_S          = 0x19;
static constexpr uint8_t SYM_HEADING_E          = 0x1A;
static constexpr uint8_t SYM_HEADING_W          = 0x1B;
static constexpr uint8_t SYM_HEADING_DIVIDED_LINE = 0x1C;
static constexpr uint8_t SYM_HEADING_LINE       = 0x1D;

// Satellite Graphics
static constexpr uint8_t SYM_SAT_L              = 0x1E;
static constexpr uint8_t SYM_SAT_R              = 0x1F;

// Direction arrows
static constexpr uint8_t SYM_ARROW_SOUTH        = 0x60;
static constexpr uint8_t SYM_ARROW_2            = 0x61;
static constexpr uint8_t SYM_ARROW_3            = 0x62;
static constexpr uint8_t SYM_ARROW_4            = 0x63;
static constexpr uint8_t SYM_ARROW_EAST         = 0x64;
static constexpr uint8_t SYM_ARROW_6            = 0x65;
static constexpr uint8_t SYM_ARROW_7            = 0x66;
static constexpr uint8_t SYM_ARROW_8            = 0x67;
static constexpr uint8_t SYM_ARROW_NORTH        = 0x68;
static constexpr uint8_t SYM_ARROW_10           = 0x69;
static constexpr uint8_t SYM_ARROW_11           = 0x6A;
static constexpr uint8_t SYM_ARROW_12           = 0x6B;
static constexpr uint8_t SYM_ARROW_WEST         = 0x6C;
static constexpr uint8_t SYM_ARROW_14           = 0x6D;
static constexpr uint8_t SYM_ARROW_15           = 0x6E;
static constexpr uint8_t SYM_ARROW_16           = 0x6F;

static constexpr uint8_t SYM_ARROW_SMALL_UP     = 0x75;
static constexpr uint8_t SYM_ARROW_SMALL_DOWN   = 0x76;

// Time
static constexpr uint8_t SYM_ON_M               = 0x9B;
static constexpr uint8_t SYM_FLY_M              = 0x9C;

// Lap Timer
static constexpr uint8_t SYM_CHECKERED_FLAG     = 0x24;
static constexpr uint8_t SYM_PREV_LAP_TIME      = 0x79;

// Speed
static constexpr uint8_t SYM_SPEED              = 0x70;
static constexpr uint8_t SYM_KPH                = 0x9E;
static constexpr uint8_t SYM_MPH                = 0x9D;
static constexpr uint8_t SYM_MPS                = 0x9F;
static constexpr uint8_t SYM_FTPS               = 0x99;

// Menu cursor
static constexpr uint8_t SYM_CURSOR             = SYM_AH_LEFT;

// Stick overlays
static constexpr uint8_t SYM_STICK_OVERLAY_SPRITE_HIGH = 0x08;
static constexpr uint8_t SYM_STICK_OVERLAY_SPRITE_MID  = 0x09;
static constexpr uint8_t SYM_STICK_OVERLAY_SPRITE_LOW  = 0x0A;
static constexpr uint8_t SYM_STICK_OVERLAY_CENTER      = 0x0B;
static constexpr uint8_t SYM_STICK_OVERLAY_VERTICAL    = 0x16;
static constexpr uint8_t SYM_STICK_OVERLAY_HORIZONTAL  = 0x17;
