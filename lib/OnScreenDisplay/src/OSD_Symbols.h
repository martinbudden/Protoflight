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

// Stick overlays
static constexpr uint8_t SYM_STICK_OVERLAY_SPRITE_HIGH = 0x08;
static constexpr uint8_t SYM_STICK_OVERLAY_SPRITE_MID  = 0x09;
static constexpr uint8_t SYM_STICK_OVERLAY_SPRITE_LOW  = 0x0A;
static constexpr uint8_t SYM_STICK_OVERLAY_CENTER      = 0x0B;
static constexpr uint8_t SYM_STICK_OVERLAY_VERTICAL    = 0x16;
static constexpr uint8_t SYM_STICK_OVERLAY_HORIZONTAL  = 0x17;

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
