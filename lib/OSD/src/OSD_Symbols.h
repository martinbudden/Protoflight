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
