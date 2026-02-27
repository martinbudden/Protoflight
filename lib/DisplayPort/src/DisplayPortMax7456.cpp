#include "DisplayPortMax7456.h"

#include <algorithm> // for std::ranges::equal
#include <cstring>
#if (__cplusplus >= 202002L)
#include <ranges>
#endif

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-constant-array-index,cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise,modernize-macro-to-enum,cppcoreguidelines-macro-usage)


// DEBUG_MAX7456_SIGNAL
static constexpr uint8_t DEBUG_MAX7456_SIGNAL_MODEREG = 0;
static constexpr uint8_t DEBUG_MAX7456_SIGNAL_SENSE   = 1;
static constexpr uint8_t DEBUG_MAX7456_SIGNAL_REINIT  = 2;
static constexpr uint8_t DEBUG_MAX7456_SIGNAL_ROWS    = 3;

// DEBUG_MAX7456_SPI_CLOCK
#define DEBUG_MAX7456_SPI_CLOCK_OVERCLOCK   0
#define DEBUG_MAX7456_SPI_CLOCK_DEVTYPE     1
#define DEBUG_MAX7456_SPI_CLOCK_DIVISOR     2
#define DEBUG_MAX7456_SPI_CLOCK_X100        3


#define SYNC_MODE_AUTO              0x00
#define SYNC_MODE_INTERNAL          0x30
#define SYNC_MODE_EXTERNAL          0x20

static constexpr uint8_t VIDEO_MODE_PAL     = 0x40;
static constexpr uint8_t VIDEO_MODE_NTSC    = 0x00;
static constexpr uint8_t VIDEO_MODE_MASK    = 0x40;
#define VIDEO_MODE_IS_PAL(val)      (((val) & VIDEO_MODE_MASK) == VIDEO_MODE_PAL)
#define VIDEO_MODE_IS_NTSC(val)     (((val) & VIDEO_MODE_MASK) == VIDEO_MODE_NTSC)

#define VIDEO_SIGNAL_DEBOUNCE_MS    100 // Time to wait for input to stabilize

// VM1 bits

// duty cycle is on_off
#define BLINK_DUTY_CYCLE_50_50 0x00
#define BLINK_DUTY_CYCLE_33_66 0x01
#define BLINK_DUTY_CYCLE_25_75 0x02
#define BLINK_DUTY_CYCLE_75_25 0x03

// blinking time
#define BLINK_TIME_0 0x00
#define BLINK_TIME_1 0x04
#define BLINK_TIME_2 0x08
#define BLINK_TIME_3 0x0C

// background mode brightness (percent)
#define BACKGROUND_BRIGHTNESS_0 0x00
#define BACKGROUND_BRIGHTNESS_7 0x01
#define BACKGROUND_BRIGHTNESS_14 0x02
#define BACKGROUND_BRIGHTNESS_21 0x03
#define BACKGROUND_BRIGHTNESS_28 0x04
#define BACKGROUND_BRIGHTNESS_35 0x05
#define BACKGROUND_BRIGHTNESS_42 0x06
#define BACKGROUND_BRIGHTNESS_49 0x07

#define BACKGROUND_MODE_GRAY 0x80

// STAT register bits

#define STAT_PAL      0x01
#define STAT_NTSC     0x02
#define STAT_LOS      0x04
#define STAT_NVR_BUSY 0x20

#define STAT_IS_PAL(val)  ((val) & STAT_PAL)
#define STAT_IS_NTSC(val) ((val) & STAT_NTSC)
#define STAT_IS_LOS(val)  ((val) & STAT_LOS)

#define VIN_IS_PAL(val)  (!STAT_IS_LOS(val) && STAT_IS_PAL(val))
#define VIN_IS_NTSC(val)  (!STAT_IS_LOS(val) && STAT_IS_NTSC(val))

// DMM register bits
#define DMM_AUTO_INC 0x01

// Kludge warning!
// There are occasions that NTSC is not detected even with !LOS (AB7456 specific?)
// When this happens, lower 3 bits of STAT register is read as zero.
// To cope with this case, this macro defines !LOS && !PAL as NTSC.
// Should be compatible with MAX7456 and non-problematic case.

#define VIN_IS_NTSC_ALT(val)  (!STAT_IS_LOS(val) && !STAT_IS_PAL(val))


// DMM special bits
#define CLEAR_DISPLAY 0x04
#define CLEAR_DISPLAY_VERT 0x06
#define INVERT_PIXEL_COLOR 0x08

// Special address for terminating incremental write
#define END_STRING 0xff

#define MAX7456ADD_READ         0x80
#define MAX7456ADD_VM0          0x00  //0b0011100// 00 // 00             ,0011100
#define MAX7456ADD_VM1          0x01
#define MAX7456ADD_HOS          0x02
#define MAX7456ADD_VOS          0x03
#define MAX7456ADD_DMM          0x04
#define MAX7456ADD_DMAH         0x05
#define MAX7456ADD_DMAL         0x06
#define MAX7456ADD_DMDI         0x07
#define MAX7456ADD_CMM          0x08
#define MAX7456ADD_CMAH         0x09
#define MAX7456ADD_CMAL         0x0a
#define MAX7456ADD_CMDI         0x0b
#define MAX7456ADD_OSDM         0x0c
#define MAX7456ADD_RB0          0x10
#define MAX7456ADD_RB1          0x11
#define MAX7456ADD_RB2          0x12
#define MAX7456ADD_RB3          0x13
#define MAX7456ADD_RB4          0x14
#define MAX7456ADD_RB5          0x15
#define MAX7456ADD_RB6          0x16
#define MAX7456ADD_RB7          0x17
#define MAX7456ADD_RB8          0x18
#define MAX7456ADD_RB9          0x19
#define MAX7456ADD_RB10         0x1a
#define MAX7456ADD_RB11         0x1b
#define MAX7456ADD_RB12         0x1c
#define MAX7456ADD_RB13         0x1d
#define MAX7456ADD_RB14         0x1e
#define MAX7456ADD_RB15         0x1f
#define MAX7456ADD_OSDBL        0x6c
#define MAX7456ADD_STAT         0xA0

#define NVM_RAM_SIZE            54
#define WRITE_NVR               0xA0

// Device type
#define MAX7456_DEVICE_TYPE_MAX 0
#define MAX7456_DEVICE_TYPE_AT  1




//Max bytes to update in one call to max7456DrawScreen()

#define MAX_BYTES2SEND          250
#define MAX_BYTES2SEND_POLLED   12
#define MAX_ENCODE_US           20
#define MAX_ENCODE_US_POLLED    10


volatile bool DisplayPortMax7456::ActiveDMA {false};


#if !defined(FRAMEWORK_TEST)
DisplayPortMax7456::DisplayPortMax7456(uint8_t SPI_index, const BusSpi::stm32_spi_pins_t& pins) :
    _bus(INITIAL_SPI_CLOCK_FREQUENCY_HZ, SPI_index, pins)
{
    _maxScreenSize = VIDEO_BUFFER_PAL_CHARACTER_COUNT;
    _smallArrowUp = 0x75;
}

DisplayPortMax7456::DisplayPortMax7456(uint8_t SPI_index, const BusSpi::spi_pins_t& pins) :
    _bus(INITIAL_SPI_CLOCK_FREQUENCY_HZ, SPI_index, pins)
{
    _maxScreenSize = VIDEO_BUFFER_PAL_CHARACTER_COUNT;
    _smallArrowUp = 0x75;
}

uint8_t* DisplayPortMax7456::getLayerBuffer(layer_e layer)
{
    return &_displayLayers[layer].buffer[0];
}

uint8_t* DisplayPortMax7456::getActiveLayerBuffer()
{
    return getLayerBuffer(_activeLayer);
}

void DisplayPortMax7456::setRegisterVM1()
{
    uint8_t backgroundGray = BACKGROUND_BRIGHTNESS_28; // this is the device default background gray level
    uint8_t vm1Register = BLINK_TIME_1 | BLINK_DUTY_CYCLE_75_25; // device defaults
    if (_backgroundType != BACKGROUND_TRANSPARENT) {
        vm1Register |= BACKGROUND_MODE_GRAY;
        switch (_backgroundType) {
        case BACKGROUND_BLACK:
            backgroundGray = BACKGROUND_BRIGHTNESS_0;
            break;
        case BACKGROUND_LIGHT_GRAY:
            backgroundGray = BACKGROUND_BRIGHTNESS_49;
            break;
        case BACKGROUND_GRAY:
            [[fallthrough]];
        default:
            backgroundGray = BACKGROUND_BRIGHTNESS_28;
            break;
        }
    }
    vm1Register |= (backgroundGray << 4);
    //spiWriteReg(dev, MAX7456ADD_VM1, vm1Register);
    _bus.write_register(MAX7456ADD_VM1, vm1Register);
}

// When clearing the shadow buffer we fill with 0 so that the characters will
// be flagged as changed when compared to the 0x20 used in the layer buffers.
void DisplayPortMax7456::clearShadowBuffer()
{
    _shadowBuffer.fill(0);
}

// Buffer is filled with the whitespace character (0x20)
void DisplayPortMax7456::clearLayer(layer_e layer)
{
    memset(getLayerBuffer(layer), 0x20, VIDEO_BUFFER_PAL_CHARACTER_COUNT);
}

void DisplayPortMax7456::reInit()
{
    switch (_videoSignalCfg) {
    case VIDEO_SYSTEM_AUTO: {
        const uint8_t data = _bus.read_register(MAX7456ADD_STAT);
        if (VIN_IS_NTSC(data)) {
            _videoSignalReg = VIDEO_MODE_NTSC | OSD_ENABLE; // cppcheck-suppress badBitmaskCheck
        } else if (VIN_IS_PAL(data)) {
            _videoSignalReg = VIDEO_MODE_PAL | OSD_ENABLE;
        } else {
            // No valid input signal, fallback to default (PAL)
            _videoSignalReg = VIDEO_MODE_PAL | OSD_ENABLE;
        }
        break;
    }
    case VIDEO_SYSTEM_PAL:
        _videoSignalReg = VIDEO_MODE_PAL | OSD_ENABLE;
        _rowCount = VIDEO_LINES_PAL;
        break;
    case VIDEO_SYSTEM_NTSC:
        _videoSignalReg = VIDEO_MODE_NTSC | OSD_ENABLE; // cppcheck-suppress badBitmaskCheck
        _rowCount = VIDEO_LINES_NTSC;
        break;
    }

    if (_videoSignalReg & VIDEO_MODE_PAL) {
        _maxScreenSize = VIDEO_BUFFER_PAL_CHARACTER_COUNT;
    } else {
        _maxScreenSize = VIDEO_BUFFER_NTSC_CHARACTER_COUNT;
    }

    // Set all rows to same character black/white level
    _previousBlackWhiteRegister = INVALID_PREVIOUS_REGISTER_STATE;
    brightness(0, 2);
    // Re-enable MAX7456 (last function call disables it)

    // Make sure the Max7456 is enabled
    //spiWriteReg(dev, MAX7456ADD_VM0, videoSignalReg);
    //spiWriteReg(dev, MAX7456ADD_HOS, hosRegValue);
    //spiWriteReg(dev, MAX7456ADD_VOS, vosRegValue);
    _bus.write_register(MAX7456ADD_VM0, _videoSignalReg);
    _bus.write_register(MAX7456ADD_HOS, _hosRegValue);
    _bus.write_register(MAX7456ADD_VOS, _vosRegValue);
    setRegisterVM1();

    // Clear shadow to force redraw all screen
    clearShadowBuffer();
}

void DisplayPortMax7456::preinit(const config_t& config)
{
    (void)config;
    //ioPreinitByTag(config.csTag, config.preInitOPU ? IOCFG_OUT_PP : IOCFG_IPU, PREINIT_PIN_STATE_HIGH);
}

// Here we init only CS and try to init MAX for first time.
// Also detect device type (MAX v.s. AT)

DisplayPortMax7456::init_status_e DisplayPortMax7456::init(const config_t* max7456Config, const vcd_profile_t* pVcdProfile, bool cpuOverclock, Debug& debug)
{
    (void)debug;

    _deviceDetected = false;
    _backgroundType = DisplayPortBase::BACKGROUND_TRANSPARENT;

    // initialize all layers
#if (__cplusplus >= 202002L)
    for (auto ii : std::views::iota(size_t{0}, size_t{MAX7456_SUPPORTED_LAYER_COUNT})) {
#else
    for (size_t ii = 0; ii < MAX7456_SUPPORTED_LAYER_COUNT; ++ii) {
#endif
        clearLayer(static_cast<layer_e>(ii));
    }

    hardwareReset();

#if false
    if (!max7456Config->csTag || !spiSetBusInstance(dev, max7456Config->spiDevice)) {
        return MAX7456_INIT_NOT_CONFIGURED;
    }

    dev->busType_u.spi.csnPin = IOGetByTag(max7456Config->csTag);

    if (!IOIsFreeOrPreinit(dev->busType_u.spi.csnPin)) {
        return MAX7456_INIT_NOT_CONFIGURED;
    }

    IOInit(dev->busType_u.spi.csnPin, OWNER_OSD_CS, 0);
    IOConfigGPIO(dev->busType_u.spi.csnPin, SPI_IO_CS_CFG);
    IOHi(dev->busType_u.spi.csnPin);

#endif
    // Detect MAX7456 existence and device type. Do this at half the speed for safety.

    // Detect MAX7456 and compatible device by reading OSDM (OSD Insertion MUX) register.
    // This register is not modified in this driver, therefore ensured to remain at its default value (0x1B).

    _bus.set_clock_divisor(_bus.calculate_clock_divider(INITIAL_SPI_CLOCK_FREQUENCY_HZ));
    concludeCurrentSPI_Transaction();

    //uint8_t osdm = spiReadRegMsk(dev, MAX7456ADD_OSDM);
    const uint8_t osdm = _bus.read_register(MAX7456ADD_OSDM);

    if (osdm != 0x1B) { // cppcheck-suppress knownConditionTrueFalse
        //IOConfigGPIO(dev->busType_u.spi.csnPin, IOCFG_IPU);
        return INIT_NOT_FOUND;
    }

    // At this point, we can claim the ownership of the CS pin
    _deviceDetected = true;
    //IOInit(dev->busType_u.spi.csnPin, OWNER_OSD_CS, 0);

    // Detect device type by writing and reading CA[8] bit at CMAL[6].
    // This is a bit for accessing second half of character glyph storage, supported only by AT variant.
    _bus.write_register(MAX7456ADD_CMAL, (1 << 6)); // CA[8] bit
    if (_bus.read_register(MAX7456ADD_CMAL) & (1 << 6)) {
        _max7456DeviceType = MAX7456_DEVICE_TYPE_AT;
    } else {
        _max7456DeviceType = MAX7456_DEVICE_TYPE_MAX;
    }

#if defined(USE_OVERCLOCK)
    // Determine SPI clock divisor based on config and the device type.

    switch (max7456Config->clockConfig) {
    case CLOCK_CONFIG_HALF:
        _spiClockDiv = _bus.calculateClockDivider(MAX_SPI_CLOCK_FREQUENCY_HZ / 2);
        break;

    case CLOCK_CONFIG_NOMINAL:
    default:
        _spiClockDiv = _bus.calculateClockDivider(MAX_SPI_CLOCK_FREQUENCY_HZ);
        break;

    case CLOCK_CONFIG_DOUBLE:
        _spiClockDiv = _bus.calculateClockDivider(MAX_SPI_CLOCK_FREQUENCY_HZ * 2);
        break;
    }

    debug.set(DEBUG_MAX7456_SPI_CLOCK, DEBUG_MAX7456_SPI_CLOCK_OVERCLOCK, cpuOverclock);
    debug.set(DEBUG_MAX7456_SPI_CLOCK, DEBUG_MAX7456_SPI_CLOCK_DEVTYPE, _max7456DeviceType);
    debug.set(DEBUG_MAX7456_SPI_CLOCK, DEBUG_MAX7456_SPI_CLOCK_DIVISOR, _spiClockDiv);
    debug.set(DEBUG_MAX7456_SPI_CLOCK, DEBUG_MAX7456_SPI_CLOCK_X100, _bus.calculateClock(_spiClockDiv) / 10000);
#else
    (void)max7456Config;
    (void)cpuOverclock;
    //max7456SpiClockDiv = spiCalculateDivider(MAX_SPI_CLOCK_FREQUENCY_HZ);
#endif

    _bus.set_clock_divisor(_spiClockDiv);

    // force soft reset on Max7456
    _bus.write_register(MAX7456ADD_VM0, MAX7456_RESET);

    // Wait for 200us before polling for completion of reset
    //delayMs(200);

    // Wait for reset to complete
    while ((_bus.read_register(MAX7456ADD_VM0) & MAX7456_RESET) != 0)
        {}

    // Setup values to write to registers
    _videoSignalCfg = pVcdProfile->video_system;
    _hosRegValue = static_cast<uint8_t>(32 - pVcdProfile->h_offset);
    _vosRegValue = static_cast<uint8_t>(16 - pVcdProfile->v_offset);

    // Real init will be made later when driver detect idle.
    return INIT_OK;
}

/*!
Sets inversion of black and white pixels.
*/
void DisplayPortMax7456::invert(bool invert)
{
    if (invert) {
        _displayMemoryModeReg |= INVERT_PIXEL_COLOR;
    } else {
        _displayMemoryModeReg &= static_cast<uint8_t>(~INVERT_PIXEL_COLOR);
    }
    if (_displayMemoryModeReg != _previousInvertRegister) {
        // clear the shadow buffer so all characters will be
        // redrawn with the proper invert state
        clearShadowBuffer();
        _previousInvertRegister = _displayMemoryModeReg;
        //spiWriteReg(dev, MAX7456ADD_DMM, displayMemoryModeReg);
        _bus.write_register(MAX7456ADD_DMM, _displayMemoryModeReg);
    }
}

/**
 * Sets the brightness of black and white pixels.
 *
 * @param black Black brightness (0-3, 0 is darkest)
 * @param white White brightness (0-3, 0 is darkest)
 */
void DisplayPortMax7456::brightness(uint8_t black, uint8_t white)
{
    const uint8_t reg = (black << 2) | (3 - white);

    if (reg != _previousBlackWhiteRegister) {
        _previousBlackWhiteRegister = reg;
        for (uint8_t i = MAX7456ADD_RB0, j = 0; i <= MAX7456ADD_RB15; i++) {
            _buf[j++] = i;
            _buf[j++] = reg;
        }
        //spiReadWriteBuf(dev, buf, NULL, sizeof(buf));
        _bus.write_bytes(&_buf[0], sizeof(_buf));
    }
}

uint32_t DisplayPortMax7456::clearScreen(display_clear_option_e options)
{
    (void)options;
    clearLayer(_activeLayer);
    return 0;
}

uint32_t DisplayPortMax7456::writeChar(uint8_t x, uint8_t y, uint8_t c, uint8_t attr)
{
    (void)attr;
    uint8_t *buf = getActiveLayerBuffer();
    if (x < CHARACTERS_PER_LINE && y < VIDEO_LINES_PAL) {
        buf[y * CHARACTERS_PER_LINE + x] = c;
    }
    return 0;
}

uint32_t DisplayPortMax7456::writeString(uint8_t x, uint8_t y, const char *text, uint8_t attr)
{
    (void)attr;
    if (y < VIDEO_LINES_PAL) {
        uint8_t *buffer = getActiveLayerBuffer();
        const uint32_t bufferYOffset = y * CHARACTERS_PER_LINE;
        for (uint32_t i = 0, bufferXOffset = x; text[i] && bufferXOffset < CHARACTERS_PER_LINE; i++, bufferXOffset++) {
            buffer[bufferYOffset + bufferXOffset] = static_cast<uint8_t>(text[i]);
        }
    }
    return 0;
}

bool DisplayPortMax7456::layerCopy(layer_e destLayer, layer_e sourceLayer)
{
    if ((sourceLayer != destLayer) && layerSupported(destLayer)) {
        memcpy(getLayerBuffer(destLayer), getLayerBuffer(sourceLayer), VIDEO_BUFFER_PAL_CHARACTER_COUNT);
        return true;
    }
    return false;
}

bool DisplayPortMax7456::dmaInProgress()
{
    return ActiveDMA;
}

bool DisplayPortMax7456::buffersSynced() const
{
#if (__cplusplus >= 202002L)
    const auto foregroundSpan = std::span(_displayLayers[LAYER_FOREGROUND].buffer).first(_maxScreenSize);
    const auto shadowSpan = std::span(_shadowBuffer).first(_maxScreenSize);
    return std::ranges::equal(foregroundSpan, shadowSpan);
#else
    for (size_t ii = 0; ii < _maxScreenSize; ++ii) {
        if (_displayLayers[LAYER_FOREGROUND].buffer[ii] != _shadowBuffer[ii]) {
            return false;
        }
    }
    return true;
#endif
}

void DisplayPortMax7456::concludeCurrentSPI_Transaction()
{
    // Write 0xff to conclude any current SPI transaction the MAX7456 is expecting
    std::array<uint8_t, 1> data = { 0xFF };
    _bus.write_bytes(&data[0], 1);
}

bool DisplayPortMax7456::reInitIfRequired(bool forceStallCheck, Debug& debug)
{
    const time_ms32_t timeNowMs = time_ms();

    bool stalled = false;
    if (forceStallCheck || (_lastStallCheckMs + STALL_CHECK_INTERVAL_MS < timeNowMs)) { // cppcheck-suppress knownConditionTrueFalse
        _lastStallCheckMs = timeNowMs;
        concludeCurrentSPI_Transaction();
        stalled = (_bus.read_register(MAX7456ADD_VM0) != _videoSignalReg);
    }
    if (stalled) {
        reInit();
    } else if ((_videoSignalCfg == VIDEO_SYSTEM_AUTO) && ((timeNowMs - _lastSigCheckMs) > SIGNAL_CHECK_INTERVAL_MS)) { // cppcheck-suppress knownConditionTrueFalse
        concludeCurrentSPI_Transaction();
        // Adjust output format based on the current input format.
        const uint8_t videoSense = _bus.read_register(MAX7456ADD_STAT);
        debug.set(DEBUG_MAX7456_SIGNAL, DEBUG_MAX7456_SIGNAL_MODEREG, static_cast<int16_t>(_videoSignalReg & VIDEO_MODE_MASK));
        debug.set(DEBUG_MAX7456_SIGNAL, DEBUG_MAX7456_SIGNAL_SENSE, static_cast<int16_t>(videoSense & 0x7));
        debug.set(DEBUG_MAX7456_SIGNAL, DEBUG_MAX7456_SIGNAL_ROWS, getRowCount());
        if (videoSense & STAT_LOS) {
            _videoDetectTimeMs = 0;
        } else {
            if ((VIN_IS_PAL(videoSense) && VIDEO_MODE_IS_NTSC(_videoSignalReg))
              || (VIN_IS_NTSC_ALT(videoSense) && VIDEO_MODE_IS_PAL(_videoSignalReg))) {
                if (_videoDetectTimeMs) {
                    if (time_ms() - _videoDetectTimeMs > VIDEO_SIGNAL_DEBOUNCE_MS) {
                        reInit();
                        ++_reInitCount;
                        debug.set(DEBUG_MAX7456_SIGNAL, DEBUG_MAX7456_SIGNAL_REINIT, _reInitCount);
                    }
                } else {
                    // Wait for signal to stabilize
                    _videoDetectTimeMs = time_ms();
                }
            }
        }
        _lastSigCheckMs = timeNowMs;
    }
    return stalled;
}

/*!
Called in ISR context
*/
uint8_t DisplayPortMax7456::callbackReady(uint32_t arg)
{
    (void)arg;
    DisplayPortMax7456::ActiveDMA = false;
    return BusSpi::BUS_READY;
}

// Return true if screen still being transferred
bool DisplayPortMax7456::drawScreen() // NOLINT(readability-function-cognitive-complexity)
{
    if (!_fontIsLoading) {
        uint8_t *buffer = getActiveLayerBuffer();
        int spiBufIndex = 0;
        bool setAddress = true;
        bool autoInc = false;
        int posLimit = _pos + (_maxScreenSize / 2);
#if defined(USE_DMA)
        const bool useDma = spiUseSDO_DMA(dev);
#else
        const bool useDma = false;
#endif
        int maxSpiBufStartIndex = useDma ? MAX_BYTES2SEND : MAX_BYTES2SEND_POLLED;
        //timeDelta_t maxEncodeTime;
        const int32_t maxEncodeTime = useDma ? MAX_ENCODE_US : MAX_ENCODE_US_POLLED;

        // Abort for now if the bus is still busy
        if (_bus.dma_is_busy()) {
            // Not finished yet
            return true;
        }

        time_us32_t startTime = time_us();

        // Allow for an ESCAPE, a reset of DMM and a two byte MAX7456ADD_DMM command at end of buffer
        maxSpiBufStartIndex -= 4;

        // Initialise the transfer buffer
        while ((spiBufIndex < maxSpiBufStartIndex) && (_pos < posLimit) && (time_difference_us(time_us(), startTime) < maxEncodeTime)) { // cppcheck-suppress knownConditionTrueFalse
            if (buffer[_pos] != _shadowBuffer[_pos]) {
                if (buffer[_pos] == 0xff) {
                    buffer[_pos] = ' ';
                }
                if (setAddress || !autoInc) {
                    if (buffer[_pos + 1] != _shadowBuffer[_pos + 1]) {
                        // It's worth auto incrementing
                        _spiBuf[spiBufIndex++] = MAX7456ADD_DMM;
                        _spiBuf[spiBufIndex++] = _displayMemoryModeReg | DMM_AUTO_INC;
                        autoInc = true;
                    } else {
                        // It's not worth auto incrementing
                        _spiBuf[spiBufIndex++] = MAX7456ADD_DMM;
                        _spiBuf[spiBufIndex++] = _displayMemoryModeReg;
                        autoInc = false;
                    }
                    _spiBuf[spiBufIndex++] = MAX7456ADD_DMAH;
                    _spiBuf[spiBufIndex++] = static_cast<uint8_t>(_pos >> 8U);
                    _spiBuf[spiBufIndex++] = MAX7456ADD_DMAL;
                    _spiBuf[spiBufIndex++] = _pos & 0xFFU;
                    setAddress = false;
                }
                _spiBuf[spiBufIndex++] = MAX7456ADD_DMDI;
                _spiBuf[spiBufIndex++] = buffer[_pos];
                _shadowBuffer[_pos] = buffer[_pos];
            } else {
                if (!setAddress) {
                    setAddress = true;
                    if (autoInc) {
                        _spiBuf[spiBufIndex++] = MAX7456ADD_DMDI;
                        _spiBuf[spiBufIndex++] = END_STRING;
                    }
                }
            }
            if (++_pos >= _maxScreenSize) {
                _pos = 0;
                break;
            }
        }
        if (autoInc) {
            if (!setAddress) {
                _spiBuf[spiBufIndex++] = MAX7456ADD_DMDI;
                _spiBuf[spiBufIndex++] = END_STRING;
            }
            _spiBuf[spiBufIndex++] = MAX7456ADD_DMM;
            _spiBuf[spiBufIndex++] = _displayMemoryModeReg;
        }
        if (spiBufIndex) {
            _segments[0].u.buffers.tx_data = &_spiBuf[0];
            _segments[0].len = spiBufIndex;
            ActiveDMA = true;
            _bus.dma_sequence(&_segments[0]); // Non-blocking, so transfer stay in progress if using DMA
        }
    }
    return (_pos != 0);
}

// should not be used when armed
void DisplayPortMax7456::refreshAll(Debug& debug)
{
    reInitIfRequired(true, debug);
    while (drawScreen()) {}
}

bool DisplayPortMax7456::writeNvm(uint8_t char_address, const uint8_t *font_data)
{
    if (!_deviceDetected) {
        return false;
    }

    // Block pending completion of any prior SPI access
    _bus.dma_wait();

    // disable display
    _fontIsLoading = true;
    _bus.write_register(MAX7456ADD_VM0, 0);

    _bus.write_register(MAX7456ADD_CMAH, char_address); // set start address high

    for (uint8_t x = 0; x < 54; ++x) {
        _bus.write_register(MAX7456ADD_CMAL, x); //set start address low
        _bus.write_register(MAX7456ADD_CMDI, font_data[x]);
#if defined(LED0_TOGGLE)
        LED0_TOGGLE;
#else
        //LED1_TOGGLE;
#endif
    }

    // Transfer 54 bytes from shadow ram to NVM

    _bus.write_register(MAX7456ADD_CMM, WRITE_NVR);

    // Wait until bit 5 in the status register returns to 0 (12ms)

    while ((_bus.read_register(MAX7456ADD_STAT) & STAT_NVR_BUSY) != 0)
        {}
    return true;
}

#if defined(MAX7456_NRST_PIN)
static IO_t max7456ResetPin        = IO_NONE;
#endif

void DisplayPortMax7456::hardwareReset()
{
#if defined(MAX7456_NRST_PIN)
#define IO_RESET_CFG      IO_CONFIG(GPIO_Mode_OUT, GPIO_Speed_2MHz, GPIO_OType_PP, GPIO_PuPd_DOWN)

    max7456ResetPin = IOGetByTag(IO_TAG(MAX7456_NRST_PIN));
    IOInit(max7456ResetPin, OWNER_OSD, 0);
    IOConfigGPIO(max7456ResetPin, IO_RESET_CFG);

    // RESET 50ms long pulse, followed by 100us pause
    IOLo(max7456ResetPin);
    delay(50);
    IOHi(max7456ResetPin);
    delayMicroseconds(100);
#else
    // Allow device 50ms to powerup
    //!!delay(50);
#endif
}

bool DisplayPortMax7456::isDeviceDetected() // NOLINT(readability-make-member-function-const)
{
    return _deviceDetected;
}

void DisplayPortMax7456::setBackgroundType(background_e backgroundType)
{
    _backgroundType = backgroundType;

    setRegisterVM1();
}
// NOLINTEND(cppcoreguidelines-pro-bounds-constant-array-index,cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise,modernize-macro-to-enum,cppcoreguidelines-macro-usage)

#endif // FRAMEWORK_TESt
