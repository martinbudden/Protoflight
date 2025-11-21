#pragma once

#include "DisplayPortBase.h"
#if defined(FRAMEWORK_TEST)
class BUS_SPI {
public:
    uint8_t readRegister(uint8_t reg) const { (void)reg; return 0; }
    uint8_t writeRegister(uint8_t reg, uint8_t data) { (void)reg; (void)data; return 0; }
    uint8_t writeBytes(const uint8_t* data, size_t length) { (void)data; (void)length; return 0; }
};
#else
#include <BUS_SPI.h>
#endif

#include <TimeMicroseconds.h>
#include <array>

class Debug;

enum video_system_e {
    VIDEO_SYSTEM_AUTO = 0,
    VIDEO_SYSTEM_PAL,
    VIDEO_SYSTEM_NTSC,
    VIDEO_SYSTEM_HD
};

class DisplayPortMax7456 : public DisplayPortBase {
public:
#if defined(FRAMEWORK_TEST)
    DisplayPortMax7456() = default;
#else
    DisplayPortMax7456(BUS_BASE::bus_index_e SPI_index, const BUS_SPI::stm32_spi_pins_t& pins, Debug& debug);
    DisplayPortMax7456(BUS_BASE::bus_index_e SPI_index, const BUS_SPI::spi_pins_t& pins, Debug& debug);
#endif
    enum init_status_e {
        INIT_OK = 0, // IO defined and MAX7456 was detected
        INIT_NOT_FOUND = -1, // IO defined, but MAX7456 could not be detected (maybe not yet powered on)
        INIT_NOT_CONFIGURED = -2, // No MAX7456 IO defined, which means either the we don't have it or it's not properly configured
    };
    enum {
        CLOCK_CONFIG_HALF,      // Force half clock
        CLOCK_CONFIG_NOMINAL,   // Nominal clock (default)
        CLOCK_CONFIG_DOUBLE     // Double clock
    };
    enum {
        SIGNAL_CHECK_INTERVAL_MS = 1000,
        STALL_CHECK_INTERVAL_MS = 1000
    };
    struct config_t {
        uint8_t clockConfig; // SPI clock modifier
        //ioTag_t csTag;
        uint8_t spiDevice;
        bool preInitOPU;
    };
    struct vcd_profile_t {
        uint8_t video_system;
        int8_t h_offset;
        int8_t v_offset;
    };
    struct extDevice_t;
    enum bus_status_e {
        BUS_READY,
        BUS_BUSY,
        BUS_ABORT
    };
    struct bus_segment_t {
        union {
            struct {
                uint8_t* txData; // Transmit buffer
                uint8_t* rxData; // Receive buffer
            } buffers;
            struct {
                const extDevice_t* dev; // Link to the device associated with the next transfer
                volatile bus_segment_t* segments; // Segments to process in the next transfer.
            } link;
        } u;
        int len;
        bool negateCS; // Should CS be negated at the end of this segment
        bus_status_e (*callbackFn)(uint32_t arg);
    };
private:
    // VM0 bits
    
    static constexpr uint8_t VIDEO_BUFFER_DISABLE       = 0x01;
    static constexpr uint8_t MAX7456_RESET              = 0x02;
    static constexpr uint8_t VERTICAL_SYNC_NEXT_VSYNC   = 0x04;
    static constexpr uint8_t OSD_ENABLE                 = 0x08;
    enum { CHARS_PER_LINE = 30 };
    enum { VIDEO_BUFFER_NTSC_CHARACTER_COUNT = 390 };
    enum { VIDEO_BUFFER_PAL_CHARACTER_COUNT = 480 };
    enum { MAX7456_SUPPORTED_LAYER_COUNT = LAYER_COUNT };

    struct layer_t {
        std::array<uint8_t, VIDEO_BUFFER_PAL_CHARACTER_COUNT> buffer;
    };
public:
    virtual uint32_t clearScreen(display_clear_option_e options) override;
    virtual bool drawScreen() override;
    virtual uint32_t writeString(uint8_t x, uint8_t y, uint8_t attr, const char *text) override;
    virtual uint32_t writeChar(uint8_t x, uint8_t y, uint8_t attr, uint8_t c) override;
    virtual bool layerSupported(layer_e layer) override;
    virtual bool layerSelect(layer_e layer) override;
    virtual bool layerCopy(layer_e destLayer, layer_e sourceLayer) override;
    virtual void setBackgroundType(background_e backgroundType) override;

    void hardwareReset();
    //void    Preinit(const struct Config_s *Config);
    init_status_e init(const config_t* config, const vcd_profile_t* vcdProfile, bool cpuOverclock);
    void preinit(const config_t& config);
    void invert(bool invert);
    void brightness(uint8_t black, uint8_t white);
    bool reInitIfRequired(bool forceStallCheck);
    bool writeNvm(uint8_t char_address, const uint8_t *font_data);
    uint8_t getRowsCount();
    void refreshAll();
    static bool dmaInProgress();
    bool buffersSynced() const;
    bool isDeviceDetected();
    void concludeCurrentSPI_Transaction();
    static bus_status_e callbackReady(uint32_t arg);
    inline uint32_t compareTimeUs(timeUs32_t a, timeUs32_t b) { return static_cast<uint32_t>(a - b); }

private:
    uint8_t* getLayerBuffer(layer_e layer);
    uint8_t* getActiveLayerBuffer();
    void setRegisterVM1();
    void clearShadowBuffer();
    void clearLayer(layer_e layer);
public:
    void reInit();
private:
    BUS_SPI _bus; //!< SPI bus interface
    Debug& _debug;
    timeMs32_t lastSigCheckMs {0};
    timeMs32_t videoDetectTimeMs {0};
    timeMs32_t lastStallCheckMs {STALL_CHECK_INTERVAL_MS / 2}; // offset so that it doesn't coincide with the signal check
    std::array<bus_segment_t, 2> segments {
        bus_segment_t {.u = {.link = {.dev = nullptr, .segments = nullptr}}, .len = 0, .negateCS = true, .callbackFn = callbackReady},
        bus_segment_t {.u = {.link = {.dev = nullptr, .segments = nullptr}}, .len = 0, .negateCS = true, .callbackFn = nullptr},
    };
    uint16_t reInitCount {0};
    uint16_t pos {0};
    uint8_t videoSignalCfg {};
    uint8_t videoSignalReg {OSD_ENABLE}; // OSD_ENABLE required to trigger first ReInit
    uint8_t displayMemoryModeReg {0};

    uint8_t hosRegValue {}; // HOS (Horizontal offset register) value
    uint8_t vosRegValue {}; // VOS (Vertical offset register) value

    bool fontIsLoading {false};

    uint8_t max7456DeviceType {};
    background_e deviceBackgroundType {BACKGROUND_TRANSPARENT};

    // previous states initialized outside the valid range to force update on first call
    enum { INVALID_PREVIOUS_REGISTER_STATE = 255 };
    uint8_t previousBlackWhiteRegister {INVALID_PREVIOUS_REGISTER_STATE};
    uint8_t previousInvertRegister {INVALID_PREVIOUS_REGISTER_STATE};
    layer_e activeLayer {LAYER_FOREGROUND};

    //extDevice_t max7456Device;
    //extDevice_t *dev = &max7456Device;

    bool max7456DeviceDetected {false};
    uint16_t max7456SpiClockDiv {};
    static volatile bool ActiveDMA;

    uint16_t maxScreenSize {VIDEO_BUFFER_PAL_CHARACTER_COUNT};

    // We write everything to the active layer and then compare
    // it to shadowBuffer and update only changed characters.
    // This is faster than redrawing entire screen.
    std::array<uint8_t, VIDEO_BUFFER_PAL_CHARACTER_COUNT> shadowBuffer {};
    std::array<layer_t, MAX7456_SUPPORTED_LAYER_COUNT> displayLayers {};
    std::array<uint8_t, 32> buf {};
    enum { MAX_BYTES_TO_SEND = 250 };
    /*DMA_DATA*/ uint8_t spiBuf[MAX_BYTES_TO_SEND];
};
