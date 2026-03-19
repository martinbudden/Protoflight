#pragma once

#include "display_port_base.h"
#if !defined(FRAMEWORK_TEST)
#include <bus_spi.h>
#include <debug.h>
#include <time_microseconds.h>
#endif

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
    DisplayPortMax7456(uint8_t SPI_index, const BusSpi::stm32_spi_pins_t& pins);
    DisplayPortMax7456(uint8_t SPI_index, const BusSpi::spi_pins_t& pins);
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
    enum { MAX7456_DEVICE_TYPE_MAX = 0, MAX7456_DEVICE_TYPE_AT = 1 };

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
private:
    // VM0 bits
    static constexpr uint8_t VIDEO_BUFFER_DISABLE       = 0x01;
    static constexpr uint8_t MAX7456_RESET              = 0x02;
    static constexpr uint8_t VERTICAL_SYNC_NEXT_VSYNC   = 0x04;
    static constexpr uint8_t OSD_ENABLE                 = 0x08;
    static constexpr uint8_t CHARACTERS_PER_LINE = 30;
    static constexpr uint32_t VIDEO_BUFFER_NTSC_CHARACTER_COUNT = 390;
    static constexpr uint32_t VIDEO_BUFFER_PAL_CHARACTER_COUNT = 480;
    static constexpr uint32_t MAX7456_SUPPORTED_LAYER_COUNT = LAYER_COUNT;
    // 10 MHz max SPI frequency
    static constexpr uint32_t MAX_SPI_CLOCK_FREQUENCY_HZ = 10'000'000;
    static constexpr uint32_t INITIAL_SPI_CLOCK_FREQUENCY_HZ = 5'000'000;

    struct layer_t {
        std::array<uint8_t, VIDEO_BUFFER_PAL_CHARACTER_COUNT> buffer;
    };
public:
    virtual uint32_t clear_screen(display_clear_option_e options) override;
    virtual bool draw_screen() override;
    virtual uint32_t write_string(uint8_t x, uint8_t y, const char *text, uint8_t attr) override;
    virtual uint32_t write_char(uint8_t x, uint8_t y, uint8_t c, uint8_t attr) override;
    virtual bool layer_copy(layer_e destLayer, layer_e sourceLayer) override;
    virtual void set_background_type(background_e background_type) override;

    void hardwareReset();
    //void    Preinit(const struct Config_s *Config);
    init_status_e init(const config_t* config, const vcd_profile_t* vcdProfile, bool cpuOverclock, Debug& debug);
    void preinit(const config_t& config);
    void invert(bool invert);
    void brightness(uint8_t black, uint8_t white);
    bool re_init_if_required(bool forceStallCheck, Debug& debug);
    bool write_nvm(uint8_t char_address, const uint8_t *font_data);
    void refresh_all(Debug& debug);
    static bool dma_in_progress();
    bool buffers_synced() const;
    bool is_device_detected();
    void conclude_current_spi_transaction();
#if !defined(FRAMEWORK_TEST)
    static uint8_t callbackReady(uint32_t arg);
#endif
private:
    uint8_t* getLayerBuffer(layer_e layer);
    uint8_t* getActiveLayerBuffer();
    void setRegisterVM1();
    void clearShadowBuffer();
    void clearLayer(layer_e layer);
public:
    void reInit();
private:
#if !defined(FRAMEWORK_TEST)
    BusSpi _bus; //!< SPI bus interface
    time_ms32_t _lastSigCheckMs {0};
    time_ms32_t _videoDetectTimeMs {0};
    time_ms32_t _lastStallCheckMs {STALL_CHECK_INTERVAL_MS / 2}; // offset so that it doesn't coincide with the signal check
    std::array<BusSpi::segment_t, 2> _segments {
        BusSpi::segment_t {.u = {.link = {.dev = nullptr, .segments = nullptr}}, .len = 0, .negate_cs = true, .callbackFn = callbackReady},
        BusSpi::segment_t {.u = {.link = {.dev = nullptr, .segments = nullptr}}, .len = 0, .negate_cs = true, .callbackFn = nullptr},
    };
#endif
    uint16_t _reInitCount {0};
    uint16_t _pos {0};
    uint8_t _max7456DeviceType {};
    uint8_t _videoSignalCfg {};
    uint8_t _videoSignalReg {OSD_ENABLE}; // OSD_ENABLE required to trigger first ReInit
    uint8_t _displayMemoryModeReg {0};

    uint8_t _hosRegValue {}; // HOS (Horizontal offset register) value
    uint8_t _vosRegValue {}; // VOS (Vertical offset register) value

    bool _fontIsLoading {false};


    // previous states initialized outside the valid range to force update on first call
    enum { INVALID_PREVIOUS_REGISTER_STATE = 255 };
    uint8_t _previousBlackWhiteRegister {INVALID_PREVIOUS_REGISTER_STATE};
    uint8_t _previousInvertRegister {INVALID_PREVIOUS_REGISTER_STATE};

    //extDevice_t max7456Device;
    //extDevice_t *dev = &max7456Device;

    bool _deviceDetected {false};
    uint16_t _spiClockDiv {};
    static volatile bool ActiveDMA;


    // We write everything to the active layer and then compare
    // it to shadowBuffer and update only changed characters.
    // This is faster than redrawing entire screen.
    std::array<uint8_t, VIDEO_BUFFER_PAL_CHARACTER_COUNT> _shadowBuffer {};
    std::array<layer_t, MAX7456_SUPPORTED_LAYER_COUNT> _displayLayers {};
    std::array<uint8_t, 32> _buf {};
    enum { MAX_BYTES_TO_SEND = 250 };
    /*DMA_DATA*/ uint8_t _spiBuf[MAX_BYTES_TO_SEND] {};
};
