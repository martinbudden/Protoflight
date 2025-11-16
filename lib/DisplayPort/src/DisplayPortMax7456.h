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
#include <array>


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
    DisplayPortMax7456(BUS_BASE::bus_index_e SPI_index, const BUS_SPI::stm32_spi_pins_t& pins);
    DisplayPortMax7456(BUS_BASE::bus_index_e SPI_index, const BUS_SPI::spi_pins_t& pins);
#endif
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
    //InitStatus_e Init(const struct Config_s *Config, const struct vcdProfile_s *vcdProfile, bool cpuOverclock);
    void invert(bool invert);
    void brightness(uint8_t black, uint8_t white);
    bool reInitIfRequired(bool forceStallCheck);
    bool writeNvm(uint8_t char_address, const uint8_t *font_data);
    uint8_t getRowsCount();
    void refreshAll();
    bool dmaInProgress() const;
    bool buffersSynced() const;
    bool isDeviceDetected();
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
    volatile bool max7456ActiveDma {false};

    uint16_t maxScreenSize {VIDEO_BUFFER_PAL_CHARACTER_COUNT};

    // We write everything to the active layer and then compare
    // it to shadowBuffer and update only changed characters.
    // This is faster than redrawing entire screen.
    std::array<uint8_t, VIDEO_BUFFER_PAL_CHARACTER_COUNT> shadowBuffer {};
    std::array<layer_t, MAX7456_SUPPORTED_LAYER_COUNT> displayLayers {};
    std::array<uint8_t, 32> buf {};
};
