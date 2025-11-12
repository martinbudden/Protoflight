#include "OSD.h"
#include "Cockpit.h"

#include <MSP_Box.h>

// NOLINTBEGIN(cppcoreguidelines-macro-usage,cppcoreguidelines-pro-bounds-constant-array-index,hicpp-signed-bitwise)

OSD::OSD(const FlightController& flightController, const Cockpit& cockpit, Debug& debug) : // cppcheck-suppress constParameterReference
    _elements(*this, flightController, cockpit, debug),
    _cockpit(cockpit)
{
}

void OSD::init(DisplayPortBase *displayPort, DisplayPortBase::device_type_e displayPortDeviceType)
{
    _displayPort = displayPort;
    _displayPortDeviceType = displayPortDeviceType;

    const uint8_t rowCount = displayPort->getRowCount();
    const uint8_t columnCount = displayPort->getColumnCount();
    if (columnCount !=0  && rowCount != 0) {
        _config.canvas_column_count = columnCount;
        _config.canvas_row_count = rowCount;
    }
    _elements.init(false, rowCount, columnCount);
}

void OSD::completeInitialization()
{
}

void OSD::setConfig(const config_t& config)
{
    _config = config;
}

void OSD::setStatsState(uint8_t statIndex, bool enabled)
{
    if (enabled) {
        _config.enabled_stats |= (1U << statIndex);
    } else {
        _config.enabled_stats &= ~(1U << statIndex);
    }
}

bool OSD::getStatsState(uint8_t statIndex) const
{
    return _config.enabled_stats & (1U << statIndex);
}

void OSD::resetStats()
{
    _stats.max_current     = 0;
    _stats.max_speed       = 0;
    _stats.min_voltage     = 5000;
    _stats.end_voltage     = 0;
    _stats.min_rssi        = 99; // percent
    _stats.max_altitude    = 0;
    _stats.max_distance    = 0;
    _stats.armed_time      = 0;
    _stats.max_g_force     = 0;
    _stats.max_esc_temp_index = 0;
    _stats.max_esc_temp    = 0;
    _stats.max_esc_rpm     = 0;

    _stats.min_link_quality = 99; //(linkQualitySource == LQ_SOURCE_NONE) ? 99 : 100; // percent
    enum { CRSF_RSSI_MIN =-130, CRSF_RSSI_MAX = 0, CRSF_SNR_MIN = -30, CRSF_SNR_MAX = 20 };
    _stats.min_rssi_dbm = CRSF_RSSI_MAX;
    _stats.min_rsnr = CRSF_SNR_MAX;
}

bool OSD::refreshStats()
{
    return true;
}

bool OSD::processStats1(timeUs32_t currentTimeUs)
{
    (void)currentTimeUs;
    return true;
}

bool OSD::processStats2(timeUs32_t currentTimeUs)
{
    (void)currentTimeUs;
    return true;
}

void OSD::processStats3()
{
}

void OSD::updateAlarms()
{
}

void OSD::syncBlink(timeUs32_t currentTimeUs)
{
    (void)currentTimeUs;
}


void OSD::setWarningState(uint8_t warningIndex, bool enabled)
{
    if (enabled) {
        _config.enabled_warnings |= (1U << warningIndex);
    } else {
        _config.enabled_warnings &= ~(1U << warningIndex);
    }
}

bool OSD::getWarningState(uint8_t warningIndex) const
{
    return _config.enabled_warnings & (1U << warningIndex);
}

void OSD::drawLogo(uint8_t x, uint8_t y, DisplayPortBase::severity_e severity)
{
    enum { SYMBOL_END_OF_FONT = 0xFF };

    // display logo and help
    uint8_t fontOffset = 160;
    for (uint8_t row = 0; row < LOGO_ROW_COUNT; ++row) {
        for (uint8_t column = 0; column < LOGO_COLUMN_COUNT; ++column) {
            if (fontOffset < SYMBOL_END_OF_FONT) {
                _displayPort->writeChar(x + column, y + row, severity, fontOffset);
                ++fontOffset;
            }
        }
    }
}

void OSD::updateDisplay(uint32_t timeMicroseconds, uint32_t timeMicrosecondsDelta) // NOLINT(readability-function-cognitive-complexity)
{
    (void)timeMicrosecondsDelta;

    switch (_state) {
    case STATE_INIT:
        if (!_displayPort->checkReady(false)) {
            // Frsky OSD needs a display redraw after search for MAX7456 devices
            if (_displayPortDeviceType == DisplayPortBase::DEVICE_TYPE_FRSKY_OSD) {
                _displayPort->redraw();
                return;
            }
        }
        completeInitialization();
        _displayPort->redraw();
        _state = STATE_COMMIT;
        break;
    case STATE_CHECK:
        // don't touch buffers if DMA transaction is in progress
        if (_displayPort->isTransferInProgress()) {
            break;
        }
        _state = STATE_UPDATE_HEARTBEAT;
        break;
    case STATE_UPDATE_HEARTBEAT:
        if (_displayPort->heartbeat()) {
            // Extraordinary action was taken, so return without allowing stateDurationFractionUs table to be updated
            return;
        }
        _state = STATE_PROCESS_STATS1;
        break;
    case STATE_PROCESS_STATS1:
        _state = processStats1(timeMicroseconds) ? STATE_REFRESH_STATS : STATE_PROCESS_STATS2; // cppcheck-suppress knownConditionTrueFalse
        break;
    case STATE_REFRESH_STATS:
        if (refreshStats()) {
            _state = STATE_PROCESS_STATS2;
        }
        break;
    case STATE_PROCESS_STATS2:
        processStats2(timeMicroseconds);
        _state = STATE_PROCESS_STATS3;
        break;
    case STATE_PROCESS_STATS3:
        processStats3();
        _state = STATE_COMMIT;
        break;
    case STATE_UPDATE_ALARMS:
        updateAlarms();
        _state = _resumeRefreshAt ? STATE_TRANSFER : STATE_UPDATE_CANVAS;
        break;
    case STATE_UPDATE_CANVAS:
        // Hide OSD when OSD SW mode is active
        if (_cockpit.isRcModeActive(MSP_Box::BOX_OSD)) {
            _displayPort->clearScreen(DISPLAY_CLEAR_NONE);
            _state = STATE_COMMIT;
            break;
        }
        if (_backgroundLayerSupported) {
            // Background layer is supported, overlay it onto the foreground
            // so that we only need to draw the active parts of the elements.
            _displayPort->layerCopy(DisplayPortBase::LAYER_FOREGROUND, DisplayPortBase::LAYER_BACKGROUND);
        } else {
            // Background layer not supported, just clear the foreground in preparation
            // for drawing the elements including their backgrounds.
            _displayPort->clearScreen(DISPLAY_CLEAR_NONE);
        }
        syncBlink(timeMicroseconds);
        _state = STATE_DRAW_ELEMENT;
        break;
    case STATE_DRAW_ELEMENT: {
        const uint8_t activeElementIndex = _elements.getActiveElementIndex();
        enum { OSD_EXEC_TIME_SHIFT = 5 };

        timeUs32_t startElementTime = timeUs();
        _moreElementsToDraw = _elements.drawNextActiveElement(*_displayPort);
        timeUs32_t executeTimeUs = timeUs() - startElementTime;

        if (executeTimeUs > (_elementDurationFractionUs[activeElementIndex] >> OSD_EXEC_TIME_SHIFT)) { // cppcheck-suppress unsignedLessThanZero
            _elementDurationFractionUs[activeElementIndex] = executeTimeUs << OSD_EXEC_TIME_SHIFT;
        } else if (_elementDurationFractionUs[activeElementIndex] > 0) {
            // Slowly decay the max time
            --_elementDurationFractionUs[activeElementIndex];
        }
        if (_elements.isRenderPending()) { // Render the element just drawn
            _state = STATE_DISPLAY_ELEMENT;
            break;
        }
        if (_moreElementsToDraw) {
            break;
        }
        _state = (_cockpit.isArmed() && _config.osd_show_spec_prearm) ? STATE_REFRESH_PREARM : STATE_COMMIT;
        break;
    }
    case STATE_DISPLAY_ELEMENT:
        if (!_elements.displayActiveElement(*_displayPort)) {
            if (_moreElementsToDraw) {
                // There is no more to draw so advance to the next element
                _state = STATE_DRAW_ELEMENT;
            } else {
                if (_cockpit.isArmed() && _config.osd_show_spec_prearm) {
                    _state = STATE_REFRESH_PREARM;
                } else {
                    _state = STATE_COMMIT;
                }
            }
        }
        break;
    case STATE_REFRESH_PREARM:
        if (_elements.drawSpec(*_displayPort)) {
            // Rendering is complete
            _state = STATE_COMMIT;
        }
        break;
    case STATE_COMMIT:
        _displayPort->commitTransaction();
        _state = _resumeRefreshAt ? STATE_IDLE : STATE_TRANSFER;
        break;
    case STATE_TRANSFER:
        // Wait for any current transfer to complete
        if (_displayPort->isTransferInProgress()) {
            break;
        }
        // Transfer may be broken into many parts
        if (_displayPort->drawScreen()) {
            break;
        }
        _state = STATE_IDLE;
        break;
    case STATE_IDLE:
        _state = STATE_CHECK;
        break;
    default:
        _state = STATE_IDLE;
        break;
    }
}
// NOLINTEND(cppcoreguidelines-macro-usage,cppcoreguidelines-pro-bounds-constant-array-index,hicpp-signed-bitwise)
