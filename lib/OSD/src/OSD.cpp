#include "OSD.h"
#include "Cockpit.h"

// NOLINTBEGIN(cppcoreguidelines-macro-usage,cppcoreguidelines-pro-bounds-constant-array-index,hicpp-signed-bitwise)

enum { OSD_PROFILE_BITS_POS = 11 };
#define OSD_PROFILE_MASK    (((1 << OSD_PROFILE_COUNT) - 1) << OSD_PROFILE_BITS_POS)


OSD::OSD(const FlightController& flightController, const Cockpit& cockpit, Debug& debug) : //cppcheck-suppress constParameterReference
    _elements(*this, flightController),
    _cockpit(cockpit),
    _debug(debug)
{
}

void OSD::init(DisplayPortBase *displayPort, DisplayPortBase::device_type_e displayPortDeviceType)
{
    _elements.init(false);

    _displayPort = displayPort;
    _displayPortDeviceType = displayPortDeviceType;

    const uint8_t rowCount = displayPort->getRowCount();
    const uint8_t columnCount = displayPort->getColumnCount();
    if (columnCount !=0  && rowCount != 0) {
        _config.canvas_column_count = columnCount;
        _config.canvas_row_count = rowCount;

        // Ensure that all OSD elements are on the canvas once the number of row and columns is known
        for (size_t ii = 0; ii < OSD_ITEM_COUNT; ++ii) { // NOLINT(modernize-loop-convert)
            const uint16_t itemPos = _elements.getConfig().item_pos[ii];
            const uint16_t elemProfileType = itemPos & (OSD_PROFILE_MASK | OSD_Elements::OSD_TYPE_MASK);

            uint8_t posX = OSD_X(itemPos);
            uint8_t posY = OSD_Y(itemPos);
            if (posX >= columnCount) {
                posX = columnCount - 1;
                _elements.getConfig().item_pos[ii] = elemProfileType | OSD_POS(posX, posY);
            }
            if (posY >= rowCount) {
                posY = rowCount - 1;
                _elements.getConfig().item_pos[ii] = elemProfileType | OSD_POS(posX, posY);
            }
        }
    }
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

void OSD::setWarningState(uint8_t warningIndex, bool enabled)
{
    if (enabled) {
        _config.enabledWarnings |= (1U << warningIndex);
    } else {
        _config.enabledWarnings &= ~(1U << warningIndex);
    }
}

bool OSD::getWarningState(uint8_t warningIndex) const
{
    return _config.enabledWarnings & (1U << warningIndex);
}

void OSD::drawLogo(uint8_t x, uint8_t y, DisplayPortBase::severity_e severity)
{
    enum { SYMBOL_END_OF_FONT = 0xFF };

    // display logo and help
    uint8_t fontOffset = 160;
    for (uint8_t row = 0; row < OSD_LOGO_ROW_COUNT; ++row) {
        for (uint8_t column = 0; column < OSD_LOGO_COLUMN_COUNT; ++column) {
            if (fontOffset < SYMBOL_END_OF_FONT) {
                _displayPort->writeChar(x + column, y + row, severity, fontOffset);
                ++fontOffset;
            }
        }
    }
}

bool OSD::updateOSD(uint32_t timeMicroseconds, uint32_t timeMicrosecondsDelta) // NOLINT(readability-function-cognitive-complexity)
{
    (void)timeMicroseconds;
    (void)timeMicrosecondsDelta;

    switch (_state) {
    case STATE_INIT:
        if (!_displayPort->checkReady(false)) {
            // Frsky OSD needs a display redraw after search for MAX7456 devices
            if (_displayPortDeviceType == DisplayPortBase::DEVICE_TYPE_FRSKY_OSD) {
                _displayPort->redraw();
                return true;
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
            // Extraordinary action was taken, so return without allowing osdStateDurationFractionUs table to be updated
            return true;
        }
        _state = STATE_PROCESS_STATS1;
        break;
    case STATE_PROCESS_STATS1:
        [[fallthrough]];
    case STATE_REFRESH_STATS:
        [[fallthrough]];
    case STATE_PROCESS_STATS2:
        _state = STATE_PROCESS_STATS3;
        break;
    case STATE_PROCESS_STATS3:
        _state = STATE_COMMIT;
        break;
    case STATE_UPDATE_ALARMS:
        _state = STATE_UPDATE_CANVAS;
        break;
    case STATE_UPDATE_CANVAS:
        _state = STATE_DRAW_ELEMENT;
        break;
    case STATE_DRAW_ELEMENT: {
        const uint8_t osdElement = _elements.getActiveElement();
        enum { OSD_EXEC_TIME_SHIFT = 5 };

        timeUs32_t startElementTime = timeUs();
        _moreElementsToDraw = _elements.drawNextActiveElement(*_displayPort);
        timeUs32_t executeTimeUs = timeUs() - startElementTime;

        if (executeTimeUs > (_elementDurationFractionUs[osdElement] >> OSD_EXEC_TIME_SHIFT)) {
            _elementDurationFractionUs[osdElement] = executeTimeUs << OSD_EXEC_TIME_SHIFT;
        } else if (_elementDurationFractionUs[osdElement] > 0) {
            // Slowly decay the max time
            --_elementDurationFractionUs[osdElement];
        }

        if (_elements.isRenderPending()) {
            _state = STATE_DISPLAY_ELEMENT;
            // Render the element just drawn
            break;
        }
        if (_moreElementsToDraw) {
            // There are more elements to draw
            break;
        }

        if (_cockpit.isArmed() && _config.osd_show_spec_prearm) {
            _state = STATE_REFRESH_PREARM;
        } else {
            _state = STATE_COMMIT;
        }
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
        break;
    case STATE_COMMIT:
        break;
    case STATE_TRANSFER:
        _state = STATE_IDLE;
        break;
    case STATE_IDLE:
        break;
    default:
        _state = STATE_IDLE;
        break;
    }

    return true;
}
// NOLINTEND(cppcoreguidelines-macro-usage,cppcoreguidelines-pro-bounds-constant-array-index,hicpp-signed-bitwise)
