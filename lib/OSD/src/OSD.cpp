#include "OSD.h"
#include "Cockpit.h"

//#include <Hardware//Serial.h>
#include <MSP_Box.h>

// NOLINTBEGIN(cppcoreguidelines-macro-usage,cppcoreguidelines-pro-bounds-constant-array-index,hicpp-signed-bitwise)

OSD::OSD(const FlightController& flightController, const Cockpit& cockpit, const AHRS_MessageQueue& ahrsMessageQueue, Debug& debug) : // cppcheck-suppress constParameterReference
    _elements(*this, flightController, cockpit, ahrsMessageQueue, debug),
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
}

void OSD::completeInitialization()
{
    uint8_t midRow = _displayPort->getRowCount() / 2;
    uint8_t midCol = _displayPort->getColumnCount() / 2;

    _isArmed = _cockpit.isArmed();

    //resetAlarms();

    _backgroundLayerSupported = _displayPort->layerSupported(DisplayPortBase::LAYER_BACKGROUND);
    _displayPort->layerSelect(DisplayPortBase::LAYER_FOREGROUND);

    _displayPort->beginTransaction(DISPLAY_TRANSACTION_OPTION_RESET_DRAWING);
    _displayPort->clearScreen(DISPLAY_CLEAR_WAIT);

    // Display betaflight logo
    drawLogo(midCol - (LOGO_COLUMN_COUNT) / 2, midRow - 5, DisplayPortBase::SEVERITY_NORMAL);

    std::array<char, 30> string_buffer;
    sprintf(&string_buffer[0], "V%s", "0.0.1");//FC_VERSION_STRING);
    _displayPort->writeString(midCol + 5, midRow, DisplayPortBase::SEVERITY_NORMAL, &string_buffer[0]);
#if defined(USE_CMS) || true
    _displayPort->writeString(midCol - 8, midRow + 2,  DisplayPortBase::SEVERITY_NORMAL, "MENU:THR MID");
    _displayPort->writeString(midCol - 4, midRow + 3, DisplayPortBase::SEVERITY_NORMAL, "+ YAW LEFT");
    _displayPort->writeString(midCol - 4, midRow + 4, DisplayPortBase::SEVERITY_NORMAL, "+ PITCH UP");
#endif

#if defined(USE_RTC_TIME)
    std::array<char, FORMATTED_DATE_TIME_BUFSIZE> dateTimeBuffer;
    if (osdFormatRtcDateTime(&dateTimeBuffer[0])) {
        _displayPort->writeString(midCol - 10, midRow + 6, DisplayPortBase::SEVERITY_NORMAL, &dateTimeBuffer[]);
    }
#endif

    _resumeRefreshAtUs = timeUs() + 4'000'000;
#if defined(USE_OSD_PROFILES)
    setOsdProfile(osdConfig()->osdProfileIndex);
#endif

    _elements.init(_backgroundLayerSupported, _displayPort->getRowCount(), _displayPort->getColumnCount());
    analyzeActiveElements();

    _displayPort->commitTransaction();

    _isReady = true;
}

void OSD::analyzeActiveElements()
{
    _elements.addActiveElements();
    _elements.drawActiveElementsBackground(*_displayPort);
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
    // the logo is in the font characters starting at 160
    enum { START_CHARACTER = 160 };
    enum { END_OF_FONT = 255 };

    // display logo and help
    uint8_t characterCode = START_CHARACTER;
    for (uint8_t row = 0; row < LOGO_ROW_COUNT; ++row) {
        for (uint8_t column = 0; column < LOGO_COLUMN_COUNT; ++column) {
            if (characterCode < END_OF_FONT) {
                _displayPort->writeChar(x + column, y + row, severity, characterCode);
                ++characterCode;
            }
        }
    }
}

void OSD::updateDisplay(uint32_t timeMicroseconds, uint32_t timeMicrosecondsDelta) // NOLINT(readability-function-cognitive-complexity)
{
    (void)timeMicrosecondsDelta;

    switch (_state) {
    case STATE_INIT:
        //Serial.printf("STATE_INIT\r\n");
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
        //Serial.printf("STATE_CHECK\r\n");
        // don't touch buffers if DMA transaction is in progress
        if (_displayPort->isTransferInProgress()) {
            break;
        }
        _state = STATE_UPDATE_HEARTBEAT;
        break;
    case STATE_UPDATE_HEARTBEAT:
        //Serial.printf("STATE_UPDATE_HEARTBEAT\r\n");
        if (_displayPort->heartbeat()) {
            // Extraordinary action was taken, so return without allowing stateDurationFractionUs table to be updated
            return;
        }
        _state = STATE_PROCESS_STATS1;
        break;
    case STATE_PROCESS_STATS1:
        //Serial.printf("STATE_PROCESS_STATS1\r\n");
        _state = processStats1(timeMicroseconds) ? STATE_REFRESH_STATS : STATE_PROCESS_STATS2; // cppcheck-suppress knownConditionTrueFalse
        break;
    case STATE_REFRESH_STATS:
        //Serial.printf("STATE_REFRESH_STATS\r\n");
        if (refreshStats()) {
            _state = STATE_PROCESS_STATS2;
        }
        break;
    case STATE_PROCESS_STATS2:
        //Serial.printf("STATE_PROCESS_STATS2\r\n");
        _displayPort->beginTransaction(DISPLAY_TRANSACTION_OPTION_RESET_DRAWING);
        processStats2(timeMicroseconds);
        _state = STATE_PROCESS_STATS3;
        break;
    case STATE_PROCESS_STATS3:
        //Serial.printf("STATE_PROCESS_STATS3\r\n");
        processStats3();
#if defined(USE_CMS)
        if (_displayPort->isGrabbed()) {
            _state = STATE_COMMIT:
        }
#endif
        _state = STATE_UPDATE_ALARMS;
        break;
    case STATE_UPDATE_ALARMS:
        //Serial.printf("STATE_UPDATE_ALARMS\r\n");
        updateAlarms();
        //!!_state = _resumeRefreshAtUs ? STATE_TRANSFER : STATE_UPDATE_CANVAS;
        _state = STATE_UPDATE_CANVAS;
        break;
    case STATE_UPDATE_CANVAS:
        //Serial.printf("STATE_UPDATE_CANVAS\r\n");
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
        _elements.updateAHRS_data();
        _state = STATE_DRAW_ELEMENT;
        break;
    case STATE_DRAW_ELEMENT: {
        const uint8_t activeElementIndex = _elements.getActiveElementIndex();

        timeUs32_t startElementTime = timeUs();
        _moreElementsToDraw = _elements.drawNextActiveElement(*_displayPort);
        timeUs32_t executeTimeUs = timeUs() - startElementTime;
        //Serial.printf("STATE_DRAW_ELEMENT ai:%d more:%d\r\n", static_cast<int>(activeElementIndex), static_cast<int>(_moreElementsToDraw));

        enum { OSD_EXEC_TIME_SHIFT = 5 };
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
        //Serial.printf("STATE_DISPLAY_ELEMENT\r\n");
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
        //Serial.printf("STATE_REFRESH_PREARM\r\n");
        if (_elements.drawSpec(*_displayPort)) {
            // Rendering is complete
            _state = STATE_COMMIT;
        }
        break;
    case STATE_COMMIT:
        //Serial.printf("STATE_COMMIT\r\n");
        _displayPort->commitTransaction();
        _state = _resumeRefreshAtUs ? STATE_IDLE : STATE_TRANSFER;
        break;
    case STATE_TRANSFER:
        //Serial.printf("STATE_TRANSFER\r\n");
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
        //Serial.printf("STATE_IDLE\r\n");
        _state = STATE_CHECK;
        break;
    default:
        //Serial.printf("STATE_default\r\n");
        _state = STATE_IDLE;
        break;
    }
}
// NOLINTEND(cppcoreguidelines-macro-usage,cppcoreguidelines-pro-bounds-constant-array-index,hicpp-signed-bitwise)
