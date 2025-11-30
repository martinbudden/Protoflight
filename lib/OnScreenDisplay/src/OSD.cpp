#include "Cockpit.h"
#include "DisplayPortBase.h"
#include "FormatInteger.h"
#include "OSD.h"

#include <AHRS_MessageQueue.h>
//#include <HardwareSerial.h>
#include <MSP_Box.h>
#include <ReceiverBase.h>
#include <cstring>

#if defined(FRAMEWORK_USE_FREERTOS)
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/FreeRTOSConfig.h>
#include <freertos/task.h>
#else
#if defined(FRAMEWORK_ARDUINO_STM32)
#include <STM32FreeRTOS.h>
#endif
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <task.h>
#endif
#endif


// NOLINTBEGIN(cppcoreguidelines-macro-usage,cppcoreguidelines-pro-bounds-constant-array-index,hicpp-signed-bitwise)

OSD::OSD(const FlightController& flightController, const Cockpit& cockpit, const AHRS_MessageQueue& ahrsMessageQueue, Debug& debug) : // cppcheck-suppress constParameterReference
    _elements(*this, flightController, cockpit, debug),
    _cockpit(cockpit),
    _ahrsMessageQueue(ahrsMessageQueue)
{
}

void OSD::init(DisplayPortBase* displayPort)
{
    _displayPort = displayPort;

    const uint8_t rowCount = displayPort->getRowCount();
    const uint8_t columnCount = displayPort->getColumnCount();
    if (columnCount !=0  && rowCount != 0) {
        _config.canvas_column_count = columnCount;
        _config.canvas_row_count = rowCount;
    }
}

void OSD::drawLogoAndCompleteInitialization()
{
    uint8_t midRow = _displayPort->getRowCount() / 2;
    uint8_t midCol = _displayPort->getColumnCount() / 2;

    //resetAlarms();

    _backgroundLayerSupported = _displayPort->layerSupported(DisplayPortBase::LAYER_BACKGROUND);
    _displayPort->layerSelect(DisplayPortBase::LAYER_FOREGROUND);

    _displayPort->clearScreen(DISPLAY_CLEAR_WAIT);

    // Display betaflight logo
    drawLogo(midCol - (LOGO_COLUMN_COUNT) / 2, midRow - 5);

    std::array<char, 30> string_buffer;
    sprintf(&string_buffer[0], "V%s", "0.0.1");//FC_VERSION_STRING);
    _displayPort->writeString(midCol + 5, midRow, &string_buffer[0]);
#if defined(USE_CMS) || true
    _displayPort->writeString(midCol - 8, midRow + 2, "MENU:THR MID");
    _displayPort->writeString(midCol - 4, midRow + 3, "+ YAW LEFT");
    _displayPort->writeString(midCol - 4, midRow + 4, "+ PITCH UP");
#endif

#if defined(USE_RTC_TIME)
    std::array<char, FORMATTED_DATE_TIME_BUFSIZE> dateTimeBuffer;
    if (osdFormatRtcDateTime(&dateTimeBuffer[0])) {
        _displayPort->writeString(midCol - 10, midRow + 6, &dateTimeBuffer[]);
    }
#endif

    _resumeRefreshAtUs = timeUs() + 4'000'000;
#if defined(USE_OSD_PROFILES)
    _elements.setProfile(_config.osdProfileIndex);
#endif

    _elements.init(_backgroundLayerSupported, _displayPort->getRowCount(), _displayPort->getColumnCount());
    analyzeActiveElements();

    _displayPort->redraw();

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

void OSD::setStatsConfig(const statsConfig_t& statsConfig)
{
    _statsConfig = statsConfig;
}

void OSD::setStatsState(uint8_t statIndex, bool enabled)
{
    if (enabled) {
        _config.enabled_stats_flags |= (1U << statIndex);
    } else {
        _config.enabled_stats_flags &= ~(1U << statIndex);
    }
}

bool OSD::getStatsState(uint8_t statIndex) const
{
    return _config.enabled_stats_flags & (1U << statIndex);
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
    _stats.max_esc_temperature_index = 0;
    _stats.max_esc_temperature = 0;
    _stats.max_esc_rpm     = 0;

    _stats.min_link_quality = 99; //(linkQualitySource == LQ_SOURCE_NONE) ? 99 : 100; // percent
    enum { CRSF_RSSI_MIN =-130, CRSF_RSSI_MAX = 0, CRSF_SNR_MIN = -30, CRSF_SNR_MAX = 20 };
    _stats.min_rssi_dbm = CRSF_RSSI_MAX;
    _stats.min_rsnr = CRSF_SNR_MAX;
}

// Controls the display order of the OSD post-flight statistics.
// Adjust the ordering here to control how the post-flight stats are presented.
// Every entry in osd_stats_e should be represented. Any that are missing will not
// be shown on the the post-flight statistics page.
// If you reorder the stats it's likely that you'll need to make likewise updates
// to the unit tests.

// If adding new stats, please add to the osdStatsNeedAccelerometer() function
// if the statistic utilizes the accelerometer.
//
#if defined (USE_OSD_STATS)
const std::array<OSD::stats_e, OSD::STATS_COUNT> OSD::StatsDisplayOrder= {
    OSD::STATS_RTC_DATE_TIME,
    OSD::STATS_TIMER_1,
    OSD::STATS_TIMER_2,
#if defined(USE_BAROMETER)
    OSD::STATS_MAX_ALTITUDE,
#endif
    OSD::STATS_MAX_SPEED,
#if defined(USE_GPS)
    OSD::STATS_MAX_DISTANCE,
    OSD::STATS_FLIGHT_DISTANCE,
#endif
    OSD::STATS_MIN_BATTERY,
    OSD::STATS_END_BATTERY,
    OSD::STATS_BATTERY,
    OSD::STATS_MIN_RSSI,
    OSD::STATS_MAX_CURRENT,
    OSD::STATS_USED_MAH,
    OSD::STATS_BLACKBOX,
    OSD::STATS_BLACKBOX_NUMBER,
    OSD::STATS_MAX_G_FORCE,
#if defined(USE_DSHOT)
    OSD::STATS_MAX_ESC_TEMPERATURE,
    OSD::STATS_MAX_ESC_RPM,
#endif
    OSD::STATS_MIN_LINK_QUALITY,
    OSD::STATS_MAX_FFT,
    OSD::STATS_MIN_RSSI_DBM,
    OSD::STATS_MIN_RSNR,
    OSD::STATS_TOTAL_FLIGHTS,
    OSD::STATS_TOTAL_TIME,
#if defined(USE_GPS)
    OSD::STATS_TOTAL_DISTANCE,
#endif
    OSD::STATS_WATT_HOURS_DRAWN,
    OSD::STATS_BEST_3_CONSEC_LAPS,
    OSD::STATS_BEST_LAP,
    OSD::STATS_FULL_THROTTLE_TIME,
    OSD::STATS_FULL_THROTTLE_COUNTER,
    OSD::STATS_AVG_THROTTLE,
};
#endif

void OSD::displayStatisticLabel(uint8_t x, uint8_t y, const char * text, const char * value)
{
    _displayPort->writeString(x - 13, y, text);
    _displayPort->writeString(x + 5, y, ":");
    _displayPort->writeString(x + 7, y, value);
}

bool OSD::displayStatistic(int statistic, uint8_t displayRow)
{
    const uint8_t midCol = _displayPort->getColumnCount() / 2;
    enum { ELEMENT_BUFFER_LENGTH = 32 };
    std::array<char, ELEMENT_BUFFER_LENGTH> buf {};

    switch (statistic) { // NOLINT(hicpp-multiway-paths-covered)
    case STATS_TOTAL_FLIGHTS:
        ui2a(_statsConfig.total_flights, &buf[0]);
        displayStatisticLabel(midCol, displayRow, "TOTAL FLIGHTS", &buf[0]);
        return true;
    default:
        return false;
    }
}
/*
Called repeatedly until it returns true which indicates that all stats have been rendered.
*/
bool OSD::renderStatsContinue()
{
#if defined(USE_OSD_STATS)
    const uint8_t midCol = _displayPort->getColumnCount() / 2;

    if (_statsRenderingState.row == 0) {
        bool displayLabel = false;
        // if rowCount is 0 then we're running an initial analysis of the active stats items
        if (_statsRenderingState.rowCount > 0) {
            const uint8_t availableRows = _displayPort->getRowCount();
            uint8_t displayRows = std::min(_statsRenderingState.rowCount, availableRows);
            if (_statsRenderingState.rowCount < availableRows) {
                displayLabel = true;
                displayRows++;
            }
            _statsRenderingState.row = static_cast<uint8_t>((availableRows - displayRows) / 2);  // center the stats vertically
        }
        if (displayLabel) {
            _displayPort->writeString(midCol - static_cast<uint8_t>(strlen("--- STATS ---") / 2), _statsRenderingState.row, "--- STATS ---");
            ++_statsRenderingState.row;
            return false;
        }
    }
    bool statisticDisplayed = false;
    while (_statsRenderingState.index < STATS_COUNT) {
        const uint8_t index = _statsRenderingState.index;
        // prepare for the next call to the method
        ++_statsRenderingState.index;
        // look for something to render
        if (_config.enabled_stats_flags & (1<< StatsDisplayOrder[index])) {
            if (displayStatistic(StatsDisplayOrder[index], _statsRenderingState.row)) {
                ++_statsRenderingState.row;
                statisticDisplayed = true;
                break;
            }
        }
    }
    const bool moreSpaceAvailable = _statsRenderingState.row < _displayPort->getRowCount();
    if (statisticDisplayed && moreSpaceAvailable) {
        return false;
    }
    if (_statsRenderingState.rowCount == 0) {
        _statsRenderingState.rowCount = _statsRenderingState.row;
    }
#endif
    return true;
}

/*!
State machine to refresh statistics.
Returns true when all iterations are complete.
*/
bool OSD::refreshStats()
{
    switch (_refreshStatsState) {
    default:
        [[fallthrough]];
    case REFRESH_STATS_STATE_INITIAL_CLEAR_SCREEN:
        //Serial.printf("REFRESH_STATS_STATE_INITIAL_CLEAR_SCREEN\r\n");
        _statsRenderingState.row = 0;
        _statsRenderingState.index = 0;
        if (_statsRenderingState.rowCount > 0) {
            _refreshStatsState = REFRESH_STATS_STATE_RENDER_STATS;
        } else {
            _refreshStatsState = REFRESH_STATS_STATE_COUNT_STATS;
        }
        _displayPort->clearScreen(DISPLAY_CLEAR_NONE);
        break;
    case REFRESH_STATS_STATE_COUNT_STATS:{
        //Serial.printf("REFRESH_STATS_STATE_COUNT_STATS\r\n");
        // No stats row count has been set yet.
        // Go through the logic one time to determine how many stats are actually displayed.
        bool count_phase_complete = renderStatsContinue();
        if (count_phase_complete) { // cppcheck-suppress knownConditionTrueFalse
            _refreshStatsState = REFRESH_STATS_STATE_CLEAR_SCREEN;
        }
        break;
    }
    case REFRESH_STATS_STATE_CLEAR_SCREEN:
        //Serial.printf("REFRESH_STATS_STATE_CLEAR_SCREEN\r\n");
        _statsRenderingState.row = 0;
        _statsRenderingState.index = 0;
        // Then clear the screen and commence with normal stats display which will
        // determine if the heading should be displayed and also center the content vertically.
        _displayPort->clearScreen(DISPLAY_CLEAR_NONE);
        _refreshStatsState = REFRESH_STATS_STATE_RENDER_STATS;
        break;
    case REFRESH_STATS_STATE_RENDER_STATS:
        //Serial.printf("REFRESH_STATS_STATE_RENDER_STATS\r\n");
        if (renderStatsContinue()) {
            _refreshStatsState = REFRESH_STATS_STATE_INITIAL_CLEAR_SCREEN;
            return true;
        }
        break;
    };

    return false;
}

/*!
Returns true if refresh stats is required.
*/
bool OSD::processStats1(timeUs32_t currentTimeUs)
{
    (void)currentTimeUs;
    return false;
}

void OSD::processStats2(timeUs32_t currentTimeUs)
{
    if (_resumeRefreshAtUs) {
        if (currentTimeUs < _resumeRefreshAtUs) {
            // in timeout period, check sticks for activity or CRASH_FLIP switch to resume display.
            if (!_cockpit.isArmed()) {
                const ReceiverBase::controls_pwm_t controls = _cockpit.getReceiver().getControlsPWM();
                if (Cockpit::pwmIsHigh(controls.throttle) || Cockpit::pwmIsHigh(controls.pitch) || _cockpit.isRcModeActive(MSP_Box::BOX_CRASH_FLIP)) {
                    _resumeRefreshAtUs = currentTimeUs;
                }
            }
            return;
        }
        _displayPort->clearScreen(DISPLAY_CLEAR_NONE);
        _resumeRefreshAtUs = 0;
        _statsEnabled = false;
        _stats.armed_time = 0;
        //schedulerIgnoreTaskExecTime();
    }
#if defined(USE_ESC_SENSOR)
    if (featureIsEnabled(FEATURE_ESC_SENSOR)) {
        osdEscDataCombined = getEscSensorData(ESC_SENSOR_COMBINED);
    }
#endif
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
        _config.enabled_warnings_flags |= (1U << warningIndex);
    } else {
        _config.enabled_warnings_flags &= ~(1U << warningIndex);
    }
}

bool OSD::getWarningState(uint8_t warningIndex) const
{
    return _config.enabled_warnings_flags & (1U << warningIndex);
}

void OSD::drawLogo(uint8_t x, uint8_t y)
{
    // the logo is in the font characters starting at 160
    enum { START_CHARACTER = 160 };
    enum { END_OF_FONT = 255 };

    // display logo and help
    uint8_t characterCode = START_CHARACTER;
    for (uint8_t row = 0; row < LOGO_ROW_COUNT; ++row) {
        for (uint8_t column = 0; column < LOGO_COLUMN_COUNT; ++column) {
            if (characterCode < END_OF_FONT) {
                _displayPort->writeChar(x + column, y + row, characterCode);
                ++characterCode;
            }
        }
    }
}

void OSD::updateDisplay(uint32_t timeMicroseconds, uint32_t timeMicrosecondsDelta)
{
    if (_displayPort->isGrabbed()) {
        return;
    }
    if (_state == STATE_IDLE) {
        _state = STATE_CHECK;
    } else if (_state != STATE_INIT) {
        return;
    }
    while (_state != STATE_IDLE) {
#if defined(FRAMEWORK_USE_FREERTOS)
        taskYIELD();
#endif
        updateDisplayIteration(timeMicroseconds, timeMicrosecondsDelta);
    }
}

void OSD::updateDisplayIteration(uint32_t timeMicroseconds, uint32_t timeMicrosecondsDelta) // NOLINT(readability-function-cognitive-complexity)
{
    (void)timeMicrosecondsDelta;

    switch (_state) {
    case STATE_INIT:
        //Serial.printf("STATE_INIT\r\n");
        if (!_displayPort->checkReady(false)) {
            // Frsky OSD needs a display redraw after search for MAX7456 devices
            if (_displayPort->getDeviceType() == DisplayPortBase::DEVICE_TYPE_FRSKY_OSD) {
                _displayPort->redraw();
                return;
            }
        }
        _displayPort->beginTransaction(DISPLAY_TRANSACTION_OPTION_RESET_DRAWING);
        drawLogoAndCompleteInitialization();
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
        // transaction begins here since refreshStats draws to the screen
        _displayPort->beginTransaction(DISPLAY_TRANSACTION_OPTION_RESET_DRAWING); 
        _state = processStats1(timeMicroseconds) ? STATE_REFRESH_STATS : STATE_PROCESS_STATS2; // cppcheck-suppress knownConditionTrueFalse
        break;
    case STATE_REFRESH_STATS:
        //Serial.printf("STATE_REFRESH_STATS\r\n");
        if (refreshStats()) { // draws the statistics to the screen
            _state = STATE_PROCESS_STATS2;
        }
        break;
    case STATE_PROCESS_STATS2:
        //Serial.printf("STATE_PROCESS_STATS2\r\n");
        processStats2(timeMicroseconds); // may clear screen
        _state = STATE_PROCESS_STATS3; // cppcheck-suppress redundantAssignment
        break;
    case STATE_PROCESS_STATS3:
        //Serial.printf("STATE_PROCESS_STATS3\r\n");
        processStats3();
#if defined(USE_CMS)
        if (_displayPort->isGrabbed()) {
            _state = STATE_COMMIT;
        }
#endif
        _state = STATE_UPDATE_ALARMS; // cppcheck-suppress redundantAssignment
        break;
    case STATE_UPDATE_ALARMS:
        //Serial.printf("STATE_UPDATE_ALARMS\r\n");
        updateAlarms();
        //!!_state = _resumeRefreshAtUs ? STATE_TRANSFER : STATE_UPDATE_CANVAS;
        _state = STATE_UPDATE_CANVAS;
        break;
    case STATE_UPDATE_CANVAS: {
        //Serial.printf("STATE_UPDATE_CANVAS\r\n");
        if (_cockpit.isRcModeActive(MSP_Box::BOX_OSD)) {
            // Hide OSD when OSD SW mode is active
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
        AHRS::ahrs_data_t ahrsData {};
        _ahrsMessageQueue.PEEK_AHRS_DATA(ahrsData);
        _elements.updateAttitude(ahrsData.orientation.calculateRollDegrees(), ahrsData.orientation.calculatePitchDegrees(), ahrsData.orientation.calculateYawDegrees()); // update the AHRS data, so it is only needed to be done once for all elements that require it
        _state = STATE_DRAW_ELEMENT;
        break;
    }
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
        _state = (_cockpit.isArmed() && _config.osd_show_spec_prearm) ? STATE_REFRESH_PRE_ARM : STATE_COMMIT;
        break;
    }
    case STATE_DISPLAY_ELEMENT: {
        //Serial.printf("STATE_DISPLAY_ELEMENT\r\n");
        const bool moreToDisplay = _elements.displayActiveElement(*_displayPort);
        if (!moreToDisplay) {
            // finished displaying this element, so move on to the next one if there is one
            if (_moreElementsToDraw) {
                _state = STATE_DRAW_ELEMENT;
            } else {
                _state = (_cockpit.isArmed() && _config.osd_show_spec_prearm) ? STATE_REFRESH_PRE_ARM : STATE_COMMIT;
            }
        }
        break;
    }
    case STATE_REFRESH_PRE_ARM:
        //Serial.printf("STATE_REFRESH_PRE-ARM\r\n");
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
        //_state = STATE_CHECK;
        break;
    default:
        //Serial.printf("STATE_default\r\n");
        _state = STATE_IDLE;
        break;
    }
}
// NOLINTEND(cppcoreguidelines-macro-usage,cppcoreguidelines-pro-bounds-constant-array-index,hicpp-signed-bitwise)
