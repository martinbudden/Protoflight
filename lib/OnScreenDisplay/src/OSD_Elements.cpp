#include "Cockpit.h"
#include "DisplayPortBase.h"
#include "OSD_Elements.h"

#if !defined(FRAMEWORK_TEST)
//#include <HardwareSerial.h>
#endif

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise)


OSD_Elements::OSD_Elements(const OSD& osd, const FlightController& flightController, const Cockpit& cockpit, const ReceiverBase& receiver, const Debug& debug, const VTX* vtx, const GPS* gps) :
    _osd(osd),
    _flightController(flightController),
    _cockpit(cockpit),
    _receiver(receiver),
    _debug(debug),
    _vtx(vtx),
    _gps(gps)
{
}

void OSD_Elements::init(bool backgroundLayerFlag, uint8_t rowCount, uint8_t columnCount)
{
    (void)backgroundLayerFlag;
    initDrawFunctions();

    if (columnCount !=0  && rowCount != 0) {
        // Ensure that all OSD elements are on the canvas once the number of row and columns is known
        //for (size_t ii = 0; ii < OSD_ELEMENT_COUNT; ++ii) { // NOLINT(modernize-loop-convert)
        for (auto& elementPos : _config.element_pos) {
            //const uint16_t elementPos = _config.element_pos[ii];
            const uint16_t elementTopBits = elementPos & (ELEMENT_TYPE_MASK | PROFILE_MASK);

            uint8_t posX = OSD_X(elementPos);
            uint8_t posY = OSD_Y(elementPos);
            if (posX >= columnCount) {
                posX = columnCount - 1;
                elementPos = elementTopBits | OSD_POS(posX, posY);
            }
            if (posY >= rowCount) {
                posY = rowCount - 1;
                elementPos = elementTopBits | OSD_POS(posX, posY);
            }
        }
    }
};

void OSD_Elements::setProfile(uint8_t profile)
{
    if (profile > 1) {
        profile = 1;
    }
    _profile = profile;
}

void OSD_Elements::setConfig(const config_t& config)
{
    _config = config;
}

void OSD_Elements::setDefaultConfig(uint8_t rowCount, uint8_t columnCount)
{
    const uint8_t midRow = rowCount/2;
    const uint8_t midCol = columnCount/2;

    // Position elements near centre of screen and disabled by default
    for (auto& element_pos : _config.element_pos) {
        element_pos = OSD_POS((midCol - 5), midRow); // cppcheck-suppress useStlAlgorithm
    }

    // Always enable warnings elements by default

    enum { OSD_WARNINGS_PREFERRED_SIZE = 12 };
    _config.element_pos[OSD_WARNINGS] = OSD_POS(midCol - OSD_WARNINGS_PREFERRED_SIZE/2, midRow + 3);

    // Default to old fixed positions for these elements
    _config.element_pos[OSD_CROSSHAIRS]         = OSD_POS(midCol - 2,  midRow - 1);
    _config.element_pos[OSD_ARTIFICIAL_HORIZON] = OSD_POS(midCol - 1,  midRow - 5);
    _config.element_pos[OSD_HORIZON_SIDEBARS]   = OSD_POS(midCol - 1,  midRow - 1);
    _config.element_pos[OSD_CAMERA_FRAME]       = OSD_POS(midCol - 12, midRow - 6);
    _config.element_pos[OSD_UP_DOWN_REFERENCE]  = OSD_POS(midCol - 2,  midRow - 1);
    _config.element_pos[OSD_DISARMED]           = OSD_POS(0, 0);
#if defined(M5_UNIFIED)
    _config.element_pos[OSD_ROLL_ANGLE]         = OSD_POS(0, 2);
    _config.element_pos[OSD_PITCH_ANGLE]        = OSD_POS(8, 2);
    _config.element_pos[OSD_NUMERICAL_HEADING]  = OSD_POS(16, 2);
    _config.element_pos[OSD_ROLL_PIDS]          = OSD_POS(2, 12);
    _config.element_pos[OSD_PITCH_PIDS]         = OSD_POS(2, 13);
    _config.element_pos[OSD_ROLL_PIDS]          = OSD_POS(2, 12);
    _config.element_pos[OSD_PITCH_PIDS]         = OSD_POS(2, 13);
    _config.element_pos[OSD_RC_CHANNELS]        = OSD_POS(0, 10);
#endif
    // enable elements in all profiles by default
    for (auto& element : _config.element_pos) {
        element |= PROFILE_MASK; // cppcheck-suppress useStlAlgorithm
   }
}

uint32_t OSD_Elements::displayWrite(DisplayPortBase& displayPort, const element_t&  element, uint8_t x, uint8_t y, const char *s, uint8_t attr)
{
    if (_blinkBits[element.index]) {
        attr |= DisplayPortBase::BLINK;
    }
    return displayPort.writeString(x, y, s, attr);
}

uint32_t OSD_Elements::displayWrite(DisplayPortBase& displayPort, const element_t& element, uint8_t x, uint8_t y, uint8_t c, uint8_t attr)
{
    if (_blinkBits[element.index]) {
        attr |= DisplayPortBase::BLINK;
    }
    return displayPort.writeChar(x, y, c, attr);
}

bool OSD_Elements::isRenderPending() const
{
    return _displayPendingForeground | _displayPendingBackground;
}

void OSD_Elements::addActiveElement(osd_elements_e element)
{
    if (elementVisible(_config.element_pos[element], _profile)) {
        _activeElements[_activeElementCount++] = element;
    }
}

void OSD_Elements::addActiveElements()
{
    addActiveElement(OSD_ARTIFICIAL_HORIZON);
    addActiveElement(OSD_G_FORCE);
    addActiveElement(OSD_UP_DOWN_REFERENCE);

    addActiveElement(OSD_MAIN_BATTERY_VOLTAGE);
    addActiveElement(OSD_RSSI_VALUE);
    addActiveElement(OSD_CROSSHAIRS);
    addActiveElement(OSD_HORIZON_SIDEBARS);
    addActiveElement(OSD_UP_DOWN_REFERENCE);
    addActiveElement(OSD_ITEM_TIMER_1);
    addActiveElement(OSD_ITEM_TIMER_2);
    addActiveElement(OSD_REMAINING_TIME_ESTIMATE);
    addActiveElement(OSD_FLYMODE);
    addActiveElement(OSD_THROTTLE_POS);
    addActiveElement(OSD_VTX_CHANNEL);
    addActiveElement(OSD_CURRENT_DRAW);
    addActiveElement(OSD_MAH_DRAWN);
    addActiveElement(OSD_WATT_HOURS_DRAWN);
    addActiveElement(OSD_CRAFT_NAME);
#if defined(USE_OSD_PROFILES)
    addActiveElement(OSD_CUSTOM_MSG0);
    addActiveElement(OSD_CUSTOM_MSG1);
    addActiveElement(OSD_CUSTOM_MSG2);
    addActiveElement(OSD_CUSTOM_MSG3);
#endif
#if defined(USE_BAROMETER) || defined(USE_GPS)
    addActiveElement(OSD_ALTITUDE);
#endif
    addActiveElement(OSD_ROLL_PIDS);
    addActiveElement(OSD_PITCH_PIDS);
    addActiveElement(OSD_YAW_PIDS);
    addActiveElement(OSD_POWER);
    addActiveElement(OSD_PID_RATE_PROFILE);
    addActiveElement(OSD_WARNINGS);
    addActiveElement(OSD_AVG_CELL_VOLTAGE);
    addActiveElement(OSD_DEBUG);
    addActiveElement(OSD_DEBUG2);
    addActiveElement(OSD_PITCH_ANGLE);
    addActiveElement(OSD_ROLL_ANGLE);
    addActiveElement(OSD_MAIN_BATTERY_USAGE);
    addActiveElement(OSD_DISARMED);
    addActiveElement(OSD_NUMERICAL_HEADING);
    addActiveElement(OSD_READY_MODE);
#if defined(USE_VARIO)
    addActiveElement(OSD_NUMERICAL_VARIO);
#endif
#if defined(USE_GPS)
    addActiveElement(OSD_COMPASS_BAR);
#endif
    addActiveElement(OSD_ANTI_GRAVITY);
#if defined(USE_BLACKBOX)
    addActiveElement(OSD_LOG_STATUS);
#endif
#if defined(USE_DSHOT)
    addActiveElement(OSD_MOTOR_DIAGNOSTICS);
#endif
    addActiveElement(OSD_FLIP_ARROW);
#if defined(USE_OSD_PROFILES)
    addActiveElement(OSD_PILOT_NAME);
#endif
#if defined(USE_RTC_TIME)
    addActiveElement(OSD_RTC_DATETIME);
#endif
#if defined(USE_OSD_ADJUSTMENTS)
    addActiveElement(OSD_ADJUSTMENT_RANGE);
#endif
#if defined(USE_ADC_INTERNAL)
    addActiveElement(OSD_CORE_TEMPERATURE);
#endif
#if defined(USE_RX_LINK_QUALITY_INFO)
    addActiveElement(OSD_LINK_QUALITY);
#endif
#if defined(USE_RX_LINK_UPLINK_POWER)
    addActiveElement(OSD_TX_UPLINK_POWER);
#endif
#if defined(USE_RX_RSSI_DBM)
    addActiveElement(OSD_RSSI_DBM_VALUE);
#endif
#if defined(USE_RX_RSNR)
    addActiveElement(OSD_RSNR_VALUE);
#endif
#if defined(USE_OSD_STICK_OVERLAY)
    addActiveElement(OSD_STICK_OVERLAY_LEFT);
    addActiveElement(OSD_STICK_OVERLAY_RIGHT);
#endif
#if defined(USE_PROFILE_NAMES)
    addActiveElement(OSD_RATE_PROFILE_NAME);
    addActiveElement(OSD_PID_PROFILE_NAME);
#endif
#if defined(USE_OSD_PROFILES)
    addActiveElement(OSD_PROFILE_NAME);
#endif
    addActiveElement(OSD_RC_CHANNELS);
    addActiveElement(OSD_CAMERA_FRAME);
#if defined(USE_PERSISTENT_STATS)
    addActiveElement(OSD_TOTAL_FLIGHTS);
#endif
    addActiveElement(OSD_AUX_VALUE);
#if defined(USE_OSD_HD)
    addActiveElement(OSD_SYS_GOGGLE_VOLTAGE);
    addActiveElement(OSD_SYS_VTX_VOLTAGE);
    addActiveElement(OSD_SYS_BITRATE);
    addActiveElement(OSD_SYS_DELAY);
    addActiveElement(OSD_SYS_DISTANCE);
    addActiveElement(OSD_SYS_LQ);
    addActiveElement(OSD_SYS_GOGGLE_DVR);
    addActiveElement(OSD_SYS_VTX_DVR);
    addActiveElement(OSD_SYS_WARNINGS);
    addActiveElement(OSD_SYS_VTX_TEMP);
    addActiveElement(OSD_SYS_FAN_SPEED);
#endif
#if defined(USE_RANGEFINDER)
    addActiveElement(OSD_LIDAR_DISTANCE);
#endif
#if defined(USE_GPS)
    //if (sensors(SENSOR_GPS)) {
        addActiveElement(OSD_GPS_SATS);
        addActiveElement(OSD_GPS_SPEED);
        addActiveElement(OSD_GPS_LAT);
        addActiveElement(OSD_GPS_LON);
        addActiveElement(OSD_HOME_DISTANCE);
        addActiveElement(OSD_HOME_DIRECTION);
        addActiveElement(OSD_FLIGHT_DISTANCE);
        addActiveElement(OSD_EFFICIENCY);
    //}
#endif
#if defined(USE_DSHOT)
    if (_cockpit.featureIsEnabled(Features::FEATURE_ESC_SENSOR)) {
        addActiveElement(OSD_ESC_TEMPERATURE);
        addActiveElement(OSD_ESC_RPM);
        addActiveElement(OSD_ESC_RPM_FREQUENCY);
    }
#endif
#if defined(USE_GPS_LAP_TIMER)
    //if (sensors(SENSOR_GPS)) {
        addActiveElement(OSD_GPS_LAP_TIME_CURRENT);
        addActiveElement(OSD_GPS_LAP_TIME_PREVIOUS);
        addActiveElement(OSD_GPS_LAP_TIME_BEST3);
    //}
#endif // USE_GPS_LAP_TIMER
    addActiveElement(OSD_TOTAL_FLIGHTS);
}

void OSD_Elements::updateAttitude(float rollAngleDegrees, float pitchAngleDegrees, float yawAngleDegrees)
{
    _rollAngleDegrees = rollAngleDegrees;
    _pitchAngleDegrees = pitchAngleDegrees;
    _yawAngleDegrees = yawAngleDegrees;
}

bool OSD_Elements::drawNextActiveElement(DisplayPortBase& displayPort)
{
    //Serial.printf("drawNextActiveElement: idx:%d cnt:%d\r\n", _activeElementIndex, _activeElementCount);
    if (_activeElementIndex >= _activeElementCount) {
        _activeElementIndex = 0;
        return false;
    }

    const uint8_t element = _activeElements[_activeElementIndex];

    if (!_backgroundLayerSupported && DrawBackgroundFunctions[element] && !_backgroundRendered) {
        // If the background layer isn't supported then we
        // have to draw the element's static layer as well.
        _backgroundRendered = drawSingleElementBackground(displayPort, element);
        // After the background always come back to check for foreground
        return true;
    }

    if (drawSingleElement(displayPort, element)) {
        // If rendering is complete then advance to the next element
        // Prepare to render the background of the next element
        _backgroundRendered = false;
        if (++_activeElementIndex >= _activeElementCount) {
            _activeElementIndex = 0;
            return false;
        }
    }
    return true;
}

/*!
Returns true if there is more to display
*/
bool OSD_Elements::displayActiveElement(DisplayPortBase& displayPort)
{
    if (_activeElementIndex >= _activeElementCount) {
        return false;
    }
    // If there's a previously drawn background string to be displayed, do that
    if (_displayPendingBackground) {
        displayWrite(displayPort, _activeElement, _activeElement.posX + _activeElement.offsetX, _activeElement.posY + _activeElement.offsetY, &_activeElement.buf[0], _activeElement.attr);
        _activeElement.buf[0] = '\0';
        _displayPendingBackground = false;
        return _displayPendingForeground;
    }
    // If there's a previously drawn foreground string to be displayed, do that
    if (_displayPendingForeground) {
        displayWrite(displayPort, _activeElement, _activeElement.posX + _activeElement.offsetX, _activeElement.posY + _activeElement.offsetY, &_activeElement.buf[0], _activeElement.attr);
        _activeElement.buf[0] = '\0';
        _displayPendingForeground = false;
    }
    return false;
}

bool OSD_Elements::drawSingleElement(DisplayPortBase& displayPort, uint8_t elementIndex) // cppcheck-suppress constParameterReference
{
    //Serial.printf("drawSingleElement:%d\r\n", elementIndex);
    // By default mark the element as rendered in case it's in the off blink state
    _activeElement.rendered = true;

    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-constant-array-index)
    if (!DrawFunctions[elementIndex]) {
        // Element has no drawing function
        return true;
    }
    if (!displayPort.getUseDeviceBlink() && _blinkBits[elementIndex]) {
        return true;
    }

    _activeElement.type = ELEMENT_TYPE(_config.element_pos[elementIndex]);
    _activeElement.index = elementIndex;
    _activeElement.posX = OSD_X(_config.element_pos[elementIndex]);
    _activeElement.posY = OSD_Y(_config.element_pos[elementIndex]);
    _activeElement.offsetX = 0;
    _activeElement.offsetY = 0;
    _activeElement.attr = DisplayPortBase::SEVERITY_NORMAL;
    _activeElement.drawElement = true;

    // Call the element drawing function
    if (isSysOSD_Element(elementIndex)) { // cppcheck-suppress knownConditionTrueFalse
#if defined(USE_OSD_HD)
        displayPort.writeSys(_activeElement.posX, _activeElement.posY, static_cast<DisplayPortBase::system_element_e>(elementIndex - OSD_SYS_GOGGLE_VOLTAGE + DisplayPortBase::SYS_GOGGLE_VOLTAGE));
#endif
    } else {
        //Serial.print("calling draw fn\r\n");
        (this->*DrawFunctions[elementIndex])(displayPort);
        if (_activeElement.drawElement) {
            _displayPendingForeground = true;
        }
    }
    // NOLINTEND(cppcoreguidelines-pro-bounds-constant-array-index)

    return _activeElement.rendered;
}

bool OSD_Elements::drawSingleElementBackground(DisplayPortBase& displayPort, uint8_t elementIndex)
{
    (void)displayPort;

     // NOLINTBEGIN(cppcoreguidelines-pro-bounds-constant-array-index)
    if (!DrawBackgroundFunctions[elementIndex]) {
        return true;
    }

    _activeElement.type = ELEMENT_TYPE(_config.element_pos[elementIndex]);
    _activeElement.index = elementIndex;
    _activeElement.posX = OSD_X(_config.element_pos[elementIndex]);
    _activeElement.posY = OSD_Y(_config.element_pos[elementIndex]);
    _activeElement.offsetX = 0;
    _activeElement.offsetY = 0;
    _activeElement.attr = DisplayPortBase::SEVERITY_NORMAL;

    (this->*DrawBackgroundFunctions[elementIndex])(displayPort);
    if (_activeElement.drawElement) {
        _displayPendingBackground = true;
    }
    // NOLINTEND(cppcoreguidelines-pro-bounds-constant-array-index)

    return _activeElement.rendered;
}

void OSD_Elements::drawActiveElementsBackground(DisplayPortBase& displayPort) // NOLINT(readability-make-member-function-const)
{
    if (_backgroundLayerSupported) {
        displayPort.layerSelect(DisplayPortBase::LAYER_BACKGROUND);
        displayPort.clearScreen(DISPLAY_CLEAR_WAIT);
        for (size_t ii = 0; ii < _activeElementCount; ++ii) {
            while (!drawSingleElementBackground(displayPort, _activeElements[ii])) {};
        }
        displayPort.layerSelect(DisplayPortBase::LAYER_FOREGROUND);
    }
}

bool OSD_Elements::drawSpec(DisplayPortBase& displayPort)
{
    (void)displayPort;
    return true;
}

// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise)
