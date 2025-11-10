#include "DisplayPortBase.h"
#include "FlightController.h"
#include "OSD_Elements.h"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise)

static std::array<OSD_Elements::osdElementDrawFn, OSD_ITEM_COUNT> osdElementDrawFunctions = {};

void OSD_Elements::init(bool backgroundLayerFlag)
{
    (void)backgroundLayerFlag;

    osdElementDrawFunctions[OSD_CAMERA_FRAME]       = nullptr;
    osdElementDrawFunctions[OSD_RSSI_VALUE]         = &OSD_Elements::drawRSSI;
    osdElementDrawFunctions[OSD_MAIN_BATT_VOLTAGE]  = &OSD_Elements::drawMainBatteryVoltage;
    osdElementDrawFunctions[OSD_CROSSHAIRS]         = &OSD_Elements::drawCrosshairs;
    osdElementDrawFunctions[OSD_ROLL_PIDS]          = &OSD_Elements::drawPIDsRoll;
    osdElementDrawFunctions[OSD_PITCH_PIDS]         = &OSD_Elements::drawPIDsPitch;
    osdElementDrawFunctions[OSD_YAW_PIDS]           = &OSD_Elements::drawPIDsYaw;
};

void OSD_Elements::setConfig(const config_t& config)
{
    _config = config;
}


int OSD_Elements::displayWrite(element_t* element, uint8_t x, uint8_t y, uint8_t attr, const char *s)
{
    if (_blinkBits[element->index]) {
        attr |= DisplayPortBase::BLINK;
    }
    return element->displayPort->writeString(x, y, attr, s);
}

int OSD_Elements::displayWrite(element_t* element, uint8_t x, uint8_t y, uint8_t attr, char c)
{
    const std::array<char, 2> buf = { c, 0 };
    return displayWrite(element, x, y, attr, &buf[0]);
}

/*!
Returns true if there is more to display
*/
bool OSD_Elements::displayActiveElement()
{
    if (_activeElementIndex >= _activeElementCount) {
        return false;
    }
    // If there's a previously drawn background string to be displayed, do that
    if (_displayPendingBackground) {
        displayWrite(&_activeElement, _activeElement.posX + _activeElement.offsetX, _activeElement.posY + _activeElement.offsetY, _activeElement.attr, &_activeElement.buf[0]);
        _activeElement.buf[0] = '\0';
        _displayPendingBackground = false;
        return _displayPendingForeground;
    }
    // If there's a previously drawn foreground string to be displayed, do that
    if (_displayPendingForeground) {
        displayWrite(&_activeElement, _activeElement.posX + _activeElement.offsetX, _activeElement.posY + _activeElement.offsetY, _activeElement.attr, &_activeElement.buf[0]);
        _activeElement.buf[0] = '\0';
        _displayPendingForeground = false;
    }
    return false;
}


bool OSD_Elements::drawSingleElement(DisplayPortBase* displayPort, uint8_t elementIndex)
{
    // By default mark the element as rendered in case it's in the off blink state
    _activeElement.rendered = true;
    if (!osdElementDrawFunctions[elementIndex]) { // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
        // Element has no drawing function
        return true;
    }
    if (!displayPort->getUseDeviceBlink() && _blinkBits[elementIndex]) {
        return true;
    }

    const uint8_t posX = OSD_X(_config.item_pos[elementIndex]); // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    const uint8_t posY = OSD_Y(_config.item_pos[elementIndex]); // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)

    _activeElement.index = elementIndex;
    _activeElement.posX = posX;
    _activeElement.posY = posY;
    _activeElement.offsetX = 0;
    _activeElement.offsetY = 0;
    _activeElement.type = static_cast<element_type_e>(OSD_TYPE(_config.item_pos[elementIndex])); // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    _activeElement.displayPort = displayPort;
    _activeElement.drawElement = true;
    _activeElement.attr = DisplayPortBase::SEVERITY_NORMAL;

    // Call the element drawing function
    if (isSysOSD_Element(elementIndex)) {
        displayPort->writeSys(posX, posY, static_cast<DisplayPortBase::system_element_e>(elementIndex - OSD_SYS_GOGGLE_VOLTAGE + DisplayPortBase::SYS_GOGGLE_VOLTAGE));
    } else {
        OSD_Elements::osdElementDrawFn drawFn = osdElementDrawFunctions[elementIndex]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
        (this->*drawFn)(_activeElement);
        if (_activeElement.drawElement) {
            _displayPendingForeground = true;
        }
    }

    return _activeElement.rendered;
}

// Draw functions
void OSD_Elements::formatPID(char * buf, const char * label, uint8_t axis) // NOLINT(readability-non-const-parameter)
{
    const FlightController::PIDF_uint16_t pid = _flightController.getPID_MSP(axis);
    sprintf(buf, "%s %3d %3d %3d %3d %3d", label,
        pid.kp,
        pid.ki,
        pid.kd,
        pid.ks,
        pid.kk
    );
}

void OSD_Elements::drawPIDsRoll(element_t&element)
{
    formatPID(&element.buf[0], "ROL", FlightController::ROLL_RATE_DPS);
}

void OSD_Elements::drawPIDsPitch(element_t& element)
{
    formatPID(&element.buf[0], "PIT", FlightController::PITCH_RATE_DPS);
}

void OSD_Elements::drawPIDsYaw(element_t&element)
{
    formatPID(&element.buf[0], "YAW", FlightController::YAW_RATE_DPS);
}

void OSD_Elements::drawRSSI(element_t&element)
{
    (void)element;
}

void OSD_Elements::drawMainBatteryVoltage(element_t&element)
{
    (void)element;
}

void OSD_Elements::drawCrosshairs(element_t&element)
{
    (void)element;
}

// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise)
