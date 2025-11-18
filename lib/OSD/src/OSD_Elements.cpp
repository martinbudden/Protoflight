#include "DisplayPortBase.h"
#include "OSD_Elements.h"

#if !defined(FRAMEWORK_TEST)
//#include <HardwareSerial.h>
#endif

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise)


OSD_Elements::OSD_Elements(const OSD& osd, const FlightController& flightController, const Cockpit& cockpit, const Debug& debug) :
    _osd(osd),
    _flightController(flightController),
    _cockpit(cockpit),
    _debug(debug)
{
}

void OSD_Elements::init(bool backgroundLayerFlag, uint8_t rowCount, uint8_t columnCount)
{
    (void)backgroundLayerFlag;
    initDrawFunctions();

    if (columnCount !=0  && rowCount != 0) {
        // Ensure that all OSD elements are on the canvas once the number of row and columns is known
        for (size_t ii = 0; ii < OSD_ITEM_COUNT; ++ii) { // NOLINT(modernize-loop-convert)
            const uint16_t elementPos = _config.element_pos[ii];
            const uint16_t elementTopBits = elementPos & (ELEMENT_TYPE_MASK | PROFILE_MASK);

            uint8_t posX = OSD_X(elementPos);
            uint8_t posY = OSD_Y(elementPos);
            if (posX >= columnCount) {
                posX = columnCount - 1;
                _config.element_pos[ii] = elementTopBits | OSD_POS(posX, posY);
            }
            if (posY >= rowCount) {
                posY = rowCount - 1;
                _config.element_pos[ii] = elementTopBits | OSD_POS(posX, posY);
            }
        }
    }
};

void OSD_Elements::setConfig(const config_t& config)
{
    _config = config;
}

void OSD_Elements::setDefaultConfig()
{
// If user includes OSD_HD in the build assume they want to use it as default
#if defined(USE_OSD_HD)
    uint8_t midRow = 10;
    uint8_t midCol = 26;
#else
    uint8_t midRow = 7;
    uint8_t midCol = 15;
#endif

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

uint32_t OSD_Elements::displayWrite(DisplayPortBase& displayPort, const element_t&  element, uint8_t x, uint8_t y, uint8_t attr, const char *s)
{
    if (_blinkBits[element.index]) {
        attr |= DisplayPortBase::BLINK;
    }
    return displayPort.writeString(x, y, attr, s);
}

uint32_t OSD_Elements::displayWrite(DisplayPortBase& displayPort, const element_t& element, uint8_t x, uint8_t y, uint8_t attr, uint8_t c)
{
    if (_blinkBits[element.index]) {
        attr |= DisplayPortBase::BLINK;
    }
    return displayPort.writeChar(x, y, attr, c);
}

bool OSD_Elements::isRenderPending() const
{
    return _displayPendingForeground | _displayPendingBackground;
}

void OSD_Elements::addActiveElement(osd_items_e element)
{
    if (elementVisible(_config.element_pos[element], _profile)) {
        _activeElementArray[_activeElementCount++] = element;
    }
}

void OSD_Elements::addActiveElements()
{
    //addActiveElement(OSD_DEBUG);
    addActiveElement(OSD_ROLL_PIDS);
    addActiveElement(OSD_PITCH_PIDS);
    addActiveElement(OSD_CROSSHAIRS);
    addActiveElement(OSD_ARTIFICIAL_HORIZON);
    addActiveElement(OSD_ROLL_ANGLE);
    addActiveElement(OSD_PITCH_ANGLE);
    addActiveElement(OSD_RC_CHANNELS);
    addActiveElement(OSD_DISARMED);
    addActiveElement(OSD_NUMERICAL_HEADING);
    addActiveElement(OSD_WARNINGS);
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

    const uint8_t item = _activeElementArray[_activeElementIndex];

    if (!_backgroundLayerSupported && elementDrawBackgroundFunctions[item] && !_backgroundRendered) {
        // If the background layer isn't supported then we
        // have to draw the element's static layer as well.
        _backgroundRendered = drawSingleElementBackground(displayPort, item);
        // After the background always come back to check for foreground
        return true;
    }

    if (drawSingleElement(displayPort, item)) {
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
        displayWrite(displayPort, _activeElement, _activeElement.posX + _activeElement.offsetX, _activeElement.posY + _activeElement.offsetY, _activeElement.attr, &_activeElement.buf[0]);
        _activeElement.buf[0] = '\0';
        _displayPendingBackground = false;
        return _displayPendingForeground;
    }
    // If there's a previously drawn foreground string to be displayed, do that
    if (_displayPendingForeground) {
        displayWrite(displayPort, _activeElement, _activeElement.posX + _activeElement.offsetX, _activeElement.posY + _activeElement.offsetY, _activeElement.attr, &_activeElement.buf[0]);
        _activeElement.buf[0] = '\0';
        _displayPendingForeground = false;
    }
    return false;
}

bool OSD_Elements::drawSingleElement(DisplayPortBase& displayPort, uint8_t elementIndex)
{
    //Serial.printf("drawSingleElement:%d\r\n", elementIndex);
    // By default mark the element as rendered in case it's in the off blink state
    _activeElement.rendered = true;

    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-constant-array-index)
    if (!elementDrawFunctions[elementIndex]) {
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
    if (isSysOSD_Element(elementIndex)) {
        displayPort.writeSys(_activeElement.posX, _activeElement.posY, static_cast<DisplayPortBase::system_element_e>(elementIndex - OSD_SYS_GOGGLE_VOLTAGE + DisplayPortBase::SYS_GOGGLE_VOLTAGE));
    } else {
        //Serial.print("calling draw fn\r\n");
        (this->*elementDrawFunctions[elementIndex])(displayPort);
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
    if (!elementDrawBackgroundFunctions[elementIndex]) {
        return true;
    }

    _activeElement.type = ELEMENT_TYPE(_config.element_pos[elementIndex]);
    _activeElement.index = elementIndex;
    _activeElement.posX = OSD_X(_config.element_pos[elementIndex]);
    _activeElement.posY = OSD_Y(_config.element_pos[elementIndex]);
    _activeElement.offsetX = 0;
    _activeElement.offsetY = 0;
    _activeElement.attr = DisplayPortBase::SEVERITY_NORMAL;

    (this->*elementDrawBackgroundFunctions[elementIndex])(displayPort);
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
            while (!drawSingleElementBackground(displayPort, _activeElementArray[ii])) {};
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
