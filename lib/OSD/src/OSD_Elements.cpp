#include "DisplayPortBase.h"
#include "OSD_Elements.h"

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
#ifdef USE_OSD_HD
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

    uint16_t profileFlags = 0;
    for (uint16_t ii = 0; ii <= PROFILE_COUNT; ++ii) {
        profileFlags |= profileFlag(ii);
    }
    // Always enable warnings elements by default

    enum { OSD_WARNINGS_PREFERRED_SIZE = 12 };
    _config.element_pos[OSD_WARNINGS] = OSD_POS(midCol - OSD_WARNINGS_PREFERRED_SIZE/2, midRow + 3) | profileFlags;

    // Default to old fixed positions for these elements
    _config.element_pos[OSD_CROSSHAIRS]         = OSD_POS(midCol - 2,  midRow - 1);
    _config.element_pos[OSD_ARTIFICIAL_HORIZON] = OSD_POS(midCol - 1,  midRow - 5);
    _config.element_pos[OSD_HORIZON_SIDEBARS]   = OSD_POS(midCol - 1,  midRow - 1);
    _config.element_pos[OSD_CAMERA_FRAME]       = OSD_POS(midCol - 12, midRow - 6);
    _config.element_pos[OSD_UP_DOWN_REFERENCE]  = OSD_POS(midCol - 2,  midRow - 1);
}

int OSD_Elements::displayWrite(DisplayPortBase& displayPort, const element_t&  element, uint8_t x, uint8_t y, uint8_t attr, const char *s)
{
    if (_blinkBits[element.index]) {
        attr |= DisplayPortBase::BLINK;
    }
    return displayPort.writeString(x, y, attr, s);
}

int OSD_Elements::displayWrite(DisplayPortBase& displayPort, const element_t& element, uint8_t x, uint8_t y, uint8_t attr, uint8_t c)
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

void OSD_Elements::addActiveElements()
{
}

bool OSD_Elements::drawNextActiveElement(DisplayPortBase& displayPort)
{
    if (_activeElementIndex >= _activeElementCount) {
        _activeElementIndex = 0;
        return false;
    }

    const uint8_t item = _activeOsdElementArray[_activeElementIndex];

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

    // Call the element drawing function
    if (isSysOSD_Element(elementIndex)) {
        displayPort.writeSys(_activeElement.posX, _activeElement.posY, static_cast<DisplayPortBase::system_element_e>(elementIndex - OSD_SYS_GOGGLE_VOLTAGE + DisplayPortBase::SYS_GOGGLE_VOLTAGE));
    } else {
        if ((this->*elementDrawFunctions[elementIndex])(displayPort, _activeElement)) {
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

    if ((this->*elementDrawBackgroundFunctions[elementIndex])(displayPort, _activeElement)) {
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
            while (!drawSingleElementBackground(displayPort, _activeOsdElementArray[ii])) {};
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
