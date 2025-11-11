#include "DisplayPortBase.h"
#include "OSD_Elements.h"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise)

OSD_Elements::OSD_Elements(const OSD& osd, const FlightController& flightController) :
    _osd(osd),
    _flightController(flightController)
{
}

void OSD_Elements::init(bool backgroundLayerFlag)
{
    (void)backgroundLayerFlag;
    initDrawFunctions();
};

void OSD_Elements::setConfig(const config_t& config)
{
    _config = config;
}


int OSD_Elements::displayWrite(DisplayPortBase& displayPort, const element_t&  element, uint8_t x, uint8_t y, uint8_t attr, const char *s)
{
    if (_blinkBits[element.index]) {
        attr |= DisplayPortBase::BLINK;
    }
    return displayPort.writeString(x, y, attr, s);
}

int OSD_Elements::displayWrite(DisplayPortBase& displayPort, const element_t& element, uint8_t x, uint8_t y, uint8_t attr, char c)
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

uint8_t OSD_Elements::getActiveElement() const 
{
    return _activeElementIndex;
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
    if (!elementDrawFunctions[elementIndex]) { // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
        // Element has no drawing function
        return true;
    }
    if (!displayPort.getUseDeviceBlink() && _blinkBits[elementIndex]) {
        return true;
    }

    _activeElement.index = elementIndex;
    _activeElement.posX = OSD_X(_config.item_pos[elementIndex]); // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    _activeElement.posY = OSD_Y(_config.item_pos[elementIndex]); // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    _activeElement.offsetX = 0;
    _activeElement.offsetY = 0;
    _activeElement.type = static_cast<element_type_e>(OSD_TYPE(_config.item_pos[elementIndex])); // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    _activeElement.drawElement = true;
    _activeElement.attr = DisplayPortBase::SEVERITY_NORMAL;

    // Call the element drawing function
    if (isSysOSD_Element(elementIndex)) {
        displayPort.writeSys(_activeElement.posX, _activeElement.posY, static_cast<DisplayPortBase::system_element_e>(elementIndex - OSD_SYS_GOGGLE_VOLTAGE + DisplayPortBase::SYS_GOGGLE_VOLTAGE));
    } else {
        const OSD_Elements::elementDrawFn drawFn = elementDrawFunctions[elementIndex]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
        (this->*drawFn)(_activeElement);
        if (_activeElement.drawElement) {
            _displayPendingForeground = true;
        }
    }

    return _activeElement.rendered;
}

bool OSD_Elements::drawSingleElementBackground(DisplayPortBase& displayPort, uint8_t elementIndex)
{
    (void)displayPort;

    if (!elementDrawBackgroundFunctions[elementIndex]) { // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
        return true;
    }

    _activeElement.index = elementIndex;
    _activeElement.posX = OSD_X(_config.item_pos[elementIndex]); // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    _activeElement.posY = OSD_Y(_config.item_pos[elementIndex]); // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    _activeElement.offsetX = 0;
    _activeElement.offsetY = 0;
    _activeElement.type = static_cast<element_type_e>(OSD_TYPE(_config.item_pos[elementIndex])); // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    _activeElement.drawElement = true;
    _activeElement.attr = DisplayPortBase::SEVERITY_NORMAL;

    const OSD_Elements::elementDrawFn drawFn = elementDrawBackgroundFunctions[elementIndex]; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    (this->*drawFn)(_activeElement);
    if (_activeElement.drawElement) {
        _displayPendingBackground = true;
    }

    return _activeElement.rendered;
}
// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise)
