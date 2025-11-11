#include "DisplayPortBase.h"
#include "OSD_Elements.h"

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise)

OSD_Elements::OSD_Elements(OSD& osd, const FlightController& flightController, const Debug& debug) :
    _osd(osd),
    _flightController(flightController),
    _debug(debug)
{
}

void OSD_Elements::init(bool backgroundLayerFlag)
{
    (void)backgroundLayerFlag;
    initDrawFunctions();
};

static constexpr uint32_t OSD_PROFILE_BITS_POS = 11;
uint16_t OSD_PROFILE_FLAG(uint32_t x);
uint16_t OSD_PROFILE_FLAG(uint32_t x)
{ 
    return 1U << (x - 1 + OSD_PROFILE_BITS_POS);
}

void OSD_Elements::setConfigDefaults()
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
    for (auto& item_pos : _config.item_pos) { 
        item_pos = OSD_POS((midCol - 5), midRow); // cppcheck-suppress useStlAlgorithm
    }

    // Always enable warnings elements by default
    uint16_t profileFlags = 0;
    for (uint32_t ii = 0; ii <= OSD_PROFILE_COUNT; ++ii) {
        profileFlags |= OSD_PROFILE_FLAG(ii);
    }
    enum { OSD_WARNINGS_PREFERRED_SIZE = 12 }; // centered on OSD
    _config.item_pos[OSD_WARNINGS] = OSD_POS((midCol - OSD_WARNINGS_PREFERRED_SIZE / 2), (midRow + 3)) | profileFlags;

    // Default to old fixed positions for these elements
    _config.item_pos[OSD_CROSSHAIRS]         = OSD_POS((midCol - 2), (midRow - 1));
    _config.item_pos[OSD_ARTIFICIAL_HORIZON] = OSD_POS((midCol - 1), (midRow - 5));
    _config.item_pos[OSD_HORIZON_SIDEBARS]   = OSD_POS((midCol - 1), (midRow - 1));
    _config.item_pos[OSD_CAMERA_FRAME]       = OSD_POS((midCol - 12), (midRow - 6));
    _config.item_pos[OSD_UP_DOWN_REFERENCE]  = OSD_POS((midCol - 2), (midRow - 1));
}

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
