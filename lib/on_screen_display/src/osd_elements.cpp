#include "cockpit.h"
#include "display_port_base.h"
#include "osd_elements.h"

#if !defined(FRAMEWORK_TEST)
//#include <HardwareSerial.h>
#endif

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise)


OSD_Elements::OSD_Elements(const OSD& osd) :
    _osd(osd)
{
}

void OSD_Elements::init(bool backgroundLayerFlag, uint8_t row_count, uint8_t column_count)
{
    (void)backgroundLayerFlag;
    initDrawFunctions();

    if (column_count !=0  && row_count != 0) {
        // Ensure that all OSD elements are on the canvas once the number of row and columns is known
        //for (size_t ii = 0; ii < OSD_ELEMENT_COUNT; ++ii) { // NOLINT(modernize-loop-convert)
        for (auto& elementPos : _config.element_pos) {
            //const uint16_t elementPos = _config.element_pos[ii];
            const uint16_t elementTopBits = elementPos & (ELEMENT_TYPE_MASK | PROFILE_MASK);

            uint8_t pos_x = OSD_X(elementPos);
            uint8_t pos_y = OSD_Y(elementPos);
            if (pos_x >= column_count) {
                pos_x = column_count - 1;
                elementPos = elementTopBits | OSD_POS(pos_x, pos_y);
            }
            if (pos_y >= row_count) {
                pos_y = row_count - 1;
                elementPos = elementTopBits | OSD_POS(pos_x, pos_y);
            }
        }
    }
};

void OSD_Elements::set_profile(uint8_t profile)
{
    if (profile > 1) {
        profile = 1;
    }
    _profile = profile;
}

void OSD_Elements::set_config(const osd_elements_config_t& config)
{
    _config = config;
}

void OSD_Elements::set_default_config(uint8_t row_count, uint8_t column_count)
{
    const uint8_t midRow = row_count/2;
    const uint8_t midCol = column_count/2;

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

uint32_t OSD_Elements::display_write_string(DisplayPortBase& display_port, const element_t&  element, uint8_t x, uint8_t y, const char *s, uint8_t attr)
{
    if (_blink_bits[element.index]) {
        attr |= DisplayPortBase::BLINK;
    }
    return display_port.write_string(x, y, s, attr);
}

uint32_t OSD_Elements::display_write_char(DisplayPortBase& display_port, const element_t& element, uint8_t x, uint8_t y, uint8_t c, uint8_t attr)
{
    if (_blink_bits[element.index]) {
        attr |= DisplayPortBase::BLINK;
    }
    return display_port.write_char(x, y, c, attr);
}

bool OSD_Elements::isRenderPending() const
{
    return _displayPendingForeground | _displayPendingBackground;
}

void OSD_Elements::add_active_element(osd_elements_e element)
{
    if (element_visible(_config.element_pos[element], _profile)) {
        _active_elements[_active_element_count++] = element;
    }
}

void OSD_Elements::add_active_elements(const osd_context_t& ctx)
{
    (void)ctx;

    add_active_element(OSD_ARTIFICIAL_HORIZON);
    add_active_element(OSD_G_FORCE);
    add_active_element(OSD_UP_DOWN_REFERENCE);

    add_active_element(OSD_MAIN_BATTERY_VOLTAGE);
    add_active_element(OSD_RSSI_VALUE);
    add_active_element(OSD_CROSSHAIRS);
    add_active_element(OSD_HORIZON_SIDEBARS);
    add_active_element(OSD_UP_DOWN_REFERENCE);
    add_active_element(OSD_ITEM_TIMER_1);
    add_active_element(OSD_ITEM_TIMER_2);
    add_active_element(OSD_REMAINING_TIME_ESTIMATE);
    add_active_element(OSD_FLYMODE);
    add_active_element(OSD_THROTTLE_POS);
    add_active_element(OSD_VTX_CHANNEL);
    add_active_element(OSD_CURRENT_DRAW);
    add_active_element(OSD_MAH_DRAWN);
    add_active_element(OSD_WATT_HOURS_DRAWN);
    add_active_element(OSD_CRAFT_NAME);
#if defined(USE_OSD_PROFILES)
    add_active_element(OSD_CUSTOM_MSG0);
    add_active_element(OSD_CUSTOM_MSG1);
    add_active_element(OSD_CUSTOM_MSG2);
    add_active_element(OSD_CUSTOM_MSG3);
#endif
#if defined(USE_BAROMETER) || defined(USE_GPS)
    add_active_element(OSD_ALTITUDE);
#endif
    add_active_element(OSD_ROLL_PIDS);
    add_active_element(OSD_PITCH_PIDS);
    add_active_element(OSD_YAW_PIDS);
    add_active_element(OSD_POWER);
    add_active_element(OSD_PID_RATE_PROFILE);
    add_active_element(OSD_WARNINGS);
    add_active_element(OSD_AVG_CELL_VOLTAGE);
    add_active_element(OSD_DEBUG);
    add_active_element(OSD_DEBUG2);
    add_active_element(OSD_PITCH_ANGLE);
    add_active_element(OSD_ROLL_ANGLE);
    add_active_element(OSD_MAIN_BATTERY_USAGE);
    add_active_element(OSD_DISARMED);
    add_active_element(OSD_NUMERICAL_HEADING);
    add_active_element(OSD_READY_MODE);
#if defined(USE_VARIO)
    add_active_element(OSD_NUMERICAL_VARIO);
#endif
#if defined(USE_GPS)
    add_active_element(OSD_COMPASS_BAR);
#endif
    add_active_element(OSD_ANTI_GRAVITY);
#if defined(USE_BLACKBOX)
    add_active_element(OSD_LOG_STATUS);
#endif
#if defined(USE_DSHOT)
    add_active_element(OSD_MOTOR_DIAGNOSTICS);
#endif
    add_active_element(OSD_FLIP_ARROW);
#if defined(USE_OSD_PROFILES)
    add_active_element(OSD_PILOT_NAME);
#endif
#if defined(USE_RTC_TIME)
    add_active_element(OSD_RTC_DATETIME);
#endif
#if defined(USE_OSD_ADJUSTMENTS)
    add_active_element(OSD_ADJUSTMENT_RANGE);
#endif
#if defined(USE_ADC_INTERNAL)
    add_active_element(OSD_CORE_TEMPERATURE);
#endif
#if defined(USE_RX_LINK_QUALITY_INFO)
    add_active_element(OSD_LINK_QUALITY);
#endif
#if defined(USE_RX_LINK_UPLINK_POWER)
    add_active_element(OSD_TX_UPLINK_POWER);
#endif
#if defined(USE_RX_RSSI_DBM)
    add_active_element(OSD_RSSI_DBM_VALUE);
#endif
#if defined(USE_RX_RSNR)
    add_active_element(OSD_RSNR_VALUE);
#endif
#if defined(USE_OSD_STICK_OVERLAY)
    add_active_element(OSD_STICK_OVERLAY_LEFT);
    add_active_element(OSD_STICK_OVERLAY_RIGHT);
#endif
#if defined(USE_PROFILE_NAMES)
    add_active_element(OSD_RATE_PROFILE_NAME);
    add_active_element(OSD_PID_PROFILE_NAME);
#endif
#if defined(USE_OSD_PROFILES)
    add_active_element(OSD_PROFILE_NAME);
#endif
    add_active_element(OSD_RC_CHANNELS);
    add_active_element(OSD_CAMERA_FRAME);
#if defined(USE_PERSISTENT_STATS)
    add_active_element(OSD_TOTAL_FLIGHTS);
#endif
    add_active_element(OSD_AUX_VALUE);
#if defined(USE_OSD_HD)
    add_active_element(OSD_SYS_GOGGLE_VOLTAGE);
    add_active_element(OSD_SYS_VTX_VOLTAGE);
    add_active_element(OSD_SYS_BITRATE);
    add_active_element(OSD_SYS_DELAY);
    add_active_element(OSD_SYS_DISTANCE);
    add_active_element(OSD_SYS_LQ);
    add_active_element(OSD_SYS_GOGGLE_DVR);
    add_active_element(OSD_SYS_VTX_DVR);
    add_active_element(OSD_SYS_WARNINGS);
    add_active_element(OSD_SYS_VTX_TEMP);
    add_active_element(OSD_SYS_FAN_SPEED);
#endif
#if defined(USE_RANGEFINDER)
    add_active_element(OSD_LIDAR_DISTANCE);
#endif
#if defined(USE_GPS)
    //if (sensors(SENSOR_GPS)) {
        add_active_element(OSD_GPS_SATS);
        add_active_element(OSD_GPS_SPEED);
        add_active_element(OSD_GPS_LAT);
        add_active_element(OSD_GPS_LON);
        add_active_element(OSD_HOME_DISTANCE);
        add_active_element(OSD_HOME_DIRECTION);
        add_active_element(OSD_FLIGHT_DISTANCE);
        add_active_element(OSD_EFFICIENCY);
    //}
#endif
#if defined(USE_DSHOT)
    if (ctx.cockpit.feature_is_enabled(Features::FEATURE_ESC_SENSOR)) {
        add_active_element(OSD_ESC_TEMPERATURE);
        add_active_element(OSD_ESC_RPM);
        add_active_element(OSD_ESC_RPM_FREQUENCY);
    }
#endif
#if defined(USE_GPS_LAP_TIMER)
    //if (sensors(SENSOR_GPS)) {
        add_active_element(OSD_GPS_LAP_TIME_CURRENT);
        add_active_element(OSD_GPS_LAP_TIME_PREVIOUS);
        add_active_element(OSD_GPS_LAP_TIME_BEST3);
    //}
#endif // USE_GPS_LAP_TIMER
    add_active_element(OSD_TOTAL_FLIGHTS);
}

void OSD_Elements::update_attitude(float roll_angle_degrees, float pitch_angle_degrees, float yaw_angle_degrees)
{
    _roll_angle_degrees = roll_angle_degrees;
    _pitch_angle_degrees = pitch_angle_degrees;
    _yaw_angle_degrees = yaw_angle_degrees;
}

bool OSD_Elements::draw_next_active_element(const osd_context_t& ctx)
{
    //Serial.printf("draw_next_active_element: idx:%d cnt:%d\r\n", _active_element_index, _active_element_count);
    if (_active_element_index >= _active_element_count) {
        _active_element_index = 0;
        return false;
    }

    const uint8_t element = _active_elements[_active_element_index];

    if (!_background_is_layer_supported && DrawBackgroundFunctions[element] && !_backgroundRendered) {
        // If the background layer isn't supported then we
        // have to draw the element's static layer as well.
        _backgroundRendered = draw_single_element_background(ctx, element);
        // After the background always come back to check for foreground
        return true;
    }

    if (draw_single_element(ctx, element)) {
        // If rendering is complete then advance to the next element
        // Prepare to render the background of the next element
        _backgroundRendered = false;
        if (++_active_element_index >= _active_element_count) {
            _active_element_index = 0;
            return false;
        }
    }
    return true;
}

/*!
Returns true if there is more to display
*/
bool OSD_Elements::display_active_element(DisplayPortBase& display_port)
{
    if (_active_element_index >= _active_element_count) {
        return false;
    }
    // If there's a previously drawn background string to be displayed, do that
    if (_displayPendingBackground) {
        display_write_string(display_port, _active_element, _active_element.pos_x + _active_element.offset_x, _active_element.pos_y + _active_element.offset_y, &_active_element.buf[0], _active_element.attr);
        _active_element.buf[0] = '\0';
        _displayPendingBackground = false;
        return _displayPendingForeground;
    }
    // If there's a previously drawn foreground string to be displayed, do that
    if (_displayPendingForeground) {
        display_write_string(display_port, _active_element, _active_element.pos_x + _active_element.offset_x, _active_element.pos_y + _active_element.offset_y, &_active_element.buf[0], _active_element.attr);
        _active_element.buf[0] = '\0';
        _displayPendingForeground = false;
    }
    return false;
}

bool OSD_Elements::draw_single_element(const osd_context_t& ctx, uint8_t element_index) // cppcheck-suppress constParameterReference
{
    //Serial.printf("draw_single_element:%d\r\n", element_index);
    // By default mark the element as rendered in case it's in the off blink state
    _active_element.rendered = true;

    // NOLINTBEGIN(cppcoreguidelines-pro-bounds-constant-array-index)
    if (!DrawFunctions[element_index]) {
        // Element has no drawing function
        return true;
    }
    if (!ctx.display_port.get_use_device_blink() && _blink_bits[element_index]) {
        return true;
    }

    _active_element.type = ELEMENT_TYPE(_config.element_pos[element_index]);
    _active_element.index = element_index;
    _active_element.pos_x = OSD_X(_config.element_pos[element_index]);
    _active_element.pos_y = OSD_Y(_config.element_pos[element_index]);
    _active_element.offset_x = 0;
    _active_element.offset_y = 0;
    _active_element.attr = DisplayPortBase::SEVERITY_NORMAL;
    _active_element.draw_element = true;

    // Call the element drawing function
    if (isSysOSD_Element(element_index)) { // cppcheck-suppress knownConditionTrueFalse
#if defined(USE_OSD_HD)
        display_port.write_sys(_active_element.pos_x, _active_element.pos_y, static_cast<DisplayPortBase::system_element_e>(element_index - OSD_SYS_GOGGLE_VOLTAGE + DisplayPortBase::SYS_GOGGLE_VOLTAGE));
#endif
    } else {
        //Serial.print("calling draw fn\r\n");
        (this->*DrawFunctions[element_index])(ctx);
        if (_active_element.draw_element) {
            _displayPendingForeground = true;
        }
    }
    // NOLINTEND(cppcoreguidelines-pro-bounds-constant-array-index)

    return _active_element.rendered;
}

bool OSD_Elements::draw_single_element_background(const osd_context_t& ctx, uint8_t element_index)
{
     // NOLINTBEGIN(cppcoreguidelines-pro-bounds-constant-array-index)
    if (!DrawBackgroundFunctions[element_index]) {
        return true;
    }

    _active_element.type = ELEMENT_TYPE(_config.element_pos[element_index]);
    _active_element.index = element_index;
    _active_element.pos_x = OSD_X(_config.element_pos[element_index]);
    _active_element.pos_y = OSD_Y(_config.element_pos[element_index]);
    _active_element.offset_x = 0;
    _active_element.offset_y = 0;
    _active_element.attr = DisplayPortBase::SEVERITY_NORMAL;

    (this->*DrawBackgroundFunctions[element_index])(ctx);
    if (_active_element.draw_element) {
        _displayPendingBackground = true;
    }
    // NOLINTEND(cppcoreguidelines-pro-bounds-constant-array-index)

    return _active_element.rendered;
}

void OSD_Elements::draw_active_elements_background(const osd_context_t& ctx) // NOLINT(readability-make-member-function-const)
{
    if (_background_is_layer_supported) {
        ctx.display_port.layer_select(DisplayPortBase::LAYER_BACKGROUND);
        ctx.display_port.clear_screen(DISPLAY_CLEAR_WAIT);
        for (size_t ii = 0; ii < _active_element_count; ++ii) {
            while (!draw_single_element_background(ctx, _active_elements[ii])) {};
        }
        ctx.display_port.layer_select(DisplayPortBase::LAYER_FOREGROUND);
    }
}

bool OSD_Elements::draw_spec(const osd_context_t& ctx)
{
    (void)ctx;
    return true;
}

// NOLINTEND(cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-signed-bitwise)
