#include "cockpit.h"
#include "display_port_base.h"
#include "format_integer.h"
#include "msp_box.h"
#include "osd.h"
#include "rc_modes.h"

//#include <HardwareSerial.h>
#include <ahrs_message_queue.h>
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

#include <receiver_base.h>

// NOLINTBEGIN(cppcoreguidelines-macro-usage,cppcoreguidelines-pro-bounds-constant-array-index,hicpp-signed-bitwise)

OSD::OSD() : // NOLINT(hicpp-use-equals-default,modernize-use-equals-default)
    _elements(*this)
{
}

void OSD::init(const DisplayPortBase* display_port)
{
    const uint8_t row_count = display_port->get_row_count();
    const uint8_t column_count = display_port->get_column_count();
    if (column_count !=0  && row_count != 0) {
        _config.canvas_column_count = column_count;
        _config.canvas_row_count = row_count;
    }
}

void OSD::draw_logo_and_complete_initialization(const osd_context_t& ctx)
{
    uint8_t midRow = ctx.display_port.get_row_count() / 2;
    uint8_t midCol = ctx.display_port.get_column_count() / 2;

    //resetAlarms();

    _background_layer_supported = ctx.display_port.layer_supported(DisplayPortBase::LAYER_BACKGROUND);
    ctx.display_port.layer_select(DisplayPortBase::LAYER_FOREGROUND);

    ctx.display_port.clear_screen(DISPLAY_CLEAR_WAIT);

    // Display betaflight logo
    draw_logo(ctx.display_port, midCol - (LOGO_COLUMN_COUNT) / 2, midRow - 5);

    std::array<char, 30> string_buffer;
    sprintf(&string_buffer[0], "V%s", "0.0.1");//FC_VERSION_STRING);
    ctx.display_port.write_string(midCol + 5, midRow, &string_buffer[0]);
#if defined(USE_CMS) || true
    ctx.display_port.write_string(midCol - 8, midRow + 2, "MENU:THR MID");
    ctx.display_port.write_string(midCol - 4, midRow + 3, "+ YAW LEFT");
    ctx.display_port.write_string(midCol - 4, midRow + 4, "+ PITCH UP");
#endif

#if defined(USE_RTC_TIME)
    std::array<char, FORMATTED_DATE_TIME_BUFSIZE> dateTimeBuffer;
    if (osdFormatRtcDateTime(&dateTimeBuffer[0])) {
        ctx.display_port.write_string(midCol - 10, midRow + 6, &dateTimeBuffer[]);
    }
#endif

    _resume_refresh_at_us = time_us() + 4'000'000;
#if defined(USE_OSD_PROFILES)
    _elements.set_profile(_config.osd_profile_index);
#endif

    _elements.init(_background_layer_supported, ctx.display_port.get_row_count(), ctx.display_port.get_column_count());
    analyze_active_elements(ctx);

    ctx.display_port.redraw();

    _is_ready = true;
}

void OSD::analyze_active_elements(const osd_context_t& ctx)
{
    _elements.add_active_elements(ctx);
    _elements.draw_active_elements_background(ctx);
}


void OSD::set_config(const osd_config_t& config)
{
    _config = config;
}

void OSD::set_stats_config(const osd_stats_config_t& stats_config)
{
    _stats_config = stats_config;
}

void OSD::set_stats_state(uint8_t stats_index, bool enabled)
{
    if (enabled) {
        _config.enabled_stats_flags |= (1U << stats_index);
    } else {
        _config.enabled_stats_flags &= ~(1U << stats_index);
    }
}

bool OSD::get_stats_state(uint8_t stats_index) const
{
    return _config.enabled_stats_flags & (1U << stats_index);
}

void OSD::reset_stats()
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

void OSD::display_statistic_label(DisplayPortBase& display_port, uint8_t x, uint8_t y, const char * text, const char * value)
{
    display_port.write_string(x - 13, y, text);
    display_port.write_string(x + 5, y, ":");
    display_port.write_string(x + 7, y, value);
}

bool OSD::display_statistic(DisplayPortBase& display_port, int statistic, uint8_t display_row)
{
    const uint8_t midCol = display_port.get_column_count() / 2;
    enum { ELEMENT_BUFFER_LENGTH = 32 };
    std::array<char, ELEMENT_BUFFER_LENGTH> buf {};

    switch (statistic) { // NOLINT(hicpp-multiway-paths-covered)
    case STATS_TOTAL_FLIGHTS:
        ui2a(_stats_config.total_flights, &buf[0]);
        display_statistic_label(display_port, midCol, display_row, "TOTAL FLIGHTS", &buf[0]);
        return true;
    default:
        return false;
    }
}
/*
Called repeatedly until it returns true which indicates that all stats have been rendered.
*/
bool OSD::render_stats_continue(DisplayPortBase& display_port)
{
#if defined(USE_OSD_STATS)
    const uint8_t midCol = display_port.get_column_count() / 2;

    if (_stats_rendering_state.row == 0) {
        bool displayLabel = false;
        // if row_count is 0 then we're running an initial analysis of the active stats items
        if (_stats_rendering_state.row_count > 0) {
            const uint8_t availableRows = display_port.get_row_count();
            uint8_t display_rows = std::min(_stats_rendering_state.row_count, availableRows);
            if (_stats_rendering_state.row_count < availableRows) {
                displayLabel = true;
                ++display_rows;
            }
            _stats_rendering_state.row = static_cast<uint8_t>((availableRows - display_rows) / 2);  // center the stats vertically
        }
        if (displayLabel) {
            display_port.write_string(midCol - static_cast<uint8_t>(strlen("--- STATS ---") / 2), _stats_rendering_state.row, "--- STATS ---");
            ++_stats_rendering_state.row;
            return false;
        }
    }
    bool statisticDisplayed = false;
    while (_stats_rendering_state.index < STATS_COUNT) {
        const uint8_t index = _stats_rendering_state.index;
        // prepare for the next call to the method
        ++_stats_rendering_state.index;
        // look for something to render
        if (_config.enabled_stats_flags & (1<< StatsDisplayOrder[index])) {
            if (display_statistic(display_port, StatsDisplayOrder[index], _stats_rendering_state.row)) {
                ++_stats_rendering_state.row;
                statisticDisplayed = true;
                break;
            }
        }
    }
    const bool moreSpaceAvailable = _stats_rendering_state.row < display_port.get_row_count();
    if (statisticDisplayed && moreSpaceAvailable) {
        return false;
    }
    if (_stats_rendering_state.row_count == 0) {
        _stats_rendering_state.row_count = _stats_rendering_state.row;
    }
#else
    (void)display_port;
#endif
    return true;
}

/*!
State machine to refresh statistics.
Returns true when all iterations are complete.
*/
bool OSD::refreshStats(DisplayPortBase& display_port)
{
    switch (_refresh_stats_state) {
    default:
        [[fallthrough]];
    case REFRESH_STATS_STATE_INITIAL_CLEAR_SCREEN:
        //Serial.printf("REFRESH_STATS_STATE_INITIAL_CLEAR_SCREEN\r\n");
        _stats_rendering_state.row = 0;
        _stats_rendering_state.index = 0;
        if (_stats_rendering_state.row_count > 0) {
            _refresh_stats_state = REFRESH_STATS_STATE_RENDER_STATS;
        } else {
            _refresh_stats_state = REFRESH_STATS_STATE_COUNT_STATS;
        }
        display_port.clear_screen(DISPLAY_CLEAR_NONE);
        break;
    case REFRESH_STATS_STATE_COUNT_STATS:{
        //Serial.printf("REFRESH_STATS_STATE_COUNT_STATS\r\n");
        // No stats row count has been set yet.
        // Go through the logic one time to determine how many stats are actually displayed.
        bool count_phase_complete = render_stats_continue(display_port);
        if (count_phase_complete) { // cppcheck-suppress knownConditionTrueFalse
            _refresh_stats_state = REFRESH_STATS_STATE_CLEAR_SCREEN;
        }
        break;
    }
    case REFRESH_STATS_STATE_CLEAR_SCREEN:
        //Serial.printf("REFRESH_STATS_STATE_CLEAR_SCREEN\r\n");
        _stats_rendering_state.row = 0;
        _stats_rendering_state.index = 0;
        // Then clear the screen and commence with normal stats display which will
        // determine if the heading should be displayed and also center the content vertically.
        display_port.clear_screen(DISPLAY_CLEAR_NONE);
        _refresh_stats_state = REFRESH_STATS_STATE_RENDER_STATS;
        break;
    case REFRESH_STATS_STATE_RENDER_STATS:
        //Serial.printf("REFRESH_STATS_STATE_RENDER_STATS\r\n");
        if (render_stats_continue(display_port)) { // cppcheck-suppress knownConditionTrueFalse
            _refresh_stats_state = REFRESH_STATS_STATE_INITIAL_CLEAR_SCREEN;
            return true;
        }
        break;
    };

    return false;
}

/*!
Returns true if refresh stats is required.
*/
bool OSD::process_stats1(const osd_context_t& ctx, time_us32_t current_time_us)
{
    (void)ctx;
    (void)current_time_us;
    return false;
}

void OSD::process_stats2(const osd_context_t& ctx, time_us32_t current_time_us)
{
    if (_resume_refresh_at_us) {
        if (current_time_us < _resume_refresh_at_us) {
            // in timeout period, check sticks for activity or CRASH_FLIP switch to resume display.
            if (!ctx.cockpit.is_armed()) {
                const receiver_controls_pwm_t controls = ctx.receiver.get_controls_pwm();
                if (RcModes::pwm_is_high(controls.throttle) || RcModes::pwm_is_high(controls.pitch) || ctx.rc_modes.is_mode_active(MspBox::BOX_CRASH_FLIP)) {
                    _resume_refresh_at_us = current_time_us;
                }
            }
            return;
        }
        ctx.display_port.clear_screen(DISPLAY_CLEAR_NONE);
        _resume_refresh_at_us = 0;
        _stats_enabled = false;
        _stats.armed_time = 0;
        //schedulerIgnoreTaskExecTime();
    }
#if defined(USE_ESC_SENSOR)
    if (feature_is_enabled(FEATURE_ESC_SENSOR)) {
        osdEscDataCombined = getEscSensorData(ESC_SENSOR_COMBINED);
    }
#endif
}

void OSD::process_stats3(const osd_context_t& ctx)
{
    (void)ctx;
}

void OSD::update_alarms()
{
}

void OSD::sync_blink(time_us32_t current_time_us)
{
    (void)current_time_us;
}


void OSD::set_warning_state(uint8_t warning_index, bool enabled)
{
    if (enabled) {
        _config.enabled_warnings_flags |= (1U << warning_index);
    } else {
        _config.enabled_warnings_flags &= ~(1U << warning_index);
    }
}

bool OSD::get_warning_state(uint8_t warning_index) const
{
    return _config.enabled_warnings_flags & (1U << warning_index);
}

void OSD::draw_logo(DisplayPortBase& display_port, uint8_t x, uint8_t y)
{
    // the logo is in the font characters starting at 160
    enum { START_CHARACTER = 160 };
    enum { END_OF_FONT = 255 };

    // display logo and help
    uint8_t characterCode = START_CHARACTER;
    for (uint8_t row = 0; row < LOGO_ROW_COUNT; ++row) {
        for (uint8_t column = 0; column < LOGO_COLUMN_COUNT; ++column) {
            if (characterCode < END_OF_FONT) {
                display_port.write_char(x + column, y + row, characterCode);
                ++characterCode;
            }
        }
    }
}

void OSD::update_display(const osd_context_t& ctx, uint32_t time_microseconds, uint32_t time_microseconds_delta)
{
    if (ctx.display_port.is_grabbed()) {
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
        update_display_iteration(ctx, time_microseconds, time_microseconds_delta);
    }
}

void OSD::update_display_iteration(const osd_context_t& ctx, uint32_t time_microseconds, uint32_t time_microseconds_delta) // NOLINT(readability-function-cognitive-complexity)
{
    (void)time_microseconds_delta;

    switch (_state) {
    case STATE_INIT:
        //Serial.printf("STATE_INIT\r\n");
        if (!ctx.display_port.check_ready(false)) {
            // Frsky OSD needs a display redraw after search for MAX7456 devices
            if (ctx.display_port.get_device_type() == DisplayPortBase::DEVICE_TYPE_FRSKY_OSD) {
                ctx.display_port.redraw();
                return;
            }
        }
        ctx.display_port.begin_transaction(DISPLAY_TRANSACTION_OPTION_RESET_DRAWING);
        draw_logo_and_complete_initialization(ctx);
        _state = STATE_COMMIT;
        break;
    case STATE_CHECK:
        //Serial.printf("STATE_CHECK\r\n");
        // don't touch buffers if DMA transaction is in progress
        if (ctx.display_port.is_transfer_in_progress()) {
            break;
        }
        _state = STATE_UPDATE_HEARTBEAT;
        break;
    case STATE_UPDATE_HEARTBEAT:
        //Serial.printf("STATE_UPDATE_HEARTBEAT\r\n");
        if (ctx.display_port.heartbeat()) {
            // Extraordinary action was taken, so return without allowing state_duration_fraction_us table to be updated
            return;
        }
        _state = STATE_PROCESS_STATS1;
        break;
    case STATE_PROCESS_STATS1:
        //Serial.printf("STATE_PROCESS_STATS1\r\n");
        // transaction begins here since refreshStats draws to the screen
        ctx.display_port.begin_transaction(DISPLAY_TRANSACTION_OPTION_RESET_DRAWING);
        _state = process_stats1(ctx, time_microseconds) ? STATE_REFRESH_STATS : STATE_PROCESS_STATS2; // cppcheck-suppress knownConditionTrueFalse
        break;
    case STATE_REFRESH_STATS:
        //Serial.printf("STATE_REFRESH_STATS\r\n");
        if (refreshStats(ctx.display_port)) { // draws the statistics to the screen
            _state = STATE_PROCESS_STATS2;
        }
        break;
    case STATE_PROCESS_STATS2:
        //Serial.printf("STATE_PROCESS_STATS2\r\n");
        process_stats2(ctx, time_microseconds); // may clear screen
        _state = STATE_PROCESS_STATS3; // cppcheck-suppress redundantAssignment
        break;
    case STATE_PROCESS_STATS3:
        //Serial.printf("STATE_PROCESS_STATS3\r\n");
        process_stats3(ctx);
#if defined(USE_CMS)
        if (ctx.display_port.is_grabbed()) {
            _state = STATE_COMMIT;
        }
#endif
        _state = STATE_UPDATE_ALARMS; // cppcheck-suppress redundantAssignment
        break;
    case STATE_UPDATE_ALARMS:
        //Serial.printf("STATE_UPDATE_ALARMS\r\n");
        update_alarms();
        //!!_state = _resume_refresh_at_us ? STATE_TRANSFER : STATE_UPDATE_CANVAS;
        _state = STATE_UPDATE_CANVAS;
        break;
    case STATE_UPDATE_CANVAS: {
        //Serial.printf("STATE_UPDATE_CANVAS\r\n");
        if (ctx.rc_modes.is_mode_active(MspBox::BOX_OSD)) {
            // Hide OSD when OSD SW mode is active
            ctx.display_port.clear_screen(DISPLAY_CLEAR_NONE);
            _state = STATE_COMMIT;
            break;
        }
        if (_background_layer_supported) {
            // Background layer is supported, overlay it onto the foreground
            // so that we only need to draw the active parts of the elements.
            ctx.display_port.layer_copy(DisplayPortBase::LAYER_FOREGROUND, DisplayPortBase::LAYER_BACKGROUND);
        } else {
            // Background layer not supported, just clear the foreground in preparation
            // for drawing the elements including their backgrounds.
            ctx.display_port.clear_screen(DISPLAY_CLEAR_NONE);
        }
        sync_blink(time_microseconds);
        ahrs_data_t ahrs_data {};
        ctx.ahrs_message_queue.PEEK_AHRS_DATA(ahrs_data);
        _elements.update_attitude(ahrs_data.orientation.calculate_roll_degrees(), ahrs_data.orientation.calculate_pitch_degrees(), ahrs_data.orientation.calculate_yaw_degrees()); // update the AHRS data, so it is only needed to be done once for all elements that require it
        _state = STATE_DRAW_ELEMENT;
        break;
    }
    case STATE_DRAW_ELEMENT: {
        const uint8_t active_element_index = _elements.get_active_element_index();

        time_us32_t startElementTime = time_us();
        _more_elements_to_draw = _elements.draw_next_active_element(ctx);
        time_us32_t executeTimeUs = time_us() - startElementTime;
        //Serial.printf("STATE_DRAW_ELEMENT ai:%d more:%d\r\n", static_cast<int>(active_element_index), static_cast<int>(_more_elements_to_draw));

        enum { OSD_EXEC_TIME_SHIFT = 5 };
        if (executeTimeUs > (_element_duration_fraction_us[active_element_index] >> OSD_EXEC_TIME_SHIFT)) { // cppcheck-suppress unsignedLessThanZero
            _element_duration_fraction_us[active_element_index] = executeTimeUs << OSD_EXEC_TIME_SHIFT;
        } else if (_element_duration_fraction_us[active_element_index] > 0) {
            // Slowly decay the max time
            --_element_duration_fraction_us[active_element_index];
        }
        if (_elements.isRenderPending()) { // Render the element just drawn
            _state = STATE_DISPLAY_ELEMENT;
            break;
        }
        if (_more_elements_to_draw) {
            break;
        }
        _state = (ctx.cockpit.is_armed() && _config.osd_show_spec_prearm) ? STATE_REFRESH_PRE_ARM : STATE_COMMIT;
        break;
    }
    case STATE_DISPLAY_ELEMENT: {
        //Serial.printf("STATE_DISPLAY_ELEMENT\r\n");
        const bool moreToDisplay = _elements.display_active_element(ctx.display_port);
        if (!moreToDisplay) {
            // finished displaying this element, so move on to the next one if there is one
            if (_more_elements_to_draw) {
                _state = STATE_DRAW_ELEMENT;
            } else {
                _state = (ctx.cockpit.is_armed() && _config.osd_show_spec_prearm) ? STATE_REFRESH_PRE_ARM : STATE_COMMIT;
            }
        }
        break;
    }
    case STATE_REFRESH_PRE_ARM:
        //Serial.printf("STATE_REFRESH_PRE-ARM\r\n");
        if (_elements.draw_spec(ctx)) {
            // Rendering is complete
            _state = STATE_COMMIT;
        }
        break;
    case STATE_COMMIT:
        //Serial.printf("STATE_COMMIT\r\n");
        ctx.display_port.commit_transaction();
        _state = _resume_refresh_at_us ? STATE_IDLE : STATE_TRANSFER;
        break;
    case STATE_TRANSFER:
        //Serial.printf("STATE_TRANSFER\r\n");
        // Wait for any current transfer to complete
        if (ctx.display_port.is_transfer_in_progress()) {
            break;
        }
        // Transfer may be broken into many parts
        if (ctx.display_port.draw_screen()) {
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
