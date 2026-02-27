#include "BlackboxCallbacks.h"
#include "Cockpit.h"
#include "FlightController.h"
#include "RC_Modes.h"
#include <GPS.h>
#include <GPS_MessageQueue.h>


#include <ahrs.h>
#include <ahrs_message_queue.h>
#include <cmath>
#include <debug.h>
#if (__cplusplus >= 202002L)
#include <ranges>
#endif
#include <motor_mixer_base.h>
#include <receiver_base.h>


bool BlackboxCallbacks::is_armed(const blackbox_parameter_group_t& pg) const
{
    return pg.cockpit.isArmed();
}

bool BlackboxCallbacks::are_motors_running(const blackbox_parameter_group_t& pg) const
{
    return pg.motorMixer.motors_is_on();
}

bool BlackboxCallbacks::is_blackbox_mode_active(const blackbox_parameter_group_t& pg) const
{
    return pg.rc_modes.is_mode_active(MspBox::BOX_BLACKBOX);
}

bool BlackboxCallbacks::is_blackbox_erase_mode_active(const blackbox_parameter_group_t& pg) const
{
    return pg.rc_modes.is_mode_active(MspBox::BOX_BLACKBOX_ERASE);
}

bool BlackboxCallbacks::is_blackbox_mode_activation_condition_present(const blackbox_parameter_group_t& pg) const
{
    return pg.rc_modes.is_mode_activation_condition_present(MspBox::BOX_BLACKBOX);
}

uint32_t BlackboxCallbacks::get_arming_beep_time_microseconds(const blackbox_parameter_group_t& pg) const
{
    (void)pg;
    return 0;
}

void BlackboxCallbacks::beep(const blackbox_parameter_group_t& pg) const
{
    (void)pg;
}

uint32_t BlackboxCallbacks::rc_mode_activation_mask(const blackbox_parameter_group_t& pg) const
{
    return pg.cockpit.getFlightModeFlags();
}

void BlackboxCallbacks::load_slow_state(blackbox_slow_state_t& slowState, const blackbox_parameter_group_t& pg)
{
    //memcpy(&slowState->flightModeFlags, &_rc_mode_activation_bitset, sizeof(slowState->flightModeFlags)); //was flightModeFlags;
    slowState.flight_mode_flags = pg.cockpit.getFlightModeFlags();
    slowState.state_flags = 0; // this is GPS state
    slowState.failsafe_phase = pg.cockpit.getFailsafePhase();
    //slowState.rxSignalReceived = pg.receiver.isRxReceivingSignal();
    slowState.rx_signal_received = (slowState.failsafe_phase == Cockpit::FAILSAFE_IDLE);
    slowState.rx_flight_channel_is_valid = (slowState.failsafe_phase == Cockpit::FAILSAFE_IDLE);
}

/*!
Called from within Blackbox::logIteration().
*/
void BlackboxCallbacks::load_main_state(blackbox_main_state_t& mainState, uint32_t currentTimeUs, const blackbox_parameter_group_t& pg)
{
    (void)currentTimeUs;

    const ahrs_data_t ahrsData = pg.ahrs_message_queue.get_received_ahrs_data();
    Quaternion orientation = ahrsData.orientation;

// NOLINTBEGIN(cppcoreguidelines-pro-bounds-constant-array-index)

    static constexpr float RADIANS_TO_DEGREES {180.0F / 3.14159265358979323846F};
    static constexpr float gyroScale {RADIANS_TO_DEGREES * 10.0F};

    mainState.gyro_adc[0] = static_cast<int16_t>(std::lroundf(ahrsData.acc_gyro_rps.gyro_rps.x * gyroScale));
    mainState.gyro_adc[1] = static_cast<int16_t>(std::lroundf(ahrsData.acc_gyro_rps.gyro_rps.y * gyroScale));
    mainState.gyro_adc[2] = static_cast<int16_t>(std::lroundf(ahrsData.acc_gyro_rps.gyro_rps.z * gyroScale));
    mainState.gyro_unfiltered[0] = static_cast<int16_t>(std::lroundf(ahrsData.gyro_rps_unfiltered.x * gyroScale));
    mainState.gyro_unfiltered[1] = static_cast<int16_t>(std::lroundf(ahrsData.gyro_rps_unfiltered.y * gyroScale));
    mainState.gyro_unfiltered[2] = static_cast<int16_t>(std::lroundf(ahrsData.gyro_rps_unfiltered.z * gyroScale));
    // just truncate for acc
    mainState.acc_adc[0] = static_cast<int16_t>(ahrsData.acc_gyro_rps.acc.x * 4096);
    mainState.acc_adc[1] = static_cast<int16_t>(ahrsData.acc_gyro_rps.acc.y * 4096);
    mainState.acc_adc[2] = static_cast<int16_t>(ahrsData.acc_gyro_rps.acc.z * 4096);

    if (orientation.get_w() < 0.0F) {
        // negate orientation if W is negative
        orientation = -orientation;
    }
    (void)orientation;
    //mainState.orientation[0] = static_cast<int16_t>(orientation.getX() * 0x7FFF);
    //mainState.orientation[1] = static_cast<int16_t>(orientation.getY() * 0x7FFF);
    //mainState.orientation[2] = static_cast<int16_t>(orientation.getZ() * 0x7FFF);

    // iterate through roll, pitch, and yaw PIDs
#if (__cplusplus >= 202002L)
    for (auto ii : std::views::iota(size_t{0}, size_t{blackbox_main_state_t::XYZ_AXIS_COUNT})) {
#else
    for (size_t ii = 0; ii < blackbox_main_state_t::XYZ_AXIS_COUNT; ++ii) {
#endif
        const auto pid_index = static_cast<FlightController::pid_index_e>(ii);
        const PidController& pid = pg.flightController.getPID(pid_index);
        const pid_error_t pidError = pid.get_error();
        mainState.axis_pid_p[ii] = static_cast<int32_t>(std::lroundf(pidError.p));
        mainState.axis_pid_i[ii] = static_cast<int32_t>(std::lroundf(pidError.i));
        mainState.axis_pid_d[ii] = static_cast<int32_t>(std::lroundf(pidError.d));
        mainState.axis_pid_s[ii] = static_cast<int32_t>(std::lroundf(pidError.s));
        mainState.axis_pid_k[ii] = static_cast<int32_t>(std::lroundf(pidError.k));
        mainState.setpoint[ii] = static_cast<int16_t>(std::lroundf(pid.get_setpoint()));
#if defined(USE_MAGNETOMETER)
        mainState.mag_adc[ii] = static_cast<int16_t>(mag.magADC.v[ii]);
#endif
    }
    // log the final throttle value used in the mixer
    mainState.setpoint[3] = static_cast<int16_t>(std::lroundf(pg.motorMixer.get_throttle_command() * 1000.0F));

    // interval [1000,2000] for THROTTLE and [-500,+500] for ROLL/PITCH/YAW
    const receiver_controls_pwm_t controls = pg.receiver.get_controls_pwm(); // returns controls in range [1000, 2000]
    mainState.rc_command[0] = static_cast<int16_t>(controls.roll - ReceiverBase::CHANNEL_MIDDLE);
    mainState.rc_command[1] = static_cast<int16_t>(controls.pitch - ReceiverBase::CHANNEL_MIDDLE);
    mainState.rc_command[2] = static_cast<int16_t>(controls.yaw - ReceiverBase::CHANNEL_MIDDLE);
    mainState.rc_command[3] = static_cast<int16_t>(controls.throttle);

    static_assert(static_cast<int>(blackbox_main_state_t::DEBUG_VALUE_COUNT) == static_cast<int>(Debug::VALUE_COUNT));
    mainState.debug = pg.debug.getValues();

#if (__cplusplus >= 202002L)
    for (auto ii : std::views::iota(size_t{0}, size_t{pg.motorMixer.get_motor_count()})) {
#else
    for (size_t ii = 0; ii < pg.motorMixer.get_motor_count(); ++ ii) {
#endif
        mainState.motor[ii] = static_cast<int16_t>(std::lroundf(pg.motorMixer.get_motor_output(ii)));
#if defined(USE_DSHOT_TELEMETRY)
        mainState.erpm[ii] = static_cast<int16_t>(mixer.getMotorRPM(ii));
#endif
    }
    mainState.vbat_latest = static_cast<uint16_t>(11.6F * 10.0F);
    mainState.amperage_latest = static_cast<uint16_t>(0.67F * 10.0F);

#if defined(USE_BAROMETER)
    //mainState.baroAlt = baro.altitude;
#endif

#if defined(USE_RANGEFINDER)
    // Store the raw sonar value without applying tilt correction
    //mainState.surfaceRaw = rangefinderGetLatestAltitude();
#endif

    mainState.rssi = 0;//getRssi();

#if defined(USE_SERVOS)
    mainState.servo[0] = 1527;
    mainState.servo[1] = 1473;
    mainState.servo[2] = 1620;
    mainState.servo[4] = 1750;
    //for (size_t ii = 0; ii < blackboxMainState_t::MAX_SUPPORTED_SERVO_COUNT; ++ii) {
    //    mainState.servo[ii] = 1527;
    //}
#endif
// NOLINTEND(cppcoreguidelines-pro-bounds-constant-array-index)
}

void BlackboxCallbacks::load_gps_state(blackbox_gps_state_t& gpsState, const blackbox_parameter_group_t& pg)
{
    if (!pg.gps) {
        return;
    }
#if defined(USE_GPS)
    gps_message_data_t gpsMessageData {};
    pg.gps->getGPS_MessageQueue().PEEK_GPS_DATA(gpsMessageData);

    gpsState.time_of_week_ms = gpsMessageData.time_of_week_ms;
    //gpsState.interval_ms;
    //gpsState.homeLongitude_degrees1E7;
    //gpsState.homeLatitude_degrees1E7;
    //gpsState.homeAltitude_cm;
    gpsState.longitude_degrees1E7 = gpsMessageData.longitude_degrees1E7;
    gpsState.latitude_degrees1E7 = gpsMessageData.latitude_degrees1E7;
    gpsState.altitude_cm = gpsMessageData.altitude_cm;
    gpsState.velocity_north_cmps = gpsMessageData.velocity_north_cmps;
    gpsState.velocity_east_cmps = gpsMessageData.velocity_east_cmps;
    gpsState.velocity_down_cmps = gpsMessageData.velocity_down_cmps;
    gpsState.speed3d_cmps = gpsMessageData.speed3d_cmps;
    gpsState.ground_speed_cmps = gpsMessageData.ground_speed_cmps;
    gpsState.ground_course_deci_degrees= gpsMessageData.heading_deci_degrees;
    gpsState.satellite_count = gpsMessageData.satellite_count;
#else
    (void)gpsState;
#endif
}
